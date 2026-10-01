#!/usr/bin/env python3
"""
Hyperparameter sweep for the ScenarioMPC_ARX C simulation (see TUNING.md).

For every combination of the --grid values, builds the C simulation with
g++ in its own directory (a copy of the sources, with the mode switches of
setup.h set from the command line and the HP_* hyperparameters passed with
-D), runs --runs closed loops on random true plants (plant.h), and
aggregates the per-run summary into one CSV row per configuration and
scenario setting. The same seed gives every configuration the same plants
and noise, so rows are paired comparisons. With --plant true every run uses
THETA_TRUE_INIT (system_configs.h) and differs only by the noise.

Example (pendulum, horizon vs. MADS iterations, with and without scenarios):

    python3 tuning/sweep.py --system pendulum --runs 20 \\
        --grid HP_NHOR=10,15,20 --grid HP_MADS_ITER=10,50 --out nhor_iter.csv

Fixed point needs the ap_fixed headers (Vitis's include directory, or
github.com/Xilinx/HLS_arbitrary_Precision_Types):

    python3 tuning/sweep.py --system pendulum --fixed --ap-include <dir> ...

Only the Python standard library is used.
"""
import argparse
import csv
import itertools
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SYSTEMS = {
    "pendulum":     "SYSTEM_INVERTED_PENDULUM",
    "buck_loss":    "SYSTEM_BUCK_LOSS",
    "buck_alberto": "SYSTEM_BUCK_ALBERTO",
    "gain_demo":    "SYSTEM_GAIN_DEMO",
}
CTRL_MODES = {"scmpc": "CTRL_MODE_SCMPC", "pl": "CTRL_MODE_PL", "al": "CTRL_MODE_AL"}

# Hyperparameters whose defaults are needed for the latency estimate.
DEFAULTS_RE = r"#define HP_DEFAULT_{}\s+\(?(-?\d+)\)?"

# Latency model of costFunctionArx-dominated synthesis, PRAGMA_PROFILE_BALANCED
# (Nscen loop unrolled, Nhor loop rolled), in clock cycles:
#   MADS_ITER * 2*NhorU polls * Nhor steps * (LAT_A + LAT_B*Nscen)
# Fitted to two Vitis HLS 2021.1 reports: buck (Nhor 5, Nscen 4, MADS_ITER 7,
# NhorU 3) = 4706 cycles and pendulum (15, 16, 10, 3) = 41470 cycles. LAT_B is
# the serial accumulation over scenarios (saturating adds cannot be
# re-associated). Two points, so treat it as a rough guide.
LAT_A, LAT_B = 14.5, 2.0


def set_switch(text, name, on):
    """Turn a '#define NAME' line of setup.h on or off (commenting it)."""
    pat = re.compile(r"^[ \t]*(//[ \t]*)?#define[ \t]+" + name + r"\b", re.M)
    if not pat.search(text):
        sys.exit("setup.h: no '#define {}' line to switch".format(name))
    return pat.sub(("#define " if on else "// #define ") + name, text, count=1)


def set_define(text, name, value):
    pat = re.compile(r"^#define[ \t]+" + name + r"[ \t]+\S+", re.M)
    if not pat.search(text):
        sys.exit("setup.h: no '#define {}' line".format(name))
    return pat.sub("#define {} {}".format(name, value), text, count=1)


def hp_default(setup_text, system_macro, name):
    """HP_DEFAULT_<name> for the given system, read from setup.h."""
    blocks = re.split(r"^#if ACTIVE_SYSTEM == |^#else /\* buck", setup_text, flags=re.M)
    for b in blocks[1:]:
        if b.startswith(system_macro) or not b.startswith("SYSTEM_"):
            m = re.search(DEFAULTS_RE.format(name), b)
            if m:
                return int(m.group(1))
    return None


def build_and_run(cfg, args, setup_text, work_root):
    """Build and run one configuration; returns the list of summary rows."""
    tag = "_".join("{}{}".format(k.replace("HP_", "").lower(), v) for k, v in cfg["hp"].items()) or "default"
    tag += "_" + cfg["scen"]
    d = os.path.join(work_root, tag)
    os.makedirs(d)
    for f in os.listdir(SRC_DIR):
        if (f.endswith(".cpp") or f.endswith(".h")) and f != "intervalHull.cpp":
            shutil.copy(os.path.join(SRC_DIR, f), d)
    text = setup_text
    text = set_define(text, "ACTIVE_SYSTEM", SYSTEMS.get(args.system, args.system))
    text = set_define(text, "CTRL_MODE", CTRL_MODES[args.ctrl])
    text = set_switch(text, "FIXED", args.fixed)
    text = set_switch(text, "NRMLZ", args.nrmlz)
    text = set_switch(text, "CONVERSIONS_MODE", args.conv)
    text = set_switch(text, "USE_SCENS_COST", cfg["scen"] == "scen")
    text = set_switch(text, "USE_SCENS_CONSTR", cfg["scen"] == "scen")
    with open(os.path.join(d, "setup.h"), "w") as fh:
        fh.write(text)

    defines = ["-DN_RUNS={}".format(args.runs),
               "-DRANDOM_TRUE_PLANT={}".format(int(args.plant == "random")),
               "-DTB_SEED={}u".format(args.seed)]
    if args.nsim:
        defines.append("-DTB_NSIM={}".format(args.nsim))
    defines += ["-D{}={}".format(k, v) for k, v in cfg["hp"].items()]
    incs = ["-I", "."]
    if args.fixed:
        incs += ["-I", args.ap_include]
        if not os.path.exists(os.path.join(args.ap_include, "hls_math.h")):
            # setup.h includes hls_math.h but uses nothing from it
            os.makedirs(os.path.join(d, "shim"))
            open(os.path.join(d, "shim", "hls_math.h"), "w").close()
            incs += ["-I", "shim"]
    exe = os.path.join(d, "sim.exe" if os.name == "nt" else "sim")
    srcs = sorted(f for f in os.listdir(d) if f.endswith(".cpp"))
    b = subprocess.run([args.cxx, "-std=c++14", "-O2"] + incs + defines + ["-o", exe] + srcs,
                       cwd=d, capture_output=True, text=True)
    if b.returncode != 0:
        return cfg, None, "build failed: " + b.stderr.strip().splitlines()[-1] if b.stderr else "build failed"
    r = subprocess.run([exe], cwd=d, capture_output=True, text=True)
    summaries = [f for f in os.listdir(d) if f.endswith("_summary.txt")]
    if r.returncode != 0 or not summaries:
        return cfg, None, "run failed (exit {})".format(r.returncode)
    with open(os.path.join(d, summaries[0])) as fh:
        rows = list(csv.DictReader(fh, delimiter=" "))
    warnings = " | ".join(l.strip() for l in r.stdout.splitlines() if l.startswith("WARNING"))
    if not args.keep:
        shutil.rmtree(d, ignore_errors=True)
    return cfg, rows, warnings


def aggregate(rows):
    f = lambda k: [float(r[k]) for r in rows]
    rmse, floor = f("rmse"), f("rmseFloor")
    ratio = [a / b if b > 0 else float("nan") for a, b in zip(rmse, floor)]
    return {
        "runs": len(rows),
        "runsViolY": sum(v > 0 for v in f("nViolY")),
        "runsViolDy": sum(v > 0 for v in f("nViolDy")),
        "samplesViolY": int(sum(f("nViolY"))),
        "samplesViolDy": int(sum(f("nViolDy"))),
        "maxViolY": max(f("maxViolY")),
        "maxViolDy": max(f("maxViolDy")),
        "meanRmse": sum(rmse) / len(rmse),
        "meanRatio": sum(ratio) / len(ratio),
        "worstRatio": max(ratio),
        "meanRmsDu": sum(f("rmsDu")) / len(rows),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--system", default="pendulum", help="|".join(SYSTEMS) + " or a SYSTEM_* macro")
    ap.add_argument("--ctrl", default="scmpc", choices=CTRL_MODES)
    ap.add_argument("--grid", action="append", default=[], metavar="HP_X=v1,v2,...",
                    help="hyperparameter values to sweep (cartesian product of all --grid)")
    ap.add_argument("--scen", default="both", choices=["both", "scen", "noscen"],
                    help="with scenarios, without, or both (paired)")
    ap.add_argument("--runs", type=int, default=20, help="closed-loop runs per configuration")
    ap.add_argument("--plant", default="random", choices=["random", "true"],
                    help="random true plant per run (plant.h), or THETA_TRUE_INIT in every run "
                         "(runs then differ only by the measurement noise)")
    ap.add_argument("--seed", type=int, default=1, help="test-bench seed (TB_SEED)")
    ap.add_argument("--nsim", type=int, default=0, help="samples per run (default: testMain.cpp's)")
    ap.add_argument("--fixed", action="store_true", help="fixed point (needs --ap-include)")
    ap.add_argument("--ap-include", help="directory containing ap_fixed.h")
    ap.add_argument("--no-nrmlz", dest="nrmlz", action="store_false", help="disable NRMLZ")
    ap.add_argument("--no-conv", dest="conv", action="store_false", help="disable CONVERSIONS_MODE")
    ap.add_argument("--fclk", type=float, default=100e6, help="clock for the latency estimate [Hz]")
    ap.add_argument("--ts", type=float, default=0.0, help="sampling period [s]: adds the share of it used")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--cxx", default="g++")
    ap.add_argument("--keep", action="store_true", help="keep the build directories")
    ap.add_argument("--out", default="sweep.csv")
    args = ap.parse_args()
    if args.fixed and not args.ap_include:
        ap.error("--fixed needs --ap-include")

    grid = []
    for g in args.grid:
        name, _, vals = g.partition("=")
        if not name.startswith("HP_") or not vals:
            ap.error("--grid expects HP_NAME=v1,v2,...: " + g)
        grid.append([(name, v) for v in vals.split(",")])
    scens = ["scen", "noscen"] if args.scen == "both" else [args.scen]
    cfgs = [{"hp": dict(c), "scen": s} for c in itertools.product(*grid) for s in scens]

    with open(os.path.join(SRC_DIR, "setup.h")) as fh:
        setup_text = fh.read()
    sysmacro = SYSTEMS.get(args.system, args.system)
    work_root = tempfile.mkdtemp(prefix="sweep_")
    print("{} configurations x {} runs, building in {}".format(len(cfgs), args.runs, work_root))

    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = [ex.submit(build_and_run, c, args, setup_text, work_root) for c in cfgs]
        for i, fut in enumerate(futs):
            cfg, rows, msg = fut.result()
            label = " ".join("{}={}".format(k, v) for k, v in cfg["hp"].items()) + " " + cfg["scen"]
            if rows is None:
                print("[{}/{}] {}: {}".format(i + 1, len(cfgs), label, msg))
                continue
            agg = aggregate(rows)
            hp = {n: int(cfg["hp"].get("HP_" + n, hp_default(setup_text, sysmacro, n)))
                  for n in ("NHOR", "NHORU", "LOG2NSCEN", "MADS_ITER")}
            cycles = hp["MADS_ITER"] * 2 * hp["NHORU"] * hp["NHOR"] * (LAT_A + LAT_B * (1 << hp["LOG2NSCEN"]))
            row = dict(cfg["hp"], scen=cfg["scen"], **agg)
            row["estCycles"] = int(round(cycles))
            if args.ts > 0:
                row["estShareOfTs"] = cycles / args.fclk / args.ts
            row["warnings"] = msg
            results.append(row)
            print("[{}/{}] {}: ratio {:.3f} (worst {:.3f}), viol runs y {} dy {} ({} samples), ~{} cycles".format(
                i + 1, len(cfgs), label, agg["meanRatio"], agg["worstRatio"], agg["runsViolY"],
                agg["runsViolDy"], agg["samplesViolDy"], row["estCycles"]))

    if not results:
        sys.exit("no configuration ran")
    keys = list(dict.fromkeys(k for r in results for k in r))
    with open(args.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(results)
    if not args.keep:
        shutil.rmtree(work_root, ignore_errors=True)
    print("wrote", args.out)


if __name__ == "__main__":
    main()
