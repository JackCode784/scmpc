# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

Scenario-based MPC (SCMPC) for ARX systems with a zonotopic parameter-uncertainty set, solved by MADS (Mesh Adaptive Direct Search) with a progressive barrier, and implemented on an FPGA with Vitis HLS.

- `cpp/ScenarioMPC_ARX/`: the active project. A single C++ source tree is both:
  - the PC closed-loop simulation (`double` types), and
  - the Vitis HLS source (`ap_fixed` types).

  Target: xc7z020-clg484-1, 10 ns clock, top function `controller`.
- `vitis/scmpc-controller2026/`: the Vitis 2026.1 (Unified IDE) HLS component. Only `hls_config.cfg` and `vitis-comp.json` are tracked. The config lists source files by absolute Windows paths on the author's machine. Build outputs live in `vitis/scmpc-controller2026/scmpc-controller2026/` and are gitignored. `vitis/scmpc-arx/` is the old Vitis HLS 2021.1 project.
- `cpp/MADS_ISCAS23/`: an earlier, separate MADS-MPC HLS design, built on its own.
- `matlab/`: reference and prototype implementations (zonotopes, passive learning, SCMPC). They need the CORA toolbox (gitignored), YALMIP and quadprog.

`README.md` (on `main`) states the control problems in mathematical form (MPC, SCMPC, passive and active learning) and keeps the roadmap ("Past, present, future"). Read it for the theory and the planned work.

## Notation: README/paper ↔ code

| README / paper | Code |
|---|---|
| $N_y$ (prediction horizon), $N_u$ (control horizon) | `Nhor`, `NhorU` (= `nOpt`) |
| $S$ scenarios, $I$ MADS iterations | `Nscen` (power of 2, `LOG2NSCEN`), `MADS_ITER` |
| $n_a$, $n_b$, $n_d$ (input delay) | `na`, `nb`, `nk` |
| zonotope centre $c$ (= $\theta^{(0)}$, nominal model), generators $G$, $n_g$ | `thetaCenter`, `thetaGens`, `nGens`; only the `nUnc` uncertain parameters have rows (`UNC_MAP`) |
| $\xi$ | `coeffs` (`generateScenarios`, `pseudoRandArx`) |
| $Q$, $R$ | `outputWeight` (applied as a shift by `log2Q` in fixed point), `R` |
| cost and constraint violation | `cost[0]`, `cost[1]` of `costFunctionArx` |

Where the code differs from the README's formulation:
- The last prediction step has its own weight, `terminalOutputWeight` (`log2P`).
- The reference is held constant over the horizon: every error is against the current `yref`.
- Constraints are not hard: MADS's progressive barrier ranks candidates by a violation measure, and the nominal model is always constrained.
- With `DELAY_COMPENSATION`, the decision variables are $u_{k+1}..u_{k+N_u}$ and $u_k$ is known; without it they are $u_k..u_{k+N_u-1}$.

## Building and running the simulation

There is no build system and no unit-test framework. The closed-loop test bench `testMain.cpp` is the test: it simulates the controller against a plant, reports constraint violations and RMSE, and writes trajectories.

```bash
cd cpp/ScenarioMPC_ARX
# floating point (default setup.h): plain g++
g++ -std=c++14 -O2 -I . -o sim $(ls *.cpp | grep -v intervalHull.cpp) && ./sim
# fixed point (FIXED in setup.h): needs the Xilinx ap_types headers
# (github.com/Xilinx/HLS_arbitrary_Precision_Types, include/) and a stub
# hls_math.h (it is included, nothing from it is used):
#   printf '#pragma once\n#include <cmath>\n' > <stub_dir>/hls_math.h
g++ -std=c++14 -O2 -I <stub_dir> -I <ap_types>/include -I . -o sim $(ls *.cpp | grep -v intervalHull.cpp) && ./sim
```

- Always leave out `intervalHull.cpp`. It is unused, it is not in the Vitis source list, and it does not compile in fixed point. `passiveLearning/` is not part of the build either.
- The pendulum in fixed point requires `NRMLZ` (there is an `#error` otherwise). The usual hardware configuration is `FIXED` + `CONVERSIONS_MODE` + `NRMLZ`.
- **Output location.** Floating-point builds write into `FloatSimulations/` (created if missing). Fixed-point builds write into the working directory, which in Vitis is `.../hls/csim/build`.
  - `N_RUNS == 1` writes `output.txt` (plotted by `plotOutputCpp.m`).
  - `N_RUNS > 1` writes `runs_<ctrl>_<plant>_<NNN>.txt` plus `..._summary.txt` (plotted by `plotMultipleOutputsCpp.m`; `opts.isVitisSim` selects the Vitis folder).
  - `<ctrl>` is `scen`, `scencost`, `scenconstr` or `noscen`, plus `_dc` (delay compensation) or `_latNN` (emulated latency). `<plant>` is `nl` or `arx`.

### Configuration is by editing `#define`s, not compiler flags

- `setup.h`, the "TARGET SELECTION" block and below:
  - mode flags: `FIXED`, `CONVERSIONS_MODE`, `NRMLZ`, `USE_SCENS_COST`, `USE_SCENS_CONSTR`, `DELAY_COMPENSATION`;
  - selectors: `ACTIVE_SYSTEM`, `CTRL_MODE` (SCMPC / passive learning / active-learning stub), `PRAGMA_PROFILE`;
  - algorithm constants: `Nhor`, `NhorU`, `Nscen` (a power of 2, with `LOG2NSCEN`), `MADS_ITER`, weights.
- `plant.h` (test bench only): `N_RUNS`, `RANDOM_TRUE_PLANT`, `TB_SEED`, `TB_COMPUTE_DELAY_FRACTION`.

### Vitis flow (on the author's machine; not available in this environment)

C simulation, then synthesis, then C/RTL co-simulation. Vitis reports co-simulation "Pass" whenever `main()` returns 0, so the test bench does the real check:

1. C simulation writes `golden_u.txt` (run 0's applied inputs).
2. Co-simulation (`__RTL_SIMULATION__`) compares the RTL's inputs against it and returns 1 on any mismatch, or if the file is missing.

So run C simulation first, and set `N_RUNS 1` for co-simulation: only run 0 is compared, and the RTL's state is not reset between runs.

## Verifying changes

The established check for refactors and pragma changes is that **C simulation output is byte-identical to the previous commit**:

1. Build the old tree (`git archive <rev> cpp/ScenarioMPC_ARX`) and the new tree with the same configuration (`N_RUNS` reduced, e.g. 3).
2. Run both and `cmp` every output file.
3. Do this across a matrix of configurations: systems (pendulum, buck converters), fixed and float, the three pragma profiles, with and without scenarios, delay compensation, passive learning.

Pragmas never change C simulation, so any difference is a behaviour change. Hardware effects (timing, latency, instance counts) can only be checked from Vitis reports (`csynth.rpt`, `hls_compile.log`, implementation reports), which the user provides.

## Architecture

**Header chain.**
- `system_configs.h`: system ids, and per system the controller's *model*: nominal θ, zonotope generators, bounds, initial histories.
- `types.h`: per-system `ap_fixed` types; everything is `double` without `FIXED`.
- `setup.h`: configuration knobs, derived dimensions and constants, ADC/normalization constants, pragma profiles, every prototype, and the `extern` declarations of the controller state.
- `plant.h`: the *true* plant for the test bench (ARX with other parameters, or the nonlinear inverted pendulum). It is never synthesized.

**`controller()` (`controller.cpp`)** is the HLS top level, with `ap_ctrl_hs` control, 12-bit ADC inputs `yCurrDig`/`yrefDig`, and the DAC value as return. State persists across calls in file-scope globals defined without `static`: zonotope `thetaCenter`/`thetaGens`, `yHist`/`uHist`, `uOptPrev` (warm start), `uNow` (delay compensation). The step order documented at the top of the file matters (time k vs k−1 data). A call does:

1. Convert and normalize the inputs.
2. Warm start: shift the previous solution.
3. Passive learning only: strip–zonotope intersection (`boundStripZonotopeIntersection`).
4. `generateScenarios`: xorshift32 PRNG, then θᵢ = c + Gξᵢ.
5. `computeArxCoeffs`: fold the normalization into per-model coefficients; row `Nscen` is the nominal model.
6. `MADSARX`: `MADS_ITER` iterations. Each runs `generatePollMatrixArx` (Householder directions), then `progressiveBarrierPollingArx`, which evaluates 2·`NhorU` poll points and selects by (violation, cost).
7. Denormalize and convert the output.

The starting point is evaluated through the same poll call site, as an iteration −1 (`startOnly`).

**`costFunctionArx`** is the hot spot: `Nhor` prediction steps (a true recurrence) × (`Nscen`+1) independent models. It returns `cost[0]` (tracking + input) and `cost[1]` (constraint violation).

With `DELAY_COMPENSATION`, a call at k computes u(k+1) and treats u(k) as known (`DC_STEPS`, `nUKnown`).

## HLS conventions and pitfalls

- **How pragmas are written.**
  - Every pragma sits inside `#ifdef PRAGMAS`, which `FIXED` defines.
  - Profile-dependent pragmas come from the `PRAGMA_*` macros in `setup.h`.
  - A pragma whose argument is a macro must use `HLS_PRAGMA(...)`, because the HLS pragma parser does not expand macros.
  - Each pragma has a comment explaining why it is there; keep that convention and the existing long rationale comments.
- **Profiles.** `PRAGMA_PROFILE` is AREA, BALANCED or LATENCY. All three compute identical numbers; they differ only in the model-loop pipelining, the ARX term unrolling, and the number of `costFunctionArx` instances (`ALLOCATION` in `MADSARX.cpp`). BALANCED is the one used in the paper.
- **Recent Vitis versions pipeline loops and inline functions on their own,** hence the explicit `PIPELINE off` / `INLINE off`. A `PIPELINE` pragma written before a loop applies to the enclosing function, so put it inside the loop body.
- **Timing (10 ns).** In an II=1 loop, every loop-carried value must be a short operation.
  - `yPastCurr` is LUTRAM (partition on dim 2 only) so that `DEPENDENCE ... false` can apply. Complete partitioning into registers created a 21 ns recurrence.
  - The model-loop accumulators `ScenAcc` and `ViolAcc` are single additions, bit-identical to the rounding, saturating `cost_type` sums they replace. Don't revert them to `cost_type +=`.
- **Single call site.** `costFunctionArx` must keep a single call site. Otherwise HLS clones it (`costFunctionArx_1`) and ignores the `ALLOCATION` pragma (warning HLS 214-300).
- **Bit-exactness.** Fixed-point changes must keep C simulation bit-exact, or be deliberately validated. Types use `AP_RND_CONV`/`AP_SAT`, so regrouping additions is not free.
- **`static const ap_fixed` constants** are initialized dynamically. Vitis HLS 2026.1 co-simulation matched C simulation with them. The older toolchain produced constant-zero RTL, fixed on branch `fix/hw-constants-on-e6e7d01`.

## Conventions

- Commit subjects use prefixes like `feat:`, `fix:`, `refactor:`, `perf:`, `chore:`, `wip:`.
- Development happens on feature branches. `vitis2026-pragmas` carries the Vitis 2026.1 work; `main` holds `README.md`.
