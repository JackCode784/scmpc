#!/usr/bin/env python3
"""
Pivot a sweep.py CSV into a table: one metric, two hyperparameters.

    python3 tuning/table.py p1.csv --rows HP_NHOR --cols HP_MADS_ITER \\
        --metric meanRatio --scen scen

Rows/columns not given are aggregated away by taking the best (lowest) value
of the metric, so a 1-D table is --rows only. Only the standard library is
used.
"""
import argparse
import csv


def num(v):
    try:
        return float(v)
    except ValueError:
        return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--rows", required=True)
    ap.add_argument("--cols")
    ap.add_argument("--metric", default="meanRatio")
    ap.add_argument("--scen", choices=["scen", "noscen"])
    ap.add_argument("--where", action="append", default=[], metavar="COL=VAL",
                    help="keep only rows with this value")
    ap.add_argument("--fmt", default="{:.3f}")
    args = ap.parse_args()

    rows = []
    for f in args.csv:
        with open(f) as fh:
            rows += list(csv.DictReader(fh))
    if args.scen:
        rows = [r for r in rows if r["scen"] == args.scen]
    for w in args.where:
        k, _, v = w.partition("=")
        rows = [r for r in rows if r.get(k) == v]

    cell = {}
    for r in rows:
        key = (r.get(args.rows, ""), r.get(args.cols, "") if args.cols else "")
        v = num(r[args.metric])
        if key not in cell or v < cell[key]:
            cell[key] = v
    rk = sorted({k[0] for k in cell}, key=num)
    ck = sorted({k[1] for k in cell}, key=num)
    width = max(10, max(len(c) for c in ck) + 2)
    print("{} ({})".format(args.metric, args.scen or "all"))
    print("{:>12}".format(args.rows + "\\" + (args.cols or "")) + "".join("{:>{w}}".format(c, w=width) for c in ck))
    for r in rk:
        line = "{:>12}".format(r)
        for c in ck:
            v = cell.get((r, c))
            s = "-" if v is None else (args.fmt.format(v) if isinstance(v, float) else str(v))
            line += "{:>{w}}".format(s, w=width)
        print(line)


if __name__ == "__main__":
    main()
