#!/usr/bin/env python3

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")           
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stats import read_column, percentile

C_IDLE = "#2a78d6"       # blue   - no load
C_LOAD = "#eb6834"       # orange - under load
INK    = "#0b0b0b"
MUTED  = "#52514e"
GRID   = "#e3e2de"

PCTS = [50, 75, 90, 95, 99, 99.9]
DIVISOR = {"ns": 1.0, "us": 1000.0, "ms": 1e6}


def style_for(name):
    low = name.lower()
    color = C_LOAD if "load" in low else C_IDLE
    dash = "--" if ("free" in low or "any" in low) else "-"
    label = ("load" if "load" in low else "idle") + \
            (", any core" if dash == "--" else ", same core")
    return color, dash, label


def main():
    ap = argparse.ArgumentParser(description="Tail latency comparison plot")
    ap.add_argument("files", nargs="+", help="CSV files")
    ap.add_argument("--col", default="per_switch_ns", help="column to plot")
    ap.add_argument("--unit", default="us", choices=list(DIVISOR))
    ap.add_argument("--out", default="figure.png", help="output PNG path")
    ap.add_argument("--title", default="Context switch cost", help="figure title")
    args = ap.parse_args()

    div = DIVISOR[args.unit]

    series = []                       # [(label, color, dash, [p50, p75, ...]), ...]
    for path in args.files:
        vals = read_column(path, args.col)
        if not vals:
            print(f"skip (no data): {path}", file=sys.stderr)
            continue
        s = sorted(vals)
        ys = [percentile(s, p) / div for p in PCTS]

        color, dash, label = style_for(os.path.basename(path))
        series.append((label, color, dash, ys, s))

    if not series:
        print("nothing to plot", file=sys.stderr)
        return 1

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(11, 4.2))
    fig.suptitle(args.title, fontsize=13, color=INK, x=0.02, ha="left")

    xs = list(range(len(PCTS)))
    for label, color, dash, ys, _ in series:
        if not ys:
            continue
        axL.plot(xs, ys, dash, color=color, label=label, linewidth=2, marker="o", markersize=5)
        pass

    axL.set_xticks(xs)
    axL.set_xticklabels([f"p{p:g}" for p in PCTS])
    axL.set_ylabel(f"latency ({args.unit})", color=MUTED, fontsize=10)
    # axL.set_yscale("log")   # range is only ~3x here; linear reads better
    axL.set_title("percentile curve", fontsize=10, color=MUTED, loc="left")
    axL.grid(True, axis="y", color=GRID, linewidth=0.8)
    axL.set_axisbelow(True)
    for side in ("top", "right"):
        axL.spines[side].set_visible(False)
    axL.legend(frameon=False, fontsize=9, labelcolor=MUTED)

    groups = ["p50", "p99", "max"]
    n = len(series)
    width = 0.8 / n
    for i, (label, color, dash, ys, s) in enumerate(series):
        if not ys:
            continue
        p50 = percentile(s, 50) / div
        p99 = percentile(s, 99) / div
        mx = s[-1] / div
        vals = [p50, p99, mx]
        pos = [g + i * width - 0.4 + width / 2 for g in range(len(groups))]

        axR.bar(pos, vals, width=width * 0.9, color=color,
                hatch="//" if dash == "--" else None,
                edgecolor="white", linewidth=1.0)
        pass

        for x, v in zip(pos, vals):
            axR.text(x, v, f"{v:.1f}", ha="center", va="bottom",
                     fontsize=8, color=MUTED)

    axR.set_yscale("log")        # max is ~4000x p50; linear would flatten every other bar
    axR.set_ylim(bottom=0.5)
    axR.set_xticks(range(len(groups)))
    axR.set_xticklabels(groups)
    axR.set_ylabel(f"latency ({args.unit})", color=MUTED, fontsize=10)
    axR.set_title("p50 / p99 / max", fontsize=10, color=MUTED, loc="left")
    axR.grid(True, axis="y", color=GRID, linewidth=0.8)
    axR.set_axisbelow(True)
    for side in ("top", "right"):
        axR.spines[side].set_visible(False)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"saved: {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
