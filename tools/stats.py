#!/usr/bin/env python3

import argparse
import csv
import math
import sys

def read_column(path, col):
    values = []
    with open(path, newline="") as f:
        rows = (line for line in f if not line.startswith("#"))
        reader = csv.DictReader(rows)

        for row in reader:
            raw = row.get(col)
            if raw is None:
                continue
            try:
                values.append(float(raw))
            except ValueError:
                continue
            pass

    return values


def percentile(sorted_values, p):
    if not sorted_values:
        return float("nan")

    n = len(sorted_values)
    rank = math.ceil((p / 100.0) * n)
    rank = max(1, min(rank, n))
    return sorted_values[rank - 1]


def summarize(values):
    if not values:
        return None

    s = sorted(values)
    n = len(s)
    mean = sum(s) / n
    var = sum((x - mean) ** 2 for x in s) / (n - 1) if n > 1 else 0.0

    return {
        "n": n,
        "min": s[0],
        "p50": percentile(s, 50),
        "p90": percentile(s, 90),
        "p95": percentile(s, 95),
        "p99": percentile(s, 99),
        "max": s[-1],
        "mean": mean,
        "stddev": math.sqrt(var),
    }


# ─────────────────────────────────────────────
DIVISOR = {"ns": 1.0, "us": 1000.0, "ms": 1000000.0}


def main():
    ap = argparse.ArgumentParser(description="Summarize a CSV column: p50 / p95 / p99 / max")
    ap.add_argument("files", nargs="+", help="CSV files")
    ap.add_argument("--col", default="overshoot_ns", help="column name to analyze")
    ap.add_argument("--unit", default="ns", choices=list(DIVISOR),
                    help="output unit (input is always assumed to be ns)")
    args = ap.parse_args()

    div = DIVISOR[args.unit]
    u = args.unit

    header = (f"{'file':<28} {'n':>7} {'min':>10} {'p50':>10} {'p90':>10} "
              f"{'p95':>10} {'p99':>10} {'max':>12} {'mean':>10} {'sd':>10}")
    print(f"column: {args.col}   unit: {u}   percentile: nearest-rank")
    print(header)
    print("-" * len(header))

    for path in args.files:
        vals = read_column(path, args.col)
        st = summarize(vals)
        if st is None:
            print(f"{path:<28} (no data - check the column name)", file=sys.stderr)
            continue
        name = path.split("/")[-1]
        print(f"{name:<28} {st['n']:>7} "
              f"{st['min']/div:>10.1f} {st['p50']/div:>10.1f} {st['p90']/div:>10.1f} "
              f"{st['p95']/div:>10.1f} {st['p99']/div:>10.1f} {st['max']/div:>12.1f} "
              f"{st['mean']/div:>10.1f} {st['stddev']/div:>10.1f}")


if __name__ == "__main__":
    main()
