#!/usr/bin/env python3
"""Summarize internal trajectory continuity and increments without GT."""

import argparse
import json
import math
import statistics
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return math.nan
    index = fraction * (len(values) - 1)
    lower = int(math.floor(index))
    upper = min(lower + 1, len(values) - 1)
    alpha = index - lower
    return values[lower] * (1.0 - alpha) + values[upper] * alpha


def summarize(values):
    values = [value for value in values if math.isfinite(value)]
    return {
        "count": len(values),
        "median": statistics.median(values) if values else math.nan,
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else math.nan,
    }


def read_tum(path):
    rows = []
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) < 8 or fields[0].startswith("#"):
            continue
        values = tuple(float(value) for value in fields[:8])
        if all(math.isfinite(value) for value in values):
            rows.append(values)
    return rows


def evaluate(path, origin, start, end):
    rows = [row for row in read_tum(path)
            if start <= row[0] - origin <= end]
    translation = []
    rotation_deg = []
    speed = []
    gaps = []
    for before, after in zip(rows, rows[1:]):
        dt = after[0] - before[0]
        if dt <= 0.0:
            continue
        dp = math.dist(before[1:4], after[1:4])
        dot = abs(sum(a * b for a, b in zip(before[4:8], after[4:8])))
        angle = 2.0 * math.acos(min(1.0, max(0.0, dot))) * 180.0 / math.pi
        gaps.append(dt)
        translation.append(dp)
        rotation_deg.append(angle)
        speed.append(dp / dt)
    return {
        "rows": len(rows),
        "first_relative_s": rows[0][0] - origin if rows else math.nan,
        "last_relative_s": rows[-1][0] - origin if rows else math.nan,
        "timestamp_gap_s": summarize(gaps),
        "translation_increment_m": summarize(translation),
        "rotation_increment_deg": summarize(rotation_deg),
        "pose_derived_speed_mps": summarize(speed),
    }


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    assert abs(2.0 * math.acos(1.0)) < 1e-12
    print("evaluate_p5_trajectory: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", action="append", nargs=2,
                        metavar=("LABEL", "TUM"))
    parser.add_argument("--origin", type=float)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--end", type=float, required=False)
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.dataset or args.origin is None or args.end is None:
        parser.error("--dataset, --origin, and --end are required")
    result = {label: evaluate(Path(path), args.origin, args.start, args.end)
              for label, path in args.dataset}
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()
