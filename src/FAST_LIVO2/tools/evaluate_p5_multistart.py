#!/usr/bin/env python3
"""Audit P5 three-seed MAP selection and selected covariance semantics."""

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
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


def evaluate(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    frames = defaultdict(list)
    for row in rows:
        frames[row["frame_id"]].append(row)

    selected = []
    exact_map_argmin = 0
    three_seed_frames = 0
    label_sets_valid = True
    for frame_rows in frames.values():
        chosen = [row for row in frame_rows if row["selected"] == "1"]
        if len(chosen) == 1:
            selected.append(chosen[0])
        if len(frame_rows) != 3:
            continue
        three_seed_frames += 1
        label_sets_valid &= {row["seed_label"] for row in frame_rows} == {
            "S0", "S-1", "S+1"
        }
        valid = [row for row in frame_rows if row["commit"] == "1"]
        if not valid:
            expected = next(row for row in frame_rows if row["seed_label"] == "S0")
        else:
            expected = min(valid, key=lambda row: float(row["j_total"]))
        exact_map_argmin += len(chosen) == 1 and chosen[0] is expected

    selected_covariance = [row for row in selected if row["commit"] == "1"]
    selected_counts = Counter(row["seed_label"] for row in selected)
    return {
        "rows": len(rows),
        "frames": len(frames),
        "one_seed_fallback_frames": sum(len(value) == 1 for value in frames.values()),
        "three_seed_frames": three_seed_frames,
        "other_candidate_count_frames": sum(
            len(value) not in (1, 3) for value in frames.values()),
        "exactly_one_selected_frames": len(selected),
        "three_seed_label_sets_valid": label_sets_valid,
        "map_argmin_exact_frames": exact_map_argmin,
        "selected_labels": dict(sorted(selected_counts.items())),
        "selected_non_s0_frames": len(selected) - selected_counts.get("S0", 0),
        "selected_commit_frames": len(selected_covariance),
        "selected_reject_frames": len(selected) - len(selected_covariance),
        "selected_covariance_all_finite": all(
            row["covariance_finite"] == "1" for row in selected_covariance),
        "selected_covariance_asymmetry_max": max(
            (float(row["covariance_asymmetry"]) for row in selected_covariance),
            default=math.nan),
        "selected_covariance_min_eigenvalue": min(
            (float(row["covariance_min_eigenvalue"]) for row in selected_covariance),
            default=math.nan),
        "candidate_solve_time_ms": summarize(
            float(row["solve_time_ms"]) for row in rows),
        "frame_multistart_time_ms": summarize(
            float(row["total_multistart_time_ms"]) for row in selected),
    }


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    assert summarize([1.0, 3.0])["median"] == 2.0
    print("evaluate_p5_multistart: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?")
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.csv:
        parser.error("csv is required")
    result = evaluate(Path(args.csv))
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        Path(args.output).write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()
