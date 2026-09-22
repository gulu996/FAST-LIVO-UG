#!/usr/bin/env python3
"""Summarize fixed P5 seed/profile CSVs without ground truth."""

import argparse
import csv
import itertools
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
    upper = min(len(values) - 1, lower + 1)
    alpha = index - lower
    return values[lower] * (1.0 - alpha) + values[upper] * alpha


def summary(values):
    values = [value for value in values if math.isfinite(value)]
    return {
        "count": len(values),
        "median": statistics.median(values) if values else math.nan,
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else math.nan,
    }


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def local_minima(rows, field):
    ordered = sorted(rows, key=lambda row: float(row["alpha"]))
    result = []
    for index in range(1, len(ordered) - 1):
        before = float(ordered[index - 1][field])
        value = float(ordered[index][field])
        after = float(ordered[index + 1][field])
        if value < before and value < after:
            result.append(float(ordered[index]["alpha"]))
    return result


def evaluate(label, directory):
    seeds = read_csv(directory / "p5_seed_basin_seeds.csv")
    profiles = read_csv(directory / "p5_seed_basin_profile.csv")
    parity = read_csv(directory / "p5_seed_basin_parity.csv")
    seed_frames = {}
    profile_frames = {}
    for row in seeds:
        seed_frames.setdefault(row["frame_id"], []).append(row)
    for row in profiles:
        profile_frames.setdefault(row["frame_id"], []).append(row)

    weak_spans = []
    position_spans = []
    minimum_overlaps = []
    lidar_joint_same_winner = 0
    lidar_winners = {}
    joint_winners = {}
    for rows in seed_frames.values():
        weak = [float(row["final_weak_offset_m"]) for row in rows]
        weak_spans.append(max(weak) - min(weak))
        positions = [tuple(float(row[key]) for key in ("final_x", "final_y", "final_z")) for row in rows]
        position_spans.append(max(
            math.dist(a, b) for a, b in itertools.combinations(positions, 2)))
        minimum_overlaps.append(min(float(row["association_jaccard_s0"]) for row in rows))
        lidar = min(rows, key=lambda row: float(row["weighted_lidar_cost"]))["seed_label"]
        joint = min(rows, key=lambda row: float(row["j_total"]))["seed_label"]
        lidar_winners[lidar] = lidar_winners.get(lidar, 0) + 1
        joint_winners[joint] = joint_winners.get(joint, 0) + 1
        lidar_joint_same_winner += lidar == joint

    rms_minima = []
    cost_minima = []
    rms_argmins = {}
    cost_argmins = {}
    for rows in profile_frames.values():
        rms_local = local_minima(rows, "weighted_rms")
        cost_local = local_minima(rows, "weighted_lidar_cost")
        rms_minima.append(len(rms_local))
        cost_minima.append(len(cost_local))
        rms_alpha = min(rows, key=lambda row: float(row["weighted_rms"]))["alpha"]
        cost_alpha = min(rows, key=lambda row: float(row["weighted_lidar_cost"]))["alpha"]
        rms_argmins[rms_alpha] = rms_argmins.get(rms_alpha, 0) + 1
        cost_argmins[cost_alpha] = cost_argmins.get(cost_alpha, 0) + 1

    return {
        "label": label,
        "sample_frames": len(seed_frames),
        "seed_rows": len(seeds),
        "profile_rows": len(profiles),
        "parity_rows": len(parity),
        "parity_pass": all(row["pass"] == "1" for row in parity),
        "leaf_scale_m": float(seeds[0]["L_m"]) if seeds else math.nan,
        "final_weak_axis_span_m": summary(weak_spans),
        "final_position_pairwise_span_m": summary(position_spans),
        "minimum_association_jaccard_to_s0": summary(minimum_overlaps),
        "lidar_winners": lidar_winners,
        "joint_map_winners": joint_winners,
        "lidar_joint_same_winner_frames": lidar_joint_same_winner,
        "profile_weighted_rms_interior_minima_count": summary(rms_minima),
        "profile_weighted_cost_interior_minima_count": summary(cost_minima),
        "profile_weighted_rms_argmin_alpha": rms_argmins,
        "profile_weighted_cost_argmin_alpha": cost_argmins,
        "seed_solve_time_ms": summary(float(row["solve_time_ms"]) for row in seeds),
        "all_seed_commits": all(row["commit"] == "1" for row in seeds),
    }


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    rows = [
        {"alpha": str(alpha), "value": str(value)}
        for alpha, value in [(-1, 2), (-0.5, 1), (0, 2), (0.5, 1), (1, 2)]
    ]
    assert local_minima(rows, "value") == [-0.5, 0.5]
    print("evaluate_p5_seed_basin: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", action="append", nargs=2,
                        metavar=("LABEL", "P5_DIR"))
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.dataset or not args.output:
        parser.error("--dataset and --output are required")
    result = {label: evaluate(label, Path(directory))
              for label, directory in args.dataset}
    Path(args.output).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
