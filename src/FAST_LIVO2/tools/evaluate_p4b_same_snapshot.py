#!/usr/bin/env python3
"""Summarize P4-B same-snapshot outputs without ground truth.

The script deliberately does not invent a causal-effect threshold. The reviewer
must freeze `--b0-reproduced` and `--root-verdict` from the internal-state plots
and mechanism checks described in the P4-B protocol.
"""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


BRANCHES = ("B0_EVOLVING", "B1_FROZEN", "B2_RECENT5")


def finite(values):
    return [value for value in values if math.isfinite(value)]


def percentile(values, fraction):
    values = sorted(finite(values))
    if not values:
        return None
    position = fraction * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan


def load_key_values(path):
    result = {}
    if not path.is_file():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result.setdefault(key, []).append(value)
    return result


def load_csv(path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def summarize_branch(rows):
    if not rows:
        return {"frames": 0}
    speeds = [math.sqrt(sum(number(row, key) ** 2 for key in
                            ("velocity_x", "velocity_y", "velocity_z")))
              for row in rows]
    first = rows[0]
    positions = [(number(row, "position_x"), number(row, "position_y"),
                  number(row, "position_z")) for row in rows]
    origin = positions[0]
    growth = [math.sqrt(sum((position[i] - origin[i]) ** 2 for i in range(3)))
              for position in positions]
    return {
        "frames": len(rows),
        "relative_time_start_s": number(first, "relative_time_s"),
        "relative_time_end_s": number(rows[-1], "relative_time_s"),
        "speed_mps": {
            "median": percentile(speeds, 0.5),
            "p95": percentile(speeds, 0.95),
            "max": max(finite(speeds), default=None),
        },
        "position_growth_from_window_start_m": {
            "median": percentile(growth, 0.5),
            "p95": percentile(growth, 0.95),
            "max": max(finite(growth), default=None),
        },
        "correspondences": {
            "median": percentile([number(row, "correspondences") for row in rows], 0.5),
            "p05": percentile([number(row, "correspondences") for row in rows], 0.05),
        },
        "translation_lambda_median": [
            percentile([number(row, f"translation_lambda_{index}") for row in rows], 0.5)
            for index in range(3)
        ],
        "residual_median_m": percentile(
            [number(row, "residual_median_m") for row in rows], 0.5),
        "residual_p95_m": percentile(
            [number(row, "residual_p95_m") for row in rows], 0.95),
        "b0_position_separation_m": {
            "median": percentile([number(row, "b0_position_separation_m") for row in rows], 0.5),
            "p95": percentile([number(row, "b0_position_separation_m") for row in rows], 0.95),
            "max": max(finite([number(row, "b0_position_separation_m") for row in rows]), default=None),
        },
        "b0_rotation_separation_deg": {
            "p95": percentile([number(row, "b0_rotation_separation_deg") for row in rows], 0.95),
            "max": max(finite([number(row, "b0_rotation_separation_deg") for row in rows]), default=None),
        },
        "association_overlap": {
            "median": percentile([number(row, "association_overlap") for row in rows], 0.5),
            "minimum": min(finite([number(row, "association_overlap") for row in rows]), default=None),
        },
        "mean_normal_cosine": {
            "median": percentile([number(row, "mean_normal_cosine") for row in rows], 0.5),
            "minimum": min(finite([number(row, "mean_normal_cosine") for row in rows]), default=None),
        },
        "mean_plane_center_difference_m": percentile(
            [number(row, "mean_plane_center_difference_m") for row in rows], 0.5),
        "mean_plane_normal_difference_deg": percentile(
            [number(row, "mean_plane_normal_difference_deg") for row in rows], 0.5),
        "provenance_age_ratio_median": {
            key: percentile([number(row, key) for row in rows], 0.5)
            for key in ("age_lt_0_5_ratio", "age_0_5_2_ratio",
                        "age_2_5_ratio", "age_5_20_ratio",
                        "age_gt_20_ratio")
        },
    }


def first_divergence(rows, keys, tolerance):
    for row in rows:
        if any(abs(number(row, key)) > tolerance for key in keys):
            return number(row, "relative_time_s")
    return None


def analyze_run(directory, focus_start=None, focus_end=None):
    directory = Path(directory)
    parity = load_key_values(directory / "p4b_parity.txt")
    rows = load_csv(directory / "p4b_frames.csv")
    validation = load_csv(directory / "p4b_validation.csv")
    by_branch = {branch: [row for row in rows if row.get("branch") == branch]
                 for branch in BRANCHES}
    if focus_start is not None:
        focused = {
            branch: [row for row in branch_rows
                     if focus_start <= number(row, "relative_time_s") <= focus_end]
            for branch, branch_rows in by_branch.items()
        }
    else:
        focused = by_branch
    validations = {}
    for row in validation:
        validations.setdefault(row.get("kind", "UNKNOWN"), []).append(
            row.get("pass") in ("1", "true", "True"))
    branch_summaries = {branch: summarize_branch(branch_rows)
                        for branch, branch_rows in focused.items()}
    divergence = {}
    for branch in BRANCHES[1:]:
        branch_rows = by_branch[branch]
        divergence[branch] = {
            "association_relative_time_s": next(
                (number(row, "relative_time_s") for row in branch_rows
                 if number(row, "association_overlap") < 1.0 - 1e-12), None),
            "plane_normal_relative_time_s": next(
                (number(row, "relative_time_s") for row in branch_rows
                 if number(row, "mean_normal_cosine") < 1.0 - 1e-12), None),
            "state_relative_time_s": first_divergence(
                branch_rows,
                ("b0_position_separation_m", "b0_rotation_separation_deg",
                 "b0_speed_separation_mps"), 1e-9),
            "map_first_frame_hash": branch_rows[0].get("map_sha256")
                if branch_rows else None,
        }
    b0_hash = by_branch["B0_EVOLVING"][0].get("map_sha256") \
        if by_branch["B0_EVOLVING"] else None
    for branch in BRANCHES[1:]:
        divergence[branch]["map_diverged_on_first_frame"] = bool(
            b0_hash and divergence[branch]["map_first_frame_hash"] and
            b0_hash != divergence[branch]["map_first_frame_hash"])
    return {
        "directory": str(directory),
        "snapshot_parity": parity.get("SNAPSHOT_PARITY", ["MISSING"])[-1],
        "horizon_complete": parity.get("HARNESS_HORIZON_COMPLETE", ["NO"])[-1],
        "hashes": {key: values[-1] for key, values in parity.items()
                   if key.endswith("sha256")},
        "validations": {key: all(values) and bool(values)
                        for key, values in validations.items()},
        "vio_accepted_during_fork": sum(
            int(value) for value in parity.get("VIO_ACCEPTED_DURING_FORK", [])),
        "branches": branch_summaries,
        "divergence": divergence,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq1", required=True)
    parser.add_argument("--seq2", required=True)
    parser.add_argument("--construction", required=True)
    parser.add_argument("--b0-reproduced", choices=("yes", "no", "unassessed"),
                        default="unassessed")
    parser.add_argument("--root-verdict",
                        choices=("ROOT-E_CONFIRMED", "ROOT-F_REMAINS",
                                 "ROOT-E_DOWNGRADED"),
                        default="ROOT-F_REMAINS")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    runs = {
        "stadtgarten_seq1": analyze_run(args.seq1, 900.0, 906.0),
        "stadtgarten_seq2": analyze_run(args.seq2),
        "construction_seq2": analyze_run(args.construction),
    }
    structural_gates = all(
        run["snapshot_parity"] == "PASS" and
        run["horizon_complete"] == "YES" and
        run["vio_accepted_during_fork"] == 0 and
        all(run["branches"][branch]["frames"] > 0 for branch in BRANCHES) and
        run["validations"].get("MEMORY_RESTORE_NEXT_FRAME", False) and
        run["validations"].get("DISK_RELOAD_NEXT_FRAME", False)
        for run in runs.values())
    ready = (structural_gates and args.b0_reproduced == "yes" and
             args.root_verdict == "ROOT-E_CONFIRMED")
    result = {
        "uses_ground_truth": False,
        "protocol": {
            "seq1_snapshot_s": 895.0,
            "seq1_horizon_s": [895.0, 910.0],
            "seq1_focus_s": [900.0, 906.0],
            "normal_horizon_s": [300.0, 315.0],
            "recent_exclusion": "last 5 accepted LiDAR map frames",
        },
        "runs": runs,
        "gates": {
            "structural": structural_gates,
            "fork_baseline_reproduced": args.b0_reproduced,
        },
        "SAME_SNAPSHOT_MAP_CAUSALITY_READY": "YES" if ready else "NO",
        "root_verdict": args.root_verdict,
        "review_boundary": (
            "No effect-size threshold is inferred by this script; the supplied "
            "baseline/root decisions must be justified from internal state, "
            "geometry, association, and normal-control evidence."),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")
    print(json.dumps({
        "SAME_SNAPSHOT_MAP_CAUSALITY_READY": result["SAME_SNAPSHOT_MAP_CAUSALITY_READY"],
        "root_verdict": result["root_verdict"],
        "structural_gates": structural_gates,
        "output": str(output),
    }, indent=2))


if __name__ == "__main__":
    main()
