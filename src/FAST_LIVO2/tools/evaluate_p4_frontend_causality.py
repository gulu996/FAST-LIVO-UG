#!/usr/bin/env python3
"""Evaluate default-off P4 LiDAR-frontend diagnostics without ground truth."""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def finite(values):
    return np.asarray([float(v) for v in values if math.isfinite(float(v))])


def summary(values):
    values = finite(values)
    if not len(values):
        return {"count": 0}
    return {
        "count": int(len(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def final_iterations(rows):
    by_frame = {}
    for row in rows:
        frame = int(row["frame_id"])
        if frame not in by_frame or int(row["iteration"]) > int(by_frame[frame]["iteration"]):
            by_frame[frame] = row
    return list(by_frame.values())


def solve_rows(rows):
    if len(rows) < 6:
        return None
    jacobian = np.asarray([[float(row[f"j_{part}_{axis}"])
                            for part in ("rot", "pos")
                            for axis in ("x", "y", "z")] for row in rows])
    residual = np.asarray([float(row["residual_m"]) for row in rows])
    variance = np.asarray([float(row["variance_m2"]) for row in rows])
    valid = (np.isfinite(jacobian).all(axis=1) & np.isfinite(residual) &
             np.isfinite(variance) & (variance > 0))
    if np.count_nonzero(valid) < 6:
        return None
    weighted_jacobian = jacobian[valid] / np.sqrt(variance[valid, None])
    weighted_rhs = -residual[valid] / np.sqrt(variance[valid])
    delta, _, rank, singular = np.linalg.lstsq(weighted_jacobian, weighted_rhs, rcond=1e-12)
    return {"delta": delta, "rank": int(rank), "singular": singular}


def counterfactuals(correspondences):
    grouped = defaultdict(list)
    for row in correspondences:
        grouped[(int(row["frame_id"]), int(row["iteration"]))].append(row)
    frames = defaultdict(dict)
    for (frame, iteration), rows in grouped.items():
        frames[frame][iteration] = rows
    result = {name: [] for name in ("c0", "cfinal", "exclude_1_frame",
                                     "exclude_3_frames", "exclude_5_frames",
                                     "exclude_0_5_s", "exclude_1_s")}
    for frame, iterations in frames.items():
        first = iterations[min(iterations)]
        last = iterations[max(iterations)]
        variants = {
            "c0": first,
            "cfinal": last,
            # Conservative plane-level exclusion: a plane is removed if any
            # retained support point is inside the requested recent window.
            "exclude_1_frame": [r for r in last if float(r["support_recent_1_ratio"]) == 0.0],
            "exclude_3_frames": [r for r in last if float(r["support_recent_3_ratio"]) == 0.0],
            "exclude_5_frames": [r for r in last if float(r["support_recent_5_ratio"]) == 0.0],
            "exclude_0_5_s": [r for r in last if float(r["plane_age_min_s"]) >= 0.5],
            "exclude_1_s": [r for r in last if float(r["plane_age_min_s"]) >= 1.0],
        }
        for name, selected in variants.items():
            solved = solve_rows(selected)
            if solved is None:
                continue
            # The per-correspondence file has normals, not the frame weak axis;
            # store the full translation delta and project later using the
            # matching iteration row.
            result[name].append({
                "frame_id": frame,
                "count": len(selected),
                "rank": solved["rank"],
                "rotation_norm": float(np.linalg.norm(solved["delta"][:3])),
                "translation_norm": float(np.linalg.norm(solved["delta"][3:])),
                "delta": solved["delta"].tolist(),
            })
    return result


def trajectory_metrics(path, windows):
    rows = []
    with Path(path).open() as stream:
        for line in stream:
            fields = line.split()
            if len(fields) >= 4 and not line.lstrip().startswith("#"):
                rows.append([float(value) for value in fields[:4]])
    data = np.asarray(rows)
    if len(data) < 2:
        return {}
    relative = data[:, 0] - data[0, 0]
    dt = np.diff(data[:, 0])
    displacement = np.linalg.norm(np.diff(data[:, 1:4], axis=0), axis=1)
    speed = np.divide(displacement, dt, out=np.full_like(displacement, np.nan), where=dt > 0)
    result = {}
    for name, start, end in windows:
        mask = (relative[1:] >= start) & (relative[1:] <= end)
        result[name] = {
            "speed_mps": summary(speed[mask]),
            "increment_m": summary(displacement[mask]),
        }
    return result


def evaluate_run(label, run_dir, windows):
    run = Path(run_dir)
    data_dir = run / "native" if (run / "native" / "p4_frontend_iterations.csv").exists() else run
    iteration_path = data_dir / "p4_frontend_iterations.csv"
    if not iteration_path.exists():
        raise FileNotFoundError(iteration_path)
    iterations = read_csv(iteration_path)
    finals = final_iterations(iterations)
    result = {
        "run_directory": str(run.resolve()),
        "iteration_rows": len(iterations),
        "frame_rows": len(finals),
        "ground_truth_used": False,
        "artifacts_bytes": sum(path.stat().st_size for path in data_dir.glob("p4_frontend_*.csv")),
    }
    metrics = ("changed_plane_ratio", "voxel_jaccard", "sign_flip_ratio",
               "weighted_signed_mean_m", "weak_signed_projection_m",
               "normal_entropy", "normal_octant_coverage",
               "conditional_eval_0", "support_recent_1_ratio",
               "support_recent_3_ratio", "support_recent_5_ratio",
               "recent_voxel_1_ratio", "recent_voxel_3_ratio",
               "recent_voxel_5_ratio", "source_distance_mean_m",
               "plane_center_shift_mean_m", "plane_normal_change_mean_deg",
               "u2_plane_jaccard", "u2_signed_mean_m", "u2_fixed_weak_dp_m")
    result["windows"] = {}
    for name, start, end in windows:
        selected = [row for row in finals if start <= float(row["relative_time_s"]) <= end]
        result["windows"][name] = {
            metric: summary(float(row[metric]) for row in selected) for metric in metrics
        }
        result["windows"][name]["frames"] = len(selected)
        u2 = [row for row in iterations
              if int(row["iteration"]) == 1 and
              start <= float(row["relative_time_s"]) <= end]
        result["windows"][name]["u2_first_iteration"] = {
            metric: summary(float(row[metric]) for row in u2
                            if metric != "u2_correspondences" or
                            float(row[metric]) >= 0)
            for metric in ("u2_correspondences", "u2_plane_jaccard",
                           "u2_signed_mean_m", "u2_fixed_weak_dp_m")
        }
    correspondence_path = data_dir / "p4_frontend_correspondences.csv"
    if correspondence_path.exists():
        correspondences = read_csv(correspondence_path)
        result["correspondence_rows"] = len(correspondences)
        result["counterfactuals"] = counterfactuals(correspondences)
    deskew_path = data_dir / "p4_frontend_deskew.csv"
    if deskew_path.exists():
        deskew = read_csv(deskew_path)
        result["deskew"] = {
            "rows": len(deskew),
            "non_monotonic_frames": sum(row["point_time_monotonic"] == "0" for row in deskew),
            "imu_uncovered_frames": sum(row["imu_covers_propagation"] == "0" for row in deskew),
            "fixed_velocity_delta_mean_m": summary(float(row["fixed_velocity_point_delta_mean_m"]) for row in deskew),
            "fixed_velocity_delta_max_m": summary(float(row["fixed_velocity_point_delta_max_m"]) for row in deskew),
        }
    trajectory = data_dir / "livo_raw_online.tum"
    if trajectory.exists():
        result["trajectory"] = trajectory_metrics(trajectory, windows)
    runtime = run / "runtime.json"
    if runtime.exists():
        result["runtime"] = json.loads(runtime.read_text())
    return label, result


def self_test():
    rows = []
    for index in range(6):
        row = {f"j_{part}_{axis}": "0" for part in ("rot", "pos") for axis in ("x", "y", "z")}
        part = "rot" if index < 3 else "pos"
        axis = ("x", "y", "z")[index % 3]
        row[f"j_{part}_{axis}"] = "1"
        row.update(residual_m=str(index + 1), variance_m2="1")
        rows.append(row)
    solved = solve_rows(rows)
    assert solved and solved["rank"] == 6
    assert np.allclose(solved["delta"], -np.arange(1, 7))
    values = summary([1, 2, 3])
    assert values["count"] == 3 and values["median"] == 2
    print("evaluate_p4_frontend_causality: SELF-TEST PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="append", default=[], metavar="LABEL=DIR")
    parser.add_argument("--window", action="append", default=[], metavar="NAME=START:END")
    parser.add_argument("--output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    windows = []
    for value in args.window:
        name, bounds = value.split("=", 1)
        start, end = bounds.split(":", 1)
        windows.append((name, float(start), float(end)))
    if not windows:
        windows = [("all", -math.inf, math.inf)]
    report = {"ground_truth_used": False, "runs": {}}
    for value in args.run:
        label, directory = value.split("=", 1)
        name, result = evaluate_run(label, directory, windows)
        report["runs"][name] = result
    output = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(output + "\n")
    else:
        print(output)


if __name__ == "__main__":
    main()
