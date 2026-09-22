#!/usr/bin/env python3
"""Summarize P3-A full-state shadow runs using internal signals only."""

import argparse
import csv
import json
import math
from pathlib import Path


def percentile(values, probability):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    position = probability * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def stats(values):
    clean = [v for v in values if math.isfinite(v)]
    if not clean:
        return {"count": 0, "median": None, "p95": None, "p99": None,
                "max": None}
    return {
        "count": len(clean),
        "median": percentile(clean, 0.5),
        "p95": percentile(clean, 0.95),
        "p99": percentile(clean, 0.99),
        "max": max(clean),
    }


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def quaternion_angle(left, right):
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def read_state(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(stream)]


def read_tum(path):
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip() or line.startswith("#"):
                continue
            values = [float(value) for value in line.split()]
            rows.append({"t": values[0], "p": values[1:4], "q": values[4:8]})
    return rows


def read_production_bias(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append({
                "timestamp": float(row["timestamp"]),
                "bg_step": float(row["delta_bias_g_norm"]),
                "ba_step": float(row["delta_bias_a_norm"]),
            })
    return rows


def select_indices(rows, begin=None, end=None):
    origin = rows[0]["timestamp"]
    return [index for index, row in enumerate(rows)
            if (begin is None or row["timestamp"] - origin >= begin)
            and (end is None or row["timestamp"] - origin <= end)]


def summarize(rows, raw, indices, production_bias=None):
    chosen = set(indices)
    fields = (
        "speed", "production_speed", "delta_v_norm", "imu_normalized_error",
        "lidar_normalized_error", "update_ms", "rss_mb", "cov_min_eigenvalue",
        "cov_condition", "pose_cov_trace", "velocity_cov_trace", "bias_cov_trace",
    )
    result = {field: stats([rows[i][field] for i in indices]) for field in fields}
    result["speed_absolute_difference"] = stats([
        abs(rows[i]["speed"] - rows[i]["production_speed"]) for i in indices])

    g2_translation = []
    g0_translation = []
    g2_rotation = []
    g0_rotation = []
    g2_speed_change = []
    g0_speed_change = []
    bias_step = []
    bg_step = []
    ba_step = []
    bias_rw_normalized = []
    for index in indices:
        if index == 0 or index - 1 not in chosen:
            continue
        dt = rows[index]["timestamp"] - rows[index - 1]["timestamp"]
        if dt <= 0.0:
            continue
        g2_translation.append(vector_norm([
            rows[index][axis] - rows[index - 1][axis] for axis in ("px", "py", "pz")]))
        g0_translation.append(vector_norm([
            raw[index]["p"][axis] - raw[index - 1]["p"][axis] for axis in range(3)]))
        g2_rotation.append(quaternion_angle(
            [rows[index - 1][axis] for axis in ("qx", "qy", "qz", "qw")],
            [rows[index][axis] for axis in ("qx", "qy", "qz", "qw")]))
        g0_rotation.append(quaternion_angle(raw[index - 1]["q"], raw[index]["q"]))
        g2_speed_change.append(abs(rows[index]["speed"] - rows[index - 1]["speed"]))
        g0_speed_change.append(abs(
            rows[index]["production_speed"] - rows[index - 1]["production_speed"]))
        delta_bias = [rows[index][axis] - rows[index - 1][axis]
                      for axis in ("bgx", "bgy", "bgz", "bax", "bay", "baz")]
        bias_step.append(vector_norm(delta_bias))
        bg_step.append(vector_norm(delta_bias[:3]))
        ba_step.append(vector_norm(delta_bias[3:]))
        bias_rw_normalized.append(vector_norm(delta_bias) / (0.01 * dt * math.sqrt(6.0)))

    result.update({
        "g2_pose_translation_increment_m": stats(g2_translation),
        "g0_pose_translation_increment_m": stats(g0_translation),
        "g2_pose_rotation_increment_rad": stats(g2_rotation),
        "g0_pose_rotation_increment_rad": stats(g0_rotation),
        "pose_translation_increment_absolute_difference_m": stats([
            abs(g2 - g0) for g2, g0 in zip(g2_translation, g0_translation)]),
        "pose_rotation_increment_absolute_difference_rad": stats([
            abs(g2 - g0) for g2, g0 in zip(g2_rotation, g0_rotation)]),
        "g2_speed_total_variation": sum(g2_speed_change),
        "g0_speed_total_variation": sum(g0_speed_change),
        "g2_speed_net_change": (rows[indices[-1]]["speed"] - rows[indices[0]]["speed"]
                                if indices else None),
        "g0_speed_net_change": (rows[indices[-1]]["production_speed"]
                                - rows[indices[0]]["production_speed"]
                                if indices else None),
        "bias_step_norm": stats(bias_step),
        "gyro_bias_step_norm": stats(bg_step),
        "accel_bias_step_norm": stats(ba_step),
        "bias_random_walk_normalized": stats(bias_rw_normalized),
        "lidar_rank_min": min((rows[i]["lidar_rank"] for i in indices), default=None),
        "lidar_weak_ratio": stats([
            rows[i]["lidar_eigen_0"] / rows[i]["lidar_eigen_5"]
            for i in indices if rows[i]["lidar_eigen_5"] > 0.0]),
    })
    if indices:
        first = rows[indices[0]]
        last = rows[indices[-1]]
        result["bias_start"] = [first[key] for key in
                                ("bgx", "bgy", "bgz", "bax", "bay", "baz")]
        result["bias_end"] = [last[key] for key in
                              ("bgx", "bgy", "bgz", "bax", "bay", "baz")]
        result["bias_net_change_norm"] = vector_norm([
            result["bias_end"][i] - result["bias_start"][i] for i in range(6)])
    if production_bias is not None:
        result["g0_gyro_bias_step_norm"] = stats(
            [production_bias[i]["bg_step"] for i in indices])
        result["g0_accel_bias_step_norm"] = stats(
            [production_bias[i]["ba_step"] for i in indices])
        result["g0_bias_step_norm"] = stats([
            math.hypot(production_bias[i]["bg_step"],
                       production_bias[i]["ba_step"]) for i in indices])
    return result


def evaluate_run(run_directory, production_diagnostic=None):
    native = run_directory / "native"
    rows = read_state(native / "fullstate_shadow_state.csv")
    raw = read_tum(native / "livo_raw_online.tum")
    if not rows or len(rows) != len(raw):
        raise ValueError(f"state/raw row mismatch: {len(rows)} != {len(raw)}")
    if any(abs(row["timestamp"] - pose["t"]) > 1e-5
           for row, pose in zip(rows, raw)):
        raise ValueError("state/raw timestamps are not aligned")
    production_bias = None
    if production_diagnostic:
        production_bias = read_production_bias(production_diagnostic)
        if len(production_bias) != len(rows):
            raise ValueError("state/production diagnostic row mismatch")
        if any(abs(row["timestamp"] - bias["timestamp"]) > 1e-9
               for row, bias in zip(rows, production_bias)):
            raise ValueError("state/production diagnostic timestamps are not aligned")

    intervals = []
    for index in range(1, len(rows)):
        dt = rows[index]["timestamp"] - rows[index - 1]["timestamp"]
        if dt <= 0.0:
            raise ValueError(f"non-monotonic state timestamp at row {index + 2}")
        intervals.append(dt)

    origin = rows[0]["timestamp"]
    acceleration = [0.0]
    turn_rate = [0.0]
    for index, dt in enumerate(intervals, start=1):
        acceleration.append(abs(rows[index]["production_speed"]
                                - rows[index - 1]["production_speed"]) / dt)
        turn_rate.append(quaternion_angle(raw[index - 1]["q"], raw[index]["q"]) / dt)
    weak_ratio = [row["lidar_eigen_0"] / row["lidar_eigen_5"]
                  if row["lidar_eigen_5"] > 0.0 else math.inf for row in rows]

    segments = {
        "overall": list(range(len(rows))),
        "acceleration_ge_1mps2": [i for i, value in enumerate(acceleration)
                                  if value >= 1.0],
        "turn_ge_15degps": [i for i, value in enumerate(turn_rate)
                             if value >= math.radians(15.0)],
        "stop_speed_le_0p1mps": [i for i, row in enumerate(rows)
                                  if row["production_speed"] <= 0.1],
        "weak_geometry_ratio_le_1e-3": [i for i, value in enumerate(weak_ratio)
                                         if value <= 1e-3],
    }
    duration = rows[-1]["timestamp"] - origin
    if duration >= 906.0:
        segments["focus_900_906s"] = select_indices(rows, 900.0, 906.0)
    if duration >= 930.0 - 0.5:
        segments["failure_900_930s"] = select_indices(rows, 900.0, 930.0)

    events_path = native / "fullstate_shadow_events.csv"
    event_count = max(0, sum(1 for _ in events_path.open(encoding="utf-8")) - 1)
    last = rows[-1]
    return {
        "run_directory": str(run_directory),
        "rows": len(rows),
        "duration_s": duration,
        "node_rate_hz": (len(intervals) / sum(intervals)),
        "event_count": event_count,
        "final_counters": {
            "active_nodes": int(last["active_nodes"]),
            "marginalized_nodes": int(last["marginalized_nodes"]),
            "active_factors": int(last["active_factors"]),
            "imu_duplicates": int(last["imu_duplicates"]),
            "imu_non_monotonic": int(last["imu_non_monotonic"]),
            "missing_imu_intervals": int(last["missing_imu_intervals"]),
            "factorization_failures": int(last["factorization_failures"]),
        },
        "production_diagnostic": (str(production_diagnostic)
                                  if production_diagnostic else None),
        "segments": {name: summarize(rows, raw, indices, production_bias)
                     for name, indices in segments.items()},
    }


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    assert percentile([3.0], 0.99) == 3.0
    assert quaternion_angle([0.0, 0.0, 0.0, 1.0],
                            [0.0, 0.0, 0.0, -1.0]) == 0.0
    assert abs(quaternion_angle([0.0, 0.0, 0.0, 1.0],
                                [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
               - math.pi / 2.0) < 1e-12
    print("PASS: full-state shadow evaluator self-test")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="*", type=Path)
    parser.add_argument("--production-diagnostic", action="append", type=Path,
                        default=[])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()
    if arguments.self_test:
        self_test()
        return
    if not arguments.runs:
        parser.error("at least one run directory is required")
    if arguments.production_diagnostic and (len(arguments.production_diagnostic)
                                            != len(arguments.runs)):
        parser.error("provide one --production-diagnostic per run")
    diagnostics = arguments.production_diagnostic or [None] * len(arguments.runs)
    report = {run.name: evaluate_run(run, diagnostic)
              for run, diagnostic in zip(arguments.runs, diagnostics)}
    text = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if arguments.output:
        arguments.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
