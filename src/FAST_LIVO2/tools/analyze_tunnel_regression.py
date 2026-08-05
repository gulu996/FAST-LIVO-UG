#!/usr/bin/env python3
"""Analyze tunnel trajectory regressions without using reference data online."""

import argparse
import csv
import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np


def quaternion_normalize(q):
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.maximum(norm, 1e-15)


def quaternion_multiply(a, b):
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ), axis=-1)


def quaternion_inverse(q):
    result = np.array(q, dtype=float, copy=True)
    result[..., :3] *= -1.0
    return quaternion_normalize(result)


def quaternion_to_matrix(q):
    x, y, z, w = quaternion_normalize(np.asarray(q, dtype=float))
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_slerp(q0, q1, alpha):
    q0 = quaternion_normalize(np.asarray(q0, dtype=float))
    q1 = quaternion_normalize(np.asarray(q1, dtype=float))
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return quaternion_normalize(q0 + alpha * (q1 - q0))
    theta = math.acos(max(-1.0, min(1.0, dot)))
    return (math.sin((1.0 - alpha) * theta) * q0 +
            math.sin(alpha * theta) * q1) / math.sin(theta)


def quaternion_angle_deg(a, b):
    dots = np.abs(np.sum(quaternion_normalize(a) * quaternion_normalize(b), axis=1))
    return np.degrees(2.0 * np.arccos(np.clip(dots, 0.0, 1.0)))


def parse_json_trajectory(value):
    if isinstance(value, dict):
        for key in ("poses", "trajectory", "samples", "data"):
            if key in value:
                return parse_json_trajectory(value[key])
        raise ValueError("JSON object has no poses/trajectory/samples/data array")
    if not isinstance(value, list) or not value:
        raise ValueError("JSON trajectory must be a non-empty array")
    if isinstance(value[0], dict):
        rows = []
        for item in value:
            position = item.get("position", item)
            orientation = item.get("orientation", item.get("quaternion", item))
            timestamp = item.get("timestamp", item.get("time", item.get("stamp")))
            row = [position[k] for k in ("x", "y", "z")]
            row += [orientation[k] for k in ("qx", "qy", "qz", "qw")]
            rows.append(([timestamp] if timestamp is not None else []) + row)
        return np.asarray(rows, dtype=float)
    return np.asarray(value, dtype=float)


def load_numeric_rows(path):
    text = Path(path).read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError("trajectory is empty: {}".format(path))
    if text[0] in "[{":
        try:
            rows = parse_json_trajectory(json.loads(text))
            return np.atleast_2d(rows)
        except (json.JSONDecodeError, ValueError, KeyError, TypeError):
            pass
    rows = []
    for line_number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            rows.append([float(value) for value in line.replace(",", " ").split()])
        except ValueError as error:
            raise ValueError("{}:{} is not numeric: {}".format(path, line_number, error))
    if not rows or len({len(row) for row in rows}) != 1:
        raise ValueError("trajectory rows are empty or have inconsistent columns: {}".format(path))
    return np.asarray(rows, dtype=float)


def load_trajectory(path):
    rows = load_numeric_rows(path)
    columns = rows.shape[1]
    if columns >= 8:
        timestamps = rows[:, 0]
        positions = rows[:, 1:4]
        quaternions = rows[:, 4:8]  # timestamp x y z qx qy qz qw
        trajectory_format = "timestamp_xyz_qxyzw"
    elif columns == 7:
        timestamps = None
        positions = rows[:, :3]
        quaternions = rows[:, [4, 5, 6, 3]]  # legacy x y z qw qx qy qz
        trajectory_format = "legacy_xyz_qwxyz"
    else:
        raise ValueError("{} columns in {}; expected 7 or at least 8".format(columns, path))
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(quaternions)):
        raise ValueError("trajectory contains non-finite pose values: {}".format(path))
    if timestamps is not None and (not np.all(np.isfinite(timestamps)) or np.any(np.diff(timestamps) <= 0.0)):
        raise ValueError("timestamps must be finite and strictly increasing: {}".format(path))
    return {
        "positions": positions,
        "quaternions": quaternion_normalize(quaternions),
        "timestamps": timestamps,
        "format": trajectory_format,
        "path": str(path),
    }


def sampling_report(trajectory, fallback_rate):
    timestamps = trajectory["timestamps"]
    if timestamps is None:
        return {"has_timestamps": False, "assumed_rate_hz": fallback_rate,
                "median_rate_hz": fallback_rate, "large_gap_count": None}
    delta = np.diff(timestamps)
    median = float(np.median(delta)) if len(delta) else math.nan
    return {
        "has_timestamps": True,
        "assumed_rate_hz": None,
        "median_rate_hz": 1.0 / median if median > 0.0 else None,
        "large_gap_count": int(np.count_nonzero(delta > 1.5 * median)) if median > 0.0 else 0,
        "maximum_gap_s": float(np.max(delta)) if len(delta) else 0.0,
    }


def interpolate_trajectory(trajectory, source_parameter, target_parameter):
    positions = np.column_stack([
        np.interp(target_parameter, source_parameter, trajectory["positions"][:, axis])
        for axis in range(3)
    ])
    quaternions = []
    for value in target_parameter:
        upper = int(np.searchsorted(source_parameter, value, side="right"))
        upper = max(1, min(upper, len(source_parameter) - 1))
        lower = upper - 1
        span = source_parameter[upper] - source_parameter[lower]
        alpha = 0.0 if span <= 0.0 else (value - source_parameter[lower]) / span
        quaternions.append(quaternion_slerp(
            trajectory["quaternions"][lower], trajectory["quaternions"][upper], alpha))
    return positions, np.asarray(quaternions)


def align_samples(estimated, reference, estimated_rate, reference_rate):
    checks = {
        "estimated_length": int(len(estimated["positions"])),
        "reference_length": int(len(reference["positions"])),
        "estimated_sampling": sampling_report(estimated, estimated_rate),
        "reference_sampling": sampling_report(reference, reference_rate),
        "interpolation_used": False,
        "alignment_mode": "first_pose_se3",
    }
    est_time = estimated["timestamps"]
    ref_time = reference["timestamps"]
    if est_time is not None and ref_time is not None:
        start = max(est_time[0], ref_time[0])
        end = min(est_time[-1], ref_time[-1])
        mask = (est_time >= start) & (est_time <= end)
        if np.count_nonzero(mask) < 2:
            raise ValueError("estimated/reference timestamp ranges do not overlap")
        times = est_time[mask]
        est_pos = estimated["positions"][mask]
        est_q = estimated["quaternions"][mask]
        ref_pos, ref_q = interpolate_trajectory(reference, ref_time, times)
        checks["interpolation_used"] = True
        checks["alignment_basis"] = "timestamps"
    else:
        count = len(estimated["positions"])
        if count < 2 or len(reference["positions"]) < 2:
            raise ValueError("both trajectories need at least two poses")
        est_parameter = np.linspace(0.0, 1.0, count)
        ref_parameter = np.linspace(0.0, 1.0, len(reference["positions"]))
        est_pos = estimated["positions"]
        est_q = estimated["quaternions"]
        ref_pos, ref_q = interpolate_trajectory(reference, ref_parameter, est_parameter)
        times = est_time if est_time is not None else np.arange(count, dtype=float) / estimated_rate
        checks["interpolation_used"] = len(reference["positions"]) != count
        checks["alignment_basis"] = "normalized_sequence_index"
        checks["warning"] = (
            "At least one trajectory has no timestamps; sequence alignment cannot prove equal sampling "
            "frequency or distinguish dropped frames. Rates are reported/assumed explicitly.")

    initial_position_offset = ref_pos[0] - est_pos[0]
    initial_orientation_error = float(quaternion_angle_deg(est_q[:1], ref_q[:1])[0])
    checks["initial_position_offset_m"] = float(np.linalg.norm(initial_position_offset))
    checks["initial_orientation_offset_deg"] = initial_orientation_error

    q_align = quaternion_multiply(ref_q[0], quaternion_inverse(est_q[0]))
    rotation_align = quaternion_to_matrix(q_align)
    aligned_pos = (rotation_align @ (est_pos - est_pos[0]).T).T + ref_pos[0]
    aligned_q = quaternion_normalize(quaternion_multiply(
        np.broadcast_to(q_align, est_q.shape), est_q))
    return times, aligned_pos, aligned_q, ref_pos, ref_q, checks


def path_length(positions):
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def maximum_error_growth(times, errors, window_s):
    candidates = deque()
    maximum = 0.0
    for index, (timestamp, error) in enumerate(zip(times, errors)):
        while candidates and times[candidates[0]] < timestamp - window_s:
            candidates.popleft()
        if candidates:
            maximum = max(maximum, float(error - errors[candidates[0]]))
        while candidates and errors[candidates[-1]] >= error:
            candidates.pop()
        candidates.append(index)
    return maximum


def false_reverse_intervals(times, estimated_positions, reference_positions,
                            minimum_duration, minimum_distance, minimum_reference_speed):
    dt = np.diff(times)
    if np.any(dt <= 0.0):
        raise ValueError("aligned sample times must be strictly increasing")
    ref_step = np.diff(reference_positions, axis=0)
    est_step = np.diff(estimated_positions, axis=0)
    ref_distance = np.linalg.norm(ref_step, axis=1)
    tangent = ref_step / np.maximum(ref_distance[:, None], 1e-12)
    reference_speed = ref_distance / dt
    signed_estimated_speed = np.sum(est_step * tangent, axis=1) / dt
    reverse = (reference_speed >= minimum_reference_speed) & (signed_estimated_speed < 0.0)

    intervals = []
    start = None
    for index in range(len(reverse) + 1):
        active = index < len(reverse) and reverse[index]
        if active and start is None:
            start = index
        if not active and start is not None:
            end = index
            duration = float(times[end] - times[start])
            distance = float(np.sum(-signed_estimated_speed[start:end] * dt[start:end]))
            maximum_speed = float(np.max(-signed_estimated_speed[start:end]))
            if duration >= minimum_duration and distance >= minimum_distance:
                intervals.append({
                    "start_time": float(times[start]), "end_time": float(times[end]),
                    "duration_s": duration, "reverse_distance_m": distance,
                    "maximum_reverse_speed_mps": maximum_speed,
                })
            start = None
    return intervals


def analyze_with_reference(args, output_dir):
    estimated = load_trajectory(args.estimated)
    reference = load_trajectory(args.reference)
    times, est_pos, est_q, ref_pos, ref_q, checks = align_samples(
        estimated, reference, args.estimated_rate, args.reference_rate)
    position_error = np.linalg.norm(est_pos - ref_pos, axis=1)
    orientation_error = quaternion_angle_deg(est_q, ref_q)
    intervals = false_reverse_intervals(
        times, est_pos, ref_pos, args.minimum_reverse_duration,
        args.minimum_reverse_distance, args.minimum_reference_speed)
    result = {
        "trajectory_length": path_length(est_pos),
        "reference_trajectory_length": path_length(ref_pos),
        "maximum_position_error": float(np.max(position_error)),
        "rmse_position": float(np.sqrt(np.mean(position_error ** 2))),
        "final_position_error": float(position_error[-1]),
        "maximum_orientation_error": float(np.max(orientation_error)),
        "false_reverse_intervals": intervals,
        "cumulative_false_reverse_distance": sum(
            item["reverse_distance_m"] for item in intervals),
        "maximum_false_reverse_speed": max(
            (item["maximum_reverse_speed_mps"] for item in intervals), default=0.0),
        "maximum_error_growth_in_10_seconds": maximum_error_growth(times, position_error, 10.0),
        "checks": checks,
    }
    with (output_dir / "aligned_trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp", "estimated_x", "estimated_y", "estimated_z",
                         "reference_x", "reference_y", "reference_z",
                         "position_error_m", "orientation_error_deg"])
        for row in zip(times, est_pos[:, 0], est_pos[:, 1], est_pos[:, 2],
                       ref_pos[:, 0], ref_pos[:, 1], ref_pos[:, 2],
                       position_error, orientation_error):
            writer.writerow(row)
    return result


def analyze_diagnostics_only(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("LIO degeneracy CSV is empty: {}".format(path))
    required = {"timestamp", "is_degenerate", "velocity_projection_on_weak_direction",
                "position_correction_on_weak_direction"}
    missing = required.difference(rows[0])
    if missing:
        raise ValueError("LIO degeneracy CSV is missing: {}".format(", ".join(sorted(missing))))
    times = np.asarray([float(row["timestamp"]) for row in rows])
    if not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0.0):
        raise ValueError("LIO diagnostic timestamps must be finite and strictly increasing")
    conflicts = []
    for row in rows:
        velocity = float(row["velocity_projection_on_weak_direction"])
        correction = float(row["position_correction_on_weak_direction"])
        if not math.isfinite(velocity) or not math.isfinite(correction):
            raise ValueError("LIO diagnostic direction projections must be finite")
        conflicts.append(int(row.get("direction_conflict",
                                     int(row["is_degenerate"]) != 0 and velocity * correction < 0.0)) != 0)
    intervals = []
    start = None
    for index in range(len(conflicts) + 1):
        active = index < len(conflicts) and conflicts[index]
        if active and start is None:
            start = index
        if not active and start is not None:
            end = index - 1
            intervals.append({"start_time": float(times[start]), "end_time": float(times[end]),
                              "duration_s": float(times[end] - times[start]),
                              "frame_count": end - start + 1})
            start = None
    return {
        "reference_available": False,
        "degenerate_lidar_imu_conflict_intervals": intervals,
        "degenerate_frame_count": sum(int(row["is_degenerate"]) != 0 for row in rows),
        "direction_conflict_diagnostic_frame_count": sum(conflicts),
        "direction_guard_trigger_count": sum(
            int(row.get("direction_guard_triggered", 0)) != 0 for row in rows),
        "state_intervention_frame_count": sum(
            int(row.get("state_intervention_applied", 0)) != 0 for row in rows),
        "map_guard_request_frame_count": sum(
            int(row.get("map_guard_requested", 0)) != 0 for row in rows),
        "map_freeze_frame_count": sum(
            int(row.get("map_guard_enforced", 0)) != 0 for row in rows),
        "checks": {"diagnostic_row_count": len(rows)},
    }


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimated", help="estimated 7-column legacy or 8-column TUM trajectory")
    parser.add_argument("--reference", help="reference trajectory; timestamps are optional")
    parser.add_argument("--lio-degeneracy-csv", help="diagnostic-only input when no reference is available")
    parser.add_argument("--output", required=True, help="output directory")
    parser.add_argument("--estimated-rate", type=float, default=10.0)
    parser.add_argument("--reference-rate", type=float, default=10.0)
    parser.add_argument("--minimum-reverse-duration", type=float, default=1.0)
    parser.add_argument("--minimum-reverse-distance", type=float, default=0.5)
    parser.add_argument("--minimum-reference-speed", type=float, default=0.05)
    args = parser.parse_args()
    if args.reference and not args.estimated:
        parser.error("--reference requires --estimated")
    if not args.reference and not args.lio_degeneracy_csv:
        parser.error("provide --estimated with --reference, or --lio-degeneracy-csv")
    for name in ("estimated_rate", "reference_rate"):
        if getattr(args, name) <= 0.0:
            parser.error("--{} must be positive".format(name.replace("_", "-")))
    return args


def main():
    args = parse_arguments()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    try:
        result = (analyze_with_reference(args, output_dir) if args.reference else
                  analyze_diagnostics_only(args.lio_degeneracy_csv))
    except (OSError, ValueError, KeyError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2
    result_path = output_dir / "result.json"
    result_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print("wrote {}".format(result_path), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
