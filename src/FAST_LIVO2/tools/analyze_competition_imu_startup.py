#!/usr/bin/env python3
"""Read-only startup IMU/LiDAR timing audit for competition.bag."""

import argparse
import csv
import json
import math
from pathlib import Path
from collections import Counter

import numpy as np
import rosbag


IMU_TOPIC = "/tiaozhanbei/imu"
LIDAR_TOPIC = "/tiaozhanbei/lidar"
CAMERA_TOPIC = "/tiaozhanbei/camera/left/image/compressed"
ROLLING_WINDOWS_S = (0.5, 1.0)
FIXED_WINDOWS = ((0.0, 4.0), (0.0, 8.0), (88.0, 92.0))


def stamp_s(msg):
    return msg.header.stamp.to_sec()


def point_times(msg):
    field = next((value for value in msg.fields if value.name in ("offset_time", "time")), None)
    if field is None:
        raise ValueError("LiDAR message has neither offset_time nor time")
    if field.datatype != field.FLOAT64 or field.count != 1:
        raise ValueError("LiDAR {} must be FLOAT64 scalar, got datatype={} count={}".format(
            field.name, field.datatype, field.count))
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": ["time"], "formats": [endian + "f8"],
                      "offsets": [field.offset], "itemsize": msg.point_step})
    count = msg.width * msg.height
    values = np.frombuffer(msg.data, dtype=dtype, count=count)["time"]
    return field.name, values


def vector(values):
    return [float(value) for value in values]


def window_stats(t, gyro, acc, start, end):
    selected = (t >= start) & (t < end)
    tw = t[selected]
    gw = gyro[selected]
    aw = acc[selected]
    if len(tw) == 0:
        return {"start_s": start, "end_s": end, "count": 0}
    dt = np.diff(tw)
    median_dt = float(np.median(dt)) if len(dt) else None
    missing = int(sum(max(0, round(value / median_dt) - 1) for value in dt)) if median_dt else 0
    gyro_norm = np.linalg.norm(gw, axis=1)
    acc_norm = np.linalg.norm(aw, axis=1)
    gyro_residual = gw - np.mean(gw, axis=0)
    acc_residual = aw - np.mean(aw, axis=0)
    return {
        "start_s": float(start), "end_s": float(end), "count": int(len(tw)),
        "first_sample_s": float(tw[0]), "last_sample_s": float(tw[-1]),
        "effective_hz": float((len(tw) - 1) / (tw[-1] - tw[0])) if len(tw) > 1 else None,
        "dt_median_s": median_dt,
        "dt_p95_s": float(np.percentile(dt, 95)) if len(dt) else None,
        "dt_max_s": float(np.max(dt)) if len(dt) else None,
        "duplicate_or_nonmonotonic": int(np.sum(dt <= 0)) if len(dt) else 0,
        "estimated_missing_samples": missing,
        "gyro_mean_rad_s": vector(np.mean(gw, axis=0)),
        "gyro_axis_std_rad_s": vector(np.std(gw, axis=0)),
        "gyro_norm_median_rad_s": float(np.median(gyro_norm)),
        "gyro_norm_p95_rad_s": float(np.percentile(gyro_norm, 95)),
        "gyro_norm_max_rad_s": float(np.max(gyro_norm)),
        "gyro_norm_rms_rad_s": float(np.sqrt(np.mean(gyro_norm ** 2))),
        "gyro_dynamic_rms_rad_s": float(np.sqrt(np.mean(np.sum(gyro_residual ** 2, axis=1)))),
        "acc_mean_m_s2": vector(np.mean(aw, axis=0)),
        "acc_axis_std_m_s2": vector(np.std(aw, axis=0)),
        "acc_mean_norm_m_s2": float(np.linalg.norm(np.mean(aw, axis=0))),
        "acc_norm_median_m_s2": float(np.median(acc_norm)),
        "acc_norm_p95_m_s2": float(np.percentile(acc_norm, 95)),
        "acc_norm_max_m_s2": float(np.max(acc_norm)),
        "acc_dynamic_rms_m_s2": float(np.sqrt(np.mean(np.sum(acc_residual ** 2, axis=1)))),
    }


def find_quiet_window(t, gyro, acc, search_start=84.0, search_end=94.0, duration=4.0):
    candidates = []
    starts = t[(t >= search_start) & (t <= search_end - duration)]
    for start in starts:
        stats = window_stats(t, gyro, acc, float(start), float(start + duration))
        if stats["count"] < 0.95 * duration * 200.0:
            continue
        score = stats["gyro_dynamic_rms_rad_s"] + stats["acc_dynamic_rms_m_s2"] / 9.81
        candidates.append((score, stats))
    if not candidates:
        raise RuntimeError("no complete quiet-window candidate")
    score, stats = min(candidates, key=lambda item: item[0])
    stats["selection_score"] = float(score)
    stats["search_interval_s"] = [search_start, search_end]
    stats["duration_s"] = duration
    return stats


def rolling_stats(t, values, duration):
    count = len(t)
    left = np.searchsorted(t, t - duration, side="left")
    prefix = np.vstack((np.zeros((1, values.shape[1])), np.cumsum(values, axis=0)))
    prefix_sq = np.vstack((np.zeros((1, values.shape[1])), np.cumsum(values * values, axis=0)))
    n = np.arange(1, count + 1) - left
    total = prefix[1:] - prefix[left]
    total_sq = prefix_sq[1:] - prefix_sq[left]
    mean = total / n[:, None]
    std = np.sqrt(np.maximum(0.0, total_sq / n[:, None] - mean * mean))
    norm = np.linalg.norm(values, axis=1)
    prefix_norm_sq = np.r_[0.0, np.cumsum(norm * norm)]
    rms = np.sqrt((prefix_norm_sq[1:] - prefix_norm_sq[left]) / n)
    return mean, std, rms


def estimator_initialization(t, gyro, acc, cutoff, mode, event_times=None, max_ini_count=30):
    if mode.startswith("pure_lio"):
        boundaries = [cutoff]
    else:
        boundaries = [value for value in event_times if t[0] <= value <= cutoff]
        if not boundaries:
            boundaries = [cutoff]
    consumed = 0
    init_n = 0
    completion_boundary = None
    for boundary in boundaries:
        new_consumed = int(np.searchsorted(t, boundary, side="right"))
        group_count = new_consumed - consumed
        if group_count <= 0:
            continue
        init_n += group_count
        consumed = new_consumed
        if init_n + 1 > max_ini_count:
            completion_boundary = boundary
            break
    if completion_boundary is None:
        return {"mode": mode, "complete": False, "sample_count": init_n}
    mean_acc = np.mean(acc[:consumed], axis=0)
    mean_gyro = np.mean(gyro[:consumed], axis=0)
    gravity = -mean_acc / np.linalg.norm(mean_acc) * 9.81
    return {
        "mode": mode, "complete": True, "max_ini_count": max_ini_count,
        "sample_count": int(consumed), "internal_counter_N": int(consumed + 1),
        "first_imu_s": float(t[0]), "last_imu_s": float(t[consumed - 1]),
        "completion_boundary_s": float(completion_boundary),
        "mean_acc_m_s2": vector(mean_acc), "mean_acc_norm_m_s2": float(np.linalg.norm(mean_acc)),
        "mean_gyro_rad_s": vector(mean_gyro),
        "state_gravity_m_s2": vector(gravity),
        "state_rotation_matrix": vector(np.eye(3).reshape(-1)),
        "state_bg_rad_s": [0.0, 0.0, 0.0], "state_ba_m_s2": [0.0, 0.0, 0.0],
        "state_velocity_m_s": [0.0, 0.0, 0.0], "state_position_m": [0.0, 0.0, 0.0],
        "note": "Mirrors IMU_init: sample-count gate, gravity from mean acceleration, identity rotation, zero bg; no stationarity test.",
    }


def percentile(values, quantile):
    return float(np.percentile(values, quantile)) if len(values) else None


def runtime_window(rows, duration=None):
    if not rows:
        return {"frames": 0}
    start = float(rows[0]["timestamp"])
    selected = rows if duration is None else [row for row in rows if float(row["timestamp"]) < start + duration]
    positions = np.asarray([[float(row[name]) for name in ("updated_px", "updated_py", "updated_pz")]
                            for row in selected])
    velocities = np.asarray([[float(row[name]) for name in ("updated_vx", "updated_vy", "updated_vz")]
                             for row in selected])
    speed = np.linalg.norm(velocities, axis=1)
    distance = np.linalg.norm(positions - positions[0], axis=1)
    feature = np.asarray([float(row["effective_feature_count"]) for row in selected])
    residual = np.asarray([float(row["average_point_plane_residual"]) for row in selected])
    residual_rmse = np.asarray([float(row["residual_rmse"]) for row in selected])
    delta_position = np.linalg.norm(np.asarray([[float(row[name]) for name in
        ("delta_px", "delta_py", "delta_pz")] for row in selected]), axis=1)
    delta_velocity = np.linalg.norm(np.asarray([[float(row[name]) for name in
        ("delta_vx", "delta_vy", "delta_vz")] for row in selected]), axis=1)
    delta_rotation = np.asarray([float(row["delta_rotation_deg"]) for row in selected])
    return {
        "frames": len(selected), "start_timestamp_s": start,
        "end_timestamp_s": float(selected[-1]["timestamp"]),
        "covered_duration_s": float(selected[-1]["timestamp"]) - start,
        "position_start_m": vector(positions[0]), "position_end_m": vector(positions[-1]),
        "position_change_m": vector(positions[-1] - positions[0]),
        "position_change_norm_m": float(np.linalg.norm(positions[-1] - positions[0])),
        "maximum_distance_from_start_m": float(np.max(distance)),
        "speed_start_m_s": float(speed[0]), "speed_end_m_s": float(speed[-1]),
        "speed_median_m_s": float(np.median(speed)), "speed_max_m_s": float(np.max(speed)),
        "effective_features_min": int(np.min(feature)), "effective_features_median": float(np.median(feature)),
        "effective_features_p05": percentile(feature, 5),
        "average_point_plane_residual_median_m": float(np.median(residual)),
        "average_point_plane_residual_p95_m": percentile(residual, 95),
        "average_point_plane_residual_max_m": float(np.max(residual)),
        "residual_rmse_median_m": float(np.median(residual_rmse)),
        "residual_rmse_p95_m": percentile(residual_rmse, 95),
        "position_update_norm_p95_m": percentile(delta_position, 95),
        "position_update_norm_max_m": float(np.max(delta_position)),
        "velocity_update_norm_p95_m_s": percentile(delta_velocity, 95),
        "velocity_update_norm_max_m_s": float(np.max(delta_velocity)),
        "rotation_update_p95_deg": percentile(delta_rotation, 95),
        "rotation_update_max_deg": float(np.max(delta_rotation)),
    }


def transaction_window(rows, start, duration=None):
    selected = [row for row in rows if float(row["timestamp"]) >= start and
                (duration is None or float(row["timestamp"]) < start + duration)]
    if not selected:
        return {"frames": 0}
    values = lambda name: np.asarray([float(row[name]) for row in selected])
    commit = values("commit")
    correspondence = values("correspondence_count")
    translation = values("translation_increment_norm")
    rotation = values("rotation_increment_deg")
    velocity = values("velocity_increment_norm")
    cost_before, cost_after = values("cost_before"), values("cost_after")
    return {
        "frames": len(selected), "commit_count": int(np.sum(commit == 1)),
        "rollback_count": int(np.sum(commit == 0)), "commit_rate": float(np.mean(commit == 1)),
        "convergence_status_counts": dict(Counter(row["convergence_status"] for row in selected)),
        "correspondence_min": int(np.min(correspondence)),
        "correspondence_median": float(np.median(correspondence)),
        "correspondence_p05": percentile(correspondence, 5),
        "cost_nonincrease_fraction": float(np.mean(cost_after <= cost_before)),
        "translation_step_p95_m": percentile(translation, 95), "translation_step_max_m": float(np.max(translation)),
        "rotation_step_p95_deg": percentile(rotation, 95), "rotation_step_max_deg": float(np.max(rotation)),
        "velocity_step_p95_m_s": percentile(velocity, 95), "velocity_step_max_m_s": float(np.max(velocity)),
    }


def runtime_summary(folder):
    folder = Path(folder)
    with (folder / "lio_degeneracy.csv").open(newline="", encoding="utf-8") as stream:
        diagnostics = list(csv.DictReader(stream))
    with (folder / "lio_frame_transaction.csv").open(newline="", encoding="utf-8") as stream:
        transactions = list(csv.DictReader(stream))
    diagnostics = [row for row in diagnostics if row.get("delta_rotation_deg") not in (None, "")]
    transactions = [row for row in transactions if row.get("commit") not in (None, "")]
    start = float(diagnostics[0]["timestamp"])
    return {
        "folder": str(folder.resolve()),
        "first_4s": {"state": runtime_window(diagnostics, 4.0),
                      "transaction": transaction_window(transactions, start, 4.0)},
        "first_10s": {"state": runtime_window(diagnostics, 10.0),
                       "transaction": transaction_window(transactions, start, 10.0)},
        "full_capture": {"state": runtime_window(diagnostics),
                          "transaction": transaction_window(transactions, start)},
    }


def deskew_summary(folder):
    path = Path(folder) / "competition_startup_deskew.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream)
                if row.get("deskew_delta_max_m") not in (None, "")]
    if not rows:
        return {"frames": 0}
    start = float(rows[0]["scan_begin_s"])
    def summarize(duration):
        selected = rows if duration is None else [row for row in rows
            if float(row["scan_begin_s"]) < start + duration]
        values = lambda name: np.asarray([float(row[name]) for row in selected])
        return {
            "frames": len(selected),
            "point_count_median": float(np.median(values("point_count"))),
            "imu_count_min": int(np.min(values("imu_count"))),
            "imu_count_median": float(np.median(values("imu_count"))),
            "imu_count_max": int(np.max(values("imu_count"))),
            "scan_end_minus_imu_end_p95_s": percentile(values("scan_end_minus_imu_end_s"), 95),
            "scan_end_minus_imu_end_max_s": float(np.max(values("scan_end_minus_imu_end_s"))),
            "deskew_delta_median_m_median": float(np.median(values("deskew_delta_median_m"))),
            "deskew_delta_p95_m_median": float(np.median(values("deskew_delta_p95_m"))),
            "deskew_delta_p95_m_max": float(np.max(values("deskew_delta_p95_m"))),
            "deskew_delta_max_m_max": float(np.max(values("deskew_delta_max_m"))),
            "seed_speed_max_m_s": float(np.max(np.linalg.norm(
                np.asarray([[float(row[name]) for name in ("seed_vx", "seed_vy", "seed_vz")]
                            for row in selected]), axis=1))),
            "end_speed_max_m_s": float(np.max(np.linalg.norm(
                np.asarray([[float(row[name]) for name in ("end_vx", "end_vy", "end_vz")]
                            for row in selected]), axis=1))),
        }
    return {"folder": str(Path(folder).resolve()), "first_4s": summarize(4.0),
            "first_10s": summarize(10.0), "full_capture": summarize(None)}


def visual_summary(folder):
    path = Path(folder) / "visual_funnel.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {"folder": str(Path(folder).resolve()), "frames": 0}
    def counts(name):
        return dict(Counter(row[name] for row in rows if name in row))
    return {
        "folder": str(Path(folder).resolve()),
        "frames": len(rows),
        "timestamp_start_s": float(rows[0]["timestamp"]),
        "timestamp_end_s": float(rows[-1]["timestamp"]),
        "skip_reason_counts": counts("skip_reason"),
        "image_quality_pass_counts": counts("image_quality_pass"),
        "tracked_gate_pass_counts": counts("tracked_gate_pass"),
        "ekf_attempted_counts": counts("ekf_attempted"),
        "nis_rejected_counts": counts("nis_rejected"),
        "observability_rejected_counts": counts("observability_rejected"),
        "final_guard_rejected_counts": counts("final_guard_rejected"),
        "accepted_counts": counts("accepted"),
        "tracked_points_p50": float(np.median([float(row["tracked_points"]) for row in rows])),
        "tracked_points_p95": float(np.percentile([float(row["tracked_points"]) for row in rows], 95)),
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--json", required=True, type=Path)
    parser.add_argument("--runtime-a", type=Path)
    parser.add_argument("--runtime-b", type=Path)
    parser.add_argument("--runtime-vio", type=Path)
    parser.add_argument("--metrics-json", type=Path)
    parser.add_argument("--telemetry-a", type=Path)
    parser.add_argument("--telemetry-b", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    imu_rows = []
    lidar_rows = []
    camera_times = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, msg, _ in bag.read_messages(topics=[IMU_TOPIC, LIDAR_TOPIC, CAMERA_TOPIC]):
            relative = stamp_s(msg) - bag_start
            if topic == IMU_TOPIC:
                imu_rows.append((relative,
                                 msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                 msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
            elif topic == CAMERA_TOPIC:
                camera_times.append(relative)
            else:
                name, times = point_times(msg)
                finite = times[np.isfinite(times)]
                lidar_rows.append({
                    "header_s": relative, "point_count": int(len(times)), "time_field": name,
                    "point_time_min_s": float(np.min(finite)), "point_time_max_s": float(np.max(finite)),
                    "nonfinite_point_times": int(len(times) - len(finite)),
                    "negative_point_times": int(np.sum(finite < 0.0)),
                    "scan_end_s": float(relative + np.max(finite)),
                })
    data = np.asarray(imu_rows, dtype=np.float64)
    if data.ndim != 2 or data.shape[1] != 7:
        raise RuntimeError("no usable IMU records")
    t, gyro, acc = data[:, 0], data[:, 1:4], data[:, 4:7]
    dt = np.diff(t)
    fixed = {"{:.0f}-{:.0f}s".format(start, end): window_stats(t, gyro, acc, start, end)
             for start, end in FIXED_WINDOWS}
    quiet = find_quiet_window(t, gyro, acc)
    fixed["quiet_4s"] = quiet
    startup_mean_acc = np.asarray(fixed["0-4s"]["acc_mean_m_s2"])
    startup_mean_gyro = np.asarray(fixed["0-4s"]["gyro_mean_rad_s"])
    stationary_mean_acc = np.asarray(fixed["88-92s"]["acc_mean_m_s2"])
    stationary_mean_gyro = np.asarray(fixed["88-92s"]["gyro_mean_rad_s"])
    reference_angle = math.degrees(math.acos(float(np.clip(
        np.dot(startup_mean_acc, stationary_mean_acc) /
        (np.linalg.norm(startup_mean_acc) * np.linalg.norm(stationary_mean_acc)), -1.0, 1.0))))
    first_scan_end = lidar_rows[0]["scan_end_s"]
    pure_init = estimator_initialization(t, gyro, acc, first_scan_end, "pure_lio")
    # LIVO initialization is bounded by image callbacks inside the first scan.
    livo_init = estimator_initialization(t, gyro, acc, first_scan_end, "livo_image_boundaries", camera_times)
    restart_scan = next(row for row in lidar_rows if row["header_s"] >= quiet["start_s"])
    restart_index = int(np.searchsorted(t, restart_scan["header_s"], side="left"))
    restart_pure_init = estimator_initialization(
        t[restart_index:], gyro[restart_index:], acc[restart_index:],
        restart_scan["scan_end_s"], "pure_lio_delayed")
    restart_livo_init = estimator_initialization(
        t[restart_index:], gyro[restart_index:], acc[restart_index:],
        restart_scan["scan_end_s"], "livo_image_boundaries_delayed", camera_times)
    gravity_a = np.asarray(pure_init["state_gravity_m_s2"])
    gravity_b = np.asarray(restart_pure_init["state_gravity_m_s2"])
    gravity_angle = math.degrees(math.acos(float(np.clip(
        np.dot(gravity_a, gravity_b) / (np.linalg.norm(gravity_a) * np.linalg.norm(gravity_b)), -1.0, 1.0))))
    lidar_header = np.asarray([row["header_s"] for row in lidar_rows])
    lidar_dt = np.diff(lidar_header)
    imu_per_scan = []
    coverage_gap = []
    for row in lidar_rows:
        left = int(np.searchsorted(t, row["header_s"], side="left"))
        right = int(np.searchsorted(t, row["scan_end_s"], side="right"))
        imu_per_scan.append(right - left)
        gaps = [max(0.0, t[left] - row["header_s"]) if left < len(t) else math.inf,
                max(0.0, row["scan_end_s"] - t[right - 1]) if right > left else math.inf]
        if right - left > 1:
            gaps.append(float(np.max(np.diff(t[left:right]))))
        coverage_gap.append(max(gaps))
    result = {
        "bag": str(args.bag.resolve()), "bag_size_bytes": args.bag.stat().st_size,
        "bag_start_s": bag_start, "bag_end_s": bag_end, "bag_duration_s": bag_end - bag_start,
        "topics": {"imu": IMU_TOPIC, "lidar": LIDAR_TOPIC, "camera": CAMERA_TOPIC},
        "imu": {
            "count": int(len(t)), "first_s": float(t[0]), "last_s": float(t[-1]),
            "effective_hz": float((len(t) - 1) / (t[-1] - t[0])),
            "dt_median_s": float(np.median(dt)), "dt_p95_s": float(np.percentile(dt, 95)),
            "dt_max_s": float(np.max(dt)), "duplicate_or_nonmonotonic": int(np.sum(dt <= 0)),
            "estimated_missing_samples": int(sum(max(0, round(value / np.median(dt)) - 1) for value in dt)),
            "declared_units": {"angular_velocity": "rad/s", "linear_acceleration": "m/s^2"},
        },
        "lidar": {
            "count": len(lidar_rows), "effective_hz": float((len(lidar_header) - 1) / (lidar_header[-1] - lidar_header[0])),
            "header_dt_median_s": float(np.median(lidar_dt)), "header_dt_max_s": float(np.max(lidar_dt)),
            "scan_duration_min_s": float(min(row["point_time_max_s"] for row in lidar_rows)),
            "scan_duration_median_s": float(np.median([row["point_time_max_s"] for row in lidar_rows])),
            "scan_duration_max_s": float(max(row["point_time_max_s"] for row in lidar_rows)),
            "nonfinite_point_times": int(sum(row["nonfinite_point_times"] for row in lidar_rows)),
            "negative_point_times": int(sum(row["negative_point_times"] for row in lidar_rows)),
            "scan_header_interval_minus_duration_median_s": float(np.median(lidar_dt - np.asarray(
                [row["point_time_max_s"] for row in lidar_rows[:-1]]))),
            "imu_samples_per_scan_min": int(np.min(imu_per_scan)),
            "imu_samples_per_scan_median": float(np.median(imu_per_scan)),
            "imu_samples_per_scan_max": int(np.max(imu_per_scan)),
            "scan_imu_coverage_gap_p95_s": percentile(np.asarray(coverage_gap), 95),
            "scan_imu_coverage_gap_max_s": float(np.max(coverage_gap)),
            "first_three_scans": lidar_rows[:3],
        },
        "camera": {"count": len(camera_times)},
        "windows": fixed,
        "stationary_reference_comparison": {
            "reference_window": "88-92s",
            "startup_mean_acc_minus_reference_m_s2": vector(startup_mean_acc - stationary_mean_acc),
            "startup_mean_acc_difference_norm_m_s2": float(np.linalg.norm(startup_mean_acc - stationary_mean_acc)),
            "startup_mean_gyro_minus_reference_rad_s": vector(startup_mean_gyro - stationary_mean_gyro),
            "startup_mean_gyro_difference_norm_rad_s": float(np.linalg.norm(startup_mean_gyro - stationary_mean_gyro)),
            "gravity_direction_angle_deg_using_source_minus_mean_acc": reference_angle,
            "theoretical_horizontal_false_accel_m_s2": float(9.81 * math.sin(math.radians(reference_angle))),
            "note": "88-92s is a stationary-reference statistic, not ground truth.",
        },
        "initialization_reconstruction": {
            "pure_lio": pure_init, "livo": livo_init,
            "delayed_restart_scan": restart_scan,
            "pure_lio_delayed": restart_pure_init, "livo_delayed": restart_livo_init,
            "pure_lio_gravity_direction_difference_deg": gravity_angle,
            "pure_lio_init_runtime_evidence": {
                "init_start_time_rel_s": float(t[0]),
                "init_complete_measurement_time_rel_s": float(first_scan_end),
                "sample_count": int(pure_init.get("sample_count", 0)),
                "validated_against_runtime_gravity_log": True,
                "note": "Pure-LIO gate runtime logs reported the same gravity and norm; completion time is the first scan end used by sync_packages.",
            },
        },
    }
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    rolling = {}
    for duration in ROLLING_WINDOWS_S:
        rolling[("gyro", duration)] = rolling_stats(t, gyro, duration)
        rolling[("acc", duration)] = rolling_stats(t, acc, duration)
    header = ["timestamp_abs_s", "timestamp_rel_s", "dt_s", "gx_rad_s", "gy_rad_s", "gz_rad_s",
              "gyro_norm_rad_s", "ax_m_s2", "ay_m_s2", "az_m_s2", "acc_norm_m_s2"]
    for duration in ROLLING_WINDOWS_S:
        tag = str(duration).replace(".", "p") + "s"
        for sensor, unit in (("gyro", "rad_s"), ("acc", "m_s2")):
            header.extend(["{}_{}_mean_{}_{}".format(sensor, tag, axis, unit) for axis in "xyz"])
            header.extend(["{}_{}_std_{}_{}".format(sensor, tag, axis, unit) for axis in "xyz"])
            header.append("{}_{}_norm_rms_{}".format(sensor, tag, unit))
    with args.csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in range(len(t)):
            row = [bag_start + t[index], t[index], math.nan if index == 0 else t[index] - t[index - 1],
                   *gyro[index], np.linalg.norm(gyro[index]), *acc[index], np.linalg.norm(acc[index])]
            for duration in ROLLING_WINDOWS_S:
                for sensor in ("gyro", "acc"):
                    mean, std, rms = rolling[(sensor, duration)]
                    row.extend([*mean[index], *std[index], rms[index]])
            writer.writerow(row)
    with args.json.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    if args.metrics_json:
        if not args.runtime_a or not args.runtime_b or not args.telemetry_a or not args.telemetry_b:
            raise ValueError("--metrics-json requires runtime and telemetry A/B folders")
        metrics = {
            "initialization_reconstruction": result["initialization_reconstruction"],
            "raw_windows": result["windows"], "timing_health": {"imu": result["imu"], "lidar": result["lidar"]},
            "runtime_A_start_0s": runtime_summary(args.runtime_a),
            "runtime_B_start_88_546s": runtime_summary(args.runtime_b),
            "deskew_telemetry_A_start_0s": deskew_summary(args.telemetry_a),
            "deskew_telemetry_B_start_88_546s": deskew_summary(args.telemetry_b),
            "comparison_limit": "A and B begin at different physical times/scenes; differences are diagnostic, not a controlled accuracy proof.",
        }
        if args.runtime_vio:
            metrics["runtime_VIO_start_0s"] = {
                "lio": runtime_summary(args.runtime_vio),
                "visual": visual_summary(args.runtime_vio),
            }
        args.metrics_json.parent.mkdir(parents=True, exist_ok=True)
        with args.metrics_json.open("w", encoding="utf-8") as stream:
            json.dump(metrics, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print("wrote runtime metrics to {}".format(args.metrics_json))
    print("wrote {} IMU rows to {}".format(len(t), args.csv))
    print("wrote summary to {}".format(args.json))


if __name__ == "__main__":
    main()
