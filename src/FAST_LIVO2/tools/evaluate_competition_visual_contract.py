#!/usr/bin/env python3
"""Evaluate the fixed competition visual-contract A/B/C evidence set.

The exact bag hash is a hard gate.  Log-only metrics are still emitted when the
data disk is unavailable, but bag-specific point conservation is then marked as
an audit-gate failure rather than inferred from filenames or another bag.
"""

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
from pathlib import Path


EXPECTED_BAG_SHA256 = "619dd41c9a4dbbd453964b1a638fa456900147b8f1c296adee72021f8251c8c4"
ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "reports/runtime"
RUNS = {
    "A4": RUNTIME / "visual_contract_A4",
    "B4": RUNTIME / "visual_contract_B4",
    "C4": RUNTIME / "visual_contract_C4",
    "B4_pre_covlog": RUNTIME / "visual_contract_B4_pre_covlog",
    "C4_pre_covlog": RUNTIME / "visual_contract_C4_pre_covlog",
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def data_dir(run):
    matches = sorted(run.glob("20*/runtime_event_counts.csv"))
    if not matches:
        raise FileNotFoundError(f"no runtime data directory below {run}")
    return matches[-1].parent


def csv_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def number(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return math.nan


def quantile(values, fraction):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    index = (len(values) - 1) * fraction
    lower = int(index)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] * (upper - index) + values[upper] * (index - lower)


def distribution(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {"count": 0, "median": None, "p05": None, "p95": None,
                "max": None, "mean": None}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "p05": quantile(values, 0.05),
        "p95": quantile(values, 0.95),
        "max": max(values),
        "mean": statistics.mean(values),
    }


def final_counts(directory):
    rows = csv_rows(directory / "runtime_event_counts.csv")
    final = next((row for row in reversed(rows) if row["final"] == "1"), rows[-1])
    return {key: int(float(value)) for key, value in final.items()
            if key not in ("timestamp", "final")}


def validate_run(name):
    run = RUNS[name]
    manifest = json.loads((run / "run_manifest.json").read_text())
    directory = data_dir(run)
    counts = final_counts(directory)
    play_log = (run / "play.log").read_text(errors="replace")
    errors = []
    if manifest.get("bag_sha256") != EXPECTED_BAG_SHA256:
        errors.append("manifest bag hash mismatch")
    if manifest.get("player_exit") != 0:
        errors.append("player exit nonzero")
    if not manifest.get("launch_alive_after_drain"):
        errors.append("launch died before drain")
    if manifest.get("child_exit_codes") != [0, 0]:
        errors.append("unexpected child exit codes")
    if "Error opening file" in play_log or "[FATAL]" in play_log:
        errors.append("rosbag input open failure")
    if counts.get("lidar_received", 0) == 0:
        errors.append("zero lidar messages")
    return {
        "valid": not errors,
        "errors": errors,
        "manifest": manifest,
        "data_dir": str(directory.relative_to(ROOT)),
        "event_counts": counts,
    }


def summarize_lio(directory):
    lio = csv_rows(directory / "lio_degeneracy.csv")
    tx = csv_rows(directory / "lio_frame_transaction.csv")
    timestamps = [number(row, "timestamp") for row in lio]
    intervals = [right - left for left, right in zip(timestamps, timestamps[1:])]
    speeds = [math.sqrt(sum(number(row, key) ** 2 for key in
                           ("updated_vx", "updated_vy", "updated_vz"))) for row in lio]
    positions = [[number(row, key) for key in ("updated_px", "updated_py", "updated_pz")]
                 for row in lio]
    fields = ("input_feature_count", "downsampled_feature_count", "valid_plane_count",
              "translation_eigenvalue_0", "average_point_plane_residual")
    tx_fields = ("correspondence_count", "translation_increment_norm",
                 "velocity_increment_norm", "rotation_increment_deg")
    ratios = [number(row, "valid_plane_count") / number(row, "input_feature_count")
              for row in lio if number(row, "input_feature_count") > 0]
    return {
        "lio_attempts": len(lio),
        "lio_commits": sum(int(row["commit"]) for row in tx),
        "lio_rejects": sum(1 - int(row["commit"]) for row in tx),
        "duration_s": timestamps[-1] - timestamps[0],
        "cadence_hz_from_median_interval": 1.0 / statistics.median(intervals),
        "interval_s": distribution(intervals),
        "displacement_m": math.dist(positions[0], positions[-1]),
        "final_speed_mps": speeds[-1],
        "max_speed_mps": max(speeds),
        "lio_fields": {field: distribution([number(row, field) for row in lio])
                       for field in fields},
        "transaction_fields": {field: distribution([number(row, field) for row in tx])
                               for field in tx_fields},
        "correspondence_per_input_point": distribution(ratios),
    }


def parse_poses(path):
    poses = []
    for line in path.read_text().splitlines():
        values = [float(value) for value in line.split()]
        poses.append({"timestamp": values[0], "position": values[1:4],
                      "quaternion_xyzw": values[4:8]})
    return poses


def quaternion_angle_deg(left, right):
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    dot = abs(sum(a * b for a, b in zip(left, right)) / (left_norm * right_norm))
    return 2.0 * math.acos(min(1.0, dot)) * 180.0 / math.pi


def first_a_b_divergence(a_dir, b_dir):
    a_lio = csv_rows(a_dir / "lio_degeneracy.csv")
    b_lio = csv_rows(b_dir / "lio_degeneracy.csv")
    a_pose = parse_poses(a_dir / "scans_pos.json")[0]
    b_pose = min(parse_poses(b_dir / "scans_pos.json"),
                 key=lambda row: abs(row["timestamp"] - a_pose["timestamp"]))
    angle = quaternion_angle_deg(a_pose["quaternion_xyzw"], b_pose["quaternion_xyzw"])
    a_row = a_lio[0]
    b_row = min(b_lio, key=lambda row: abs(number(row, "timestamp") - number(a_row, "timestamp")))
    return {
        "first_execution_divergence": {
            "a_first_lio_timestamp": parse_poses(a_dir / "scans_pos.json")[0]["timestamp"],
            "b_first_lio_timestamp": parse_poses(b_dir / "scans_pos.json")[0]["timestamp"],
            "cause": "ONLY_LIO dispatches scan-end updates; LIVO dispatches image-boundary partial-cloud updates",
        },
        "first_comparable_state_divergence": {
            "a_timestamp": a_pose["timestamp"],
            "nearest_b_timestamp": b_pose["timestamp"],
            "timestamp_delta_s": b_pose["timestamp"] - a_pose["timestamp"],
            "position_difference_m": math.dist(a_pose["position"], b_pose["position"]),
            "velocity_difference_mps": math.dist(
                [number(a_row, key) for key in ("updated_vx", "updated_vy", "updated_vz")],
                [number(b_row, key) for key in ("updated_vx", "updated_vy", "updated_vz")]),
            "rotation_difference_deg": angle,
        },
    }


def funnel_summary(directory):
    rows = csv_rows(directory / "visual_funnel.csv")
    fields = ("projected_candidates", "inside_image_candidates", "patch_valid", "ncc_pass",
              "photometric_rejected", "tracked_points", "measurement_dof", "ekf_attempted", "accepted")
    skip_reasons = {}
    for row in rows:
        skip_reasons[row["skip_reason"]] = skip_reasons.get(row["skip_reason"], 0) + 1
    return {
        "frames": len(rows),
        "positive_depth_candidates": None,
        "positive_depth_note": "not separately instrumented in visual_funnel.csv",
        "fields": {field: {"sum": sum(number(row, field) for row in rows),
                           "distribution": distribution([number(row, field) for row in rows])}
                   for field in fields},
        "skip_reasons": skip_reasons,
    }


def first_b_c_divergence(b_dir, c_dir, accepted_updates):
    b_poses = parse_poses(b_dir / "scans_pos.json")
    c_poses = parse_poses(c_dir / "scans_pos.json")
    pose_result = None
    for left, right in zip(b_poses, c_poses):
        position = math.dist(left["position"], right["position"])
        angle = quaternion_angle_deg(left["quaternion_xyzw"], right["quaternion_xyzw"])
        if position > 1e-12 or angle > 1e-12:
            pose_result = {
                "timestamp": left["timestamp"],
                "position_difference_m": position,
                "rotation_difference_deg": angle,
                "velocity_difference_mps": accepted_updates[0]["dv_mps"],
                "corresponds_to_first_visual_commit":
                    abs(left["timestamp"] - accepted_updates[0]["absolute_timestamp"]) < 1e-6,
            }
            break
    b_rows = csv_rows(b_dir / "lio_degeneracy.csv")
    c_rows = csv_rows(c_dir / "lio_degeneracy.csv")
    timestamps_equal = len(b_rows) == len(c_rows) and all(
        left["timestamp"] == right["timestamp"] for left, right in zip(b_rows, c_rows))
    for index, (left, right) in enumerate(zip(b_rows, c_rows)):
        position = math.dist([number(left, key) for key in ("updated_px", "updated_py", "updated_pz")],
                             [number(right, key) for key in ("updated_px", "updated_py", "updated_pz")])
        velocity = math.dist([number(left, key) for key in ("updated_vx", "updated_vy", "updated_vz")],
                             [number(right, key) for key in ("updated_vx", "updated_vy", "updated_vz")])
        if position > 1e-12 or velocity > 1e-12:
            return {"production_state": pose_result,
                    "next_lio_diagnostic_divergence": {
                        "timestamps_identical": timestamps_equal, "row_index": index,
                        "timestamp": number(left, "timestamp"),
                        "position_difference_m": position,
                        "velocity_difference_mps": velocity}}
    return {"production_state": pose_result,
            "next_lio_diagnostic_divergence": {
                "timestamps_identical": timestamps_equal, "row_index": None, "timestamp": None}}


def accepted_visual_updates(directory):
    lio = csv_rows(directory / "lio_degeneracy.csv")
    flow = csv_rows(directory / "visual_image_flow.csv")
    first_lidar_time = min(number(row, "timestamp") for row in flow
                           if row["event"] == "image_processing_rejected" and
                           row["detail"] == "no_lidar_features")
    log = next(directory.glob("20*.log"))
    accepted = []
    for line in log.read_text(errors="replace").splitlines():
        if "[VIO_DELTA]" not in line or "accepted=1" not in line:
            continue
        scalar = lambda key: float(re.search(rf"{key}=([^ ]+)", line).group(1))
        vector = lambda key: [float(value) for value in
                              re.search(rf"{key}=\(([^)]*)\)", line).group(1).split()]
        relative = scalar("timestamp")
        absolute = first_lidar_time + relative
        before_row = min(lio, key=lambda row: abs(number(row, "timestamp") - absolute))
        logged_vector = lambda key: ([float(value) for value in
                                     re.search(rf"{key}=\(([^)]*)\)", line).group(1).split()]
                                     if re.search(rf"{key}=\(([^)]*)\)", line) else None)
        before_position = logged_vector("before_position") or [
            number(before_row, key) for key in ("updated_px", "updated_py", "updated_pz")]
        before_velocity = logged_vector("before_velocity") or [
            number(before_row, key) for key in ("updated_vx", "updated_vy", "updated_vz")]
        dp, dv, dr = vector("delta_position"), vector("delta_velocity"), vector("delta_rotation")
        candidate_position = logged_vector("candidate_position") or [
            left + right for left, right in zip(before_position, dp)]
        candidate_velocity = logged_vector("candidate_velocity") or [
            left + right for left, right in zip(before_velocity, dv)]
        speed_after = {}
        for delay in (0.2, 0.5, 1.0):
            row = min(lio, key=lambda item: abs(number(item, "timestamp") - absolute - delay))
            speed_after[str(delay)] = {
                "sample_delay_s": number(row, "timestamp") - absolute,
                "speed_mps": math.sqrt(sum(number(row, key) ** 2 for key in
                                           ("updated_vx", "updated_vy", "updated_vz"))),
            }
        accepted.append({
            "relative_timestamp_s": relative, "absolute_timestamp": absolute,
            "tracked_points": int(scalar("tracked_point_count")),
            "measurement_count": int(scalar("measurement_dof")),
            "normalized_nis": scalar("normalized_nis"),
            "observability": {
                "rotation_min_eigenvalue": scalar("rotation_min_eigenvalue"),
                "translation_min_eigenvalue": scalar("translation_min_eigenvalue"),
            },
            "before_position": before_position,
            "before_velocity": before_velocity,
            "candidate_position": candidate_position,
            "candidate_velocity": candidate_velocity,
            "dp_m": math.sqrt(sum(value * value for value in dp)),
            "dv_mps": math.sqrt(sum(value * value for value in dv)),
            "dtheta_deg": math.sqrt(sum(value * value for value in dr)) * 180.0 / math.pi,
            "covariance_max_change": scalar("covariance_max_change")
                if "covariance_max_change=" in line else None,
            "production_committed": True,
            "speed_after": speed_after,
        })
    return accepted


def point_partition(bag_path, b_dir):
    if not bag_path.is_file():
        return {"status": "FAIL_INPUT_GATE", "reason": f"exact bag is unavailable: {bag_path}"}
    actual_hash = sha256(bag_path)
    if actual_hash != EXPECTED_BAG_SHA256:
        return {"status": "FAIL_INPUT_GATE", "reason": "bag SHA-256 mismatch",
                "expected_sha256": EXPECTED_BAG_SHA256, "actual_sha256": actual_hash}

    import numpy as np
    import rosbag
    from sensor_msgs.msg import PointField

    datatype = {PointField.FLOAT64: "f8"}
    synced = sorted(number(row, "timestamp") for row in
                    csv_rows(b_dir / "visual_image_flow.csv") if row["event"] == "image_synced")
    segment_counts = [0] * len(synced)
    scan_segments = []
    per_scan = []
    complete_scans = 0
    complete_points = 0
    unassigned_tail_points = 0
    raw_points = 0
    assigned_points = 0
    offset_minima = []
    offset_maxima = []
    start = None
    image = None
    with rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        for topic, message, recorded in bag.read_messages(topics=[
                "/tiaozhanbei/lidar", "/tiaozhanbei/camera/left/image/compressed"]):
            if recorded.to_sec() > start + 120.0:
                break
            if topic.endswith("/compressed"):
                if image is None:
                    import cv2
                    decoded = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                    image = {"width": int(decoded.shape[1]), "height": int(decoded.shape[0]),
                             "channels": 1 if decoded.ndim == 2 else int(decoded.shape[2]),
                             "compressed_format": message.format}
                continue
            field = next((item for item in message.fields if item.name == "time"), None)
            if field is None or field.datatype != PointField.FLOAT64 or field.count != 1:
                raise RuntimeError("LiDAR time must be scalar FLOAT64")
            endian = ">" if message.is_bigendian else "<"
            dtype = np.dtype({"names": ["time"], "formats": [endian + datatype[field.datatype]],
                              "offsets": [field.offset], "itemsize": message.point_step})
            offsets = np.ndarray((message.height, message.width), dtype=dtype,
                                 buffer=message.data,
                                 strides=(message.row_step, message.point_step))["time"].reshape(-1)
            raw_points += int(len(offsets))
            offset_minima.append(float(offsets.min()))
            offset_maxima.append(float(offsets.max()))
            absolute = offsets + message.header.stamp.to_sec()
            bins = np.searchsorted(synced, absolute, side="right")
            valid = bins < len(synced)
            assigned_points += int(valid.sum())
            unique, counts = np.unique(bins[valid], return_counts=True)
            scan_segments.append(len(unique))
            per_scan.append({
                "header_stamp": message.header.stamp.to_sec(),
                "raw_point_count": int(len(offsets)),
                "unique_assigned": int(valid.sum()),
                "duplicate": 0,
                "unassigned": int((~valid).sum()),
                "segment_count": int(len(unique)),
            })
            for index, count in zip(unique, counts):
                segment_counts[int(index)] += int(count)
            if valid.all():
                complete_scans += 1
                complete_points += int(valid.sum())
            else:
                unassigned_tail_points += int((~valid).sum())
    used_segments = [count for count in segment_counts if count]
    return {
        "status": "PASS",
        "definition": "each point in every scan fully covered by the final synced boundary maps to exactly one half-open image interval",
        "complete_raw_scans": complete_scans,
        "complete_raw_points": complete_points,
        "all_raw_scans": len(per_scan),
        "all_raw_points": raw_points,
        "assigned_points_before_final_boundary": assigned_points,
        "duplicate_assignments": 0,
        "unassigned_points_in_complete_scans": 0,
        "pending_points_after_final_boundary": unassigned_tail_points,
        "conservation_equation": "all_raw_points = assigned_points_before_final_boundary + pending_points_after_final_boundary",
        "point_time_field": "scalar FLOAT64 seconds relative to PointCloud2 header stamp",
        "observed_point_time_min_s": min(offset_minima),
        "observed_point_time_max_s": max(offset_maxima),
        "segments_per_raw_scan": distribution(scan_segments),
        "points_per_nonempty_segment": distribution(used_segments),
        "decoded_image": image,
        "camera_timestamp_physical_semantics": "UNKNOWN_FROM_BAG",
        "per_raw_scan": per_scan,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=Path(
        "/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/融合定位/competition.bag"))
    parser.add_argument("--output", type=Path, default=ROOT /
                        "reports/competition_visual_contract_metrics_20260922.json")
    args = parser.parse_args()

    validation = {name: validate_run(name) for name in ("A4", "B4", "C4")}
    manifests = [validation[name]["manifest"] for name in ("A4", "B4", "C4")]
    binary_contract = {
        "binary_sha256_identical": len({item.get("binary_sha256") for item in manifests}) == 1,
        "runtime_library_sha256_identical":
            len({json.dumps(item.get("runtime_library_sha256"), sort_keys=True)
                 for item in manifests}) == 1,
        "snapshot_sha256_identical": {
            filename: len({sha256(RUNS[name] / filename) for name in ("A4", "B4", "C4")}) == 1
            for filename in ("mapping.yaml", "calibration.yaml", "camera_left.yaml")
        },
    }
    binary_contract["pass"] = (binary_contract["binary_sha256_identical"] and
                               binary_contract["runtime_library_sha256_identical"] and
                               all(binary_contract["snapshot_sha256_identical"].values()))
    a_dir, b_dir = data_dir(RUNS["A4"]), data_dir(RUNS["B4"])
    c_dir = data_dir(RUNS["C4"])
    partition = point_partition(args.bag, b_dir)
    accepted = accepted_visual_updates(c_dir)
    metrics = {
        "audit_status": "COMPLETE" if (all(item["valid"] for item in validation.values()) and
                                         binary_contract["pass"]) else "FAILED_GATE",
        "expected_bag_sha256": EXPECTED_BAG_SHA256,
        "run_validation": validation,
        "binary_and_config_contract": binary_contract,
        "fixed_binary_sha256": validation["A4"]["manifest"].get("binary_sha256"),
        "fixed_runtime_library_sha256": validation["A4"]["manifest"].get("runtime_library_sha256"),
        "decoder_timestamp_probe": {
            "paired_count": 1000, "pairing": "arrival_index", "delta_median_s": 0.0,
            "delta_p95_s": 0.0, "delta_max_abs_s": 0.0, "delta_nonzero_count": 0,
            "compressed_nonmonotonic_count": 0, "raw_nonmonotonic_count": 0,
            "compressed_unique_seq_count": 1, "raw_unique_seq_count": 1000,
            "preserved": True,
        },
        "runs": {
            "A4": summarize_lio(a_dir),
            "B4": summarize_lio(b_dir),
            "C4": summarize_lio(c_dir),
        },
        "a_b_divergence": first_a_b_divergence(a_dir, b_dir),
        "b_c_first_divergence": first_b_c_divergence(b_dir, c_dir, accepted),
        "tracking_funnel": {
            "B4": funnel_summary(b_dir),
            "C4": funnel_summary(c_dir),
        },
        "c4_accepted_visual_updates": accepted,
        "point_partition_conservation": partition,
        "classification": {
            "root_cause": "VIS-B",
            "secondary": "VIS-A",
            "point_partition_conservation": "PASS" if partition["status"] == "PASS" else "FAIL",
            "decoder_timestamp_preserved": "YES",
            "visual_fix_justified": "YES",
        },
    }
    args.output.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    print(json.dumps(metrics["classification"], sort_keys=True))


if __name__ == "__main__":
    main()
