#!/usr/bin/env python3
"""Cross-dataset, internal-metric-only LIO sequential shadow evaluation.

Calibration consumes only registry entries marked ``calibration_normal``.
Failure entries are evaluated after calibration and never influence feature
normalization or thresholds.  No trajectory/reference/GNSS fields are read.
"""

import argparse
import bisect
import csv
import gzip
import json
import math
import statistics
from collections import deque
from pathlib import Path


FIXED_CHI2_19_P999 = 43.82019596451753
METHODS = (
    "A_fixed_single_q_state",
    "A_empirical_single_q_state",
    "B_consecutive_q_state",
    "C_q_state_ewma",
    "D_q_state_cusum",
    "E_state_direction",
    "F_state_direction_geometry",
    "G_rolling_robust_z",
)


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def percentile(values, fraction):
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        raise ValueError("cannot compute percentile of empty data")
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = min(len(ordered) - 1, lower + 1)
    alpha = position - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def robust_location_scale(values):
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        raise ValueError("cannot fit robust statistics to empty data")
    location = statistics.median(clean)
    mad = statistics.median(abs(value - location) for value in clean)
    # ponytail: five percent of the mean absolute magnitude prevents a constant
    # channel from manufacturing huge z-scores; replace with a noise model if
    # sensor-specific calibration becomes available.
    floor = max(1e-9, 0.05 * statistics.mean(abs(value) for value in clean))
    return location, max(1.4826 * mad, floor)


def robust_z(value, center_scale):
    return (value - center_scale[0]) / center_scale[1]


def raw_features(row):
    q_state = finite(row["q_state_prior_value"])
    q_velocity = finite(row["q_velocity_prior_value"])
    pose_min = max(finite(row["pose_information_eigenvalue_0"]), 1e-15)
    pose_condition = max(finite(row["pose_information_condition_number"]), 0.0)
    delta_position = [finite(row[f"delta_position_{axis}"]) for axis in "xyz"]
    weak_direction = [
        finite(row[f"weak_translation_direction_world_{index}"])
        for index in range(3)
    ]
    position_norm = math.sqrt(sum(value * value for value in delta_position))
    projection = abs(sum(value * direction for value, direction in
                         zip(delta_position, weak_direction)))
    weak_projection = projection / position_norm if position_norm > 1e-15 else 0.0
    weak_projection = min(max(weak_projection, 1e-9), 1.0 - 1e-9)
    translation_ratio = max(finite(row["translation_information_ratio"]), 1e-15)
    direction = finite(row["dv_direction_persistence_1s"])
    values = {
        "state": math.log1p(max(q_state, 0.0)),
        "velocity": math.log1p(max(q_velocity, 0.0)),
        "direction": direction,
        "geometry_pose_min": -math.log(pose_min),
        "geometry_condition": math.log1p(pose_condition),
        "geometry_projection": math.log(weak_projection / (1.0 - weak_projection)),
        "geometry_translation_ratio": -math.log(translation_ratio),
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError(f"non-finite required evidence at frame {row.get('frame_id')}")
    return values


def load_dataset(entry):
    path = Path(entry["motion_csv"])
    with path.open(newline="") as stream:
        source = list(csv.DictReader(stream))
    required = {
        "timestamp", "frame_id", "q_state_prior_valid", "q_state_prior_value",
        "q_velocity_prior_valid", "q_velocity_prior_value",
        "dv_direction_persistence_1s", "cumulative_dv_025_norm",
        "cumulative_dv_050_norm", "cumulative_dv_100_norm",
        "pose_information_eigenvalue_0", "pose_information_condition_number",
        "translation_information_ratio", "delta_position_x", "delta_position_y",
        "delta_position_z", "weak_translation_direction_world_0",
        "weak_translation_direction_world_1", "weak_translation_direction_world_2",
    }
    if not source or not required.issubset(source[0]):
        missing = sorted(required - set(source[0] if source else ()))
        raise ValueError(f"{path}: missing required columns {missing}")
    rows = []
    sensor_start = float(entry["sensor_start"])
    for item in source:
        if item["q_state_prior_valid"] != "1" or item["q_velocity_prior_valid"] != "1":
            continue
        evidence = raw_features(item)
        rows.append({
            "timestamp": finite(item["timestamp"]),
            "relative_time_s": finite(item["timestamp"]) - sensor_start,
            "frame_id": int(item["frame_id"]),
            "q_state": finite(item["q_state_prior_value"]),
            "q_velocity": finite(item["q_velocity_prior_value"]),
            "cumulative_dv_025": finite(item["cumulative_dv_025_norm"]),
            "cumulative_dv_050": finite(item["cumulative_dv_050_norm"]),
            "cumulative_dv_100": finite(item["cumulative_dv_100_norm"]),
            **evidence,
        })
    if not rows:
        raise ValueError(f"{path}: no valid motion-consistency rows")
    if any(right["timestamp"] <= left["timestamp"] for left, right in zip(rows, rows[1:])):
        raise ValueError(f"{path}: timestamps are not strictly increasing")
    return rows


def fit_feature_model(training_sets):
    keys = (
        "state", "velocity", "direction", "geometry_pose_min",
        "geometry_condition", "geometry_projection", "geometry_translation_ratio",
    )
    return {
        key: robust_location_scale([
            row[key] for rows in training_sets.values() for row in rows
        ]) for key in keys
    }


def empirical_upper_tail(value, ordered_reference):
    first_greater_equal = bisect.bisect_left(ordered_reference, value)
    return (len(ordered_reference) - first_greater_equal + 1.0) / (
        len(ordered_reference) + 1.0)


def transform_dataset(rows, model, empirical_reference, rolling_window_s,
                      ewma_tau_s, cusum_drift):
    transformed = []
    history = deque()
    ewma = 0.0
    cusum = 0.0
    previous_time = None
    rolling_floor = 0.05 * model["state"][1]
    for source in rows:
        row = dict(source)
        timestamp = row["timestamp"]
        dt = 0.0 if previous_time is None else min(max(timestamp - previous_time, 0.0), 0.25)
        previous_time = timestamp
        row["state_z"] = robust_z(row["state"], model["state"])
        row["velocity_z"] = robust_z(row["velocity"], model["velocity"])
        row["state_empirical_upper_tail"] = empirical_upper_tail(
            row["state"], empirical_reference["state"])
        row["velocity_empirical_upper_tail"] = empirical_upper_tail(
            row["velocity"], empirical_reference["velocity"])
        row["direction_z"] = robust_z(row["direction"], model["direction"])
        geometry_z = [
            robust_z(row[key], model[key]) for key in (
                "geometry_pose_min", "geometry_condition", "geometry_projection",
                "geometry_translation_ratio",
            )
        ]
        # At least two independent geometry symptoms must be elevated.
        row["weak_geometry_score"] = statistics.median(geometry_z)
        while history and history[0][0] < timestamp - rolling_window_s:
            history.popleft()
        if len(history) >= 20:
            prior_values = [value for _, value in history]
            center = statistics.median(prior_values)
            mad = statistics.median(abs(value - center) for value in prior_values)
            scale = max(1.4826 * mad, rolling_floor, 1e-9)
            row["rolling_robust_z"] = (row["state"] - center) / scale
        else:
            row["rolling_robust_z"] = 0.0
        history.append((timestamp, row["state"]))
        alpha = 1.0 if not transformed else 1.0 - math.exp(-dt / ewma_tau_s)
        ewma += alpha * (max(0.0, row["state_z"]) - ewma)
        cusum = max(0.0, cusum + (row["state_z"] - cusum_drift) * dt)
        row["q_state_ewma"] = ewma
        row["q_state_cusum"] = cusum
        row["dt"] = dt
        transformed.append(row)
    return transformed


def fit_thresholds(training_signals, tail_quantile, context_quantile):
    rows = [row for signals in training_signals.values() for row in signals]
    return {
        "q_state_fixed": FIXED_CHI2_19_P999,
        "state_z": percentile([row["state_z"] for row in rows], tail_quantile),
        "ewma": percentile([row["q_state_ewma"] for row in rows], tail_quantile),
        "cusum": percentile([row["q_state_cusum"] for row in rows], tail_quantile),
        "rolling_robust_z": percentile(
            [row["rolling_robust_z"] for row in rows], tail_quantile),
        "direction_z": percentile([row["direction_z"] for row in rows], context_quantile),
        "weak_geometry_score": percentile(
            [row["weak_geometry_score"] for row in rows], context_quantile),
    }


def method_series(rows, thresholds, consecutive_count):
    output = {method: [] for method in METHODS}
    run = 0
    for row in rows:
        state_exceeds = row["state_z"] > thresholds["state_z"]
        run = run + 1 if state_exceeds else 0
        definitions = {
            "A_fixed_single_q_state": (
                row["q_state"] > thresholds["q_state_fixed"],
                row["q_state"], thresholds["q_state_fixed"],
            ),
            "A_empirical_single_q_state": (
                state_exceeds, row["state_z"], thresholds["state_z"],
            ),
            "B_consecutive_q_state": (
                run >= consecutive_count, float(run), float(consecutive_count),
            ),
            "C_q_state_ewma": (
                row["q_state_ewma"] > thresholds["ewma"],
                row["q_state_ewma"], thresholds["ewma"],
            ),
            "D_q_state_cusum": (
                row["q_state_cusum"] > thresholds["cusum"],
                row["q_state_cusum"], thresholds["cusum"],
            ),
            "E_state_direction": (
                state_exceeds and row["direction_z"] > thresholds["direction_z"],
                min(row["state_z"] - thresholds["state_z"],
                    row["direction_z"] - thresholds["direction_z"]), 0.0,
            ),
            "F_state_direction_geometry": (
                state_exceeds and row["direction_z"] > thresholds["direction_z"] and
                row["weak_geometry_score"] > thresholds["weak_geometry_score"],
                min(row["state_z"] - thresholds["state_z"],
                    row["direction_z"] - thresholds["direction_z"],
                    row["weak_geometry_score"] - thresholds["weak_geometry_score"]), 0.0,
            ),
            "G_rolling_robust_z": (
                row["rolling_robust_z"] > thresholds["rolling_robust_z"],
                row["rolling_robust_z"], thresholds["rolling_robust_z"],
            ),
        }
        for method, (active, score, threshold) in definitions.items():
            output[method].append({**row, "active": bool(active),
                                   "score": score, "threshold": threshold})
    return output


def run_state_machine(rows, persistent_after_s, would_reject_after_s,
                      recovery_m, recovery_n):
    state = "NORMAL"
    state_entered = rows[0]["timestamp"] if rows else math.nan
    active_since = math.nan
    first_trigger = math.nan
    recovery_start = math.nan
    recovery_active_since = math.nan
    recovery_timestamp = math.nan
    clean_history = deque(maxlen=recovery_n)
    output = []
    for row in rows:
        timestamp = row["timestamp"]
        active = row["active"]
        reason = "none"
        previous_state = state
        if state == "NORMAL" and active:
            state = "SUSPECT"
            active_since = timestamp
            first_trigger = timestamp
            reason = "evidence_started"
        elif state == "SUSPECT":
            if not active:
                state = "NORMAL"
                reason = "evidence_cleared_before_persistence"
                active_since = first_trigger = math.nan
            elif timestamp - active_since >= persistent_after_s:
                state = "PERSISTENT_SUSPECT"
                reason = "persistent_duration_reached"
        elif state == "PERSISTENT_SUSPECT":
            if not active:
                state = "RECOVERY"
                recovery_start = timestamp
                active_since = recovery_active_since = math.nan
                clean_history.clear()
                clean_history.append(True)
                reason = "evidence_cleared"
            elif timestamp - active_since >= would_reject_after_s:
                state = "WOULD_REJECT"
                reason = "would_reject_duration_reached"
        elif state == "WOULD_REJECT" and not active:
            state = "RECOVERY"
            recovery_start = timestamp
            active_since = recovery_active_since = math.nan
            clean_history.clear()
            clean_history.append(True)
            reason = "evidence_cleared"
        elif state == "RECOVERY":
            clean_history.append(not active)
            if active:
                if not math.isfinite(recovery_active_since):
                    recovery_active_since = timestamp
                if timestamp - recovery_active_since >= would_reject_after_s:
                    state = "WOULD_REJECT"
                    active_since = recovery_active_since
                    recovery_start = recovery_active_since = math.nan
                    clean_history.clear()
                    reason = "recovery_retriggered_would_reject"
                else:
                    reason = "recovery_window_not_clean"
            else:
                recovery_active_since = math.nan
            if (state == "RECOVERY" and not active and
                    len(clean_history) == recovery_n and
                    sum(clean_history) >= recovery_m):
                state = "NORMAL"
                recovery_timestamp = timestamp
                reason = "m_of_n_clean_recovery"
                active_since = first_trigger = recovery_start = math.nan
                recovery_active_since = math.nan
                clean_history.clear()
        if state != previous_state:
            state_entered = timestamp
        state_duration = timestamp - state_entered
        evidence_duration = (
            timestamp - active_since if math.isfinite(active_since) else 0.0)
        output.append({**row, "state": state, "transition_reason": reason,
                       "state_duration_s": state_duration,
                       "accumulated_active_duration_s": evidence_duration,
                       "first_trigger_timestamp": first_trigger,
                       "recovery_start_timestamp": recovery_start,
                       "recovery_timestamp": recovery_timestamp})
    return output


def episode_durations(rows, predicate):
    durations = []
    start = None
    last = previous = None
    last_dt = 0.0
    for row in rows:
        if predicate(row):
            if start is None:
                start = row["timestamp"]
            last = row["timestamp"]
            last_dt = row.get(
                "dt", 0.0 if previous is None else row["timestamp"] - previous)
        elif start is not None:
            durations.append(max(0.0, row["timestamp"] - start))
            start = last = None
        previous = row["timestamp"]
    if start is not None:
        durations.append(max(0.0, last - start + last_dt))
    return durations


def transitions(rows, state):
    count = 0
    previous = None
    for row in rows:
        if row["state"] == state and previous != state:
            count += 1
        previous = row["state"]
    return count


def summarize_machine(rows, anomaly_start=None):
    active_count = sum(row["active"] for row in rows)
    non_normal = episode_durations(rows, lambda row: row["state"] != "NORMAL")
    would = episode_durations(rows, lambda row: row["state"] == "WOULD_REJECT")
    recovery_latencies = []
    pending = None
    for row in rows:
        if row["transition_reason"] == "evidence_cleared":
            pending = row["timestamp"]
        if row["transition_reason"] == "m_of_n_clean_recovery" and pending is not None:
            recovery_latencies.append(row["timestamp"] - pending)
            pending = None
    result = {
        "total_frames": len(rows),
        "warning_frames": active_count,
        "warning_rate": active_count / len(rows),
        "suspect_episode_count": transitions(rows, "SUSPECT"),
        "persistent_suspect_episode_count": transitions(rows, "PERSISTENT_SUSPECT"),
        "would_reject_count": sum(row["state"] == "WOULD_REJECT" for row in rows),
        "would_reject_episode_count": transitions(rows, "WOULD_REJECT"),
        "longest_non_normal_duration_s": max(non_normal, default=0.0),
        "maximum_would_reject_duration_s": max(would, default=0.0),
        "median_recovery_latency_s": (
            statistics.median(recovery_latencies) if recovery_latencies else None),
        "maximum_recovery_latency_s": max(recovery_latencies, default=None),
    }
    if anomaly_start is not None:
        post = [row for row in rows if row["relative_time_s"] >= anomaly_start]
        pre = [row for row in rows if row["relative_time_s"] < anomaly_start]
        first_active = next((row for row in post if row["active"]), None)
        first_would = next((row for row in post if row["state"] == "WOULD_REJECT"), None)
        post_active = episode_durations(post, lambda row: row["active"])
        post_would = episode_durations(post, lambda row: row["state"] == "WOULD_REJECT")
        result.update({
            "anomaly_start_s": anomaly_start,
            "active_at_anomaly_start": bool(post and post[0]["active"]),
            "would_reject_at_anomaly_start": bool(
                post and post[0]["state"] == "WOULD_REJECT"),
            "pre_anomaly_warning_rate": (
                sum(row["active"] for row in pre) / len(pre) if pre else None),
            "pre_anomaly_would_reject_count": sum(
                row["state"] == "WOULD_REJECT" for row in pre),
            "first_detection_s": first_active["relative_time_s"] if first_active else None,
            "detection_delay_s": (
                first_active["relative_time_s"] - anomaly_start if first_active else None),
            "first_would_reject_s": first_would["relative_time_s"] if first_would else None,
            "would_reject_delay_s": (
                first_would["relative_time_s"] - anomaly_start if first_would else None),
            "post_anomaly_warning_rate": (
                sum(row["active"] for row in post) / len(post) if post else None),
            "post_anomaly_active_duration_s": sum(post_active),
            "post_anomaly_maximum_active_duration_s": max(post_active, default=0.0),
            "post_anomaly_would_reject_duration_s": sum(post_would),
            "post_anomaly_maximum_would_reject_duration_s": max(
                post_would, default=0.0),
        })
    return result


def recovery_comparison(rows, would_after_s, recovery_m, recovery_n,
                        time_recovery_s):
    results = []
    for strategy in ("immediate", "m_of_n", "time_based", "evidence_decay"):
        latched = False
        active_start = None
        clean_start = None
        clean_history = deque(maxlen=recovery_n)
        decay_load = 0.0
        reject_start = None
        first_clean = None
        episodes = 0
        recoveries = []
        durations = []
        previous_time = None
        for row in rows:
            timestamp = row["timestamp"]
            dt = 0.0 if previous_time is None else min(timestamp - previous_time, 0.25)
            previous_time = timestamp
            active = row["active"]
            if active:
                active_start = timestamp if active_start is None else active_start
            else:
                active_start = None
            if not latched and active_start is not None and timestamp - active_start >= would_after_s:
                latched = True
                reject_start = timestamp
                first_clean = clean_start = None
                clean_history.clear()
                decay_load = would_after_s
                episodes += 1
            if not latched:
                continue
            if active:
                clean_start = first_clean = None
                clean_history.append(False)
                decay_load += dt
                continue
            first_clean = timestamp if first_clean is None else first_clean
            clean_start = timestamp if clean_start is None else clean_start
            clean_history.append(True)
            decay_load = max(0.0, decay_load - 2.0 * dt)
            recovered = (
                strategy == "immediate" or
                (strategy == "m_of_n" and len(clean_history) == recovery_n and
                 sum(clean_history) >= recovery_m) or
                (strategy == "time_based" and timestamp - clean_start >= time_recovery_s) or
                (strategy == "evidence_decay" and decay_load <= 0.0)
            )
            if recovered:
                recoveries.append(timestamp - first_clean)
                durations.append(timestamp - reject_start)
                latched = False
                reject_start = first_clean = clean_start = None
                clean_history.clear()
        if latched and reject_start is not None:
            durations.append(rows[-1]["timestamp"] - reject_start)
        results.append({
            "strategy": strategy,
            "reject_episode_count": episodes,
            "recovered_episode_count": len(recoveries),
            "maximum_reject_duration_s": max(durations, default=0.0),
            "median_recovery_latency_s": (
                statistics.median(recoveries) if recoveries else None),
            "maximum_recovery_latency_s": max(recoveries, default=None),
        })
    return results


def json_ready(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_ready(item) for item in value]
    return value


def write_csv(path, rows, fieldnames=None):
    rows = list(rows)
    if not rows:
        return
    fields = fieldnames or list(dict.fromkeys(
        key for row in rows for key in row))
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "wt", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(json_ready(rows))


def evaluate(registry, output_dir, args):
    entries = registry["datasets"]
    allowed_roles = {"calibration_normal", "failure_evaluation", "unclassified"}
    ids = [entry.get("id") for entry in entries]
    if any(not item for item in ids) or len(ids) != len(set(ids)):
        raise ValueError("dataset ids must be present and unique")
    invalid_roles = sorted({entry.get("role") for entry in entries} - allowed_roles,
                           key=str)
    if invalid_roles:
        raise ValueError(f"unsupported dataset roles: {invalid_roles}")
    classified = [entry for entry in entries if entry["role"] != "unclassified"]
    datasets = {entry["id"]: load_dataset(entry) for entry in classified}
    normal_ids = [entry["id"] for entry in entries if entry["role"] == "calibration_normal"]
    failure_ids = [entry["id"] for entry in entries if entry["role"] == "failure_evaluation"]
    if len(normal_ids) < 2:
        raise ValueError("at least two calibration_normal datasets are required for LODO evaluation")
    parameters = {
        "tail_quantile": args.tail_quantile,
        "context_quantile": args.context_quantile,
        "consecutive_count": args.consecutive_count,
        "rolling_window_s": args.rolling_window_s,
        "ewma_tau_s": args.ewma_tau_s,
        "cusum_drift_robust_sigma": args.cusum_drift,
        "persistent_after_s": args.persistent_after_s,
        "would_reject_after_s": args.would_reject_after_s,
        "canonical_recovery": f"{args.recovery_m}-of-{args.recovery_n} clean frames",
        "time_recovery_s": args.time_recovery_s,
    }
    evaluations = []
    timelines = []
    thresholds_by_fold = {}
    recovery_rows = []

    targets = []
    for heldout in normal_ids:
        targets.append((heldout, [item for item in normal_ids if item != heldout], "normal_lodo"))
    for failure in failure_ids:
        targets.append((failure, normal_ids, "failure_pooled_normal_calibration"))

    entry_by_id = {entry["id"]: entry for entry in entries}
    for target_id, training_ids, scheme in targets:
        training = {item: datasets[item] for item in training_ids}
        model = fit_feature_model(training)
        empirical_reference = {
            key: sorted(row[key] for rows in training.values() for row in rows)
            for key in ("state", "velocity")
        }
        training_signals = {
            item: transform_dataset(rows, model, empirical_reference,
                                    args.rolling_window_s, args.ewma_tau_s,
                                    args.cusum_drift)
            for item, rows in training.items()
        }
        thresholds = fit_thresholds(training_signals, args.tail_quantile,
                                    args.context_quantile)
        fold = f"train_{'+'.join(training_ids)}__test_{target_id}"
        thresholds_by_fold[fold] = {
            "feature_model": model,
            "empirical_reference_sample_count": len(empirical_reference["state"]),
            "thresholds": thresholds,
        }
        signals = transform_dataset(
            datasets[target_id], model, empirical_reference,
            args.rolling_window_s, args.ewma_tau_s, args.cusum_drift)
        series = method_series(signals, thresholds, args.consecutive_count)
        anomaly_windows = entry_by_id[target_id].get("confirmed_failure_windows", [])
        anomaly_start = anomaly_windows[0]["start_s"] if anomaly_windows else None
        for method, method_rows in series.items():
            machine = run_state_machine(
                method_rows, args.persistent_after_s, args.would_reject_after_s,
                args.recovery_m, args.recovery_n)
            metrics = summarize_machine(machine, anomaly_start)
            evaluations.append({"dataset": target_id, "role": entry_by_id[target_id]["role"],
                                "scheme": scheme, "fold": fold, "method": method,
                                **metrics})
            for row in machine:
                timelines.append({"dataset": target_id, "method": method,
                                  "scheme": scheme, **row})
            for recovery in recovery_comparison(
                    method_rows, args.would_reject_after_s, args.recovery_m,
                    args.recovery_n, args.time_recovery_s):
                recovery_rows.append({"dataset": target_id, "role": entry_by_id[target_id]["role"],
                                      "method": method, **recovery})

    pareto = []
    for method in METHODS:
        normals = [row for row in evaluations if row["method"] == method and
                   row["role"] == "calibration_normal"]
        failures = [row for row in evaluations if row["method"] == method and
                    row["role"] == "failure_evaluation"]
        normal_frames = sum(row["total_frames"] for row in normals)
        normal_warnings = sum(row["warning_frames"] for row in normals)
        detection_delays = [row["detection_delay_s"] for row in failures
                            if row.get("detection_delay_s") is not None]
        delays = [row["would_reject_delay_s"] for row in failures
                  if row.get("would_reject_delay_s") is not None]
        recovery_latencies = [
            row["median_recovery_latency_s"] for row in normals + failures
            if row.get("median_recovery_latency_s") is not None
        ]
        method_parameters = {
            "A_fixed_single_q_state": f"q_state>{FIXED_CHI2_19_P999:.12g}",
            "A_empirical_single_q_state": f"normal quantile={args.tail_quantile}",
            "B_consecutive_q_state": (
                f"normal quantile={args.tail_quantile}; K={args.consecutive_count}"),
            "C_q_state_ewma": (
                f"normal quantile={args.tail_quantile}; tau_s={args.ewma_tau_s}"),
            "D_q_state_cusum": (
                f"normal quantile={args.tail_quantile}; drift_sigma={args.cusum_drift}"),
            "E_state_direction": (
                f"state quantile={args.tail_quantile}; direction quantile={args.context_quantile}"),
            "F_state_direction_geometry": (
                f"state quantile={args.tail_quantile}; direction/geometry quantile="
                f"{args.context_quantile}"),
            "G_rolling_robust_z": (
                f"normal quantile={args.tail_quantile}; window_s={args.rolling_window_s}"),
        }[method]
        pareto.append({
            "method": method,
            "normal_warning_rate_lodo": normal_warnings / normal_frames,
            "normal_false_episode_count_lodo": sum(
                row["suspect_episode_count"] for row in normals),
            "normal_persistent_episode_count_lodo": sum(
                row["persistent_suspect_episode_count"] for row in normals),
            "normal_would_reject_count_lodo": sum(
                row["would_reject_count"] for row in normals),
            "normal_would_reject_episode_count_lodo": sum(
                row["would_reject_episode_count"] for row in normals),
            "normal_maximum_false_duration_s_lodo": max(
                (row["longest_non_normal_duration_s"] for row in normals), default=0.0),
            "failure_detected_count": len(detection_delays),
            "failure_would_reject_count": len(delays),
            "failure_count": len(failures),
            "failure_detection_delay_s": (
                max(detection_delays)
                if len(detection_delays) == len(failures) else None),
            "failure_would_reject_delay_s": max(delays) if len(delays) == len(failures) else None,
            "canonical_recovery_median_latency_s": (
                statistics.median(recovery_latencies) if recovery_latencies else None),
            "canonical_recovery_maximum_latency_s": max(
                (row["maximum_recovery_latency_s"] for row in normals + failures
                 if row.get("maximum_recovery_latency_s") is not None), default=None),
            "parameters": method_parameters,
            "mathematical_interpretation": {
                "A_fixed_single_q_state": "fixed chi-square-like reference; not a calibrated probability gate",
                "A_empirical_single_q_state": "normal-data robust z tail of log(1+q_state)",
                "B_consecutive_q_state": "K consecutive empirical q_state tail exceedances",
                "C_q_state_ewma": "sensor-time EWMA of positive robust q_state evidence",
                "D_q_state_cusum": "sensor-time one-sided CUSUM-like robust q_state accumulation",
                "E_state_direction": "empirical q_state tail intersected with directional persistence",
                "F_state_direction_geometry": "state plus direction plus independent weak-geometry context",
                "G_rolling_robust_z": "causal local median/MAD surprise of log(1+q_state)",
            }[method],
        })
    def on_frontier(candidate, delay_key):
        candidate_delay = (candidate[delay_key]
                           if candidate[delay_key] is not None else math.inf)
        return not any(
            other is not candidate and
            other["normal_warning_rate_lodo"] <= candidate["normal_warning_rate_lodo"] and
            other["normal_maximum_false_duration_s_lodo"] <= candidate["normal_maximum_false_duration_s_lodo"] and
            (other[delay_key]
             if other[delay_key] is not None else math.inf) <= candidate_delay and
            (other["normal_warning_rate_lodo"] < candidate["normal_warning_rate_lodo"] or
             other["normal_maximum_false_duration_s_lodo"] < candidate["normal_maximum_false_duration_s_lodo"] or
             (other[delay_key]
              if other[delay_key] is not None else math.inf) < candidate_delay)
            for other in pareto)
    for candidate in pareto:
        candidate["pareto_frontier"] = on_frontier(
            candidate, "failure_detection_delay_s")
        candidate["gate_pareto_frontier"] = on_frontier(
            candidate, "failure_would_reject_delay_s")

    output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "policy": {
            "calibration_uses_roles": ["calibration_normal"],
            "failure_metrics_used_for_thresholds": False,
            "ground_truth_gnss_reference_fields_read": False,
            "normal_evaluation": "leave-one-dataset-out",
            "failure_evaluation": "thresholds fitted to all calibration-normal datasets",
        },
        "parameters": parameters,
        "registry": registry,
        "thresholds_by_fold": thresholds_by_fold,
        "evaluations": evaluations,
        "pareto": pareto,
        "recovery_comparison": recovery_rows,
    }
    with (output_dir / "sequential_shadow_summary.json").open("w") as stream:
        json.dump(json_ready(report), stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    write_csv(output_dir / "dataset_method_metrics.csv", evaluations)
    write_csv(output_dir / "pareto.csv", pareto)
    write_csv(output_dir / "recovery_comparison.csv", recovery_rows)
    timeline_fields = (
        "dataset", "method", "scheme", "timestamp", "relative_time_s", "frame_id",
        "active", "score", "threshold", "state", "transition_reason",
        "state_duration_s", "accumulated_active_duration_s",
        "first_trigger_timestamp", "recovery_start_timestamp",
        "recovery_timestamp", "q_state", "q_velocity", "state_z", "velocity_z",
        "state_empirical_upper_tail", "velocity_empirical_upper_tail",
        "q_state_ewma", "q_state_cusum", "rolling_robust_z", "direction",
        "direction_z", "weak_geometry_score", "cumulative_dv_025",
        "cumulative_dv_050", "cumulative_dv_100",
    )
    write_csv(output_dir / "sequential_shadow_timeline.csv.gz", timelines, timeline_fields)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--tail-quantile", type=float, default=0.999)
    parser.add_argument("--context-quantile", type=float, default=0.75)
    parser.add_argument("--consecutive-count", type=int, default=3)
    parser.add_argument("--rolling-window-s", type=float, default=5.0)
    parser.add_argument("--ewma-tau-s", type=float, default=0.5)
    parser.add_argument("--cusum-drift", type=float, default=1.0)
    parser.add_argument("--persistent-after-s", type=float, default=0.15)
    parser.add_argument("--would-reject-after-s", type=float, default=0.35)
    parser.add_argument("--recovery-m", type=int, default=8)
    parser.add_argument("--recovery-n", type=int, default=10)
    parser.add_argument("--time-recovery-s", type=float, default=0.5)
    args = parser.parse_args()
    if not 0.5 < args.tail_quantile < 1.0:
        parser.error("--tail-quantile must be between 0.5 and 1")
    if not 0.5 <= args.context_quantile < 1.0:
        parser.error("--context-quantile must be between 0.5 and 1")
    if args.consecutive_count < 1:
        parser.error("--consecutive-count must be positive")
    if args.rolling_window_s <= 0.0 or args.ewma_tau_s <= 0.0:
        parser.error("rolling window and EWMA tau must be positive")
    if args.cusum_drift < 0.0:
        parser.error("--cusum-drift must be nonnegative")
    if not 0.0 <= args.persistent_after_s <= args.would_reject_after_s:
        parser.error("state durations must satisfy 0 <= persistent <= would-reject")
    if not 1 <= args.recovery_m <= args.recovery_n:
        parser.error("recovery must satisfy 1 <= M <= N")
    if args.time_recovery_s < 0.0:
        parser.error("--time-recovery-s must be nonnegative")
    registry = json.loads(args.registry.read_text())
    report = evaluate(registry, args.output_dir, args)
    print(json.dumps({"datasets": len(report["registry"]["datasets"]),
                      "evaluations": len(report["evaluations"]),
                      "output_dir": str(args.output_dir)}, indent=2))


if __name__ == "__main__":
    main()
