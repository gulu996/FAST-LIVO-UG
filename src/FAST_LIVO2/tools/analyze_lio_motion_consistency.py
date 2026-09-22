#!/usr/bin/env python3
"""Summarize one completed motion_consistency_shadow.csv without GT/reference data."""

import argparse
import csv
import json
import math
from pathlib import Path


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def norm(row, prefix, indices):
    values = [finite(row[f"{prefix}_{index}"]) for index in indices]
    return math.sqrt(sum(value * value for value in values)) if all(
        math.isfinite(value) for value in values
    ) else math.nan


def percentile(sorted_values, fraction):
    if not sorted_values:
        return math.nan
    position = fraction * (len(sorted_values) - 1)
    lower = int(math.floor(position))
    upper = min(len(sorted_values) - 1, lower + 1)
    alpha = position - lower
    return sorted_values[lower] * (1.0 - alpha) + sorted_values[upper] * alpha


def summary(rows, key):
    values = sorted(value for row in rows if math.isfinite(value := row.get(key, math.nan)))
    return {
        "count": len(values),
        "min": values[0] if values else math.nan,
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "p999": percentile(values, 0.999),
        "max": values[-1] if values else math.nan,
    }


def add_derived(row, degeneracy, sensor_start):
    row["relative_time_s"] = finite(row["timestamp"]) - sensor_start
    row["delta_velocity_norm"] = finite(row["delta_velocity_norm_mps"])
    row["q_velocity_prior"] = finite(row["q_velocity_prior_value"])
    row["q_state_prior"] = finite(row["q_state_prior_value"])
    row["q_velocity_correction_approx"] = finite(
        row["q_velocity_correction_approx_value"]
    )
    row["innovation_velocity_norm"] = norm(
        row, "last_iteration_innovation_component", range(7, 10)
    )
    row["relinearization_velocity_norm"] = norm(
        row, "last_iteration_relinearization_component", range(7, 10)
    )
    row["accumulated_innovation_velocity_norm"] = norm(
        row, "accumulated_innovation_component", range(7, 10)
    )
    row["accumulated_relinearization_velocity_norm"] = norm(
        row, "accumulated_relinearization_component", range(7, 10)
    )
    accumulated_velocity = [
        finite(row[f"accumulated_innovation_component_{index}"])
        + finite(row[f"accumulated_relinearization_component_{index}"])
        for index in range(7, 10)
    ]
    delta_velocity = [finite(row[f"delta_velocity_{axis}"]) for axis in "xyz"]
    row["velocity_decomposition_closure_error"] = math.sqrt(sum(
        (actual - decomposed) ** 2
        for actual, decomposed in zip(delta_velocity, accumulated_velocity)
    ))
    row["innovation_pose_norm"] = norm(
        row, "last_iteration_innovation_component", range(0, 6)
    )
    row["relinearization_pose_norm"] = norm(
        row, "last_iteration_relinearization_component", range(0, 6)
    )
    row["linearized_nis_per_dof"] = finite(row["linearized_nis_per_dof"])
    row["velocity_equivalent_gain"] = finite(row["velocity_equivalent_gain_norm"])
    for key in (
        "p_v_theta_frobenius_norm",
        "p_v_position_frobenius_norm",
        "p_v_bg_frobenius_norm",
        "p_v_ba_frobenius_norm",
        "p_v_gravity_frobenius_norm",
        "cumulative_dv_025_norm",
        "cumulative_dv_050_norm",
        "cumulative_dv_100_norm",
        "dv_direction_persistence_1s",
        "translation_information_ratio",
        "translation_information_condition",
        "rotation_information_ratio",
        "rotation_information_condition",
        "pose_information_condition_number",
        "pose_information_eigenvalue_0",
        "maximum_iteration_linearized_nis_per_dof",
        "mean_iteration_linearized_nis_per_dof",
        "maximum_iteration_velocity_equivalent_gain_norm",
        "maximum_iteration_velocity_innovation_component_norm",
    ):
        row[key] = finite(row[key])
    delta_position = [finite(row[f"delta_position_{axis}"]) for axis in "xyz"]
    weak_translation = [
        finite(row[f"weak_translation_direction_world_{index}"]) for index in range(3)
    ]
    projection = sum(a * b for a, b in zip(delta_position, weak_translation))
    position_norm = finite(row["delta_position_norm_m"])
    row["weak_translation_projection_abs"] = abs(projection)
    row["weak_translation_projection_fraction"] = (
        abs(projection) / position_norm if position_norm > 1e-15 else math.nan
    )
    row["shadow_warn_value"] = finite(row["shadow_warn"])
    row["raw_is_degenerate_value"] = finite(row["raw_is_degenerate"])
    row["is_degenerate_value"] = finite(row["is_degenerate"])
    if degeneracy:
        velocity = [finite(degeneracy[f"updated_v{axis}"]) for axis in "xyz"]
        row["candidate_speed_mps"] = math.sqrt(sum(value * value for value in velocity))
    else:
        row["candidate_speed_mps"] = math.nan


METRICS = (
    "candidate_speed_mps",
    "delta_velocity_norm",
    "q_velocity_prior",
    "q_state_prior",
    "q_velocity_correction_approx",
    "linearized_nis_per_dof",
    "innovation_velocity_norm",
    "relinearization_velocity_norm",
    "accumulated_innovation_velocity_norm",
    "accumulated_relinearization_velocity_norm",
    "velocity_decomposition_closure_error",
    "maximum_iteration_linearized_nis_per_dof",
    "mean_iteration_linearized_nis_per_dof",
    "maximum_iteration_velocity_equivalent_gain_norm",
    "maximum_iteration_velocity_innovation_component_norm",
    "velocity_equivalent_gain",
    "p_v_theta_frobenius_norm",
    "p_v_position_frobenius_norm",
    "p_v_bg_frobenius_norm",
    "p_v_ba_frobenius_norm",
    "p_v_gravity_frobenius_norm",
    "cumulative_dv_025_norm",
    "cumulative_dv_050_norm",
    "cumulative_dv_100_norm",
    "dv_direction_persistence_1s",
    "weak_translation_projection_abs",
    "weak_translation_projection_fraction",
    "translation_information_ratio",
    "translation_information_condition",
    "rotation_information_ratio",
    "rotation_information_condition",
    "pose_information_eigenvalue_0",
    "pose_information_condition_number",
)


TIMELINE_FIELDS = (
    "relative_time_s",
    "frame_id",
    "shadow_warn",
    "shadow_reason",
    "candidate_speed_mps",
    "delta_velocity_norm",
    "q_velocity_prior",
    "q_state_prior",
    "q_velocity_correction_approx",
    "linearized_nis_per_dof",
    "innovation_velocity_norm",
    "relinearization_velocity_norm",
    "accumulated_innovation_velocity_norm",
    "accumulated_relinearization_velocity_norm",
    "velocity_decomposition_closure_error",
    "maximum_iteration_linearized_nis_per_dof",
    "mean_iteration_linearized_nis_per_dof",
    "maximum_iteration_velocity_equivalent_gain_norm",
    "maximum_iteration_velocity_innovation_component_norm",
    "velocity_equivalent_gain",
    "p_v_theta_frobenius_norm",
    "p_v_position_frobenius_norm",
    "p_v_bg_frobenius_norm",
    "p_v_ba_frobenius_norm",
    "p_v_gravity_frobenius_norm",
    "cumulative_dv_025_norm",
    "cumulative_dv_050_norm",
    "cumulative_dv_100_norm",
    "consecutive_dv_direction_cosine",
    "dv_direction_persistence_1s",
    "dv_velocity_angle_deg",
    "dv_dp_angle_deg",
    "weak_translation_projection_abs",
    "weak_translation_projection_fraction",
    "translation_information_ratio",
    "translation_information_condition",
    "rotation_information_ratio",
    "rotation_information_condition",
    "pose_information_eigenvalue_0",
    "pose_information_condition_number",
    "pose_information_rank",
    "raw_is_degenerate",
    "is_degenerate",
    "correction_covariance_psd",
    "correction_covariance_rank",
    "commit",
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion", required=True, type=Path)
    parser.add_argument("--degeneracy", type=Path)
    parser.add_argument("--sensor-start", required=True, type=float)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    degeneracy_by_frame = {}
    if args.degeneracy:
        with args.degeneracy.open(newline="") as stream:
            degeneracy_by_frame = {
                row["frame_id"]: row for row in csv.DictReader(stream)
            }
    with args.motion.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        add_derived(row, degeneracy_by_frame.get(row["frame_id"]), args.sensor_start)

    windows = {
        "baseline_0_880": [row for row in rows if 0.0 <= row["relative_time_s"] < 880.0],
        "anomaly_880_930": [row for row in rows if 880.0 <= row["relative_time_s"] <= 930.0],
        "focus_888_901": [row for row in rows if 888.0 <= row["relative_time_s"] <= 901.0],
        "focus_900_906": [row for row in rows if 900.0 <= row["relative_time_s"] <= 906.0],
    }
    report = {
        "input": str(args.motion),
        "sensor_start": args.sensor_start,
        "row_count": len(rows),
        "windows": {
            name: {
                "row_count": len(window_rows),
                "shadow_warn_count": sum(int(row["shadow_warn"]) for row in window_rows),
                "raw_degenerate_count": sum(int(row["raw_is_degenerate"]) for row in window_rows),
                "latched_degenerate_count": sum(int(row["is_degenerate"]) for row in window_rows),
                "metrics": {key: summary(window_rows, key) for key in METRICS},
            }
            for name, window_rows in windows.items()
        },
    }

    baseline = windows["baseline_0_880"]
    anomaly = windows["anomaly_880_930"]
    first_exceedance = {}
    for key in METRICS:
        baseline_summary = summary(baseline, key)
        upper = baseline_summary["max"]
        lower = baseline_summary["min"]
        above = next((row for row in anomaly if row[key] > upper), None)
        below = next((row for row in anomaly if row[key] < lower), None)
        first_exceedance[key] = {
            "baseline_min": lower,
            "baseline_max": upper,
            "first_above_baseline_max_s": above["relative_time_s"] if above else None,
            "first_above_value": above[key] if above else None,
            "first_below_baseline_min_s": below["relative_time_s"] if below else None,
            "first_below_value": below[key] if below else None,
        }
    report["first_anomaly_exceedance"] = first_exceedance

    targets = (900.269, 900.917, 905.321)
    report["nearest_target_frames"] = {
        str(target): min(rows, key=lambda row: abs(row["relative_time_s"] - target))
        for target in targets
    }
    report["nearest_target_frames"] = {
        target: {key: row.get(key) for key in TIMELINE_FIELDS}
        for target, row in report["nearest_target_frames"].items()
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "motion_consistency_summary.json").open("w") as stream:
        json.dump(report, stream, indent=2, allow_nan=True)
        stream.write("\n")
    timeline = [row for row in rows if 888.0 <= row["relative_time_s"] <= 906.0]
    with (args.output_dir / "motion_consistency_timeline_888_906.csv").open(
        "w", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=TIMELINE_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(timeline)
    print(json.dumps({
        "rows": len(rows),
        "baseline_rows": len(baseline),
        "anomaly_rows": len(anomaly),
        "output_dir": str(args.output_dir),
    }, indent=2))


if __name__ == "__main__":
    main()
