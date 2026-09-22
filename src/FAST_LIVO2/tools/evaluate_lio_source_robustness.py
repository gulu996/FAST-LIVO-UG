#!/usr/bin/env python3
"""Internal-metric-only P2-D source robustness evaluation.

The input registry assigns dataset roles before this script runs.  This tool
never reads trajectories, GNSS, or reference errors.  Counterfactual columns
come from the final production linearization and therefore estimate local
source-update effects, not replayed trajectories.
"""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


BLOCKS = {
    "pose": range(0, 6),
    "exposure": range(6, 7),
    "velocity": range(7, 10),
    "bias_g": range(10, 13),
    "bias_a": range(13, 16),
    "gravity": range(16, 19),
}
METHODS = (
    "transfer_mild", "transfer_medium", "transfer_stronger",
    "damping_mild", "damping_medium", "damping_stronger",
)
ITERATION_METHODS = (
    "iteration_all_075", "iteration_all_050",
    "iteration_innovation_075", "iteration_innovation_050",
)
C_METHODS = ("max_iter_confidence_075", "max_iter_confidence_050")


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def vector(row, prefix, size):
    return [finite(row[f"{prefix}_{index}"]) for index in range(size)]


def raw_state(row):
    names = (
        "delta_rotation_x", "delta_rotation_y", "delta_rotation_z",
        "delta_position_x", "delta_position_y", "delta_position_z",
        "delta_exposure",
        "delta_velocity_x", "delta_velocity_y", "delta_velocity_z",
        "delta_bias_g_x", "delta_bias_g_y", "delta_bias_g_z",
        "delta_bias_a_x", "delta_bias_a_y", "delta_bias_a_z",
        "delta_gravity_x", "delta_gravity_y", "delta_gravity_z",
    )
    return [finite(row[name]) for name in names]


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def subtract(left, right):
    return [a - b for a, b in zip(left, right)]


def add(left, right):
    return [a + b for a, b in zip(left, right)]


def scale(values, factor):
    return [factor * value for value in values]


def percentile(values, fraction):
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return math.nan
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = min(len(ordered) - 1, lower + 1)
    alpha = position - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def distribution(values):
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {"count": 0, "median": None, "p95": None, "p99": None, "max": None}
    return {
        "count": len(clean),
        "median": statistics.median(clean),
        "p95": percentile(clean, 0.95),
        "p99": percentile(clean, 0.99),
        "max": max(clean),
    }


def assert_internal_schema(path, fields):
    forbidden = ("ground_truth", "gnss_error", "reference_error", "trajectory_error")
    found = [field for field in fields if any(token in field.lower() for token in forbidden)]
    if found:
        raise ValueError(f"{path}: forbidden external-error columns present: {found}")


def load_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = set(reader.fieldnames or ())
    assert_internal_schema(path, fields)
    return rows, fields


def load_dataset(entry):
    motion, fields = load_csv(entry["motion_csv"])
    required = {
        "timestamp", "frame_id", "diagnostic_valid", "delta_velocity_x",
        "delta_velocity_y", "delta_velocity_z", "translation_information_ratio",
        "pose_direction_decomposition_valid", "source_counterfactual_compute_time_ms",
    }
    for method in METHODS:
        required.update(f"{method}_delta_state_{index}" for index in range(19))
        required.add(f"{method}_valid")
    missing = sorted(required - fields)
    if missing:
        raise ValueError(f"{entry['motion_csv']}: missing current P2-D columns {missing}")

    transaction, _ = load_csv(entry["transaction_csv"])
    degeneracy, _ = load_csv(entry["degeneracy_csv"])
    transaction_by_frame = {row["frame_id"]: row for row in transaction}
    degeneracy_by_frame = {row["frame_id"]: row for row in degeneracy}
    start = float(entry["sensor_start"])
    rows = []
    previous = None
    for source in motion:
        if source["diagnostic_valid"] != "1" or source["pose_direction_decomposition_valid"] != "1":
            continue
        if any(source[f"{method}_valid"] != "1" for method in METHODS):
            continue
        frame = source["frame_id"]
        tx = transaction_by_frame.get(frame)
        geometry = degeneracy_by_frame.get(frame)
        if tx is None or geometry is None:
            continue
        raw = raw_state(source)
        timestamp = finite(source["timestamp"])
        updated_velocity = [finite(geometry[f"updated_v{axis}"]) for axis in "xyz"]
        updated_position = [finite(geometry[f"updated_p{axis}"]) for axis in "xyz"]
        row = {
            "timestamp": timestamp,
            "relative_time_s": timestamp - start,
            "frame_id": int(frame),
            "raw": raw,
            "translation_ratio": finite(source["translation_information_ratio"]),
            "predicted_speed": finite(geometry["predicted_speed_mps"]),
            "updated_velocity": updated_velocity,
            "updated_position": updated_position,
            "convergence_status": tx["convergence_status"],
            "source_compute_ms": finite(source["source_counterfactual_compute_time_ms"]),
            "source": source,
        }
        row["methods"] = {method: vector(source, f"{method}_delta_state", 19)
                          for method in METHODS}
        innovation = vector(source, "accumulated_innovation_component", 19)
        relinearization = vector(source, "accumulated_relinearization_component", 19)
        row["iteration_closure_velocity"] = norm(subtract(
            raw[7:10], add(innovation[7:10], relinearization[7:10])))
        row["methods"].update({
            "iteration_all_075": scale(raw, 0.75),
            "iteration_all_050": scale(raw, 0.50),
            "iteration_innovation_075": add(scale(innovation, 0.75), relinearization),
            "iteration_innovation_050": add(scale(innovation, 0.50), relinearization),
        })
        confidence = 1.0 if tx["convergence_status"] != "max_iter_healthy" else 0.75
        row["methods"]["max_iter_confidence_075"] = scale(raw, confidence)
        confidence = 1.0 if tx["convergence_status"] != "max_iter_healthy" else 0.50
        row["methods"]["max_iter_confidence_050"] = scale(raw, confidence)
        if previous is not None:
            dt = timestamp - previous["timestamp"]
            if dt > 0.0:
                row["acceleration_proxy"] = norm(subtract(
                    updated_velocity, previous["updated_velocity"])) / dt
                a = previous["updated_velocity"]
                b = updated_velocity
                denominator = norm(a) * norm(b)
                row["turn_proxy_deg"] = (math.degrees(math.acos(max(-1.0, min(
                    1.0, sum(x * y for x, y in zip(a, b)) / denominator))))
                    if denominator > 1e-12 else math.nan)
                row["speed_change_proxy"] = abs(norm(b) - norm(a)) / dt
            else:
                row["acceleration_proxy"] = row["turn_proxy_deg"] = math.nan
                row["speed_change_proxy"] = math.nan
        else:
            row["acceleration_proxy"] = row["turn_proxy_deg"] = math.nan
            row["speed_change_proxy"] = math.nan
        rows.append(row)
        previous = row
    if not rows:
        raise ValueError(f"{entry['id']}: no valid joined P2-D rows")
    return rows


def assign_normal_regimes(rows):
    thresholds = {
        "weak_geometry": percentile([row["translation_ratio"] for row in rows], 0.10),
        "stop": percentile([row["predicted_speed"] for row in rows], 0.10),
        "acceleration": percentile([row["acceleration_proxy"] for row in rows], 0.90),
        "turn": percentile([row["turn_proxy_deg"] for row in rows], 0.90),
        "start_stop": percentile([row["speed_change_proxy"] for row in rows], 0.90),
    }
    for row in rows:
        row["regimes"] = {"all"}
        if row["translation_ratio"] <= thresholds["weak_geometry"]:
            row["regimes"].add("weak_geometry_decile")
        if row["predicted_speed"] <= thresholds["stop"]:
            row["regimes"].add("stop_low_speed_decile")
        if row["acceleration_proxy"] >= thresholds["acceleration"]:
            row["regimes"].add("acceleration_top_decile")
        if row["turn_proxy_deg"] >= thresholds["turn"]:
            row["regimes"].add("turn_top_decile")
        if row["speed_change_proxy"] >= thresholds["start_stop"]:
            row["regimes"].add("start_stop_top_decile")
        if row["convergence_status"] == "max_iter_healthy":
            row["regimes"].add("max_iter_healthy")
    return thresholds


def block_distortion(rows, method):
    result = {}
    for name, indices in BLOCKS.items():
        result[name] = distribution([
            norm([row["methods"][method][index] - row["raw"][index]
                  for index in indices]) for row in rows
        ])
    return result


def cumulative_velocity(rows, method=None):
    total = [0.0, 0.0, 0.0]
    peak = 0.0
    for row in rows:
        state = row["raw"] if method is None else row["methods"][method]
        velocity = state[7:10]
        total = add(total, velocity)
        peak = max(peak, norm(velocity))
    return {"vector": total, "norm": norm(total), "peak_frame_norm": peak}


def direction_summary(rows):
    output = []
    for direction in range(6):
        total = [0.0, 0.0, 0.0]
        rotation_fraction = []
        eigenvalues = []
        for row in rows:
            source = row["source"]
            contribution = vector(
                source, f"pose_direction_{direction}_velocity_contribution", 3)
            total = add(total, contribution)
            axis = vector(source, f"pose_direction_{direction}_axis", 6)
            rotation_fraction.append(norm(axis[:3]))
            eigenvalues.append(finite(source[f"pose_direction_eigenvalue_{direction}"]))
        output.append({
            "direction": direction,
            "cumulative_velocity_contribution": total,
            "cumulative_velocity_contribution_norm": norm(total),
            "median_rotation_axis_fraction": statistics.median(rotation_fraction),
            "median_translation_axis_fraction": statistics.median(
                [math.sqrt(max(0.0, 1.0 - value * value)) for value in rotation_fraction]),
            "median_information_eigenvalue": statistics.median(eigenvalues),
        })
    output.sort(key=lambda item: item["cumulative_velocity_contribution_norm"], reverse=True)
    return output


def evaluate_dataset(entry, rows):
    role = entry["role"]
    result = {
        "id": entry["id"], "role": role, "row_count": len(rows),
        "source_compute_ms": distribution([row["source_compute_ms"] for row in rows]),
        "iteration_velocity_closure": distribution([
            row["iteration_closure_velocity"] for row in rows]),
        "convergence_counts": {},
    }
    for row in rows:
        status = row["convergence_status"]
        result["convergence_counts"][status] = result["convergence_counts"].get(status, 0) + 1
    if role == "normal":
        result["regime_quantile_thresholds_internal_only"] = assign_normal_regimes(rows)
        result["normal_distortion"] = {}
        for method in METHODS + ITERATION_METHODS + C_METHODS:
            result["normal_distortion"][method] = {}
            for regime in (
                "all", "weak_geometry_decile", "stop_low_speed_decile",
                "acceleration_top_decile", "turn_top_decile",
                "start_stop_top_decile", "max_iter_healthy",
            ):
                subset = [row for row in rows if regime in row["regimes"]]
                result["normal_distortion"][method][regime] = {
                    "row_count": len(subset),
                    "blocks": block_distortion(subset, method),
                }
    elif role == "failure":
        result["windows"] = {}
        for window in entry["analysis_windows"]:
            subset = [row for row in rows if window["start_s"] <= row["relative_time_s"] <= window["end_s"]]
            raw = cumulative_velocity(subset)
            methods = {}
            for method in METHODS + ITERATION_METHODS + C_METHODS:
                candidate = cumulative_velocity(subset, method)
                candidate["cumulative_suppression_fraction"] = (
                    1.0 - candidate["norm"] / raw["norm"] if raw["norm"] > 1e-15 else None)
                candidate["peak_suppression_fraction"] = (
                    1.0 - candidate["peak_frame_norm"] / raw["peak_frame_norm"]
                    if raw["peak_frame_norm"] > 1e-15 else None)
                methods[method] = candidate
            result["windows"][window["name"]] = {
                "row_count": len(subset), "raw": raw, "methods": methods,
                "dominant_pose_information_directions": direction_summary(subset),
                "max_iter_healthy_count": sum(
                    row["convergence_status"] == "max_iter_healthy" for row in subset),
            }
    return result


def flatten_pareto(summary):
    failure = next(item for item in summary["datasets"] if item["role"] == "failure")
    focus = failure["windows"]["focus_900_906"]
    normals = [item for item in summary["datasets"] if item["role"] == "normal"]
    rows = []
    for method in METHODS + ITERATION_METHODS + C_METHODS:
        normal_velocity_p95 = [
            item["normal_distortion"][method]["all"]["blocks"]["velocity"]["p95"]
            for item in normals
        ]
        rows.append({
            "method": method,
            "failure_cumulative_velocity_suppression":
                focus["methods"][method]["cumulative_suppression_fraction"],
            "failure_peak_velocity_suppression":
                focus["methods"][method]["peak_suppression_fraction"],
            "worst_normal_velocity_distortion_p95": max(normal_velocity_p95),
            "new_parameters": ("1 strength" if method in METHODS else
                               "1 scale" if method in ITERATION_METHODS + C_METHODS else "unknown"),
            "covariance_consistency": (
                "localized Gaussian prior and posterior" if method.startswith("transfer_") else
                "only if damping is treated as stronger physical prior" if method.startswith("damping_") else
                "undefined for mean-only scaling"),
            "implementation_risk": (
                "medium" if method.startswith("transfer_") else
                "high" if method.startswith("damping_") else "high"),
        })
    return rows


def write_report(summary, pareto, output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "source_robustness_summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n")
    with (output_dir / "pareto.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=pareto[0].keys())
        writer.writeheader()
        writer.writerows(pareto)
    failure = next(item for item in summary["datasets"] if item["role"] == "failure")
    focus = failure["windows"]["focus_900_906"]
    lines = [
        "# P2-D ESIKF source robustness shadow evaluation", "",
        "This report uses estimator-internal metrics only. Counterfactuals reuse the",
        "production final linearization and are not trajectory replays.", "",
        "## Focus 900-906 s", "",
        "| method | cumulative dv suppression | peak dv suppression | worst normal velocity p95 distortion |",
        "|---|---:|---:|---:|",
    ]
    for row in pareto:
        lines.append(
            f"| {row['method']} | {row['failure_cumulative_velocity_suppression']:.3%} | "
            f"{row['failure_peak_velocity_suppression']:.3%} | "
            f"{row['worst_normal_velocity_distortion_p95']:.6g} |")
    lines += ["", "## Dominant algebraic pose-information directions", ""]
    for item in focus["dominant_pose_information_directions"]:
        lines.append(
            f"- direction {item['direction']}: cumulative |dv|="
            f"{item['cumulative_velocity_contribution_norm']:.6g}, median rotation/translation "
            f"axis fractions={item['median_rotation_axis_fraction']:.3f}/"
            f"{item['median_translation_axis_fraction']:.3f}")
    (output_dir / "REPORT.md").write_text("\n".join(lines) + "\n")


def self_test():
    assert abs(percentile([0.0, 10.0], 0.5) - 5.0) < 1e-12
    raw = {"raw": [0.0] * 19, "methods": {"x": [0.0] * 19}}
    raw["raw"][7] = 2.0
    raw["methods"]["x"][7] = 1.0
    assert abs(block_distortion([raw], "x")["velocity"]["max"] - 1.0) < 1e-12
    assert abs(cumulative_velocity([raw], "x")["norm"] - 1.0) < 1e-12
    print("evaluate_lio_source_robustness: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.registry or not args.output_dir:
        parser.error("--registry and --output-dir are required unless --self-test is used")
    registry = json.loads(args.registry.read_text())
    datasets = []
    for entry in registry["datasets"]:
        datasets.append(evaluate_dataset(entry, load_dataset(entry)))
    summary = {
        "policy": {
            "external_error_fields_read": False,
            "roles_assigned_by_registry": True,
            "counterfactual_scope": "same final production linearization; not a replayed trajectory",
            "normal_regimes": "within-dataset quantiles of estimator state/geometry only",
        },
        "registry": registry,
        "datasets": datasets,
    }
    pareto = flatten_pareto(summary)
    summary["pareto"] = pareto
    write_report(summary, pareto, args.output_dir)
    print(json.dumps({"datasets": len(datasets), "output_dir": str(args.output_dir)}, indent=2))


if __name__ == "__main__":
    main()
