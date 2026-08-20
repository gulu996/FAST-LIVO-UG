#!/usr/bin/env python3
"""Summarize one fixed-lag replay directory as machine-readable JSON."""

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def load_tum(path):
    rows = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip() and not line.lstrip().startswith("#"):
                values = [float(value) for value in line.split()[:8]]
                if len(values) == 8 and np.isfinite(values).all():
                    rows.append(values)
    return np.asarray(rows, dtype=float)


def quaternion_angles(first, second):
    dots = np.abs(np.sum(first * second, axis=1))
    return 2.0 * np.arccos(np.clip(dots, 0.0, 1.0))


def distribution(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {}
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)),
        "max": float(np.max(values)),
    }


def trajectory_summary(trajectory):
    if not len(trajectory):
        return {}
    position_steps = np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1)
    angle_steps = np.degrees(quaternion_angles(
        trajectory[:-1, 4:8], trajectory[1:, 4:8]))
    return {
        "poses": int(len(trajectory)),
        "timestamps_strictly_increasing": bool(
            np.all(np.diff(trajectory[:, 0]) > 0.0)),
        "duration_s": float(trajectory[-1, 0] - trajectory[0, 0]),
        "start_xyz_m": trajectory[0, 1:4].tolist(),
        "end_xyz_m": trajectory[-1, 1:4].tolist(),
        "endpoint_displacement_m": float(np.linalg.norm(
            trajectory[-1, 1:4] - trajectory[0, 1:4])),
        "path_length_m": float(np.sum(position_steps)),
        "max_position_step_m": float(np.max(position_steps, initial=0.0)),
        "max_orientation_step_deg": float(np.max(angle_steps, initial=0.0)),
        "z_min_m": float(np.min(trajectory[:, 3])),
        "z_max_m": float(np.max(trajectory[:, 3])),
        "z_span_m": float(np.ptp(trajectory[:, 3])),
    }


def interpolate_raw(raw, times):
    upper = np.clip(np.searchsorted(raw[:, 0], times), 1, len(raw) - 1)
    lower = upper - 1
    alpha = ((times - raw[lower, 0]) /
             (raw[upper, 0] - raw[lower, 0]))
    positions = ((1.0 - alpha)[:, None] * raw[lower, 1:4] +
                 alpha[:, None] * raw[upper, 1:4])
    q0 = raw[lower, 4:8]
    q1 = raw[upper, 4:8].copy()
    q1[np.sum(q0 * q1, axis=1) < 0.0] *= -1.0
    quaternions = (1.0 - alpha)[:, None] * q0 + alpha[:, None] * q1
    quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
    return positions, quaternions


def correction_summary(raw, optimized):
    if len(raw) < 2 or not len(optimized):
        return {}
    valid = ((optimized[:, 0] >= raw[0, 0]) &
             (optimized[:, 0] <= raw[-1, 0]))
    selected = optimized[valid]
    raw_positions, raw_quaternions = interpolate_raw(raw, selected[:, 0])
    position = np.linalg.norm(selected[:, 1:4] - raw_positions, axis=1)
    orientation = np.degrees(quaternion_angles(
        selected[:, 4:8], raw_quaternions))
    return {
        "position_correction_m": distribution(position),
        "orientation_correction_deg": distribution(orientation),
        "final_position_correction_m": float(position[-1]),
        "final_orientation_correction_deg": float(orientation[-1]),
    }


def uwb_summary(path):
    if not path.exists():
        return {}
    decisions = Counter()
    reasons = Counter()
    accepted = defaultdict(list)
    association = []
    with open(path, "r", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            decisions[row["decision"]] += 1
            reasons[row["reason"]] += 1
            if row["decision"] == "ACCEPTED":
                accepted[row["anchor_id"]].append(abs(float(row["residual_prefit_m"])))
                association.append(abs(float(row["association_dt_s"])))
    return {
        "decisions": dict(decisions),
        "reject_reasons": dict(reasons),
        "association_abs_dt_s": distribution(association),
        "accepted_abs_prefit_residual_m": {
            anchor_id: distribution(values)
            for anchor_id, values in sorted(accepted.items())
        },
    }


def backend_summary(path):
    if not path.exists():
        return {}
    optimization = []
    active_states = []
    active_uwb = []
    final = None
    with open(path, "r", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            final = row
            value = float(row["optimization_ms"])
            if value > 0.0:
                optimization.append(value)
            active_states.append(int(row["active_states"]))
            active_uwb.append(int(row["active_uwb_factors"]))
    if final is None:
        return {}
    return {
        "optimization_ms": distribution(optimization),
        "max_active_states": max(active_states, default=0),
        "max_active_uwb_factors": max(active_uwb, default=0),
        "total_livo_factors": int(final["total_livo_factors"]),
        "total_uwb_factors": int(final["uwb_factors"]),
        "uwb_received": int(final["uwb_received"]),
        "uwb_rejected": int(final["uwb_rejected"]),
        "backend_error": final["backend_error"],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory")
    parser.add_argument("--output-json")
    arguments = parser.parse_args()
    run = Path(arguments.run_directory)
    raw = load_tum(run / "livo_raw_online.tum")
    optimized_path = run / "rtk_optimized_final.tum"
    optimized_source = optimized_path.name
    if optimized_path.exists():
        optimized = load_tum(optimized_path)
    else:
        optimized = raw.copy()
        optimized_source = "livo_raw_online.tum (backend disabled)"
    result = {
        "run_directory": str(run.resolve()),
        "optimized_source": optimized_source,
        "raw": trajectory_summary(raw),
        "optimized": trajectory_summary(optimized),
        "optimized_vs_raw": correction_summary(raw, optimized),
        "uwb": uwb_summary(run / "uwb_backend_measurements.csv"),
        "backend": backend_summary(run / "rtk_backend_status.csv"),
    }
    rendered = json.dumps(result, indent=2, ensure_ascii=True)
    print(rendered)
    if arguments.output_json:
        Path(arguments.output_json).write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
