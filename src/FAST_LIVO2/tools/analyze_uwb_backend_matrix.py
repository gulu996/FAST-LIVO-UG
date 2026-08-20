#!/usr/bin/env python3
"""Summarize a replay matrix manifest without treating UWB as ground truth."""

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from analyze_uwb_backend_run import (
    backend_summary,
    correction_summary,
    load_tum,
    trajectory_summary,
    uwb_summary,
)


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_manifest(path):
    rows = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            dataset, half_map, enabled, allowlist, repeat, label, run = (
                line.rstrip("\n").split("\t"))
            rows.append({
                "dataset": dataset,
                "half_map_size_m": int(half_map),
                "uwb_backend": enabled == "true",
                "allowlist": allowlist,
                "repeat": int(repeat),
                "label": label,
                "run_directory": run,
            })
    return rows


def summarize(row):
    run = Path(row["run_directory"])
    raw_path = run / "livo_raw_online.tum"
    optimized_path = run / "rtk_optimized_final.tum"
    raw = load_tum(raw_path)
    optimized = load_tum(optimized_path) if optimized_path.exists() else raw.copy()
    raw_stats = trajectory_summary(raw)
    optimized_stats = trajectory_summary(optimized)
    correction = correction_summary(raw, optimized)
    uwb = uwb_summary(run / "uwb_backend_measurements.csv")
    backend = backend_summary(run / "rtk_backend_status.csv")
    return {
        **row,
        "optimized_source": (optimized_path.name if optimized_path.exists()
                             else "livo_raw_online.tum (backend disabled)"),
        "raw_sha256": file_sha256(raw_path),
        "raw": raw_stats,
        "optimized": optimized_stats,
        "optimized_vs_raw": correction,
        "uwb": uwb,
        "backend": backend,
    }


def repeatability(rows):
    groups = defaultdict(list)
    for row in rows:
        if row["half_map_size_m"] == 10 and row["allowlist"] in ("[]", "[0,1]"):
            key = (row["dataset"], row["uwb_backend"], row["allowlist"])
            groups[key].append(row)
    result = []
    for (dataset, enabled, allowlist), group in sorted(groups.items()):
        if len(group) < 3:
            continue
        endpoints = np.asarray([item["optimized"]["end_xyz_m"] for item in group])
        factors = [item["backend"].get("total_uwb_factors", 0) for item in group]
        result.append({
            "dataset": dataset,
            "uwb_backend": enabled,
            "allowlist": allowlist,
            "runs": len(group),
            "unique_raw_hashes": len({item["raw_sha256"] for item in group}),
            "max_endpoint_pair_distance_m": float(np.max(
                np.linalg.norm(endpoints[:, None, :] - endpoints[None, :, :], axis=2))),
            "uwb_factor_count_min": min(factors),
            "uwb_factor_count_max": max(factors),
        })
    return result


def flatten(row):
    optimized = row["optimized"]
    correction = row["optimized_vs_raw"]
    backend = row["backend"]
    timing = backend.get("optimization_ms", {})
    decisions = row["uwb"].get("decisions", {})
    return {
        "dataset": row["dataset"],
        "half_map_size_m": row["half_map_size_m"],
        "uwb_backend": row["uwb_backend"],
        "allowlist": row["allowlist"],
        "repeat": row["repeat"],
        "run_directory": row["run_directory"],
        "raw_sha256": row["raw_sha256"],
        "optimized_closure_m": optimized.get("endpoint_displacement_m"),
        "optimized_path_length_m": optimized.get("path_length_m"),
        "optimized_z_span_m": optimized.get("z_span_m"),
        "optimized_max_step_m": optimized.get("max_position_step_m"),
        "optimized_max_angle_step_deg": optimized.get("max_orientation_step_deg"),
        "max_position_correction_m": correction.get("position_correction_m", {}).get("max"),
        "uwb_accepted": decisions.get("ACCEPTED", 0),
        "uwb_rejected": decisions.get("REJECTED", 0),
        "optimization_mean_ms": timing.get("mean"),
        "optimization_p95_ms": timing.get("p95"),
        "optimization_max_ms": timing.get("max"),
        "max_active_states": backend.get("max_active_states"),
        "backend_error": backend.get("backend_error"),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest")
    parser.add_argument("--output-prefix", required=True)
    arguments = parser.parse_args()
    rows = [summarize(row) for row in load_manifest(arguments.manifest)]
    result = {"runs": rows, "repeatability": repeatability(rows)}
    prefix = Path(arguments.output_prefix)
    prefix.with_suffix(".json").write_text(
        json.dumps(result, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    flat = [flatten(row) for row in rows]
    with open(prefix.with_suffix(".csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    print(json.dumps(result["repeatability"], indent=2, ensure_ascii=True))


if __name__ == "__main__":
    main()
