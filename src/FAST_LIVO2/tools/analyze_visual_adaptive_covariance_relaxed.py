#!/usr/bin/env python3
"""Summarize stage 3C-B relaxed visual covariance experiments."""

import argparse
import json
import math
import statistics
from collections import Counter
from pathlib import Path

from analyze_visual_tracking_dry_run import (
    INTERVALS,
    attach_absolute_times,
    distribution,
    frame_key,
    integer,
    number,
    percent,
    read_csv,
)


BRANCHES = (
    "RELAXED_15_19",
    "RELAXED_20_24",
    "RELAXED_25_29",
    "NORMAL_GE30",
)


def finite_number(row, key):
    try:
        value = float(row.get(key, "nan"))
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def adaptive_scale(ncc, photo_mse, search_level, k_ncc=1.0, k_level=1.0,
                   k_photo=0.5, scale_max=4.0):
    q_ncc = min(1.0, max(0.0, (1.0 - ncc) / 0.60))
    q_photo = min(1.0, max(0.0, photo_mse / 1000.0))
    q_level = min(1.0, max(0.0, search_level / 2.0))
    return min(scale_max, max(1.0,
        1.0 + k_ncc * q_ncc + k_level * q_level + k_photo * q_photo))


def dist(rows, key, multiplier=1.0):
    values = [finite_number(row, key) for row in rows]
    return distribution([value * multiplier for value in values if value is not None])


def summarize_rows(rows, duration):
    attempted = [row for row in rows if integer(row, "ekf_attempted")]
    accepted = [row for row in rows if integer(row, "accepted")]
    whitened = [row for row in rows if integer(row, "whitening_applied")]
    nis_rows = [row for row in attempted if finite_number(row, "normalized_nis") is not None]
    level_totals = [sum(integer(row, f"search_level_{level}") for row in rows)
                    for level in range(3)]
    level_all = sum(level_totals)
    return {
        "frames": len(rows),
        "hz": len(rows) / duration if duration > 0.0 else 0.0,
        "attempted": len(attempted),
        "attempted_hz": len(attempted) / duration if duration > 0.0 else 0.0,
        "accepted": len(accepted),
        "accepted_hz": len(accepted) / duration if duration > 0.0 else 0.0,
        "accepted_percent_of_attempted": percent(len(accepted), len(attempted)),
        "tracked": dist(rows, "tracked_points"),
        "scale": dist(whitened, "scale_mean"),
        "scale_p90_per_frame": dist(whitened, "scale_p90"),
        "scale_max_per_frame": dist(whitened, "scale_max"),
        "ncc_mean_per_frame": dist(rows, "ncc_mean"),
        "photo_mse_mean_per_frame": dist(rows, "photo_mse_mean"),
        "normalized_nis": dist(nis_rows, "normalized_nis"),
        "nis_rejected": sum(not integer(row, "nis_pass") for row in attempted),
        "observability_suppressed": dist(attempted, "observability_suppressed"),
        "observability_rejected": sum(integer(row, "observability_rejected") for row in attempted),
        "guard_rejected": sum(integer(row, "guard_rejected") for row in rows),
        "rollback": sum(integer(row, "rollback") for row in rows),
        "delta_p_m": dist(attempted, "delta_p_norm"),
        "delta_rotation_deg": dist(attempted, "delta_rotation_deg"),
        "processing_ms": dist(rows, "processing_time_s", 1000.0),
        "search_level_percent": [percent(value, level_all) for value in level_totals],
        "decisions": dict(sorted(Counter(row.get("decision", "") for row in rows).items())),
    }


def analyze_run(label, run_dir):
    flow = read_csv(run_dir / "visual_image_flow.csv")
    funnel = read_csv(run_dir / "visual_funnel.csv")
    supply = read_csv(run_dir / "visual_map_supply.csv")
    candidates = read_csv(run_dir / "visual_patch_quality.csv")
    relaxed = read_csv(run_dir / "visual_adaptive_covariance_relaxed.csv")
    pairing = attach_absolute_times(flow, funnel, supply, candidates)

    normal = [row for row in funnel if row.get("frame_mode") == "NORMAL_LIDAR_SUPPORTED"]
    normal_by_key = {frame_key(row): row for row in normal}
    if len(normal_by_key) != len(normal):
        raise RuntimeError("duplicate NORMAL funnel frame key")
    expected_keys = {frame_key(row) for row in normal if integer(row, "tracked_points") >= 15}
    relaxed_by_key = {frame_key(row): row for row in relaxed}
    if len(relaxed_by_key) != len(relaxed):
        raise RuntimeError("duplicate relaxed diagnostic frame key")
    if set(relaxed_by_key) != expected_keys:
        raise RuntimeError(
            f"relaxed/funnel mismatch: missing={list(expected_keys - set(relaxed_by_key))[:5]} "
            f"extra={list(set(relaxed_by_key) - expected_keys)[:5]}"
        )
    for key, row in relaxed_by_key.items():
        funnel_row = normal_by_key[key]
        if integer(row, "tracked_points") != integer(funnel_row, "tracked_points"):
            raise RuntimeError(f"tracked count mismatch at {key}")
        row["absolute_timestamp"] = funnel_row["absolute_timestamp"]

    received = [row for row in flow if row.get("event") == "image_received"]
    full_start = min(number(row, "timestamp") for row in received)
    full_end = max(number(row, "timestamp") for row in received)
    windows = {"full": (full_start, full_end), **INTERVALS}

    regions = {}
    for region, (start, end) in windows.items():
        duration = max(1e-9, end - start)
        region_funnel = [row for row in normal if start <= row["absolute_timestamp"] <= end]
        region_relaxed = [row for row in relaxed if start <= row["absolute_timestamp"] <= end]
        regions[region] = {
            "duration_s": duration,
            "processed_frames": len(region_funnel),
            "processed_hz": len(region_funnel) / duration,
            "attempted_hz_all": sum(integer(row, "ekf_attempted") for row in region_funnel) / duration,
            "accepted_hz_all": sum(integer(row, "accepted") for row in region_funnel) / duration,
            "branches": {
                branch: summarize_rows(
                    [row for row in region_relaxed if row.get("branch") == branch], duration
                )
                for branch in BRANCHES
            },
        }

    memory = read_csv(run_dir / "runtime_memory.csv")
    runtime = {
        "rss_mb": dist(memory, "rss_mb"),
        "visual_processing_ms": dist(supply, "processing_time_s", 1000.0),
    }
    resource_path = run_dir / "process_resource.csv"
    if resource_path.exists():
        resource = read_csv(resource_path)
        runtime["cpu_percent"] = dist(resource, "cpu_percent")

    return {
        "label": label,
        "run_directory": str(run_dir),
        "integrity": {
            "normal_frames": len(normal),
            "relaxed_rows": len(relaxed),
            "relaxed_funnel_pairing": "PASS",
            "timestamp_pairing_error": pairing,
        },
        "runtime": runtime,
        "regions": regions,
    }


def render(report):
    lines = ["# Stage 3C-B visual relaxed update comparison", ""]
    for run in report["runs"]:
        lines.extend([f"## {run['label']}", ""])
        lines.append("| region | attempted Hz | accepted Hz | 15-19 accepted | 20-24 accepted | 25-29 accepted | >=30 accepted |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|")
        for region, item in run["regions"].items():
            branches = item["branches"]
            lines.append(
                f"| {region} | {item['attempted_hz_all']:.3f} | {item['accepted_hz_all']:.3f} | "
                f"{branches['RELAXED_15_19']['accepted']} | {branches['RELAXED_20_24']['accepted']} | "
                f"{branches['RELAXED_25_29']['accepted']} | {branches['NORMAL_GE30']['accepted']} |"
            )
        lines.append("")
        lines.append("| full branch | frames | attempted/accepted | scale mean/P90/max | nNIS median/P95 | dP P95/max m | dR P95/max deg | obs/guard/rollback reject |")
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
        for branch, item in run["regions"]["full"]["branches"].items():
            lines.append(
                f"| {branch} | {item['frames']} | {item['attempted']}/{item['accepted']} | "
                f"{item['scale']['mean']:.3f}/{item['scale_p90_per_frame']['p90']:.3f}/{item['scale_max_per_frame']['max']:.3f} | "
                f"{item['normalized_nis']['median']:.3f}/{item['normalized_nis']['p95']:.3f} | "
                f"{item['delta_p_m']['p95']:.4f}/{item['delta_p_m']['max']:.4f} | "
                f"{item['delta_rotation_deg']['p95']:.4f}/{item['delta_rotation_deg']['max']:.4f} | "
                f"{item['observability_rejected']}/{item['guard_rejected']}/{item['rollback']} |"
            )
        lines.append("")
    return "\n".join(lines)


def self_test():
    assert adaptive_scale(1.0, 0.0, 0) == 1.0
    assert abs(adaptive_scale(0.40, 1000.0, 2) - 3.5) < 1e-12
    assert adaptive_scale(0.40, 1000.0, 2, 2.0, 2.0, 2.0, 4.0) == 4.0
    sample = summarize_rows([
        {"ekf_attempted": "1", "accepted": "1", "whitening_applied": "1",
         "tracked_points": "20", "scale_mean": "2", "normalized_nis": "0.5",
         "nis_pass": "1", "search_level_0": "10", "search_level_1": "10",
         "search_level_2": "0", "delta_p_norm": "0.01", "delta_rotation_deg": "0.1"},
    ], 20.0)
    assert sample["accepted"] == 1 and sample["accepted_hz"] == 0.05
    assert sample["scale"]["mean"] == 2.0
    print("visual adaptive covariance relaxed analyzer self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="*", help="LABEL=/absolute/run/directory")
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.runs:
        parser.error("at least one LABEL=RUN_DIR is required")
    runs = []
    for spec in args.runs:
        if "=" not in spec:
            parser.error(f"invalid run specification: {spec}")
        label, path = spec.split("=", 1)
        runs.append(analyze_run(label, Path(path)))
    report = {"runs": runs}
    markdown = render(report) + "\n"
    if args.json:
        args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    if args.markdown:
        args.markdown.write_text(markdown)
    print(markdown)


if __name__ == "__main__":
    main()
