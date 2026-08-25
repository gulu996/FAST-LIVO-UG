#!/usr/bin/env python3
"""Summarize visual-map supply stages without changing visual decisions."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


INTERVALS = {
    "A_unobstructed": (1785900626.0, 1785900646.0),
    "B_occlusion_denied": (1785900675.0, 1785900695.0),
    "C_denied_half_occlusion": (1785900748.0, 1785900768.0),
}
GLOBAL_STAGES = (
    "map_points_total",
    "global_in_front",
    "global_inside_image",
    "global_border_valid",
    "global_usable_tile",
    "global_grid_cells",
)
SPATIAL_STAGES = (
    "map_points_total",
    "spatial_map_points",
    "spatial_in_front",
    "spatial_inside_image",
    "spatial_border_valid",
    "spatial_usable_tile",
    "spatial_grid_cells",
    "combined_grid_cells",
    "depth_valid",
    "normal_valid",
    "patch_input",
)


def read_csv(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if not reader.fieldnames:
            raise RuntimeError(f"missing CSV header: {path}")
        expected = len(reader.fieldnames)
    with path.open(newline="") as stream:
        raw = list(csv.reader(stream))
    bad = [index + 1 for index, row in enumerate(raw) if len(row) != expected]
    if bad:
        raise RuntimeError(f"CSV column mismatch in {path}: lines {bad[:5]}")
    return rows


def number(row, key):
    try:
        value = float(row.get(key, 0) or 0)
        return value if math.isfinite(value) else 0.0
    except (TypeError, ValueError):
        return 0.0


def percentile(values, quantile):
    values = sorted(values)
    if not values:
        return 0.0
    position = (len(values) - 1) * quantile
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] * (upper - position) + values[upper] * (position - lower)


def distribution(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {key: 0.0 for key in ("mean", "median", "p10", "p90", "p95")}
    return {
        "mean": statistics.fmean(values),
        "median": percentile(values, 0.50),
        "p10": percentile(values, 0.10),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
    }


def stage_summary(rows, stages):
    result = {}
    previous = None
    for stage in stages:
        values = [number(row, stage) for row in rows]
        total = sum(values)
        item = distribution(values)
        item["sum"] = total
        if previous is not None:
            previous_total = result[previous]["sum"]
            item["retained_from_previous_percent"] = (
                100.0 * total / previous_total if previous_total else 0.0
            )
        result[stage] = item
        previous = stage
    return result


def aggregate_logged_distribution(rows, prefix):
    count = sum(number(row, prefix + "_count") for row in rows)
    weighted_mean = sum(
        number(row, prefix + "_count") * number(row, prefix + "_mean") for row in rows
    ) / count if count else 0.0
    return {
        "sample_count": count,
        "weighted_mean": weighted_mean,
        "per_frame_median": distribution([number(row, prefix + "_median") for row in rows]),
        "per_frame_p90": distribution([number(row, prefix + "_p90") for row in rows]),
        "per_frame_p95": distribution([number(row, prefix + "_p95") for row in rows]),
    }


def summarize_region(supply, funnel, start, end):
    rows = [row for row in supply if start <= row["absolute_timestamp"] <= end]
    funnel_rows = [row for row in funnel if start <= row["absolute_timestamp"] <= end]
    duration = max(1e-9, end - start)
    patch_valid = sum(number(row, "patch_valid") for row in funnel_rows)
    ncc_pass = sum(number(row, "ncc_pass") for row in funnel_rows)
    patch_input = sum(number(row, "patch_quality_input") for row in funnel_rows)
    attempted = sum(number(row, "ekf_attempted") for row in funnel_rows)
    accepted = sum(
        number(row, "ekf_attempted") and number(row, "accepted") for row in funnel_rows
    )
    map_total = sum(number(row, "map_points_total") for row in rows)
    projected = sum(number(row, "projected_candidates") for row in funnel_rows)
    return {
        "window": {"start": start, "end": end, "duration_s": duration, "frames": len(rows)},
        "global_map_funnel": stage_summary(rows, GLOBAL_STAGES),
        "actual_spatial_funnel": stage_summary(rows, SPATIAL_STAGES),
        "projected_over_map_percent": 100.0 * projected / map_total if map_total else 0.0,
        "patch_input_ge_30_percent": (
            100.0 * sum(number(row, "patch_input") >= 30 for row in rows) / len(rows) if rows else 0.0
        ),
        "tracked_points": distribution([number(row, "tracked_points") for row in funnel_rows]),
        "visual_attempted_hz": attempted / duration,
        "visual_accepted_hz": accepted / duration,
        "candidate_reject_percent": {
            "patch_quality": 100.0 * sum(number(row, "patch_quality_rejected") for row in funnel_rows) / patch_input if patch_input else 0.0,
            "ncc": 100.0 * sum(number(row, "ncc_rejected") for row in funnel_rows) / patch_valid if patch_valid else 0.0,
            "photometric_after_ncc": 100.0 * sum(number(row, "photometric_rejected") for row in funnel_rows) / ncc_pass if ncc_pass else 0.0,
        },
        "map_point_properties": {
            prefix: aggregate_logged_distribution(rows, prefix)
            for prefix in (
                "global_all_distance_m", "global_view_distance_m",
                "spatial_all_distance_m", "spatial_view_distance_m",
                "global_view_abs_cos", "spatial_view_abs_cos",
                "global_view_reference_age_frames", "spatial_view_reference_age_frames",
                "global_view_last_seen_age_frames", "spatial_view_last_seen_age_frames",
                "global_view_observation_count", "spatial_view_observation_count",
            )
        },
        "maintenance": {
            key: distribution([number(row, key) for row in rows])
            for key in (
                "map_pg_size", "map_candidate_slots", "map_patch_rejects", "map_added_points",
                "map_evicted_points", "map_evicted_voxels", "map_insert_reject_invalid",
                "map_insert_reject_voxel_full", "map_insert_reject_voxel_cap",
                "fallback_added_grid_cells",
            )
        },
        "processing_time_s": distribution([number(row, "processing_time_s") for row in rows]),
    }


def analyze(run_dir):
    supply = read_csv(run_dir / "visual_map_supply.csv")
    funnel = read_csv(run_dir / "visual_funnel.csv")
    flow = read_csv(run_dir / "visual_image_flow.csv")
    processed = [row for row in flow if row.get("event") == "image_processed"]
    if len(processed) != len(funnel) or len(supply) != len(funnel):
        raise RuntimeError(
            f"processed/supply/funnel mismatch: {len(processed)}/{len(supply)}/{len(funnel)}"
        )
    for processed_row, supply_row, funnel_row in zip(processed, supply, funnel):
        absolute = number(processed_row, "timestamp")
        supply_row["absolute_timestamp"] = absolute
        funnel_row["absolute_timestamp"] = absolute
        if abs(number(supply_row, "timestamp") - number(funnel_row, "timestamp")) > 1e-9:
            raise RuntimeError("visual_map_supply/visual_funnel timestamp mismatch")
    times = [number(row, "timestamp") for row in flow]
    windows = {"full": (min(times), max(times)), **INTERVALS}
    return {
        "run_directory": str(run_dir),
        "csv_integrity": {
            "processed_frames": len(processed),
            "supply_rows": len(supply),
            "funnel_rows": len(funnel),
        },
        "regions": {
            name: summarize_region(supply, funnel, start, end)
            for name, (start, end) in windows.items()
        },
    }


def render_markdown(report):
    lines = ["# Visual map supply diagnostics", ""]
    for name, region in report["regions"].items():
        lines += [f"## {name}", ""]
        lines.append(
            f"frames={region['window']['frames']}, projected/map={region['projected_over_map_percent']:.3f}%, "
            f"patch>=30={region['patch_input_ge_30_percent']:.2f}%, "
            f"attempted={region['visual_attempted_hz']:.3f} Hz, accepted={region['visual_accepted_hz']:.3f} Hz"
        )
        lines += ["", "| actual stage | mean | median | P10 | P90 | P95 | retained |", "|---|---:|---:|---:|---:|---:|---:|"]
        for stage, values in region["actual_spatial_funnel"].items():
            retained = values.get("retained_from_previous_percent")
            lines.append(
                f"| {stage} | {values['mean']:.2f} | {values['median']:.2f} | {values['p10']:.2f} | "
                f"{values['p90']:.2f} | {values['p95']:.2f} | "
                f"{'' if retained is None else f'{retained:.2f}%'} |"
            )
        lines += ["", "| global FOV stage | mean | median | P10 | P90 | P95 | retained |", "|---|---:|---:|---:|---:|---:|---:|"]
        for stage, values in region["global_map_funnel"].items():
            retained = values.get("retained_from_previous_percent")
            lines.append(
                f"| {stage} | {values['mean']:.2f} | {values['median']:.2f} | {values['p10']:.2f} | "
                f"{values['p90']:.2f} | {values['p95']:.2f} | "
                f"{'' if retained is None else f'{retained:.2f}%'} |"
            )
        lines.append("")
    return "\n".join(lines)


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    rows = [{"a": "10", "b": "5"}, {"a": "20", "b": "10"}]
    summary = stage_summary(rows, ("a", "b"))
    assert summary["a"]["median"] == 15.0
    assert summary["b"]["retained_from_previous_percent"] == 50.0
    print("visual map supply analyzer self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.run_dir is None:
        parser.error("run_dir is required unless --self-test is used")
    report = analyze(args.run_dir)
    json_path = args.run_dir / "visual_map_supply_summary.json"
    markdown_path = args.run_dir / "visual_map_supply_summary.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    markdown_path.write_text(render_markdown(report) + "\n")
    print(markdown_path)


if __name__ == "__main__":
    main()
