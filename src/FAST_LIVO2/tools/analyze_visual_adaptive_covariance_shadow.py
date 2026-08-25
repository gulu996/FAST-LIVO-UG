#!/usr/bin/env python3
"""Summarize read-only visual EKF shadow measurements by tracked-point bin."""

import argparse
import json
import math
import statistics
from collections import Counter, defaultdict
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


BINS = (
    ("lt15", 0, 14),
    ("15_19", 15, 19),
    ("20_24", 20, 24),
    ("25_29", 25, 29),
    ("ge30", 30, math.inf),
)


def tracked_bin(tracked):
    for name, lower, upper in BINS:
        if lower <= tracked <= upper:
            return name
    raise RuntimeError(f"invalid tracked-point count: {tracked}")


def candidate_distribution(rows, column):
    return distribution([number(row, column, math.nan) for row in rows])


def summarize_bin(frames, shadows, candidates, duration):
    tracked_candidates = [
        row for row in candidates if row.get("decision") == "accepted_preliminary"
    ]
    search_levels = Counter(integer(row, "search_level") for row in tracked_candidates)
    sources = Counter(row.get("source", "unknown") for row in tracked_candidates)
    valid_shadows = [row for row in shadows if integer(row, "valid")]
    evaluated_shadows = [row for row in shadows if integer(row, "evaluated")]
    spatial_geometry_pass = sum(
        integer(row, "occupied_good_tiles") >= 8
        and number(row, "good_tile_ratio") >= 0.85
        and number(row, "horizontal_coverage") >= 0.50
        and number(row, "vertical_coverage") >= 0.50
        for row in frames
    )
    return {
        "frames": len(frames),
        "hz": len(frames) / duration if duration > 0.0 else 0.0,
        "tracked": distribution([number(row, "tracked_points") for row in frames]),
        "occupied_good_tiles": distribution(
            [number(row, "occupied_good_tiles") for row in frames]
        ),
        "occupied_tile_ratio": distribution(
            [number(row, "occupied_tile_ratio") for row in frames]
        ),
        "horizontal_coverage": distribution(
            [number(row, "horizontal_coverage") for row in frames]
        ),
        "vertical_coverage": distribution(
            [number(row, "vertical_coverage") for row in frames]
        ),
        "existing_spatial_geometry_pass_percent": percent(
            spatial_geometry_pass, len(frames)
        ),
        "tracked_observations": len(tracked_candidates),
        "ncc": candidate_distribution(tracked_candidates, "ncc"),
        "photometric_mse": candidate_distribution(
            tracked_candidates, "photometric_mse"
        ),
        "warp_frobenius": candidate_distribution(
            tracked_candidates, "warp_frobenius"
        ),
        "warp_condition": candidate_distribution(
            tracked_candidates, "warp_condition"
        ),
        "depth_z": candidate_distribution(tracked_candidates, "depth_z"),
        "search_level_percent": {
            str(level): percent(count, len(tracked_candidates))
            for level, count in sorted(search_levels.items())
        },
        "source_percent": {
            source: percent(count, len(tracked_candidates))
            for source, count in sorted(sources.items())
        },
        "shadow": {
            "evaluated_frames": len(evaluated_shadows),
            "valid_frames": len(valid_shadows),
            "valid_percent": percent(len(valid_shadows), len(evaluated_shadows)),
            "min_measurement_pass_percent": percent(
                sum(integer(row, "min_measurement_pass") for row in evaluated_shadows),
                len(evaluated_shadows),
            ),
            "observability_pass_percent": percent(
                sum(integer(row, "observability_pass") for row in evaluated_shadows),
                len(evaluated_shadows),
            ),
            "nis_pass_percent": percent(
                sum(integer(row, "nis_pass") for row in valid_shadows),
                len(valid_shadows),
            ),
            "measurement_dof": candidate_distribution(
                evaluated_shadows, "measurement_dof"
            ),
            "residual_rms": candidate_distribution(valid_shadows, "residual_rms"),
            "robust_residual_rms": candidate_distribution(
                valid_shadows, "robust_residual_rms"
            ),
            "normalized_nis": candidate_distribution(
                valid_shadows, "normalized_nis"
            ),
            "suppressed_directions": candidate_distribution(
                evaluated_shadows, "observability_suppressed"
            ),
            "rotation_min_eigenvalue": candidate_distribution(
                valid_shadows, "rotation_min_eigenvalue"
            ),
            "rotation_condition": candidate_distribution(
                valid_shadows, "rotation_condition"
            ),
            "translation_min_eigenvalue": candidate_distribution(
                valid_shadows, "translation_min_eigenvalue"
            ),
            "translation_condition": candidate_distribution(
                valid_shadows, "translation_condition"
            ),
        },
    }


def analyze(run_dir):
    flow = read_csv(run_dir / "visual_image_flow.csv")
    funnel = read_csv(run_dir / "visual_funnel.csv")
    supply = read_csv(run_dir / "visual_map_supply.csv")
    candidates = read_csv(run_dir / "visual_patch_quality.csv")
    shadows = read_csv(run_dir / "visual_adaptive_covariance_shadow.csv")
    pairing = attach_absolute_times(flow, funnel, supply, candidates)

    normal_funnel = [
        row for row in funnel if row.get("frame_mode") == "NORMAL_LIDAR_SUPPORTED"
    ]
    funnel_by_key = {frame_key(row): row for row in normal_funnel}
    if len(funnel_by_key) != len(normal_funnel):
        raise RuntimeError("duplicate normal funnel frame key")
    normal_keys = {frame_key(row) for row in normal_funnel}
    shadow_by_key = {}
    for row in shadows:
        key = frame_key(row)
        if key in shadow_by_key:
            raise RuntimeError(f"duplicate shadow frame: {key}")
        shadow_by_key[key] = row
    if set(shadow_by_key) != normal_keys:
        missing = sorted(normal_keys - set(shadow_by_key))[:5]
        extra = sorted(set(shadow_by_key) - normal_keys)[:5]
        raise RuntimeError(f"shadow/funnel key mismatch: missing={missing}, extra={extra}")

    absolute_by_key = {frame_key(row): row["absolute_timestamp"] for row in normal_funnel}
    for row in shadows:
        row["absolute_timestamp"] = absolute_by_key[frame_key(row)]
        funnel_tracked = integer(funnel_by_key[frame_key(row)], "tracked_points")
        if integer(row, "tracked_points") != funnel_tracked:
            raise RuntimeError(
                f"shadow tracked count mismatch at {frame_key(row)}: "
                f"{integer(row, 'tracked_points')} != {funnel_tracked}"
            )

    candidates_by_key = defaultdict(list)
    for row in candidates:
        if row.get("frame_mode") == "NORMAL_LIDAR_SUPPORTED":
            candidates_by_key[frame_key(row)].append(row)

    received = [row for row in flow if row.get("event") == "image_received"]
    if not received or not normal_funnel:
        raise RuntimeError("missing image_received or normal visual frames")
    full_start = min(
        min(number(row, "timestamp") for row in received),
        min(row["absolute_timestamp"] for row in normal_funnel),
    )
    full_end = max(
        max(number(row, "timestamp") for row in received),
        max(row["absolute_timestamp"] for row in normal_funnel),
    )
    windows = {"full": (full_start, full_end), **INTERVALS}

    regions = {}
    for region, (start, end) in windows.items():
        duration = max(1e-9, end - start)
        region_frames = [
            row for row in normal_funnel
            if start <= row["absolute_timestamp"] <= end
        ]
        regions[region] = {}
        for name, lower, upper in BINS:
            frames = [
                row for row in region_frames
                if lower <= integer(row, "tracked_points") <= upper
            ]
            keys = {frame_key(row) for row in frames}
            bin_shadows = [shadow_by_key[key] for key in keys]
            bin_candidates = [
                candidate
                for key in keys
                for candidate in candidates_by_key.get(key, ())
            ]
            regions[region][name] = summarize_bin(
                frames, bin_shadows, bin_candidates, duration
            )

    return {
        "run_directory": str(run_dir),
        "csv_integrity": {
            "normal_frames": len(normal_funnel),
            "shadow_rows": len(shadows),
            "candidate_rows": len(candidates),
            "shadow_funnel_pairing": "PASS",
            "timestamp_pairing_error": pairing,
        },
        "regions": regions,
    }


def compact(item, key):
    values = item[key]
    return f"{values['mean']:.3f}/{values['median']:.3f}/{values['p90']:.3f}"


def render_markdown(report):
    lines = [
        "# Visual adaptive covariance shadow evaluation",
        "",
        "Distributions are mean/median/P90. NCC and photometric values use only "
        "candidates that passed the existing gates and became tracked points.",
        "",
    ]
    for region, bins in report["regions"].items():
        lines += [f"## {region}", ""]
        lines += [
            "| tracked bin | frames | Hz | tracked | tiles | H/V coverage | spatial geometry pass | NCC | photo MSE | warp Frob/cond | depth | search L0/L1/L2 | fallback | shadow valid/obs/NIS pass | shadow nNIS |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for name, item in bins.items():
            search = item["search_level_percent"]
            source = item["source_percent"]
            shadow = item["shadow"]
            lines.append(
                f"| {name} | {item['frames']} | {item['hz']:.3f} | "
                f"{compact(item, 'tracked')} | {compact(item, 'occupied_good_tiles')} | "
                f"{item['horizontal_coverage']['median']:.3f}/{item['vertical_coverage']['median']:.3f} | "
                f"{item['existing_spatial_geometry_pass_percent']:.1f}% | {compact(item, 'ncc')} | "
                f"{compact(item, 'photometric_mse')} | "
                f"{item['warp_frobenius']['median']:.3f}/{item['warp_condition']['median']:.3f} | "
                f"{item['depth_z']['median']:.3f} | "
                f"{search.get('0', 0.0):.1f}/{search.get('1', 0.0):.1f}/{search.get('2', 0.0):.1f}% | "
                f"{source.get('fov_fallback', 0.0):.1f}% | "
                f"{shadow['valid_percent']:.1f}/{shadow['observability_pass_percent']:.1f}/"
                f"{shadow['nis_pass_percent']:.1f}% | {compact(shadow, 'normalized_nis')} |"
            )
        lines.append("")
    return "\n".join(lines)


def self_test():
    assert [tracked_bin(value) for value in (0, 14, 15, 19, 20, 24, 25, 29, 30)] == [
        "lt15", "lt15", "15_19", "15_19", "20_24", "20_24", "25_29", "25_29", "ge30"
    ]
    sample = summarize_bin(
        [{"tracked_points": "20", "occupied_good_tiles": "9", "good_tile_ratio": "0.9",
          "horizontal_coverage": "0.6", "vertical_coverage": "0.7"}],
        [{"evaluated": "1", "valid": "1", "min_measurement_pass": "1",
          "observability_pass": "1", "nis_pass": "1", "normalized_nis": "0.5"}],
        [{"decision": "accepted_preliminary", "ncc": "0.8", "photometric_mse": "10",
          "search_level": "1", "source": "lidar_submap"}],
        20.0,
    )
    assert sample["frames"] == 1 and sample["shadow"]["nis_pass_percent"] == 100.0
    print("visual adaptive covariance shadow analyzer self-test: PASS")


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
    json_path = args.run_dir / "visual_adaptive_covariance_shadow_summary.json"
    markdown_path = args.run_dir / "visual_adaptive_covariance_shadow_summary.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    markdown_path.write_text(render_markdown(report) + "\n")
    print(markdown_path)


if __name__ == "__main__":
    main()
