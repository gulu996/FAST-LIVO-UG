#!/usr/bin/env python3
"""Summarize stage 3C-C1 visual correction and observability diagnostics."""

import argparse
import json
import math
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


BRANCHES = ("NORMAL_GE30", "RELAXED_20_24", "RELAXED_25_29")


def finite(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def values(rows, key, multiplier=1.0):
    return [value * multiplier for row in rows
            if (value := finite(row, key)) is not None]


def dist(rows, key, multiplier=1.0):
    return distribution(values(rows, key, multiplier))


def safe_ratio(numerator, denominator):
    return numerator / denominator if denominator and math.isfinite(denominator) else None


def projector_class(row):
    weights = [
        finite(row, f"{space}_weight_{index}")
        for space in ("rotation", "translation") for index in range(3)
    ]
    if any(weight is None for weight in weights):
        return "unknown"
    if all(abs(weight - 1.0) < 1e-12 for weight in weights):
        return "unchanged"
    if any(abs(weight) < 1e-12 for weight in weights):
        return "full"
    return "partial"


def cumulative(rows):
    accepted = [row for row in rows if integer(row, "accepted")]
    signed_keys = (
        "delta_px", "delta_py", "delta_pz",
        "delta_roll_deg", "delta_pitch_deg", "delta_yaw_deg",
    )
    signed = {key: sum(values(accepted, key)) for key in signed_keys}
    absolute = {key: sum(abs(value) for value in values(accepted, key)) for key in signed_keys}
    dz_abs = absolute["delta_pz"]
    return {
        "accepted": len(accepted),
        "sum": signed,
        "sum_abs": absolute,
        "dz_directional_consistency": safe_ratio(abs(signed["delta_pz"]), dz_abs),
    }


def summarize(rows, duration):
    attempted = [row for row in rows if integer(row, "ekf_attempted")]
    accepted = [row for row in rows if integer(row, "accepted")]
    dz_fraction = []
    rp_magnitude = []
    weak_delta_fraction = []
    weighted_huber_ratio = []
    suppressed_weighted_ratio = []
    translation_eigen_ratio_0 = []
    rotation_eigen_ratio_0 = []
    for row in accepted:
        dp = finite(row, "delta_p_norm")
        dz = finite(row, "delta_pz")
        if dp is not None and dz is not None and dp > 1e-12:
            dz_fraction.append(abs(dz) / dp)
        roll = finite(row, "delta_roll_deg")
        pitch = finite(row, "delta_pitch_deg")
        if roll is not None and pitch is not None:
            rp_magnitude.append(math.hypot(roll, pitch))
        delta_weak = finite(row, "delta_position_weak0")
        if dp is not None and delta_weak is not None and dp > 1e-12:
            weak_delta_fraction.append(abs(delta_weak) / dp)
    for row in attempted:
        huber = finite(row, "huber_pose_information_trace")
        weighted = finite(row, "weighted_pose_information_trace")
        suppressed = finite(row, "suppressed_pose_information_trace")
        ratio = safe_ratio(weighted, huber)
        if ratio is not None:
            weighted_huber_ratio.append(ratio)
        ratio = safe_ratio(suppressed, weighted)
        if ratio is not None:
            suppressed_weighted_ratio.append(ratio)
        t0 = finite(row, "translation_eigenvalue_0")
        t2 = finite(row, "translation_eigenvalue_2")
        r0 = finite(row, "rotation_eigenvalue_0")
        r2 = finite(row, "rotation_eigenvalue_2")
        if t0 is not None and t2 is not None and t2 > 0.0:
            translation_eigen_ratio_0.append(t0 / t2)
        if r0 is not None and r2 is not None and r2 > 0.0:
            rotation_eigen_ratio_0.append(r0 / r2)

    return {
        "rows": len(rows),
        "attempted": len(attempted),
        "accepted": len(accepted),
        "attempted_hz": len(attempted) / duration,
        "accepted_hz": len(accepted) / duration,
        "accepted_percent": percent(len(accepted), len(attempted)),
        "tracked": dist(rows, "tracked_points"),
        "coverage": {
            "occupied_tiles": dist(attempted, "occupied_good_tiles"),
            "horizontal": dist(attempted, "horizontal_coverage"),
            "vertical": dist(attempted, "vertical_coverage"),
        },
        "depth_m": {
            "frame_p10": dist(attempted, "depth_p10"),
            "frame_median": dist(attempted, "depth_p50"),
            "frame_p90": dist(attempted, "depth_p90"),
        },
        "quality": {
            "ncc_mean": dist(attempted, "ncc_mean"),
            "ncc_p10": dist(attempted, "ncc_p10"),
            "photo_mse_mean": dist(attempted, "photo_mse_mean"),
            "photo_mse_p90": dist(attempted, "photo_mse_p90"),
            "scale_mean": dist(attempted, "scale_mean"),
            "scale_p90": dist(attempted, "scale_p90"),
        },
        "normalized_nis": dist(attempted, "normalized_nis"),
        "nis_rejected": sum(not integer(row, "nis_pass") for row in attempted),
        "projector": {
            name: sum(projector_class(row) == name for row in attempted)
            for name in ("unchanged", "partial", "full", "unknown")
        },
        "observability": {
            "suppressed_direction_count": dist(attempted, "observability_suppressed"),
            "rejected": sum(integer(row, "observability_rejected") for row in attempted),
            "huber_information_trace": dist(attempted, "huber_pose_information_trace"),
            "weighted_information_trace": dist(attempted, "weighted_pose_information_trace"),
            "suppressed_information_trace": dist(attempted, "suppressed_pose_information_trace"),
            "weighted_over_huber": distribution(weighted_huber_ratio),
            "suppressed_over_weighted": distribution(suppressed_weighted_ratio),
            "rotation_eigenvalue_0": dist(attempted, "rotation_eigenvalue_0"),
            "rotation_eigenvalue_1": dist(attempted, "rotation_eigenvalue_1"),
            "rotation_eigenvalue_2": dist(attempted, "rotation_eigenvalue_2"),
            "rotation_eigen_ratio_0": distribution(rotation_eigen_ratio_0),
            "rotation_condition": dist(attempted, "rotation_condition"),
            "rotation_weight_0": dist(attempted, "rotation_weight_0"),
            "translation_eigenvalue_0": dist(attempted, "translation_eigenvalue_0"),
            "translation_eigenvalue_1": dist(attempted, "translation_eigenvalue_1"),
            "translation_eigenvalue_2": dist(attempted, "translation_eigenvalue_2"),
            "translation_eigen_ratio_0": distribution(translation_eigen_ratio_0),
            "translation_condition": dist(attempted, "translation_condition"),
            "translation_weight_0": dist(attempted, "translation_weight_0"),
            "translation_weak_vertical_abs": dist(attempted, "translation_weak_vertical_abs"),
        },
        "correction": {
            "delta_p_norm_m": dist(accepted, "delta_p_norm"),
            "abs_dz_over_dp": distribution(dz_fraction),
            "roll_pitch_magnitude_deg": distribution(rp_magnitude),
            "delta_rotation_deg": dist(accepted, "delta_rotation_deg"),
            "abs_weak_translation_over_dp": distribution(weak_delta_fraction),
            "delta_position_weak0_m": dist(accepted, "delta_position_weak0"),
            "delta_rotation_weak0_rad": dist(accepted, "delta_rotation_weak0"),
            "cumulative": cumulative(rows),
        },
        "guard_rejected": sum(integer(row, "guard_rejected") for row in rows),
        "rollback": sum(integer(row, "rollback") for row in rows),
    }


def analyze(run_dir):
    flow = read_csv(run_dir / "visual_image_flow.csv")
    funnel = read_csv(run_dir / "visual_funnel.csv")
    supply = read_csv(run_dir / "visual_map_supply.csv")
    candidates = read_csv(run_dir / "visual_patch_quality.csv")
    diagnostics = read_csv(run_dir / "visual_adaptive_covariance_relaxed.csv")
    pairing = attach_absolute_times(flow, funnel, supply, candidates)

    normal = [row for row in funnel if row.get("frame_mode") == "NORMAL_LIDAR_SUPPORTED"]
    funnel_by_key = {frame_key(row): row for row in normal}
    diagnostics_by_key = {frame_key(row): row for row in diagnostics}
    if len(diagnostics_by_key) != len(diagnostics):
        raise RuntimeError("duplicate directional diagnostic frame key")
    expected_keys = {frame_key(row) for row in normal if integer(row, "tracked_points") >= 15}
    if set(diagnostics_by_key) != expected_keys:
        raise RuntimeError(
            f"diagnostic/funnel mismatch: missing={list(expected_keys - set(diagnostics_by_key))[:5]} "
            f"extra={list(set(diagnostics_by_key) - expected_keys)[:5]}"
        )
    for key, row in diagnostics_by_key.items():
        row["absolute_timestamp"] = funnel_by_key[key]["absolute_timestamp"]

    received = [row for row in flow if row.get("event") == "image_received"]
    full_start = min(number(row, "timestamp") for row in received)
    full_end = max(number(row, "timestamp") for row in received)
    windows = {"full": (full_start, full_end), **INTERVALS}
    regions = {}
    for region, (start, end) in windows.items():
        duration = max(1e-9, end - start)
        region_rows = [row for row in diagnostics if start <= row["absolute_timestamp"] <= end]
        regions[region] = {
            "start": start,
            "end": end,
            "duration_s": duration,
            "branches": {
                branch: summarize(
                    [row for row in region_rows if row.get("branch") == branch], duration
                )
                for branch in BRANCHES
            },
        }
    relaxed_rows = [
        row for row in diagnostics
        if row.get("branch") in ("RELAXED_20_24", "RELAXED_25_29")
    ]
    temporal_windows = {
        "before_B": (full_start, INTERVALS["B_occlusion_denied"][0]),
        "B": INTERVALS["B_occlusion_denied"],
        "B_to_C": (INTERVALS["B_occlusion_denied"][1],
                   INTERVALS["C_denied_half_occlusion"][0]),
        "C": INTERVALS["C_denied_half_occlusion"],
    }
    temporal_relaxed_cumulative = {
        name: cumulative([
            row for row in relaxed_rows if start <= row["absolute_timestamp"] <= end
        ])
        for name, (start, end) in temporal_windows.items()
    }
    thresholds = {
        "relative": sorted(set(values(diagnostics, "observability_relative_threshold"))),
        "absolute": sorted(set(values(diagnostics, "observability_absolute_threshold"))),
    }
    return {
        "run_directory": str(run_dir),
        "integrity": {
            "normal_frames": len(normal),
            "diagnostic_rows": len(diagnostics),
            "pairing": "PASS",
            "timestamp_pairing_error": pairing,
        },
        "observability_thresholds": thresholds,
        "regions": regions,
        "temporal_relaxed_cumulative": temporal_relaxed_cumulative,
    }


def render(report):
    lines = ["# Stage 3C-C1 directional observability report", ""]
    lines.append(
        f"Run: `{report['run_directory']}`; thresholds: "
        f"relative={report['observability_thresholds']['relative']}, "
        f"absolute={report['observability_thresholds']['absolute']}."
    )
    lines.append("")
    for region, region_report in report["regions"].items():
        lines.extend([f"## {region}", ""])
        lines.append(
            "| branch | attempted/accepted Hz | sum dx/dy/dz m | sum abs dx/dy/dz m | "
            "|dz|/|dp| median/P95 | RP median/P95 deg | T eig0/eig2 median | T cond median | "
            "weak vertical median | nNIS median/P95 |"
        )
        lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
        for branch in BRANCHES:
            item = region_report["branches"][branch]
            cumulative_item = item["correction"]["cumulative"]
            signed = cumulative_item["sum"]
            absolute = cumulative_item["sum_abs"]
            dz = item["correction"]["abs_dz_over_dp"]
            rp = item["correction"]["roll_pitch_magnitude_deg"]
            obs = item["observability"]
            nis = item["normalized_nis"]
            eig_ratio = obs["translation_eigen_ratio_0"]
            lines.append(
                f"| {branch} | {item['attempted_hz']:.3f}/{item['accepted_hz']:.3f} | "
                f"{signed['delta_px']:.4f}/{signed['delta_py']:.4f}/{signed['delta_pz']:.4f} | "
                f"{absolute['delta_px']:.4f}/{absolute['delta_py']:.4f}/{absolute['delta_pz']:.4f} | "
                f"{dz['median']:.3f}/{dz['p95']:.3f} | {rp['median']:.4f}/{rp['p95']:.4f} | "
                f"{eig_ratio['median']:.5f} | {obs['translation_condition']['median']:.1f} | "
                f"{obs['translation_weak_vertical_abs']['median']:.3f} | "
                f"{nis['median']:.3f}/{nis['p95']:.3f} |"
            )
        lines.append("")
    lines.extend(["## RELAXED accepted temporal cumulative", ""])
    lines.append("| window | accepted | sum dx/dy/dz m | sum roll/pitch/yaw deg |")
    lines.append("|---|---:|---:|---:|")
    for window, item in report["temporal_relaxed_cumulative"].items():
        signed = item["sum"]
        lines.append(
            f"| {window} | {item['accepted']} | "
            f"{signed['delta_px']:.4f}/{signed['delta_py']:.4f}/{signed['delta_pz']:.4f} | "
            f"{signed['delta_roll_deg']:.4f}/{signed['delta_pitch_deg']:.4f}/"
            f"{signed['delta_yaw_deg']:.4f} |"
        )
    lines.append("")
    return "\n".join(lines)


def self_test():
    rows = [{
        "ekf_attempted": "1", "accepted": "1", "delta_px": "1", "delta_py": "-2",
        "delta_pz": "3", "delta_p_norm": str(math.sqrt(14)), "delta_roll_deg": "0.1",
        "delta_pitch_deg": "0.2", "delta_yaw_deg": "-0.3", "delta_position_weak0": "1",
        "huber_pose_information_trace": "10", "weighted_pose_information_trace": "5",
        "suppressed_pose_information_trace": "4", "translation_eigenvalue_0": "1",
        "translation_eigenvalue_2": "10", "rotation_eigenvalue_0": "2",
        "rotation_eigenvalue_2": "20", "nis_pass": "1",
    }]
    result = summarize(rows, 2.0)
    assert result["accepted_hz"] == 0.5
    assert result["correction"]["cumulative"]["sum"]["delta_pz"] == 3.0
    assert result["observability"]["weighted_over_huber"]["mean"] == 0.5
    assert result["observability"]["suppressed_over_weighted"]["mean"] == 0.8
    print("visual directional observability analyzer self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", nargs="?", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.run_dir is None:
        parser.error("run_dir is required unless --self-test is used")
    report = analyze(args.run_dir)
    markdown = render(report) + "\n"
    if args.json:
        args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    if args.markdown:
        args.markdown.write_text(markdown)
    print(markdown)


if __name__ == "__main__":
    main()
