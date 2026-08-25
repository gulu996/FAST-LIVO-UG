#!/usr/bin/env python3
"""Analyze candidate-level NCC, patch, warp, and photometric diagnostics."""

import argparse
import bisect
import csv
import json
import math
import statistics
import tempfile
from collections import Counter
from pathlib import Path


INTERVALS = {
    "A_unobstructed": (1785900626.0, 1785900646.0),
    "B_occlusion_denied": (1785900675.0, 1785900695.0),
    "C_denied_half_occlusion": (1785900748.0, 1785900768.0),
}
NCC_DECISIONS = {"ncc_reject", "photometric_reject", "accepted_preliminary"}


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def finite(row, key, default=math.nan):
    try:
        value = float(row.get(key, default))
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def integer(row, key):
    try:
        return int(float(row.get(key, 0) or 0))
    except ValueError:
        return 0


def percentile(values, q):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return 0.0
    position = (len(values) - 1) * q
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] * (upper - position) + values[upper] * (position - lower)


def distribution(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {key: 0.0 for key in ("mean", "p10", "p25", "median", "p75", "p90", "p95", "max")}
    return {
        "mean": statistics.fmean(values),
        "p10": percentile(values, 0.10),
        "p25": percentile(values, 0.25),
        "median": percentile(values, 0.50),
        "p75": percentile(values, 0.75),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def candidate_diagnostics(rows):
    ref_age = [finite(row, "ref_age") for row in rows]
    ref_time_difference = [finite(row, "reference_time_difference_s") for row in rows]
    warp_abs = [abs(finite(row, "warp_determinant")) for row in rows]
    exposure_ratio = []
    for row in rows:
        ref = finite(row, "ref_inv_exposure")
        current = finite(row, "current_inv_exposure")
        if math.isfinite(ref) and math.isfinite(current) and abs(current) > 1e-12:
            exposure_ratio.append(ref / current)
    return {
        "ref_age": distribution(ref_age),
        "reference_time_difference_s": distribution(ref_time_difference),
        "abs_warp_determinant": distribution(warp_abs),
        "exposure_ratio_ref_over_current": distribution(exposure_ratio),
        "search_level_counts": dict(sorted(Counter(integer(row, "search_level") for row in rows).items())),
        "warp_invalid": sum(not integer(row, "warp_valid") for row in rows if "warp_valid" in row),
    }


def summarize_region(flow_rows, funnel_rows, candidates, start, end, ncc_threshold,
                     photometric_threshold):
    duration = max(1e-9, end - start)
    flow = [row for row in flow_rows if start <= finite(row, "timestamp") <= end]
    funnel = [row for row in funnel_rows if start <= row["absolute_timestamp"] <= end]
    candidate_rows = [row for row in candidates if start <= row["absolute_timestamp"] <= end]
    ncc_all = [row for row in candidate_rows if row.get("decision") in NCC_DECISIONS]
    ncc_reject = [row for row in ncc_all if row.get("decision") == "ncc_reject"]
    ncc_pass = [row for row in ncc_all if row.get("decision") != "ncc_reject"]
    photo_reject = [row for row in ncc_pass if row.get("decision") == "photometric_reject"]
    photo_pass = [row for row in ncc_pass if row.get("decision") == "accepted_preliminary"]
    patch_reject = [row for row in candidate_rows if row.get("decision", "").startswith("patch_")]

    def ncc_values(rows):
        return [finite(row, "ncc") for row in rows]

    reject_margins = [value - ncc_threshold for value in ncc_values(ncc_reject)]
    processed = sum(row.get("event") == "image_processed" for row in flow)
    attempted = sum(integer(row, "ekf_attempted") for row in funnel)
    accepted = sum(integer(row, "ekf_attempted") and integer(row, "accepted") for row in funnel)
    tracked = [finite(row, "tracked_points", 0.0) for row in funnel]
    projected = [finite(row, "projected_candidates", 0.0) for row in funnel]

    geometry_keys = (
        "projected_candidates", "inside_image_candidates", "good_tile_candidates", "grid_candidates",
        "depth_discontinuity_rejected", "normal_uninitialized_rejected", "warp_invalid_candidates",
        "patch_quality_input", "patch_quality_rejected", "patch_valid",
    )
    geometry = {key: sum(integer(row, key) for row in funnel) for key in geometry_keys}

    return {
        "window": {"start": start, "end": end, "duration_s": duration},
        "tracking": {
            "image_received": sum(row.get("event") == "image_received" for row in flow),
            "image_synced": sum(row.get("event") == "image_synced" for row in flow),
            "image_processed": processed,
            "tracked_points": distribution(tracked),
            "projected_candidates": distribution(projected),
            "visual_attempted": attempted,
            "visual_attempted_hz": attempted / duration,
            "visual_accepted": accepted,
            "visual_accepted_hz": accepted / duration,
        },
        "ncc": {
            "threshold": ncc_threshold,
            "all_count": len(ncc_all),
            "pass_count": len(ncc_pass),
            "reject_count": len(ncc_reject),
            "reject_ratio_percent": 100.0 * len(ncc_reject) / len(ncc_all) if ncc_all else 0.0,
            "all": distribution(ncc_values(ncc_all)),
            "pass": distribution(ncc_values(ncc_pass)),
            "reject": distribution(ncc_values(ncc_reject)),
            "margin_all": distribution([value - ncc_threshold for value in ncc_values(ncc_all)]),
            "margin_pass": distribution([value - ncc_threshold for value in ncc_values(ncc_pass)]),
            "margin_reject": distribution(reject_margins),
            "reject_within_0_05_percent": 100.0 * sum(value >= -0.05 for value in reject_margins) / len(reject_margins) if reject_margins else 0.0,
            "reject_within_0_10_percent": 100.0 * sum(value >= -0.10 for value in reject_margins) / len(reject_margins) if reject_margins else 0.0,
            "reject_below_0_20_margin_percent": 100.0 * sum(value < -0.20 for value in reject_margins) / len(reject_margins) if reject_margins else 0.0,
            "reject_nonpositive_ncc_percent": 100.0 * sum(finite(row, "ncc") <= 0.0 for row in ncc_reject) / len(ncc_reject) if ncc_reject else 0.0,
            "pass_diagnostics": candidate_diagnostics(ncc_pass),
            "reject_diagnostics": candidate_diagnostics(ncc_reject),
        },
        "patch_quality": {
            "reject_count": len(patch_reject),
            "reason_counts": dict(sorted(Counter(row.get("decision", "unknown") for row in patch_reject).items())),
            "reason_percent": {
                key: 100.0 * value / len(patch_reject) if patch_reject else 0.0
                for key, value in sorted(Counter(row.get("decision", "unknown") for row in patch_reject).items())
            },
            "diagnostics": candidate_diagnostics(patch_reject),
            "diagnostics_by_reason": {
                reason: candidate_diagnostics([
                    row for row in patch_reject if row.get("decision", "unknown") == reason
                ])
                for reason in sorted(set(row.get("decision", "unknown") for row in patch_reject))
            },
        },
        "photometric": {
            "threshold": photometric_threshold,
            "ncc_pass_count": len(ncc_pass),
            "pass_count": len(photo_pass),
            "reject_count": len(photo_reject),
            "reject_ratio_percent": 100.0 * len(photo_reject) / len(ncc_pass) if ncc_pass else 0.0,
            "all_ncc_pass": distribution([finite(row, "photometric_mse") for row in ncc_pass]),
            "pass": distribution([finite(row, "photometric_mse") for row in photo_pass]),
            "reject": distribution([finite(row, "photometric_mse") for row in photo_reject]),
            "ncc_rejected_candidate_mse": distribution([finite(row, "photometric_mse") for row in ncc_reject]),
        },
        "geometry": geometry,
    }


def analyze(run_dir, ncc_threshold, photometric_threshold):
    flow_rows = read_csv(run_dir / "visual_image_flow.csv")
    funnel_rows = read_csv(run_dir / "visual_funnel.csv")
    candidates = read_csv(run_dir / "visual_patch_quality.csv")
    processed = [row for row in flow_rows if row.get("event") == "image_processed"]
    if len(processed) != len(funnel_rows):
        raise RuntimeError(f"image_processed/funnel mismatch: {len(processed)} != {len(funnel_rows)}")
    origins = []
    for event, funnel in zip(processed, funnel_rows):
        absolute = finite(event, "timestamp")
        relative = finite(funnel, "timestamp")
        funnel["absolute_timestamp"] = absolute
        origins.append(absolute - relative)
    origin = statistics.median(origins)
    origin_errors = [value - origin for value in origins]
    absolute_origin_errors = [abs(value) for value in origin_errors]
    max_origin_deviation = max(absolute_origin_errors)
    # Absolute timestamps were historically logged with 12 significant digits,
    # which quantizes Unix-epoch values at roughly the millisecond level.
    if max_origin_deviation > 1e-2:
        raise RuntimeError(f"visual timestamp origin is not constant: max deviation {max_origin_deviation}")
    for row in candidates:
        row["absolute_timestamp"] = finite(row, "timestamp") + origin

    funnel_times = [finite(row, "timestamp") for row in funnel_rows]
    candidate_frame_pairing_errors = []
    candidate_reference_time_unavailable = 0
    for row in candidates:
        candidate_time = finite(row, "timestamp")
        index = bisect.bisect_left(funnel_times, candidate_time)
        choices = [candidate_index for candidate_index in (index - 1, index)
                   if 0 <= candidate_index < len(funnel_times)]
        if not choices:
            candidate_reference_time_unavailable += 1
            continue
        current_index = min(choices,
                            key=lambda candidate_index: abs(funnel_times[candidate_index] - candidate_time))
        pairing_error = abs(funnel_times[current_index] - candidate_time)
        candidate_frame_pairing_errors.append(pairing_error)
        ref_index = current_index - integer(row, "ref_age")
        if pairing_error > 1e-6 or ref_index < 0:
            candidate_reference_time_unavailable += 1
            continue
        row["reference_time_difference_s"] = (
            funnel_rows[current_index]["absolute_timestamp"] -
            funnel_rows[ref_index]["absolute_timestamp"]
        )

    all_times = [finite(row, "timestamp") for row in flow_rows]
    windows = {"full": (min(all_times), max(all_times)), **INTERVALS}
    return {
        "run_directory": str(run_dir),
        "visual_time_origin": origin,
        "pairing_tolerance_s": 1e-2,
        "pairing_error_signed_s": distribution(origin_errors),
        "pairing_error_absolute_s": distribution(absolute_origin_errors),
        "max_origin_deviation_s": max_origin_deviation,
        "candidate_frame_pairing_error_absolute_s": distribution(candidate_frame_pairing_errors),
        "candidate_reference_time_unavailable": candidate_reference_time_unavailable,
        "regions": {
            name: summarize_region(flow_rows, funnel_rows, candidates, start, end,
                                   ncc_threshold, photometric_threshold)
            for name, (start, end) in windows.items()
        },
    }


def fmt_dist(values):
    return " | ".join(f"{values[key]:.3f}" for key in ("mean", "p10", "p25", "median", "p75", "p90", "p95"))


def markdown(summary):
    regions = summary["regions"]
    lines = [
        "# Visual tracking diagnostic summary", "",
        "## Timestamp pairing", "",
        f"Tolerance: {summary['pairing_tolerance_s']:.6f} s; visual-time origin: "
        f"{summary['visual_time_origin']:.9f} s.", "",
        "| Error | Mean | Median | P95 | Max |", "|---|---:|---:|---:|---:|",
        f"| Absolute pairing error (s) | {summary['pairing_error_absolute_s']['mean']:.9f} | "
        f"{summary['pairing_error_absolute_s']['median']:.9f} | "
        f"{summary['pairing_error_absolute_s']['p95']:.9f} | "
        f"{summary['max_origin_deviation_s']:.9f} |",
        f"| Candidate-to-current-frame pairing error (s) | "
        f"{summary['candidate_frame_pairing_error_absolute_s']['mean']:.9f} | "
        f"{summary['candidate_frame_pairing_error_absolute_s']['median']:.9f} | "
        f"{summary['candidate_frame_pairing_error_absolute_s']['p95']:.9f} | "
        f"{summary['candidate_frame_pairing_error_absolute_s']['max']:.9f} |", "",
        f"Candidate rows without a reconstructable reference timestamp: "
        f"{summary['candidate_reference_time_unavailable']}.", "",
        "## NCC distributions", "",
        "| Region / set | Mean | P10 | P25 | Median | P75 | P90 | P95 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, region in regions.items():
        for subset in ("all", "pass", "reject", "margin_reject"):
            lines.append(f"| {name} / {subset} | {fmt_dist(region['ncc'][subset])} |")
    lines += ["", "## NCC rejection shape", "",
              "| Region | Reject ratio | Within 0.05 | Within 0.10 | Margin < -0.20 | NCC <= 0 |",
              "|---|---:|---:|---:|---:|---:|"]
    for name, region in regions.items():
        ncc = region["ncc"]
        lines.append(
            f"| {name} | {ncc['reject_ratio_percent']:.1f}% | {ncc['reject_within_0_05_percent']:.1f}% | "
            f"{ncc['reject_within_0_10_percent']:.1f}% | {ncc['reject_below_0_20_margin_percent']:.1f}% | "
            f"{ncc['reject_nonpositive_ncc_percent']:.1f}% |"
        )
    lines += ["", "## Patch quality rejection reasons", "",
              "| Region | Count | Reasons |", "|---|---:|---|"]
    for name, region in regions.items():
        patch = region["patch_quality"]
        reasons = ", ".join(
            f"{key}={value} ({patch['reason_percent'][key]:.1f}%)"
            for key, value in patch["reason_counts"].items()
        ) or "not logged"
        lines.append(f"| {name} | {patch['reject_count']} | {reasons} |")
    lines += ["", "| Region / patch reason | Ref age median | Ref dt median (s) | "
              "abs(warp det) median | Exposure ratio median |",
              "|---|---:|---:|---:|---:|"]
    for name, region in regions.items():
        for reason, diagnostics in region["patch_quality"]["diagnostics_by_reason"].items():
            lines.append(
                f"| {name} / {reason} | {diagnostics['ref_age']['median']:.2f} | "
                f"{diagnostics['reference_time_difference_s']['median']:.3f} | "
                f"{diagnostics['abs_warp_determinant']['median']:.3f} | "
                f"{diagnostics['exposure_ratio_ref_over_current']['median']:.3f} |"
            )
    lines += ["", "## Candidate geometry and exposure", "",
              "| Region / NCC set / metric | Mean | P10 | P25 | Median | P75 | P90 | P95 |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for name, region in regions.items():
        for subset in ("pass_diagnostics", "reject_diagnostics"):
            diagnostics = region["ncc"][subset]
            label = subset[:-len("_diagnostics")]
            for key in ("ref_age", "reference_time_difference_s", "abs_warp_determinant",
                        "exposure_ratio_ref_over_current"):
                lines.append(f"| {name} / {label} / {key} | {fmt_dist(diagnostics[key])} |")
    lines += ["", "## Photometric MSE", "",
              "| Region / set | Mean | P10 | P25 | Median | P75 | P90 | P95 |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for name, region in regions.items():
        for subset in ("all_ncc_pass", "pass", "reject"):
            lines.append(f"| {name} / {subset} | {fmt_dist(region['photometric'][subset])} |")
    lines += ["", "## Sweep metrics", "",
              "| Region | Tracked median/P90/P95 | NCC reject | Photometric reject | Attempted Hz | Accepted Hz |",
              "|---|---:|---:|---:|---:|---:|"]
    for name, region in regions.items():
        tracked = region["tracking"]["tracked_points"]
        lines.append(
            f"| {name} | {tracked['median']:.2f}/{tracked['p90']:.2f}/{tracked['p95']:.2f} | "
            f"{region['ncc']['reject_ratio_percent']:.1f}% | {region['photometric']['reject_ratio_percent']:.1f}% | "
            f"{region['tracking']['visual_attempted_hz']:.3f} | {region['tracking']['visual_accepted_hz']:.3f} |"
        )
    lines.append("")
    return "\n".join(lines)


def self_test():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "visual_image_flow.csv").write_text(
            "timestamp,event,detail\n100,image_received,x\n100,image_synced,x\n100,image_processed,x\n"
        )
        (root / "visual_funnel.csv").write_text(
            "timestamp,tracked_points,projected_candidates,ekf_attempted,accepted\n0,10,20,1,1\n"
        )
        (root / "visual_patch_quality.csv").write_text(
            "timestamp,photometric_mse,ncc,decision,ref_age,search_level,warp_determinant,warp_valid,ref_inv_exposure,current_inv_exposure\n"
            "0,10,0.39,ncc_reject,2,0,1,1,1,1\n0,20,0.8,accepted_preliminary,3,1,2,1,1,1\n"
            "0,nan,nan,patch_current_saturation,4,0,1,1,1,1\n"
        )
        result = analyze(root, 0.4, 1000.0)
        full = result["regions"]["full"]
        assert full["ncc"]["reject_count"] == 1
        assert abs(full["ncc"]["margin_reject"]["median"] + 0.01) < 1e-9
        assert full["patch_quality"]["reason_counts"]["patch_current_saturation"] == 1
        assert result["pairing_error_absolute_s"]["p95"] == 0.0
        assert "NCC distributions" in markdown(result)
    print("visual tracking diagnostics self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--ncc-threshold", type=float, default=0.4)
    parser.add_argument("--photometric-threshold", type=float, default=1000.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.run_dir is None:
        parser.error("--run-dir is required unless --self-test is used")
    summary = analyze(args.run_dir, args.ncc_threshold, args.photometric_threshold)
    json_path = args.run_dir / "visual_tracking_diagnostic_summary.json"
    md_path = args.run_dir / "visual_tracking_diagnostic_summary.md"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    md_path.write_text(markdown(summary))
    print(md_path)


if __name__ == "__main__":
    main()
