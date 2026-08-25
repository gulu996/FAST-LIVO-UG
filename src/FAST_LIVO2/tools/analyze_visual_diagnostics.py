#!/usr/bin/env python3
"""Summarize the observed FAST-LIVO2 visual funnel without changing its gates."""

import argparse
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
CANONICAL_REASONS = (
    "IMAGE_QUALITY_REJECT",
    "LOW_PROJECTED_POINTS",
    "LOW_PATCH_CANDIDATES",
    "LOW_TRACKED_POINTS",
    "LOW_SPATIAL_COVERAGE",
    "NCC_REJECT",
    "OBSERVABILITY_REJECT",
    "NIS_REJECT",
    "BACKWARD_GUARD_REJECT",
    "LATERAL_GUARD_REJECT",
)


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def integer(row, key):
    try:
        return int(float(row.get(key, 0) or 0))
    except ValueError:
        return 0


def number(row, key):
    try:
        value = float(row.get(key, 0) or 0)
        return value if math.isfinite(value) else 0.0
    except ValueError:
        return 0.0


def percentile(values, quantile):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def distribution(rows, key):
    values = [number(row, key) for row in rows]
    if not values:
        return {name: 0.0 for name in ("mean", "median", "p10", "p50", "p90", "p95")}
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p10": percentile(values, 0.10),
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
    }


def canonical_reason(row):
    raw = row.get("skip_reason", "none") or "none"
    if not integer(row, "image_quality_pass"):
        return "IMAGE_QUALITY_REJECT"
    if integer(row, "observability_rejected"):
        return "OBSERVABILITY_REJECT"
    if integer(row, "nis_rejected") or raw in ("normalized_nis", "large_visual_normalized_nis"):
        return "NIS_REJECT"
    aliases = {
        "low_projected_points": "LOW_PROJECTED_POINTS",
        "low_patch_candidates": "LOW_PATCH_CANDIDATES",
        "low_tracked_points": "LOW_TRACKED_POINTS",
        "insufficient_tracked_spatial_coverage": "LOW_SPATIAL_COVERAGE",
        "ncc_reject": "NCC_REJECT",
        "backward_step": "BACKWARD_GUARD_REJECT",
        "lateral_step": "LATERAL_GUARD_REJECT",
    }
    return aliases.get(raw, "OTHER:" + raw)


def summarize(flow_rows, funnel_rows, start, end):
    duration = max(1e-9, end - start)
    flow = [row for row in flow_rows if start <= float(row["timestamp"]) <= end]
    funnel = [row for row in funnel_rows if start <= row["absolute_timestamp"] <= end]
    events = Counter(row["event"] for row in flow)
    drops = Counter(row["detail"] for row in flow if row["event"] == "image_dropped")
    processing_rejects = Counter(
        row["detail"] for row in flow if row["event"] == "image_processing_rejected"
    )

    counts = {
        "image_received": events["image_received"],
        "image_buffered": events["image_buffered"],
        "image_synced": events["image_synced"],
        "image_processed": events["image_processed"],
        "image_quality_pass": sum(integer(row, "image_quality_pass") for row in funnel),
        "frames_with_projected_candidates": sum(number(row, "projected_candidates") > 0 for row in funnel),
        "frames_with_patch_candidates": sum(number(row, "patch_quality_input") > 0 for row in funnel),
        "frames_with_valid_patches": sum(number(row, "patch_valid") > 0 for row in funnel),
        "frames_with_tracked_points": sum(number(row, "tracked_points") > 0 for row in funnel),
        "tracked_gate_pass": sum(integer(row, "tracked_gate_pass") for row in funnel),
        "visual_update_attempted": sum(integer(row, "ekf_attempted") for row in funnel),
        "visual_update_accepted": sum(
            integer(row, "accepted") and integer(row, "ekf_attempted") for row in funnel
        ),
    }
    rates = {key: value / duration for key, value in counts.items()}

    rejected = [
        row for row in funnel
        if not (integer(row, "accepted") and integer(row, "ekf_attempted"))
    ]
    canonical = Counter(canonical_reason(row) for row in rejected)
    raw = Counter((row.get("skip_reason", "none") or "none") for row in rejected)
    canonical_counts = {reason: canonical.get(reason, 0) for reason in CANONICAL_REASONS}
    canonical_counts.update({key: value for key, value in canonical.items() if key not in canonical_counts})
    reject_total = len(rejected)

    patch_input = sum(number(row, "patch_quality_input") for row in funnel)
    patch_rejected = sum(number(row, "patch_quality_rejected") for row in funnel)
    patch_valid = sum(number(row, "patch_valid") for row in funnel)
    ncc_rejected = sum(number(row, "ncc_rejected") for row in funnel)
    ncc_pass = sum(number(row, "ncc_pass") for row in funnel)
    photometric_rejected = sum(number(row, "photometric_rejected") for row in funnel)

    return {
        "window": {"start": start, "end": end, "duration_s": duration},
        "counts": counts,
        "rates_hz": rates,
        "distributions": {
            "projected_candidates": distribution(funnel, "projected_candidates"),
            "patch_candidates": distribution(funnel, "patch_quality_input"),
            "tracked_points": distribution(funnel, "tracked_points"),
        },
        "terminal_rejections": {
            "total": reject_total,
            "canonical_counts": canonical_counts,
            "canonical_percent": {
                key: (100.0 * value / reject_total if reject_total else 0.0)
                for key, value in canonical_counts.items()
            },
            "raw_counts": dict(sorted(raw.items())),
        },
        "candidate_rejections": {
            "patch_quality_input": patch_input,
            "patch_quality_rejected": patch_rejected,
            "patch_quality_reject_percent": 100.0 * patch_rejected / patch_input if patch_input else 0.0,
            "patch_valid": patch_valid,
            "ncc_rejected": ncc_rejected,
            "ncc_reject_percent_of_valid_patches": 100.0 * ncc_rejected / patch_valid if patch_valid else 0.0,
            "ncc_pass": ncc_pass,
            "photometric_rejected": photometric_rejected,
            "photometric_reject_percent_of_ncc_pass": 100.0 * photometric_rejected / ncc_pass if ncc_pass else 0.0,
        },
        "image_drop_reasons": dict(sorted(drops.items())),
        "preprocess_reject_reasons": dict(sorted(processing_rejects.items())),
    }


def cell(count, rate):
    return f"{count} / {rate:.3f}"


def percent(numerator, denominator):
    return 100.0 * numerator / denominator if denominator else 0.0


def markdown_report(summary):
    regions = summary["regions"]
    lines = [
        "# Visual diagnostic summary",
        "",
        "Counts/rates use each window's sensor-time duration. Rate cells are `count / Hz`.",
        "",
        "## Visual chain",
        "",
        "| Region | Received | Synced | Processed | Quality pass | Attempted | EKF accepted |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, region in regions.items():
        c, r = region["counts"], region["rates_hz"]
        lines.append(
            f"| {name} | {cell(c['image_received'], r['image_received'])} | "
            f"{cell(c['image_synced'], r['image_synced'])} | {cell(c['image_processed'], r['image_processed'])} | "
            f"{cell(c['image_quality_pass'], r['image_quality_pass'])} | "
            f"{cell(c['visual_update_attempted'], r['visual_update_attempted'])} | "
            f"{cell(c['visual_update_accepted'], r['visual_update_accepted'])} |"
        )

    lines += [
        "",
        "## Sequential pass rates",
        "",
        "| Region | Received->synced | Synced->processed | Processed->quality | Quality->projected | Projected->patch | Patch->valid | Valid->tracked | Tracked->attempted | Attempted->accepted |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, region in regions.items():
        c = region["counts"]
        pairs = (
            (c["image_synced"], c["image_received"]),
            (c["image_processed"], c["image_synced"]),
            (c["image_quality_pass"], c["image_processed"]),
            (c["frames_with_projected_candidates"], c["image_quality_pass"]),
            (c["frames_with_patch_candidates"], c["frames_with_projected_candidates"]),
            (c["frames_with_valid_patches"], c["frames_with_patch_candidates"]),
            (c["frames_with_tracked_points"], c["frames_with_valid_patches"]),
            (c["visual_update_attempted"], c["frames_with_tracked_points"]),
            (c["visual_update_accepted"], c["visual_update_attempted"]),
        )
        lines.append("| " + name + " | " + " | ".join(f"{percent(a, b):.1f}%" for a, b in pairs) + " |")

    lines += [
        "",
        "## Non-zero stage passage",
        "",
        "| Region | Projected > 0 | Patch candidates > 0 | Valid patches > 0 | Tracked > 0 | Tracked gate pass |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, region in regions.items():
        c, r = region["counts"], region["rates_hz"]
        keys = ("frames_with_projected_candidates", "frames_with_patch_candidates",
                "frames_with_valid_patches", "frames_with_tracked_points", "tracked_gate_pass")
        lines.append("| " + name + " | " + " | ".join(cell(c[key], r[key]) for key in keys) + " |")

    lines += [
        "",
        "## Candidate distributions",
        "",
        "| Region / metric | Mean | Median | P10 | P50 | P90 | P95 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, region in regions.items():
        for metric, values in region["distributions"].items():
            lines.append(
                f"| {name} / {metric} | {values['mean']:.2f} | {values['median']:.2f} | "
                f"{values['p10']:.2f} | {values['p50']:.2f} | {values['p90']:.2f} | {values['p95']:.2f} |"
            )

    lines += [
        "",
        "## Terminal rejection attribution",
        "",
        "These are recorded terminal decisions. Candidate-level NCC/patch attrition is reported separately.",
        "",
        "| Reason | " + " | ".join(regions) + " |",
        "|---|" + "---:|" * len(regions),
    ]
    reason_order = list(CANONICAL_REASONS)
    extras = sorted({
        reason for region in regions.values()
        for reason in region["terminal_rejections"]["canonical_counts"]
        if reason not in CANONICAL_REASONS
    })
    for reason in reason_order + extras:
        values = []
        for region in regions.values():
            rejected = region["terminal_rejections"]
            count = rejected["canonical_counts"].get(reason, 0)
            rejection_percent = rejected["canonical_percent"].get(reason, 0.0)
            values.append(f"{count} ({rejection_percent:.1f}%)")
        lines.append("| " + reason + " | " + " | ".join(values) + " |")

    lines += [
        "",
        "## Candidate-level rejection",
        "",
        "| Region | Patch quality rejected | NCC rejected | Photometric rejected |",
        "|---|---:|---:|---:|",
    ]
    for name, region in regions.items():
        c = region["candidate_rejections"]
        lines.append(
            f"| {name} | {int(c['patch_quality_rejected'])} / {c['patch_quality_reject_percent']:.1f}% | "
            f"{int(c['ncc_rejected'])} / {c['ncc_reject_percent_of_valid_patches']:.1f}% | "
            f"{int(c['photometric_rejected'])} / {c['photometric_reject_percent_of_ncc_pass']:.1f}% |"
        )

    lines += [
        "",
        "## Before-process attrition (recorded reasons)",
        "",
        "| Region | Buffer/input drops | Synced but not processed |",
        "|---|---|---|",
    ]
    for name, region in regions.items():
        drops = ", ".join(f"{key}={value}" for key, value in region["image_drop_reasons"].items()) or "none"
        rejects = ", ".join(f"{key}={value}" for key, value in region["preprocess_reject_reasons"].items()) or "none"
        lines.append(f"| {name} | {drops} | {rejects} |")
    lines.append("")
    return "\n".join(lines)


def analyze(run_dir):
    flow_rows = read_rows(run_dir / "visual_image_flow.csv")
    funnel_rows = read_rows(run_dir / "visual_funnel.csv")
    processed = [row for row in flow_rows if row["event"] == "image_processed"]
    if len(processed) != len(funnel_rows):
        raise RuntimeError(
            f"image_processed/funnel mismatch: {len(processed)} != {len(funnel_rows)}"
        )
    for event, funnel in zip(processed, funnel_rows):
        funnel["absolute_timestamp"] = float(event["timestamp"])

    received_times = [
        float(row["timestamp"]) for row in flow_rows if row["event"] == "image_received"
    ]
    if len(received_times) < 2:
        raise RuntimeError("visual_image_flow.csv has fewer than two received images")
    all_flow_times = [float(row["timestamp"]) for row in flow_rows]
    windows = {"full": (min(all_flow_times), max(all_flow_times)), **INTERVALS}
    regions = {
        name: summarize(flow_rows, funnel_rows, start, end)
        for name, (start, end) in windows.items()
    }
    return {
        "run_directory": str(run_dir),
        "pairing": {"image_processed": len(processed), "visual_funnel_rows": len(funnel_rows)},
        "regions": regions,
    }


def self_test():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "visual_image_flow.csv").write_text(
            "timestamp,event,detail\n"
            "100,image_received,subscriber_callback\n100,image_synced,vio_measurement\n"
            "100,image_processed,adaptive_off\n101,image_received,subscriber_callback\n"
            "101,image_synced,vio_measurement\n101,image_processed,adaptive_off\n"
        )
        (root / "visual_funnel.csv").write_text(
            "image_quality_pass,projected_candidates,patch_quality_input,patch_valid,tracked_points,"
            "tracked_gate_pass,ekf_attempted,accepted,nis_rejected,observability_rejected,skip_reason,"
            "patch_quality_rejected,ncc_rejected,ncc_pass,photometric_rejected\n"
            "1,20,10,8,6,1,1,1,0,0,none,2,1,7,1\n"
            "1,4,2,1,1,0,0,0,0,0,low_tracked_points,1,0,1,0\n"
        )
        summary = analyze(root)
        result = summary["regions"]["full"]
        assert result["counts"]["image_received"] == 2
        assert result["counts"]["visual_update_accepted"] == 1
        assert result["terminal_rejections"]["canonical_counts"]["LOW_TRACKED_POINTS"] == 1
        assert result["distributions"]["tracked_points"]["median"] == 3.5
        assert "Sequential pass rates" in markdown_report(summary)
    print("visual diagnostics self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.run_dir is None:
        parser.error("--run-dir is required unless --self-test is used")
    summary = analyze(args.run_dir)
    json_path = args.run_dir / "visual_diagnostic_summary.json"
    markdown_path = args.run_dir / "visual_diagnostic_summary.md"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    markdown_path.write_text(markdown_report(summary))
    print(markdown_path)


if __name__ == "__main__":
    main()
