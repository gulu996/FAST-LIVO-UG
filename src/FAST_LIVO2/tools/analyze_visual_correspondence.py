#!/usr/bin/env python3
"""Analyze visual correspondence quality without changing matching gates."""

import argparse
import bisect
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path


INTERVALS = {
    "A_unobstructed": (1785900626.0, 1785900646.0),
    "B_occlusion_denied": (1785900675.0, 1785900695.0),
    "C_denied_half_occlusion": (1785900748.0, 1785900768.0),
}
NCC_DECISIONS = {"ncc_reject", "photometric_reject", "accepted_preliminary"}
VARIABLES = (
    "view_angle_deg",
    "warp_condition",
    "warp_frobenius",
    "warp_max_singular",
    "ref_age",
    "last_seen_age",
    "observation_count",
    "depth_z",
    "range_m",
)


def read_csv(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        header = reader.fieldnames
    if not header:
        raise RuntimeError(f"missing CSV header: {path}")
    with path.open(newline="") as stream:
        raw = list(csv.reader(stream))
    bad = [index + 1 for index, row in enumerate(raw) if len(row) != len(header)]
    if bad:
        raise RuntimeError(f"CSV column mismatch in {path}: lines {bad[:5]}")
    return rows


def number(row, key, default=math.nan):
    try:
        value = float(row.get(key, default))
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def integer(row, key):
    try:
        return int(float(row.get(key, 0) or 0))
    except (TypeError, ValueError):
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


def pearson(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return 0.0
    xs, ys = zip(*pairs)
    mean_x, mean_y = statistics.fmean(xs), statistics.fmean(ys)
    dx = [value - mean_x for value in xs]
    dy = [value - mean_y for value in ys]
    denominator = math.sqrt(sum(value * value for value in dx) * sum(value * value for value in dy))
    return sum(x * y for x, y in zip(dx, dy)) / denominator if denominator > 1e-15 else 0.0


def ranks(values):
    order = sorted(range(len(values)), key=values.__getitem__)
    result = [0.0] * len(values)
    index = 0
    while index < len(order):
        end = index + 1
        while end < len(order) and values[order[end]] == values[order[index]]:
            end += 1
        rank = 0.5 * (index + end - 1)
        for position in range(index, end):
            result[order[position]] = rank
        index = end
    return result


def spearman(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return 0.0
    xs, ys = zip(*pairs)
    return pearson(ranks(list(xs)), ranks(list(ys)))


def quartile_effect(rows, variable):
    values = [number(row, variable) for row in rows]
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return {"bounds": [], "bins": []}
    bounds = [percentile(finite_values, q) for q in (0.25, 0.50, 0.75)]
    bins = [[] for _ in range(4)]
    for row, value in zip(rows, values):
        if not math.isfinite(value):
            continue
        index = bisect.bisect_right(bounds, value)
        bins[index].append(row)
    return {
        "bounds": bounds,
        "bins": [
            {
                "count": len(group),
                "ncc_reject_percent": 100.0 * sum(row["decision"] == "ncc_reject" for row in group) / len(group) if group else 0.0,
                "ncc": distribution([number(row, "ncc") for row in group]),
            }
            for group in bins
        ],
    }


def group_summary(rows):
    ncc_rows = [row for row in rows if row.get("decision") in NCC_DECISIONS]
    ncc_pass = [row for row in ncc_rows if row.get("decision") != "ncc_reject"]
    tracked = [row for row in ncc_rows if row.get("decision") == "accepted_preliminary"]
    low_contrast = [row for row in rows if row.get("decision") == "patch_current_low_contrast"]
    return {
        "patch_input": len(rows),
        "ncc_input": len(ncc_rows),
        "tracked": len(tracked),
        "patch_to_tracked_percent": 100.0 * len(tracked) / len(rows) if rows else 0.0,
        "ncc_reject_percent": 100.0 * sum(row.get("decision") == "ncc_reject" for row in ncc_rows) / len(ncc_rows) if ncc_rows else 0.0,
        "photometric_reject_percent_after_ncc": 100.0 * sum(row.get("decision") == "photometric_reject" for row in ncc_pass) / len(ncc_pass) if ncc_pass else 0.0,
        "current_low_contrast_percent": 100.0 * len(low_contrast) / len(rows) if rows else 0.0,
        "ncc": distribution([number(row, "ncc") for row in ncc_rows]),
        "view_angle_deg": distribution([number(row, "view_angle_deg") for row in rows]),
        "warp_condition": distribution([number(row, "warp_condition") for row in rows]),
        "warp_frobenius": distribution([number(row, "warp_frobenius") for row in rows]),
        "ref_age": distribution([number(row, "ref_age") for row in rows]),
        "last_seen_age": distribution([number(row, "last_seen_age") for row in rows]),
        "observation_count": distribution([number(row, "observation_count") for row in rows]),
        "depth_z": distribution([number(row, "depth_z") for row in rows]),
        "image_u": distribution([number(row, "image_u") for row in rows]),
        "image_v": distribution([number(row, "image_v") for row in rows]),
        "unique_grid_count": len({integer(row, "grid_index") for row in rows}),
        "decision_counts": dict(sorted(Counter(row.get("decision", "unknown") for row in rows).items())),
    }


def summarize_region(candidates, funnel, supply, start, end):
    rows = [row for row in candidates if start <= row["absolute_timestamp"] <= end]
    ncc_rows = [row for row in rows if row.get("decision") in NCC_DECISIONS]
    funnel_rows = [row for row in funnel if start <= row["absolute_timestamp"] <= end]
    supply_rows = [row for row in supply if start <= row["absolute_timestamp"] <= end]
    reject_flags = [1.0 if row.get("decision") == "ncc_reject" else 0.0 for row in ncc_rows]
    ncc_scores = [number(row, "ncc") for row in ncc_rows]
    correlations = {}
    for variable in VARIABLES:
        values = [number(row, variable) for row in ncc_rows]
        correlations[variable] = {
            "pearson_with_ncc": pearson(values, ncc_scores),
            "spearman_with_ncc": spearman(values, ncc_scores),
            "pearson_with_reject": pearson(values, reject_flags),
            "quartiles": quartile_effect(ncc_rows, variable),
        }
    by_source = {
        source: group_summary([row for row in rows if row.get("source") == source])
        for source in ("lidar_submap", "fov_fallback")
    }
    by_search_level = {
        str(level): group_summary([row for row in rows if integer(row, "search_level") == level])
        for level in sorted({integer(row, "search_level") for row in rows})
    }
    low_contrast_rows = [row for row in rows if row.get("decision") == "patch_current_low_contrast"]
    attempted = sum(integer(row, "ekf_attempted") for row in funnel_rows)
    accepted = sum(integer(row, "ekf_attempted") and integer(row, "accepted") for row in funnel_rows)
    duration = max(1e-9, end - start)
    return {
        "window": {"start": start, "end": end, "duration_s": duration},
        "all_candidates": group_summary(rows),
        "ncc_input_count": len(ncc_rows),
        "ncc_pass": group_summary([row for row in ncc_rows if row.get("decision") != "ncc_reject"]),
        "ncc_reject": group_summary([row for row in ncc_rows if row.get("decision") == "ncc_reject"]),
        "by_source": by_source,
        "by_search_level": by_search_level,
        "correlations": correlations,
        "current_low_contrast": group_summary(low_contrast_rows),
        "tracked_points_per_frame": distribution([number(row, "tracked_points", 0.0) for row in funnel_rows]),
        "visual_attempted_hz": attempted / duration,
        "visual_accepted_hz": accepted / duration,
        "processing_time_s": distribution([number(row, "processing_time_s", 0.0) for row in supply_rows]),
    }


def analyze(run_dir):
    candidates = read_csv(run_dir / "visual_patch_quality.csv")
    funnel = read_csv(run_dir / "visual_funnel.csv")
    flow = read_csv(run_dir / "visual_image_flow.csv")
    supply = read_csv(run_dir / "visual_map_supply.csv")
    required = {
        "observation_count", "last_seen_age", "reference_level", "grid_index", "source",
        "view_angle_deg", "warp_condition", "warp_frobenius", "warp_max_singular",
        "depth_z", "range_m", "image_u", "image_v",
    }
    missing = required - set(candidates[0]) if candidates else required
    if missing:
        raise RuntimeError(f"candidate diagnostics missing columns: {sorted(missing)}")
    processed = [row for row in flow if row.get("event") == "image_processed"]
    if len(processed) != len(funnel) or len(supply) != len(funnel):
        raise RuntimeError(f"processed/supply/funnel mismatch: {len(processed)}/{len(supply)}/{len(funnel)}")
    origins = []
    for event, funnel_row, supply_row in zip(processed, funnel, supply):
        absolute = number(event, "timestamp")
        funnel_row["absolute_timestamp"] = absolute
        supply_row["absolute_timestamp"] = absolute
        origins.append(absolute - number(funnel_row, "timestamp"))
    origin = statistics.median(origins)
    for row in candidates:
        row["absolute_timestamp"] = number(row, "timestamp") + origin
    times = [number(row, "timestamp") for row in flow]
    windows = {"full": (min(times), max(times)), **INTERVALS}
    return {
        "run_directory": str(run_dir),
        "visual_time_origin": origin,
        "csv_integrity": {
            "candidate_rows": len(candidates),
            "processed_frames": len(processed),
            "funnel_rows": len(funnel),
            "supply_rows": len(supply),
        },
        "regions": {
            name: summarize_region(candidates, funnel, supply, start, end)
            for name, (start, end) in windows.items()
        },
    }


def render_markdown(report):
    lines = ["# Visual correspondence diagnostics", ""]
    for name, region in report["regions"].items():
        all_rows = region["all_candidates"]
        lines += [f"## {name}", ""]
        lines.append(
            f"patch input={all_rows['patch_input']}, patch->tracked={all_rows['patch_to_tracked_percent']:.2f}%, "
            f"NCC reject={all_rows['ncc_reject_percent']:.2f}%, "
            f"photometric reject={all_rows['photometric_reject_percent_after_ncc']:.2f}%, "
            f"attempted={region['visual_attempted_hz']:.3f} Hz, accepted={region['visual_accepted_hz']:.3f} Hz"
        )
        lines += ["", "| source | patch input | patch->tracked | NCC reject | photo reject | current low contrast |", "|---|---:|---:|---:|---:|---:|"]
        for source, summary in region["by_source"].items():
            lines.append(
                f"| {source} | {summary['patch_input']} | {summary['patch_to_tracked_percent']:.2f}% | "
                f"{summary['ncc_reject_percent']:.2f}% | {summary['photometric_reject_percent_after_ncc']:.2f}% | "
                f"{summary['current_low_contrast_percent']:.2f}% |"
            )
        lines += ["", "| variable | Spearman with NCC | Pearson with reject | Q1 reject | Q4 reject |", "|---|---:|---:|---:|---:|"]
        for variable, correlation in region["correlations"].items():
            bins = correlation["quartiles"]["bins"]
            q1 = bins[0]["ncc_reject_percent"] if bins else 0.0
            q4 = bins[-1]["ncc_reject_percent"] if bins else 0.0
            lines.append(
                f"| {variable} | {correlation['spearman_with_ncc']:.3f} | "
                f"{correlation['pearson_with_reject']:.3f} | {q1:.2f}% | {q4:.2f}% |"
            )
        lines.append("")
    return "\n".join(lines)


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    assert abs(pearson([1, 2, 3], [2, 4, 6]) - 1.0) < 1e-12
    assert abs(spearman([1, 3, 2], [2, 6, 4]) - 1.0) < 1e-12
    rows = [
        {"decision": "ncc_reject", "ncc": "0.1", "ref_age": "10"},
        {"decision": "accepted_preliminary", "ncc": "0.8", "ref_age": "1"},
    ]
    assert quartile_effect(rows, "ref_age")["bins"]
    print("visual correspondence analyzer self-test: PASS")


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
    json_path = args.run_dir / "visual_correspondence_summary.json"
    markdown_path = args.run_dir / "visual_correspondence_summary.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    markdown_path.write_text(render_markdown(report) + "\n")
    print(markdown_path)


if __name__ == "__main__":
    main()
