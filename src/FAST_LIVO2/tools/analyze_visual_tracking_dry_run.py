#!/usr/bin/env python3
"""Compare normal and tracking-only visual frames without scoring truth data."""

import argparse
import csv
import json
import math
import statistics
import tempfile
from collections import Counter, defaultdict
from pathlib import Path


INTERVALS = {
    "A_unobstructed": (1785900626.0, 1785900646.0),
    "B_occlusion_denied": (1785900675.0, 1785900695.0),
    "C_denied_half_occlusion": (1785900748.0, 1785900768.0),
}
MODES = ("NORMAL_LIDAR_SUPPORTED", "TRACKING_ONLY_DRY_RUN")


def read_csv(path):
    with path.open(newline="") as stream:
        raw = list(csv.reader(stream))
    if not raw:
        raise RuntimeError(f"empty CSV: {path}")
    width = len(raw[0])
    bad = [line for line, row in enumerate(raw, 1) if len(row) != width]
    if bad:
        raise RuntimeError(f"CSV column mismatch in {path}: lines {bad[:5]}")
    return [dict(zip(raw[0], row)) for row in raw[1:]]


def number(row, key, default=0.0):
    try:
        value = float(row.get(key, default) or default)
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def integer(row, key):
    return int(number(row, key, 0.0))


def percentile(values, q):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return 0.0
    position = (len(values) - 1) * q
    lo, hi = math.floor(position), math.ceil(position)
    if lo == hi:
        return values[lo]
    return values[lo] * (hi - position) + values[hi] * (position - lo)


def distribution(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {key: 0.0 for key in ("mean", "median", "p10", "p50", "p90", "p95", "max")}
    return {
        "mean": statistics.fmean(values),
        "median": percentile(values, 0.50),
        "p10": percentile(values, 0.10),
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def percent(numerator, denominator):
    return 100.0 * numerator / denominator if denominator else 0.0


def frame_key(row):
    return row.get("timestamp", ""), row.get("frame_mode", "NORMAL_LIDAR_SUPPORTED")


def attach_absolute_times(flow, funnel, supply, candidates):
    processed_events = [
        row for row in flow
        if row.get("event") in {"image_processed", "image_tracking_only_dry_run"}
    ]
    if len(processed_events) != len(funnel) or len(funnel) != len(supply):
        raise RuntimeError(
            f"flow/funnel/supply mismatch: {len(processed_events)}/{len(funnel)}/{len(supply)}"
        )

    origins = []
    absolute_by_key = {}
    pairing_errors = []
    for event, funnel_row, supply_row in zip(processed_events, funnel, supply):
        expected_mode = (
            "TRACKING_ONLY_DRY_RUN"
            if event.get("event") == "image_tracking_only_dry_run"
            else "NORMAL_LIDAR_SUPPORTED"
        )
        mode = funnel_row.get("frame_mode", "NORMAL_LIDAR_SUPPORTED")
        if mode != expected_mode or supply_row.get("frame_mode", mode) != mode:
            raise RuntimeError(f"frame mode mismatch at visual timestamp {funnel_row.get('timestamp')}")
        absolute = number(event, "timestamp")
        relative = number(funnel_row, "timestamp")
        origin = absolute - relative
        origins.append(origin)
        funnel_row["absolute_timestamp"] = absolute
        supply_row["absolute_timestamp"] = absolute
        absolute_by_key[frame_key(funnel_row)] = absolute

    median_origin = statistics.median(origins)
    pairing_errors = [abs(value - median_origin) for value in origins]
    if pairing_errors and max(pairing_errors) > 0.010:
        raise RuntimeError(f"visual timestamp pairing error exceeds 10 ms: {max(pairing_errors):.6f} s")
    for row in candidates:
        key = frame_key(row)
        if key not in absolute_by_key:
            raise RuntimeError(f"candidate has no funnel frame: {key}")
        row["absolute_timestamp"] = absolute_by_key[key]
    return {
        "mean_s": statistics.fmean(pairing_errors) if pairing_errors else 0.0,
        "median_s": percentile(pairing_errors, 0.50),
        "p95_s": percentile(pairing_errors, 0.95),
        "max_s": max(pairing_errors) if pairing_errors else 0.0,
    }


def check_conservation(funnel, candidates):
    by_frame = defaultdict(Counter)
    for row in candidates:
        decision = row.get("decision", "")
        counts = by_frame[frame_key(row)]
        counts["all"] += 1
        counts[decision] += 1
        if not decision.startswith("patch_"):
            counts["patch_valid"] += 1
    problems = []
    for row in funnel:
        counts = by_frame[frame_key(row)]
        expected = {
            "patch_quality_input": counts["all"],
            "patch_valid": counts["patch_valid"],
            "ncc_rejected": counts["ncc_reject"],
            "photometric_rejected": counts["photometric_reject"],
            "tracked_points": counts["accepted_preliminary"],
        }
        for column, value in expected.items():
            if integer(row, column) != value:
                problems.append((frame_key(row), column, integer(row, column), value))
    if problems:
        raise RuntimeError(f"candidate decision conservation failed: {problems[:5]}")


def summarize(rows, supply_rows, candidate_rows, duration):
    frame_count = len(rows)
    patch_input = sum(integer(row, "patch_quality_input") for row in rows)
    patch_valid = sum(integer(row, "patch_valid") for row in rows)
    tracked_total = sum(integer(row, "tracked_points") for row in rows)
    ncc_rejected = sum(integer(row, "ncc_rejected") for row in rows)
    ncc_pass = max(0, patch_valid - ncc_rejected)
    photometric_rejected = sum(integer(row, "photometric_rejected") for row in rows)
    tracked_values = [number(row, "tracked_points") for row in rows]
    patch_values = [number(row, "patch_quality_input") for row in rows]
    source_counts = Counter(row.get("source", "unknown") for row in candidate_rows)
    search_levels = Counter(integer(row, "search_level") for row in candidate_rows)
    attempted = sum(integer(row, "ekf_attempted") for row in rows)
    accepted = sum(integer(row, "accepted") for row in rows)
    return {
        "frames": frame_count,
        "hz": frame_count / duration if duration > 0.0 else 0.0,
        "visual_map_total": distribution([number(row, "map_points") for row in rows]),
        "projected": distribution([number(row, "projected_candidates") for row in rows]),
        "patch_input": distribution(patch_values),
        "patch_ge_30_percent": percent(sum(value >= 30 for value in patch_values), frame_count),
        "tracked": distribution(tracked_values),
        "tracked_ge_15_percent": percent(sum(value >= 15 for value in tracked_values), frame_count),
        "tracked_ge_20_percent": percent(sum(value >= 20 for value in tracked_values), frame_count),
        "tracked_ge_30_percent": percent(sum(value >= 30 for value in tracked_values), frame_count),
        "patch_to_tracked_percent": percent(tracked_total, patch_input),
        "ncc_reject_percent": percent(ncc_rejected, patch_valid),
        "photometric_reject_percent_after_ncc": percent(photometric_rejected, ncc_pass),
        "visual_attempted_hz": attempted / duration if duration > 0.0 else 0.0,
        "visual_accepted_hz": accepted / duration if duration > 0.0 else 0.0,
        "processing_time_ms": distribution([1000.0 * number(row, "processing_time_s") for row in supply_rows]),
        "search_level_counts": dict(sorted(search_levels.items())),
        "search_level_percent": {
            str(level): percent(count, sum(search_levels.values()))
            for level, count in sorted(search_levels.items())
        },
        "fov_fallback_candidate_percent": percent(
            source_counts["fov_fallback"], sum(source_counts.values())
        ),
        "fov_fallback_frame_percent": percent(
            sum(number(row, "fallback_added_grid_cells") > 0 for row in supply_rows), frame_count
        ),
    }


def analyze(run_dir):
    flow = read_csv(run_dir / "visual_image_flow.csv")
    funnel = read_csv(run_dir / "visual_funnel.csv")
    supply = read_csv(run_dir / "visual_map_supply.csv")
    candidates = read_csv(run_dir / "visual_patch_quality.csv")
    for name, rows in (("funnel", funnel), ("supply", supply), ("candidates", candidates)):
        if rows and "frame_mode" not in rows[0]:
            raise RuntimeError(f"{name} CSV lacks frame_mode")
    pairing = attach_absolute_times(flow, funnel, supply, candidates)
    check_conservation(funnel, candidates)

    received = [row for row in flow if row.get("event") == "image_received"]
    if not received:
        raise RuntimeError("visual_image_flow.csv has no image_received rows")
    # Include startup-buffered visual frames even if their callback event was
    # logged just before the first image_received row.
    full_start = min(
        min(number(row, "timestamp") for row in received),
        min(row["absolute_timestamp"] for row in funnel),
    )
    full_end = max(
        max(number(row, "timestamp") for row in received),
        max(row["absolute_timestamp"] for row in funnel),
    )
    windows = {"full": (full_start, full_end), **INTERVALS}
    regions = {}
    for region, (start, end) in windows.items():
        duration = max(1e-9, end - start)
        regions[region] = {}
        for mode in MODES:
            frame_rows = [
                row for row in funnel
                if row.get("frame_mode") == mode and start <= row["absolute_timestamp"] <= end
            ]
            supply_rows = [
                row for row in supply
                if row.get("frame_mode") == mode and start <= row["absolute_timestamp"] <= end
            ]
            candidate_rows = [
                row for row in candidates
                if row.get("frame_mode") == mode and start <= row["absolute_timestamp"] <= end
            ]
            regions[region][mode] = summarize(frame_rows, supply_rows, candidate_rows, duration)

    memory_path = run_dir / "runtime_memory.csv"
    memory = read_csv(memory_path) if memory_path.exists() else []
    rss = [number(row, "rss_mb") for row in memory]
    return {
        "run_directory": str(run_dir),
        "csv_integrity": {
            "flow_rows": len(flow),
            "funnel_rows": len(funnel),
            "supply_rows": len(supply),
            "candidate_rows": len(candidates),
            "decision_conservation": "PASS",
            "timestamp_pairing_error": pairing,
        },
        "camera_received_hz": len(received) / max(1e-9, full_end - full_start),
        "rss_mb": distribution(rss),
        "regions": regions,
    }


def render_markdown(report):
    lines = [
        "# Visual tracking-only dry-run diagnostics",
        "",
        f"Camera received: {report['camera_received_hz']:.3f} Hz",
        "",
    ]
    for region, modes in report["regions"].items():
        lines += [f"## {region}", ""]
        lines += [
            "| mode | Hz | projected mean/median/P95 | patch mean/median/P95 | patch>=30 | tracked mean/median/P90/P95 | >=15 | >=20 | >=30 | survival | NCC reject | photo reject | attempted/accepted Hz | processing mean/P95/max ms | fallback cand/frame |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for mode in MODES:
            item = modes[mode]
            projected, patch, tracked = item["projected"], item["patch_input"], item["tracked"]
            processing = item["processing_time_ms"]
            lines.append(
                f"| {mode} | {item['hz']:.3f} | "
                f"{projected['mean']:.2f}/{projected['median']:.2f}/{projected['p95']:.2f} | "
                f"{patch['mean']:.2f}/{patch['median']:.2f}/{patch['p95']:.2f} | "
                f"{item['patch_ge_30_percent']:.1f}% | "
                f"{tracked['mean']:.2f}/{tracked['median']:.2f}/{tracked['p90']:.2f}/{tracked['p95']:.2f} | "
                f"{item['tracked_ge_15_percent']:.1f}% | {item['tracked_ge_20_percent']:.1f}% | "
                f"{item['tracked_ge_30_percent']:.1f}% | {item['patch_to_tracked_percent']:.1f}% | "
                f"{item['ncc_reject_percent']:.1f}% | {item['photometric_reject_percent_after_ncc']:.1f}% | "
                f"{item['visual_attempted_hz']:.3f}/{item['visual_accepted_hz']:.3f} | "
                f"{processing['mean']:.2f}/{processing['p95']:.2f}/{processing['max']:.2f} | "
                f"{item['fov_fallback_candidate_percent']:.1f}%/{item['fov_fallback_frame_percent']:.1f}% |"
            )
        lines.append("")
    return "\n".join(lines)


def self_test():
    assert percentile([0.0, 10.0], 0.5) == 5.0
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "test.csv"
        path.write_text("a,b\n1,2\n")
        assert read_csv(path) == [{"a": "1", "b": "2"}]
    rows = [{"tracked_points": str(value)} for value in (10, 20, 30)]
    summary = summarize(rows, [], [], 1.0)
    assert summary["tracked"]["median"] == 20.0
    assert abs(summary["tracked_ge_20_percent"] - 200.0 / 3.0) < 1e-12
    print("visual tracking-only dry-run analyzer self-test: PASS")


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
    output = json.dumps(report, indent=2, sort_keys=True)
    markdown = render_markdown(report)
    if args.json:
        args.json.write_text(output + "\n")
    if args.markdown:
        args.markdown.write_text(markdown + "\n")
    print(markdown)


if __name__ == "__main__":
    main()
