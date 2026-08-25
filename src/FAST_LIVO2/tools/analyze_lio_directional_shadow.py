#!/usr/bin/env python3
"""Summarize Stage 4B LiDAR directional-projector shadow solves.

This tool intentionally consumes only internal estimator diagnostics. It has no
reference-trajectory input, so threshold selection cannot accidentally use truth.
"""

import argparse
import csv
import math
import statistics
from pathlib import Path


WINDOWS = (
    ("Full", None, None),
    ("A", 1785900626.0, 1785900646.0),
    ("B", 1785900675.0, 1785900695.0),
    ("C", 1785900748.0, 1785900768.0),
)

SCORE_TIMES = (
    ("P02", 1785900591.0), ("P03", 1785900626.0),
    ("P04", 1785900645.0), ("P05", 1785900672.0),
    ("P06", 1785900700.0), ("P07", 1785900720.0),
    ("P08", 1785900741.0), ("P09", 1785900773.0),
    ("P10", 1785900796.0),
)

REQUIRED_FIELDS = {
    "timestamp", "frame_id", "iteration_index", "valid", "relative_threshold",
    "effective_feature_count", "residual_rmse", "translation_condition",
    "rotation_condition", "weak_translation_vertical_abs", "weak_rotation_x",
    "weak_rotation_y", "weak_rotation_z", "affected_direction_count",
    "partial_suppression_count", "full_suppression_count",
    "information_trace_retained_ratio", "raw_dx", "raw_dy", "raw_dz",
    "shadow_dx", "shadow_dy", "shadow_dz", "removed_dx", "removed_dy",
    "removed_dz", "raw_droll_deg", "raw_dpitch_deg", "raw_dyaw_deg",
    "shadow_droll_deg", "shadow_dpitch_deg", "shadow_dyaw_deg",
    "removed_droll_deg", "removed_dpitch_deg", "removed_dyaw_deg",
    "removed_dR_deg",
    "raw_weak_translation_projection", "shadow_weak_translation_projection",
    "raw_weak_rotation_projection_deg", "shadow_weak_rotation_projection_deg",
    "raw_posterior_pose_cov_trace", "shadow_posterior_pose_cov_trace",
    "shadow_solve_time_ms",
}


def number(row, field):
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return math.nan


def percentile(values, fraction):
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = fraction * (len(finite) - 1)
    lower = int(math.floor(position))
    upper = min(lower + 1, len(finite) - 1)
    alpha = position - lower
    return finite[lower] * (1.0 - alpha) + finite[upper] * alpha


def values(rows, field, absolute=False):
    result = [number(row, field) for row in rows]
    return [abs(value) if absolute else value for value in result if math.isfinite(value)]


def distribution(rows, field, absolute=False):
    finite = values(rows, field, absolute)
    if not finite:
        return {key: math.nan for key in
                ("sum", "abs_sum", "mean", "median", "p10", "p90", "p95", "max")}
    return {
        "sum": sum(finite),
        "abs_sum": sum(abs(value) for value in finite),
        "mean": statistics.fmean(finite),
        "median": percentile(finite, 0.50),
        "p10": percentile(finite, 0.10),
        "p90": percentile(finite, 0.90),
        "p95": percentile(finite, 0.95),
        "max": max(finite),
    }


def pearson(rows, field_x, field_y, abs_x=False, abs_y=False, log_x=False):
    pairs = []
    for row in rows:
        x = number(row, field_x)
        y = number(row, field_y)
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if abs_x:
            x = abs(x)
        if abs_y:
            y = abs(y)
        if log_x:
            if x <= 0.0:
                continue
            x = math.log10(x)
        pairs.append((x, y))
    if len(pairs) < 2:
        return math.nan
    mean_x = statistics.fmean(pair[0] for pair in pairs)
    mean_y = statistics.fmean(pair[1] for pair in pairs)
    covariance = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    variance_x = sum((x - mean_x) ** 2 for x, _ in pairs)
    variance_y = sum((y - mean_y) ** 2 for _, y in pairs)
    denominator = math.sqrt(variance_x * variance_y)
    return covariance / denominator if denominator > 0.0 else math.nan


def select(rows, start, end):
    if start is None:
        return rows
    return [row for row in rows if start <= number(row, "timestamp") <= end]


def read_rows(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        missing = REQUIRED_FIELDS - fields
        if missing:
            raise ValueError("missing CSV fields: " + ", ".join(sorted(missing)))
        rows = list(reader)
        expected_columns = len(reader.fieldnames or ())
    if not rows:
        raise ValueError("CSV has no data rows")
    for line_number, row in enumerate(rows, 2):
        if None in row or len(row) != expected_columns:
            raise ValueError(f"CSV column mismatch at line {line_number}")
        if not math.isfinite(number(row, "timestamp")):
            raise ValueError(f"invalid timestamp at line {line_number}")
    rows.sort(key=lambda row: (number(row, "relative_threshold"), number(row, "timestamp")))
    thresholds = sorted(set(number(row, "relative_threshold") for row in rows))
    frame_counts = []
    for threshold in thresholds:
        group = [row for row in rows if number(row, "relative_threshold") == threshold]
        frame_ids = [int(number(row, "frame_id")) for row in group]
        if len(frame_ids) != len(set(frame_ids)):
            raise ValueError(f"duplicate frame rows at threshold {threshold:g}")
        frame_counts.append(len(group))
    if len(set(frame_counts)) != 1:
        raise ValueError(f"threshold row-count mismatch: {dict(zip(thresholds, frame_counts))}")
    return rows, thresholds


def fmt(value, precision=4):
    if not math.isfinite(value):
        return "nan" if math.isnan(value) else "inf"
    return f"{value:.{precision}f}"


def ratio_percent(rows, field, predicate=lambda value: value > 0.0):
    finite = values(rows, field)
    return 100.0 * sum(predicate(value) for value in finite) / len(finite) if finite else math.nan


def signed_triplet(rows, prefix):
    return "/".join(fmt(distribution(rows, prefix + axis)["sum"], 5)
                    for axis in ("x", "y", "z"))


def rpy_triplet(rows, prefix):
    return "/".join(fmt(distribution(rows, prefix + axis + "_deg")["sum"], 4)
                    for axis in ("droll", "dpitch", "dyaw"))


def report(rows, thresholds, point_half_window):
    print("# LiDAR directional observability shadow solve")
    valid_rows = [row for row in rows if number(row, "valid") >= 0.5]
    print(f"rows={len(rows)} valid={len(valid_rows)} thresholds={','.join(f'{x:g}' for x in thresholds)}")
    print("Each row is the terminating real ESIKF iteration, solved counterfactually with the same H/R/P.")

    print("\n## Threshold sweep (internal metrics only)")
    print("|threshold|frames|affected %|partial %|full %|info retained mean/median/P10 %|"
          "posterior pose trace shadow/raw median/P95|removed dp abs sum m|removed dR abs sum deg|solve mean/P95 ms|")
    print("|---:|---:|---:|---:|---:|---|---|---:|---:|---|")
    for threshold in thresholds:
        group = [row for row in valid_rows if number(row, "relative_threshold") == threshold]
        retained = distribution(group, "information_trace_retained_ratio")
        covariance_ratios = []
        for row in group:
            raw = number(row, "raw_posterior_pose_cov_trace")
            shadow = number(row, "shadow_posterior_pose_cov_trace")
            if math.isfinite(raw) and math.isfinite(shadow) and raw > 0.0:
                covariance_ratios.append(shadow / raw)
        solve = distribution(group, "shadow_solve_time_ms")
        removed_dp = sum(math.sqrt(number(row, "removed_dx") ** 2 +
                                   number(row, "removed_dy") ** 2 +
                                   number(row, "removed_dz") ** 2) for row in group)
        removed_dr = sum(abs(number(row, "removed_dR_deg")) for row in group)
        print(f"|{threshold:g}|{len(group)}|"
              f"{fmt(ratio_percent(group, 'affected_direction_count'),2)}|"
              f"{fmt(ratio_percent(group, 'partial_suppression_count'),2)}|"
              f"{fmt(ratio_percent(group, 'full_suppression_count'),2)}|"
              f"{fmt(retained['mean']*100,3)}/{fmt(retained['median']*100,3)}/{fmt(retained['p10']*100,3)}|"
              f"{fmt(percentile(covariance_ratios,0.5),5)}/{fmt(percentile(covariance_ratios,0.95),5)}|"
              f"{fmt(removed_dp,4)}|{fmt(removed_dr,3)}|"
              f"{fmt(solve['mean'],3)}/{fmt(solve['p95'],3)}|")

    print("\n## A/B/C correction sums")
    print("Triplets are signed sums x/y/z in metres and roll/pitch/yaw in degrees.")
    print("|threshold|window|frames|affected %|retained median %|raw dxyz|shadow dxyz|removed dxyz|"
          "raw rpy|shadow rpy|removed rpy|cov trace ratio median|")
    print("|---:|---|---:|---:|---:|---|---|---|---|---|---|---:|")
    for threshold in thresholds:
        threshold_rows = [row for row in valid_rows if number(row, "relative_threshold") == threshold]
        for name, start, end in WINDOWS:
            group = select(threshold_rows, start, end)
            covariance_ratios = [number(row, "shadow_posterior_pose_cov_trace") /
                                 number(row, "raw_posterior_pose_cov_trace") for row in group
                                 if number(row, "raw_posterior_pose_cov_trace") > 0.0]
            print(f"|{threshold:g}|{name}|{len(group)}|"
                  f"{fmt(ratio_percent(group, 'affected_direction_count'),1)}|"
                  f"{fmt(distribution(group, 'information_trace_retained_ratio')['median']*100,3)}|"
                  f"{signed_triplet(group,'raw_d')}|{signed_triplet(group,'shadow_d')}|"
                  f"{signed_triplet(group,'removed_d')}|{rpy_triplet(group,'raw_')}|"
                  f"{rpy_triplet(group,'shadow_')}|{rpy_triplet(group,'removed_')}|"
                  f"{fmt(percentile(covariance_ratios,0.5),5)}|")

    print("\n## B removed vertical and roll")
    print("|threshold|removed dz sum/abs-sum/mean/median/P90/max m|"
          "removed roll sum/abs-sum/mean/median/P90/max deg|"
          "corr log10(T condition),abs raw weak T|corr log10(T condition),abs removed dz|"
          "corr log10(R condition),abs raw weak R|corr log10(R condition),abs removed roll|")
    print("|---:|---|---|---:|---:|---:|---:|")
    for threshold in thresholds:
        group = select([row for row in valid_rows if number(row, "relative_threshold") == threshold],
                       1785900675.0, 1785900695.0)
        dz = distribution(group, "removed_dz")
        roll = distribution(group, "removed_droll_deg")
        def six(stats):
            return "/".join(fmt(stats[key], 6) for key in
                            ("sum", "abs_sum", "mean", "median", "p90", "max"))
        print(f"|{threshold:g}|{six(dz)}|{six(roll)}|"
              f"{fmt(pearson(group,'translation_condition','raw_weak_translation_projection',abs_y=True,log_x=True),3)}|"
              f"{fmt(pearson(group,'translation_condition','removed_dz',abs_y=True,log_x=True),3)}|"
              f"{fmt(pearson(group,'rotation_condition','raw_weak_rotation_projection_deg',abs_y=True,log_x=True),3)}|"
              f"{fmt(pearson(group,'rotation_condition','removed_droll_deg',abs_y=True,log_x=True),3)}|")

    print(f"\n## P02-P10 plus/minus {point_half_window:g}s")
    print("|threshold|point|frames|affected %|T condition median/P95|vertical median|"
          "raw/shadow/removed dz sum m|raw/shadow/removed roll sum deg|retained median %|")
    print("|---:|---|---:|---:|---|---:|---|---|---:|")
    for threshold in thresholds:
        threshold_rows = [row for row in valid_rows if number(row, "relative_threshold") == threshold]
        for label, timestamp in SCORE_TIMES:
            group = select(threshold_rows, timestamp-point_half_window, timestamp+point_half_window)
            condition = distribution(group, "translation_condition")
            print(f"|{threshold:g}|{label}|{len(group)}|"
                  f"{fmt(ratio_percent(group,'affected_direction_count'),1)}|"
                  f"{fmt(condition['median'],2)}/{fmt(condition['p95'],2)}|"
                  f"{fmt(distribution(group,'weak_translation_vertical_abs')['median'],3)}|"
                  f"{fmt(distribution(group,'raw_dz')['sum'],5)}/"
                  f"{fmt(distribution(group,'shadow_dz')['sum'],5)}/"
                  f"{fmt(distribution(group,'removed_dz')['sum'],5)}|"
                  f"{fmt(distribution(group,'raw_droll_deg')['sum'],4)}/"
                  f"{fmt(distribution(group,'shadow_droll_deg')['sum'],4)}/"
                  f"{fmt(distribution(group,'removed_droll_deg')['sum'],4)}|"
                  f"{fmt(distribution(group,'information_trace_retained_ratio')['median']*100,3)}|")


def self_test():
    assert percentile([0.0, 10.0, 20.0, 30.0], 0.5) == 15.0
    rows = [{"x": "1", "y": "2"}, {"x": "2", "y": "4"}, {"x": "3", "y": "6"}]
    assert math.isclose(pearson(rows, "x", "y"), 1.0)
    stats = distribution([{"v": "-2"}, {"v": "3"}], "v")
    assert stats["sum"] == 1.0 and stats["abs_sum"] == 5.0
    print("analyze_lio_directional_shadow.py self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", type=Path)
    parser.add_argument("--point-half-window", type=float, default=2.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.csv is None:
        parser.error("csv path is required unless --self-test is used")
    rows, thresholds = read_rows(args.csv)
    report(rows, thresholds, max(0.0, args.point_half_window))


if __name__ == "__main__":
    main()
