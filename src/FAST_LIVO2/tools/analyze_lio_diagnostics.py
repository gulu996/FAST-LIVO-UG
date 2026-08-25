#!/usr/bin/env python3
"""Summarize diagnostic-only FAST-LIVO2 LiDAR observability CSV output."""

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
    ("P02", 1785900591.0),
    ("P03", 1785900626.0),
    ("P04", 1785900645.0),
    ("P05", 1785900672.0),
    ("P06", 1785900700.0),
    ("P07", 1785900720.0),
    ("P08", 1785900741.0),
    ("P09", 1785900773.0),
    ("P10", 1785900796.0),
)

REQUIRED_FIELDS = {
    "timestamp", "input_feature_count", "downsampled_feature_count",
    "effective_feature_count", "valid_plane_count", "observability_feature_count",
    "inlier_ratio", "average_point_plane_residual", "abs_residual_median",
    "abs_residual_p90", "abs_residual_p95", "residual_rmse", "abs_residual_max",
    "measurement_variance_median", "translation_eigenvalue_0",
    "translation_eigenvalue_1", "translation_eigenvalue_2",
    "translation_eigenvalue_ratio", "translation_condition_number",
    "rotation_eigenvalue_0", "rotation_eigenvalue_1", "rotation_eigenvalue_2",
    "rotation_eigenvalue_ratio", "rotation_condition_number",
    "weak_translation_direction_body_x", "weak_translation_direction_body_y",
    "weak_translation_direction_body_z", "weak_translation_vertical_abs",
    "weak_rotation_direction_body_x", "weak_rotation_direction_body_y",
    "weak_rotation_direction_body_z", "delta_px", "delta_py", "delta_pz",
    "delta_roll_deg", "delta_pitch_deg", "delta_yaw_deg", "delta_rotation_deg",
    "raw_is_degenerate", "is_degenerate",
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
    index = fraction * (len(finite) - 1)
    lower = int(math.floor(index))
    upper = min(len(finite) - 1, lower + 1)
    alpha = index - lower
    return finite[lower] * (1.0 - alpha) + finite[upper] * alpha


def distribution(rows, field, absolute=False):
    values = [number(row, field) for row in rows]
    values = [abs(value) if absolute else value for value in values if math.isfinite(value)]
    if not values:
        return {key: math.nan for key in ("mean", "p10", "median", "p90", "p95", "max")}
    return {
        "mean": statistics.fmean(values),
        "p10": percentile(values, 0.10),
        "median": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def select(rows, start, end):
    if start is None:
        return rows
    return [row for row in rows if start <= number(row, "timestamp") <= end]


def axis_fractions(rows, prefix, labels):
    counts = {label: 0 for label in labels}
    valid = 0
    for row in rows:
        components = [abs(number(row, f"{prefix}_{axis}")) for axis in ("x", "y", "z")]
        if not all(math.isfinite(value) for value in components):
            continue
        counts[labels[max(range(3), key=lambda index: components[index])]] += 1
        valid += 1
    return {label: 100.0 * counts[label] / valid if valid else math.nan for label in labels}


def frame_rate(rows, start, end):
    if not rows:
        return math.nan
    duration = (end - start) if start is not None else (
        number(rows[-1], "timestamp") - number(rows[0], "timestamp"))
    return len(rows) / duration if duration > 0.0 else math.nan


def fmt(value, precision=3):
    if not math.isfinite(value):
        return "nan" if math.isnan(value) else "inf"
    return f"{value:.{precision}f}"


def compact_dist(rows, field, scale=1.0, absolute=False):
    stats = distribution(rows, field, absolute)
    return "/".join(fmt(stats[key] * scale) for key in ("mean", "median", "p10", "p90", "p95"))


def degraded_runs(rows):
    runs = []
    current = []
    for row in rows:
        degenerate = number(row, "raw_is_degenerate") >= 0.5
        timestamp = number(row, "timestamp")
        contiguous = current and timestamp - number(current[-1], "timestamp") <= 0.25
        if degenerate and (not current or contiguous):
            current.append(row)
        else:
            if current:
                runs.append(current)
                current = []
            if degenerate:
                current = [row]
    if current:
        runs.append(current)
    return sorted(runs, key=lambda run: (-len(run), number(run[0], "timestamp")))


def read_rows(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED_FIELDS - set(reader.fieldnames or ())
        if missing:
            raise ValueError("missing CSV fields: " + ", ".join(sorted(missing)))
        rows = list(reader)
    if not rows:
        raise ValueError("CSV has no data rows")
    expected_columns = len(reader.fieldnames)
    for line_number, row in enumerate(rows, 2):
        if None in row or len(row) != expected_columns:
            raise ValueError(f"CSV column mismatch at line {line_number}")
        if not math.isfinite(number(row, "timestamp")):
            raise ValueError(f"invalid timestamp at line {line_number}")
    rows.sort(key=lambda row: number(row, "timestamp"))
    return rows


def print_report(rows, point_half_window):
    groups = {name: select(rows, start, end) for name, start, end in WINDOWS}
    print("# LiDAR measurement and observability diagnostics")
    print(f"frames={len(rows)} start={number(rows[0], 'timestamp'):.9f} "
          f"end={number(rows[-1], 'timestamp'):.9f} "
          f"duration={number(rows[-1], 'timestamp') - number(rows[0], 'timestamp'):.3f}s")

    print("\n## Frame supply and residuals")
    print("Values are frame-wise mean/median/P10/P90/P95.")
    print("|window|frames|Hz|input points|downsampled|effective planes|inlier ratio %|abs residual mean m|residual RMSE m|abs residual P95 m|")
    print("|---|---:|---:|---|---|---|---|---|---|---|")
    for name, start, end in WINDOWS:
        group = groups[name]
        print(f"|{name}|{len(group)}|{fmt(frame_rate(group, start, end), 2)}|"
              f"{compact_dist(group, 'input_feature_count')}|"
              f"{compact_dist(group, 'downsampled_feature_count')}|"
              f"{compact_dist(group, 'effective_feature_count')}|"
              f"{compact_dist(group, 'inlier_ratio', 100.0)}|"
              f"{compact_dist(group, 'average_point_plane_residual')}|"
              f"{compact_dist(group, 'residual_rmse')}|"
              f"{compact_dist(group, 'abs_residual_p95')}|")

    print("\n## Information and weak directions")
    print("|window|translation lambda min|translation ratio|translation condition|rotation lambda min|rotation ratio|rotation condition|weak T dominant F/L/U %|weak R dominant roll/pitch/yaw %|raw/stable degenerate %|")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for name, _, _ in WINDOWS:
        group = groups[name]
        trans_axes = axis_fractions(group, "weak_translation_direction_body", ("F", "L", "U"))
        rot_axes = axis_fractions(group, "weak_rotation_direction_body", ("roll", "pitch", "yaw"))
        raw_deg = 100.0 * sum(number(row, "raw_is_degenerate") >= 0.5 for row in group) / len(group) if group else math.nan
        stable_deg = 100.0 * sum(number(row, "is_degenerate") >= 0.5 for row in group) / len(group) if group else math.nan
        print(f"|{name}|{compact_dist(group, 'translation_eigenvalue_0')}|"
              f"{compact_dist(group, 'translation_eigenvalue_ratio')}|"
              f"{compact_dist(group, 'translation_condition_number')}|"
              f"{compact_dist(group, 'rotation_eigenvalue_0')}|"
              f"{compact_dist(group, 'rotation_eigenvalue_ratio')}|"
              f"{compact_dist(group, 'rotation_condition_number')}|"
              f"{fmt(trans_axes['F'],1)}/{fmt(trans_axes['L'],1)}/{fmt(trans_axes['U'],1)}|"
              f"{fmt(rot_axes['roll'],1)}/{fmt(rot_axes['pitch'],1)}/{fmt(rot_axes['yaw'],1)}|"
              f"{fmt(raw_deg,1)}/{fmt(stable_deg,1)}|")

    print("\n## State increments and existing per-point variance")
    print("|window|abs dx m|abs dy m|abs dz m|delta rotation deg|abs droll deg|abs dpitch deg|abs dyaw deg|measurement variance|")
    print("|---|---|---|---|---|---|---|---|---|")
    for name, _, _ in WINDOWS:
        group = groups[name]
        print(f"|{name}|{compact_dist(group, 'delta_px', absolute=True)}|"
              f"{compact_dist(group, 'delta_py', absolute=True)}|"
              f"{compact_dist(group, 'delta_pz', absolute=True)}|"
              f"{compact_dist(group, 'delta_rotation_deg')}|"
              f"{compact_dist(group, 'delta_roll_deg', absolute=True)}|"
              f"{compact_dist(group, 'delta_pitch_deg', absolute=True)}|"
              f"{compact_dist(group, 'delta_yaw_deg', absolute=True)}|"
              f"{compact_dist(group, 'measurement_variance_median')}|")

    print(f"\n## P02-P10 neighborhoods (plus/minus {point_half_window:.1f}s)")
    print("|point|time|frames|effective median|inlier median %|residual RMSE median|T ratio median|T condition median/P95|weak vertical median/P90|weak T F/L/U %|weak R r/p/y %|abs dz median|abs roll/pitch median deg|")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---:|---|")
    for label, timestamp in SCORE_TIMES:
        group = select(rows, timestamp - point_half_window, timestamp + point_half_window)
        trans_condition = distribution(group, "translation_condition_number")
        weak_vertical = distribution(group, "weak_translation_vertical_abs")
        trans_axes = axis_fractions(group, "weak_translation_direction_body", ("F", "L", "U"))
        rot_axes = axis_fractions(group, "weak_rotation_direction_body", ("r", "p", "y"))
        roll = distribution(group, "delta_roll_deg", True)
        pitch = distribution(group, "delta_pitch_deg", True)
        print(f"|{label}|{timestamp:.1f}|{len(group)}|"
              f"{fmt(distribution(group, 'effective_feature_count')['median'])}|"
              f"{fmt(distribution(group, 'inlier_ratio')['median'] * 100.0)}|"
              f"{fmt(distribution(group, 'residual_rmse')['median'],4)}|"
              f"{fmt(distribution(group, 'translation_eigenvalue_ratio')['median'],4)}|"
              f"{fmt(trans_condition['median'],2)}/{fmt(trans_condition['p95'],2)}|"
              f"{fmt(weak_vertical['median'],3)}/{fmt(weak_vertical['p90'],3)}|"
              f"{fmt(trans_axes['F'],0)}/{fmt(trans_axes['L'],0)}/{fmt(trans_axes['U'],0)}|"
              f"{fmt(rot_axes['r'],0)}/{fmt(rot_axes['p'],0)}/{fmt(rot_axes['y'],0)}|"
              f"{fmt(distribution(group, 'delta_pz', True)['median'],4)}|"
              f"{fmt(roll['median'],3)}/{fmt(pitch['median'],3)}|")

    print("\n## Worst translation condition frames")
    finite_rows = [row for row in rows if math.isfinite(number(row, "translation_condition_number"))]
    for row in sorted(finite_rows, key=lambda item: number(item, "translation_condition_number"), reverse=True)[:12]:
        print(f"- t={number(row, 'timestamp'):.6f} cond={number(row, 'translation_condition_number'):.2f} "
              f"ratio={number(row, 'translation_eigenvalue_ratio'):.6f} "
              f"effective={number(row, 'effective_feature_count'):.0f} "
              f"weak_world=({number(row, 'weak_translation_direction_world_x'):.3f},"
              f"{number(row, 'weak_translation_direction_world_y'):.3f},"
              f"{number(row, 'weak_translation_direction_world_z'):.3f})")

    print("\n## Longest raw-degenerate runs")
    for run in degraded_runs(rows)[:12]:
        start = number(run[0], "timestamp")
        end = number(run[-1], "timestamp")
        print(f"- {start:.6f}..{end:.6f}: frames={len(run)} span={end-start:.3f}s")


def self_test():
    values = [0.0, 10.0, 20.0, 30.0, math.nan]
    assert percentile(values, 0.50) == 15.0
    assert math.isclose(percentile(values, 0.90), 27.0)
    rows = [
        {"timestamp": "1.0", "raw_is_degenerate": "1"},
        {"timestamp": "1.1", "raw_is_degenerate": "1"},
        {"timestamp": "1.5", "raw_is_degenerate": "0"},
        {"timestamp": "2.0", "raw_is_degenerate": "1"},
    ]
    runs = degraded_runs(rows)
    assert len(runs) == 2 and len(runs[0]) == 2
    axis_rows = [{
        "weak_translation_direction_body_x": "0.2",
        "weak_translation_direction_body_y": "-0.9",
        "weak_translation_direction_body_z": "0.1",
    }]
    assert axis_fractions(axis_rows, "weak_translation_direction_body", ("F", "L", "U"))["L"] == 100.0
    print("analyze_lio_diagnostics.py self-test: PASS")


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
    print_report(read_rows(args.csv), max(0.0, args.point_half_window))


if __name__ == "__main__":
    main()
