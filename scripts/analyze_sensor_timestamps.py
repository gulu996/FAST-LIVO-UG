#!/usr/bin/env python3
"""Read-only ROS1 bag timestamp analysis with startup/change-point handling."""

import argparse
import csv
import os
from pathlib import Path
import struct
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


DEFAULT_TOPICS = (
    "/left_camera/image",
    "/livox/lidar",
    "/livox/imu",
)


class TopicSeries:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.bag_ns: List[int] = []
        self.header_ns: List[int] = []
        self.seq: List[int] = []


def serialized_header(raw_message) -> Tuple[int, int]:
    """Read std_msgs/Header seq/stamp without deserializing large payloads."""
    serialized = raw_message[1]
    if len(serialized) < 12:
        raise RuntimeError("serialized ROS message is shorter than Header")
    seq, seconds, nanoseconds = struct.unpack_from("<III", serialized)
    return int(seq), int(seconds) * 1_000_000_000 + int(nanoseconds)


def percentile(values: Sequence[float], quantile: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def interval_summary(interval_ns: Sequence[int]) -> Dict[str, float]:
    values_ms = np.asarray(interval_ns, dtype=np.float64) / 1e6
    if values_ms.size == 0:
        return {key: float("nan") for key in (
            "min_ms", "p1_ms", "median_ms", "mean_ms",
            "p95_ms", "p99_ms", "max_ms",
        )}
    return {
        "min_ms": float(np.min(values_ms)),
        "p1_ms": float(np.percentile(values_ms, 1)),
        "median_ms": float(np.median(values_ms)),
        "mean_ms": float(np.mean(values_ms)),
        "p95_ms": float(np.percentile(values_ms, 95)),
        "p99_ms": float(np.percentile(values_ms, 99)),
        "max_ms": float(np.max(values_ms)),
    }


def detect_change_points(header_ns: Sequence[int]) -> Tuple[List[int], List[int], int]:
    """Return all anomalous interval indices, startup anomalies, stable start."""
    if len(header_ns) < 3:
        return [], [], 0
    intervals = np.diff(np.asarray(header_ns, dtype=np.int64))
    median = float(np.median(intervals))
    mad = float(np.median(np.abs(intervals - median)))
    robust_sigma = 1.4826 * mad
    # ponytail: an interval is a change point only if it is both far beyond
    # normal jitter and at least 50% of the nominal period away.
    threshold = max(10.0 * robust_sigma, 0.5 * abs(median), 1.0)
    anomalies = [
        int(index)
        for index, value in enumerate(intervals)
        if abs(float(value) - median) > threshold
    ]
    startup_window = min(len(intervals), max(20, len(intervals) // 50))
    startup = [index for index in anomalies if index < startup_window]
    stable_start = max((index + 1 for index in startup), default=0)
    return anomalies, startup, stable_start


def robust_linear_fit(x_ns: Sequence[int], y_ns: Sequence[int]) -> Tuple[float, float]:
    """Huber IRLS fit y_seconds = intercept + slope * x_seconds."""
    if len(x_ns) < 2:
        return float("nan"), float("nan")
    x = np.asarray(x_ns, dtype=np.float64) / 1e9
    y = np.asarray(y_ns, dtype=np.float64) / 1e9
    x_origin = float(x[0])
    y_origin = float(y[0])
    centered_x = x - x_origin
    centered_y = y - y_origin
    design = np.column_stack((np.ones_like(centered_x), centered_x))
    coefficients = np.linalg.lstsq(design, centered_y, rcond=None)[0]
    for _ in range(20):
        residual = centered_y - design.dot(coefficients)
        scale = 1.4826 * float(np.median(np.abs(
            residual - np.median(residual))))
        if scale <= np.finfo(float).eps:
            break
        cutoff = 1.345 * scale
        absolute = np.abs(residual)
        weights = np.ones_like(absolute)
        outside = absolute > cutoff
        weights[outside] = cutoff / absolute[outside]
        weighted_design = design * np.sqrt(weights)[:, None]
        weighted_y = centered_y * np.sqrt(weights)
        updated = np.linalg.lstsq(
            weighted_design, weighted_y, rcond=None)[0]
        if np.max(np.abs(updated - coefficients)) < 1e-14:
            coefficients = updated
            break
        coefficients = updated
    slope = float(coefficients[1])
    intercept = float(
        y_origin + coefficients[0] - slope * x_origin)
    return intercept, slope


def endpoint_ppm(bag_ns: Sequence[int], header_ns: Sequence[int]) -> float:
    if len(bag_ns) < 2:
        return float("nan")
    bag_duration = bag_ns[-1] - bag_ns[0]
    if bag_duration == 0:
        return float("nan")
    return ((header_ns[-1] - header_ns[0]) / bag_duration - 1.0) * 1e6


def sequence_gap_count(seq: Sequence[int]) -> int:
    gaps = 0
    for previous, current in zip(seq, seq[1:]):
        delta = (current - previous) & 0xFFFFFFFF
        if 1 < delta < 0x80000000:
            gaps += delta - 1
    return gaps


def exact_timestamp_matches(
    camera_ns: Sequence[int], lidar_ns: Sequence[int]
) -> Tuple[List[Tuple[int, int, int]], Optional[Tuple[int, int, int, int, int]]]:
    lidar_indices: Dict[int, List[int]] = {}
    for lidar_index, stamp in enumerate(lidar_ns):
        lidar_indices.setdefault(stamp, []).append(lidar_index)
    matches: List[Tuple[int, int, int]] = []
    for camera_index, stamp in enumerate(camera_ns):
        for lidar_index in lidar_indices.get(stamp, []):
            matches.append((camera_index, lidar_index, stamp))

    best = None
    run_start = 0
    for index in range(1, len(matches) + 1):
        continues = (
            index < len(matches)
            and matches[index][0] == matches[index - 1][0] + 1
            and matches[index][1] == matches[index - 1][1] + 1
        )
        if continues:
            continue
        if index > run_start:
            first = matches[run_start]
            last = matches[index - 1]
            candidate = (
                first[0], last[0], first[1], last[1],
                index - run_start,
            )
            if best is None or candidate[4] > best[4]:
                best = candidate
        run_start = index
    return matches, best


def read_bag(path: Path, topics: Sequence[str]) -> Dict[str, TopicSeries]:
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "rosbag Python module is unavailable; source ROS Noetic first"
        ) from exc

    before = path.stat()
    series = {topic: TopicSeries(topic) for topic in topics}
    message_count = 0
    with rosbag.Bag(str(path), "r") as bag:
        for topic, raw_message, bag_time in bag.read_messages(
                topics=list(topics), raw=True):
            item = series[topic]
            seq, header_ns = serialized_header(raw_message)
            item.bag_ns.append(int(bag_time.to_nsec()))
            item.header_ns.append(header_ns)
            item.seq.append(seq)
            message_count += 1
            if message_count % 20_000 == 0:
                print(
                    f"scanned {message_count} messages from {path}",
                    flush=True)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (
            after.st_size, after.st_mtime_ns):
        raise RuntimeError("input bag changed while being analyzed")
    return series


def analyze_series(item: TopicSeries) -> Dict[str, object]:
    header_intervals = [
        current - previous
        for previous, current in zip(item.header_ns, item.header_ns[1:])
    ]
    bag_intervals = [
        current - previous
        for previous, current in zip(item.bag_ns, item.bag_ns[1:])
    ]
    anomalies, startup_anomalies, stable_start = detect_change_points(
        item.header_ns)
    stable_headers = item.header_ns[stable_start:]
    stable_bag = item.bag_ns[stable_start:]
    _, full_slope = robust_linear_fit(item.bag_ns, item.header_ns)
    _, stable_slope = robust_linear_fit(stable_bag, stable_headers)
    return {
        "topic": item.topic,
        "count": len(item.header_ns),
        "duplicates": sum(
            current == previous
            for previous, current in zip(
                item.header_ns, item.header_ns[1:])),
        "non_monotonic": sum(
            current < previous
            for previous, current in zip(
                item.header_ns, item.header_ns[1:])),
        "seq_gap_count": sequence_gap_count(item.seq),
        "anomaly_indices": anomalies,
        "startup_anomaly_indices": startup_anomalies,
        "stable_start": stable_start,
        "full_header_intervals": interval_summary(header_intervals),
        "stable_header_intervals": interval_summary(
            [current - previous for previous, current in zip(
                stable_headers, stable_headers[1:])]),
        "full_bag_intervals": interval_summary(bag_intervals),
        "full_endpoint_ppm": endpoint_ppm(item.bag_ns, item.header_ns),
        "stable_endpoint_ppm": endpoint_ppm(stable_bag, stable_headers),
        "full_robust_ppm": (full_slope - 1.0) * 1e6,
        "stable_robust_ppm": (stable_slope - 1.0) * 1e6,
        "bag_minus_header_trend_us_per_s": -(stable_slope - 1.0) * 1e6,
    }


def write_summary_csv(path: Path, analyses: Sequence[Dict[str, object]]) -> None:
    fields = [
        "topic", "count", "duplicates", "non_monotonic",
        "seq_gap_count", "stable_start", "startup_anomaly_indices",
        "full_endpoint_ppm", "stable_endpoint_ppm",
        "full_robust_ppm", "stable_robust_ppm",
        "bag_minus_header_trend_us_per_s",
        "full_min_ms", "full_median_ms", "full_mean_ms",
        "full_p1_ms", "full_p95_ms", "full_p99_ms", "full_max_ms",
        "stable_min_ms", "stable_median_ms", "stable_mean_ms",
        "stable_p1_ms", "stable_p95_ms", "stable_p99_ms",
        "stable_max_ms",
    ]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for analysis in analyses:
            full = analysis["full_header_intervals"]
            stable = analysis["stable_header_intervals"]
            row = {key: analysis.get(key, "") for key in fields}
            row["startup_anomaly_indices"] = ";".join(
                str(value)
                for value in analysis["startup_anomaly_indices"])
            for prefix, values in (("full", full), ("stable", stable)):
                for key in (
                    "min_ms", "median_ms", "mean_ms", "p1_ms",
                    "p95_ms", "p99_ms", "max_ms",
                ):
                    row[f"{prefix}_{key}"] = values[key]
            writer.writerow(row)


def write_intervals_csv(
    path: Path, series: Iterable[TopicSeries],
    analyses_by_topic: Dict[str, Dict[str, object]],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow([
            "topic", "from_index", "to_index", "bag_dt_ms",
            "header_dt_ms", "change_point", "startup_anomaly",
        ])
        for item in series:
            analysis = analyses_by_topic[item.topic]
            anomalies = set(analysis["anomaly_indices"])
            startup = set(analysis["startup_anomaly_indices"])
            for index in range(len(item.header_ns) - 1):
                writer.writerow([
                    item.topic, index, index + 1,
                    (item.bag_ns[index + 1] - item.bag_ns[index]) / 1e6,
                    (item.header_ns[index + 1] -
                     item.header_ns[index]) / 1e6,
                    int(index in anomalies), int(index in startup),
                ])


def write_matches_csv(
    path: Path, matches: Sequence[Tuple[int, int, int]]
) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(["camera_index", "lidar_index", "header_stamp_ns"])
        writer.writerows(matches)


def format_stats(stats: Dict[str, float]) -> str:
    return (
        f"min={stats['min_ms']:.6f}, p1={stats['p1_ms']:.6f}, "
        f"median={stats['median_ms']:.6f}, mean={stats['mean_ms']:.6f}, "
        f"p95={stats['p95_ms']:.6f}, p99={stats['p99_ms']:.6f}, "
        f"max={stats['max_ms']:.6f} ms"
    )


def write_markdown(
    path: Path, bag_path: Path, analyses: Sequence[Dict[str, object]],
    longest_match: Optional[Tuple[int, int, int, int, int]],
) -> None:
    by_topic = {analysis["topic"]: analysis for analysis in analyses}
    camera = by_topic.get("/left_camera/image")
    lines = [
        "# Sensor timestamp analysis",
        "",
        f"- Input bag (read-only): `{bag_path}`",
        "- Frequency estimates use Huber robust regression and report startup "
        "change points separately.",
        "",
        "## Topic summary",
        "",
        "| Topic | Count | Duplicate | Backward | Stable start | "
        "Full endpoint ppm | Stable robust ppm |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for analysis in analyses:
        lines.append(
            f"| `{analysis['topic']}` | {analysis['count']} | "
            f"{analysis['duplicates']} | {analysis['non_monotonic']} | "
            f"{analysis['stable_start']} | "
            f"{analysis['full_endpoint_ppm']:.6f} | "
            f"{analysis['stable_robust_ppm']:.6f} |"
        )
    lines.extend(["", "## Interval distributions", ""])
    for analysis in analyses:
        lines.extend([
            f"### `{analysis['topic']}`",
            "",
            f"- Full: {format_stats(analysis['full_header_intervals'])}",
            f"- Stable: {format_stats(analysis['stable_header_intervals'])}",
            f"- Change-point interval indices: "
            f"`{analysis['anomaly_indices']}`",
            f"- Startup anomaly interval indices: "
            f"`{analysis['startup_anomaly_indices']}`",
            f"- Stable `bag_time-header` trend: "
            f"`{analysis['bag_minus_header_trend_us_per_s']:.6f} µs/s`",
            "",
        ])
    lines.extend(["## Camera–LiDAR exact integer-ns matching", ""])
    if longest_match is None:
        lines.append("No exact contiguous Camera–LiDAR timestamp run found.")
    else:
        cam_first, cam_last, lidar_first, lidar_last, count = longest_match
        lines.append(
            f"Longest exact run: Camera index **{cam_first}…{cam_last}** "
            f"equals LiDAR index **{lidar_first}…{lidar_last}** "
            f"for **{count}** consecutive messages."
        )
    lines.extend(["", "## Interpretation", ""])
    if camera and camera["startup_anomaly_indices"]:
        index = camera["startup_anomaly_indices"][0]
        lines.append(
            f"- Camera startup change point is interval {index}→{index + 1}; "
            f"the interval is "
            f"`{camera['full_header_intervals']['max_ms'] / 1000.0:.9f} s`."
        )
    if camera:
        lines.append(
            f"- Stable camera mean interval is "
            f"`{camera['stable_header_intervals']['mean_ms']:.9f} ms`."
        )
        lines.append(
            f"- The full endpoint estimate "
            f"(`{camera['full_endpoint_ppm']:.3f} ppm`) is not a sustained "
            "clock-rate estimate because it includes the startup change point."
        )
    if longest_match and longest_match[4] > 100:
        lines.append(
            "- The long exact integer-ns run shows Camera and LiDAR headers "
            "share values; it does not prove physical trigger association."
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze_bag(
    bag_path: Path, output_prefix: Path,
    topics: Sequence[str] = DEFAULT_TOPICS,
) -> Tuple[List[Dict[str, object]], Optional[Tuple[int, int, int, int, int]]]:
    series_by_topic = read_bag(bag_path, topics)
    analyses = [
        analyze_series(series_by_topic[topic])
        for topic in topics
    ]
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    write_summary_csv(
        output_prefix.with_name(output_prefix.name + "_summary.csv"),
        analyses)
    analyses_by_topic = {
        analysis["topic"]: analysis for analysis in analyses
    }
    write_intervals_csv(
        output_prefix.with_name(output_prefix.name + "_intervals.csv"),
        series_by_topic.values(), analyses_by_topic)

    camera = series_by_topic.get("/left_camera/image")
    lidar = series_by_topic.get("/livox/lidar")
    matches: List[Tuple[int, int, int]] = []
    longest_match = None
    if camera is not None and lidar is not None:
        matches, longest_match = exact_timestamp_matches(
            camera.header_ns, lidar.header_ns)
    write_matches_csv(
        output_prefix.with_name(output_prefix.name + "_matches.csv"),
        matches)
    write_markdown(
        output_prefix.with_suffix(".md"), bag_path, analyses,
        longest_match)
    return analyses, longest_match


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="input ROS1 bag (read-only)")
    parser.add_argument(
        "--output-prefix", type=Path, required=True,
        help="new output path prefix for Markdown and CSV files")
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error(f"bag does not exist: {args.bag}")
    analyses, longest_match = analyze_bag(args.bag, args.output_prefix)
    for analysis in analyses:
        print(
            f"{analysis['topic']}: count={analysis['count']} "
            f"startup={analysis['startup_anomaly_indices']} "
            f"stable_mean_ms="
            f"{analysis['stable_header_intervals']['mean_ms']:.9f} "
            f"stable_robust_ppm={analysis['stable_robust_ppm']:.6f}"
        )
    if longest_match:
        print(
            "exact Camera/LiDAR run: "
            f"camera {longest_match[0]}..{longest_match[1]} = "
            f"lidar {longest_match[2]}..{longest_match[3]} "
            f"({longest_match[4]} messages)"
        )
    print(f"reports: {args.output_prefix}.md and CSV companions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
