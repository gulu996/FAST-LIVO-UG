#!/usr/bin/env python3
"""Stream a petrochemical/Tiaozhanbei dataset into the audited ROS1 schema."""

import argparse
import csv
import gzip
import heapq
import json
import math
import os
import statistics
import time
from pathlib import Path

import numpy as np
import pandas as pd
import rosbag
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
from std_msgs.msg import Header


NSEC = 1_000_000_000
TOPICS = {
    "lidar": "/tiaozhanbei/lidar",
    "imu": "/tiaozhanbei/imu",
    "camera": "/tiaozhanbei/camera/left/image/compressed",
    "gnss": "/tiaozhanbei/gnss/pose",
}
OUTPUT_TYPES = {
    TOPICS["lidar"]: "sensor_msgs/PointCloud2",
    TOPICS["imu"]: "sensor_msgs/Imu",
    TOPICS["camera"]: "sensor_msgs/CompressedImage",
    TOPICS["gnss"]: "geometry_msgs/PoseWithCovarianceStamped",
}
PRIORITY = {"lidar": 0, "imu": 1, "camera": 2, "gnss": 3}
IMU_COLUMNS = (
    "timestamp_ns",
    "angular_velocity_x_rad_s",
    "angular_velocity_y_rad_s",
    "angular_velocity_z_rad_s",
    "linear_acceleration_x_m_s2",
    "linear_acceleration_y_m_s2",
    "linear_acceleration_z_m_s2",
)
LIDAR_COLUMNS = (
    "frame_timestamp_ns",
    "frame_index",
    "point_index",
    "x_m",
    "y_m",
    "z_m",
    "intensity",
    "ring",
    "point_time_s",
)
GNSS_COLUMNS = (
    "timestamp_ns",
    "east_m",
    "north_m",
    "up_m",
    "cov_ee",
    "cov_en",
    "cov_eu",
    "cov_nn",
    "cov_nu",
    "cov_uu",
    "quality",
    "num_satellites",
)
POINT_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "intensity", "time", "ring"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<f8", "<u2"],
        "offsets": [0, 4, 8, 12, 16, 24],
        "itemsize": 26,
    }
)
POINT_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
    PointField("time", 16, PointField.FLOAT64, 1),
    PointField("ring", 24, PointField.UINT16, 1),
]


def format_duration(seconds):
    if seconds is None or not math.isfinite(seconds) or seconds < 0:
        return "--:--"
    seconds = int(seconds + 0.5)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    if hours:
        return "{:d}:{:02d}:{:02d}".format(hours, minutes, seconds)
    return "{:02d}:{:02d}".format(minutes, seconds)


class ProgressReporter:
    """Rate-limited line progress that remains visible through pipes and log files."""

    def __init__(self, label, total, unit="items", interval_s=1.0):
        self.label = label
        self.total = max(0, int(total))
        self.unit = unit
        self.interval_s = interval_s
        self.started = time.monotonic()
        self.last_printed = self.started
        self.last_value_printed = None
        self.current = 0
        self._print(0)

    def _print(self, current):
        elapsed = max(time.monotonic() - self.started, 0.0)
        fraction = min(1.0, current / self.total) if self.total else 0.0
        complete = min(24, int(fraction * 24))
        rate = current / elapsed if elapsed >= 0.1 else 0.0
        eta = (self.total - current) / rate if rate > 0.0 and self.total else None
        print(
            "[{label}] [{bar:<24}] {percent:6.2f}%  {current:,}/{total:,} {unit}  "
            "elapsed={elapsed}  rate={rate:,.1f}/s  ETA={eta}".format(
                label=self.label,
                bar="#" * complete,
                percent=fraction * 100.0,
                current=current,
                total=self.total,
                unit=self.unit,
                elapsed=format_duration(elapsed),
                rate=rate,
                eta=format_duration(eta),
            ),
            flush=True,
        )
        self.last_value_printed = current

    def update(self, current, force=False):
        current = max(self.current, min(int(current), self.total) if self.total else int(current))
        self.current = current
        if force and current == self.last_value_printed:
            return
        now = time.monotonic()
        if force or (self.total and current >= self.total) or now - self.last_printed >= self.interval_s:
            self._print(current)
            self.last_printed = now

    def finish(self):
        self.update(self.total if self.total else self.current, force=True)


def stamp_from_ns(timestamp_ns):
    timestamp_ns = int(timestamp_ns)
    return rospy.Time(timestamp_ns // NSEC, timestamp_ns % NSEC)


def percentile(values, fraction):
    if not values:
        return 0
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


class TimestampAudit:
    def __init__(self, label):
        self.label = label
        self.values = []
        self.duplicate = 0
        self.nonmonotonic = 0

    def add(self, timestamp_ns):
        timestamp_ns = int(timestamp_ns)
        if timestamp_ns <= 0:
            raise ValueError("{} has a non-positive timestamp: {}".format(self.label, timestamp_ns))
        if self.values:
            delta = timestamp_ns - self.values[-1]
            if delta == 0:
                self.duplicate += 1
            elif delta < 0:
                self.nonmonotonic += 1
        self.values.append(timestamp_ns)

    def require_strict(self):
        if self.duplicate or self.nonmonotonic:
            raise ValueError(
                "{} timestamps are not strictly increasing: duplicate={} nonmonotonic={}".format(
                    self.label, self.duplicate, self.nonmonotonic
                )
            )

    def summary(self):
        if not self.values:
            return {
                "count": 0,
                "first_ns": None,
                "last_ns": None,
                "duration_s": 0.0,
                "mean_hz": 0.0,
                "median_dt_s": 0.0,
                "p95_dt_s": 0.0,
                "max_dt_s": 0.0,
                "duplicate_timestamps": self.duplicate,
                "nonmonotonic_timestamps": self.nonmonotonic,
            }
        deltas = [b - a for a, b in zip(self.values, self.values[1:]) if b > a]
        duration_ns = self.values[-1] - self.values[0]
        return {
            "count": len(self.values),
            "first_ns": self.values[0],
            "last_ns": self.values[-1],
            "duration_s": duration_ns / NSEC,
            "mean_hz": ((len(self.values) - 1) * NSEC / duration_ns) if duration_ns > 0 else 0.0,
            "median_dt_s": (statistics.median(deltas) / NSEC) if deltas else 0.0,
            "p95_dt_s": percentile(deltas, 0.95) / NSEC,
            "max_dt_s": (max(deltas) / NSEC) if deltas else 0.0,
            "duplicate_timestamps": self.duplicate,
            "nonmonotonic_timestamps": self.nonmonotonic,
        }


def read_header(path, compressed=False):
    opener = gzip.open if compressed else open
    with opener(path, "rt", encoding="utf-8-sig", errors="strict", newline="") as stream:
        return tuple(next(csv.reader(stream)))


def locate_inputs(dataset, gnss_solution, skip_gnss):
    lidar_candidates = [
        path for path in (dataset / "lidar" / "points.csv", dataset / "lidar" / "points.csv.gz")
        if path.is_file()
    ]
    if len(lidar_candidates) != 1:
        raise RuntimeError(
            "expected exactly one LiDAR CSV (points.csv or points.csv.gz), found: {}".format(
                [str(path) for path in lidar_candidates]
            )
        )
    paths = {
        "imu": dataset / "imu" / "imu.csv",
        "lidar": lidar_candidates[0],
        "camera": dataset / "camera" / "left",
    }
    if not skip_gnss:
        if gnss_solution is None:
            raise RuntimeError("--gnss-solution is required unless --skip-gnss is used")
        paths["gnss"] = gnss_solution
    missing = [str(path) for path in paths.values() if not path.exists()]
    if missing:
        raise FileNotFoundError("required inputs are missing:\n  " + "\n  ".join(missing))
    imu_header = read_header(paths["imu"])
    missing_imu_columns = [name for name in IMU_COLUMNS if name not in imu_header]
    if missing_imu_columns:
        raise ValueError(
            "IMU CSV is missing columns {} in {}".format(missing_imu_columns, paths["imu"])
        )
    if read_header(paths["lidar"], paths["lidar"].suffix == ".gz") != LIDAR_COLUMNS:
        raise ValueError("unexpected LiDAR header in {}".format(paths["lidar"]))
    if not skip_gnss:
        gnss_header = read_header(paths["gnss"])
        missing_columns = [name for name in GNSS_COLUMNS if name not in gnss_header]
        if missing_columns:
            raise ValueError("GNSS solution is missing columns: {}".format(missing_columns))
    return paths


def make_imu(values):
    timestamp_ns, angular_velocity, linear_acceleration = values
    msg = Imu()
    msg.header = Header(stamp=stamp_from_ns(timestamp_ns), frame_id="imu0")
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = angular_velocity
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = linear_acceleration
    return msg


def imu_events(path, audit, counters):
    previous = None
    reader = pd.read_csv(
        str(path), usecols=list(IMU_COLUMNS), chunksize=100_000,
        dtype={"timestamp_ns": np.int64},
    )
    for chunk in reader:
        for row in chunk.itertuples(index=False, name=None):
            timestamp_ns = int(row[0])
            values = tuple(float(value) for value in row[1:])
            if not all(math.isfinite(value) for value in values):
                raise ValueError("IMU contains non-finite data at timestamp {}".format(timestamp_ns))
            audit.add(timestamp_ns)
            if previous is not None and timestamp_ns <= previous:
                audit.require_strict()
            previous = timestamp_ns
            counters["imu_rows"] += 1
            if any(value != 0.0 for value in values[:3]):
                counters["imu_angular_velocity_nonzero"] += 1
            if any(value != 0.0 for value in values[3:]):
                counters["imu_linear_acceleration_nonzero"] += 1
            yield timestamp_ns, (timestamp_ns, values[:3], values[3:])


def jpeg_has_markers(path):
    try:
        with path.open("rb") as stream:
            if stream.read(2) != b"\xff\xd8":
                return False
            stream.seek(-2, os.SEEK_END)
            return stream.read(2) == b"\xff\xd9"
    except (OSError, ValueError):
        return False


def camera_events(directory, audit, counters):
    images = []
    for path in directory.iterdir():
        if not path.is_file() or path.suffix.lower() not in (".jpg", ".jpeg"):
            continue
        try:
            timestamp_ns = int(path.stem)
        except ValueError as exc:
            raise ValueError("camera filename is not a nanosecond timestamp: {}".format(path)) from exc
        images.append((timestamp_ns, path))
    images.sort(key=lambda item: item[0])
    previous = None
    for timestamp_ns, path in images:
        audit.add(timestamp_ns)
        if previous is not None and timestamp_ns <= previous:
            audit.require_strict()
        previous = timestamp_ns
        if not jpeg_has_markers(path):
            raise ValueError("camera file does not have complete JPEG markers: {}".format(path))
        counters["camera_jpeg_marker_valid"] += 1
        yield timestamp_ns, path


def make_camera(timestamp_ns, path):
    msg = CompressedImage()
    msg.header = Header(stamp=stamp_from_ns(timestamp_ns), frame_id="cam0")
    msg.format = "jpeg"
    msg.data = path.read_bytes()
    return msg


def gnss_events(path, audit, counters):
    previous = None
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            counters["gnss_rows"] += 1
            try:
                timestamp_ns = int(row["timestamp_ns"])
                position = tuple(float(row[name]) for name in ("east_m", "north_m", "up_m"))
                covariance = tuple(
                    float(row[name])
                    for name in ("cov_ee", "cov_en", "cov_eu", "cov_nn", "cov_nu", "cov_uu")
                )
                quality = int(row["quality"])
                satellites = int(row["num_satellites"])
            except (TypeError, ValueError):
                counters["gnss_rejected_malformed"] += 1
                continue
            finite = all(math.isfinite(value) for value in position + covariance)
            if not finite or min(covariance[0], covariance[3], covariance[5]) <= 0.0:
                counters["gnss_rejected_covariance_or_finite"] += 1
                continue
            if quality > 2 or satellites < 6:
                counters["gnss_rejected_quality_or_satellites"] += 1
                continue
            audit.add(timestamp_ns)
            if previous is not None and timestamp_ns <= previous:
                audit.require_strict()
            previous = timestamp_ns
            counters["gnss_valid"] += 1
            yield timestamp_ns, (timestamp_ns, position, covariance)


def make_gnss(values):
    timestamp_ns, position, source_covariance = values
    msg = PoseWithCovarianceStamped()
    msg.header = Header(stamp=stamp_from_ns(timestamp_ns), frame_id="gnss_enu")
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = position
    msg.pose.pose.orientation.w = 1.0
    covariance = [0.0] * 36
    covariance[0] = source_covariance[0]
    covariance[1] = covariance[6] = source_covariance[1]
    covariance[2] = covariance[12] = source_covariance[2]
    covariance[7] = source_covariance[3]
    covariance[8] = covariance[13] = source_covariance[4]
    covariance[14] = source_covariance[5]
    covariance[21] = covariance[28] = covariance[35] = 1e6
    msg.pose.covariance = covariance
    return msg


def make_cloud(frame, audit, counters, frame_summaries, build_message):
    frame_ids = frame["frame_index"].unique()
    timestamps = frame["frame_timestamp_ns"].unique()
    if len(frame_ids) != 1 or len(timestamps) != 1:
        raise ValueError("LiDAR frame grouping changed inside one frame")
    timestamp_ns = int(timestamps[0])
    point_indices = frame["point_index"].to_numpy(dtype=np.int64, copy=False)
    if len(point_indices) and (
        point_indices[0] != 0 or np.any(np.diff(point_indices) != 1)
    ):
        raise ValueError("LiDAR point_index is not contiguous in frame {}".format(frame_ids[0]))
    x = frame["x_m"].to_numpy(copy=False)
    y = frame["y_m"].to_numpy(copy=False)
    z = frame["z_m"].to_numpy(copy=False)
    intensity = frame["intensity"].to_numpy(copy=False)
    point_time = frame["point_time_s"].to_numpy(copy=False)
    ring = frame["ring"].to_numpy(copy=False)
    valid = (
        np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity)
        & np.isfinite(point_time) & (point_time >= 0.0) & (ring >= 1) & (ring <= 16)
    )
    selected = frame.loc[valid].sort_values("point_time_s", kind="stable")
    if selected.empty:
        raise ValueError("LiDAR frame {} contains no valid points".format(frame_ids[0]))
    selected_times = selected["point_time_s"].to_numpy(dtype=np.float64, copy=False)
    audit.add(timestamp_ns)
    audit.require_strict()
    counters["lidar_input_rows"] += len(frame)
    counters["lidar_valid_points"] += len(selected)
    counters["lidar_filtered_points"] += len(frame) - len(selected)
    frame_summaries.append(
        {
            "frame_index": int(frame_ids[0]),
            "timestamp_ns": timestamp_ns,
            "input_points": int(len(frame)),
            "valid_points": int(len(selected)),
            "point_time_min_s": float(selected_times.min()),
            "point_time_max_s": float(selected_times.max()),
        }
    )
    if not build_message:
        return timestamp_ns, None
    points = np.empty(len(selected), dtype=POINT_DTYPE)
    points["x"] = selected["x_m"].to_numpy(dtype=np.float32)
    points["y"] = selected["y_m"].to_numpy(dtype=np.float32)
    points["z"] = selected["z_m"].to_numpy(dtype=np.float32)
    points["intensity"] = selected["intensity"].to_numpy(dtype=np.float32)
    points["time"] = selected_times
    points["ring"] = selected["ring"].to_numpy(dtype=np.uint16) - 1
    msg = PointCloud2()
    msg.header = Header(stamp=stamp_from_ns(timestamp_ns), frame_id="lidar0")
    msg.height = 1
    msg.width = len(points)
    msg.fields = POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_DTYPE.itemsize
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return timestamp_ns, msg


def lidar_events(path, audit, counters, frame_summaries, build_message, progress_label):
    total_bytes = path.stat().st_size
    progress = ProgressReporter(progress_label, total_bytes, "bytes")
    raw = path.open("rb")
    source = gzip.GzipFile(fileobj=raw, mode="rb") if path.suffix == ".gz" else raw
    try:
        reader = pd.read_csv(
            source,
            usecols=list(LIDAR_COLUMNS),
            chunksize=500_000,
            dtype={
                "frame_timestamp_ns": np.int64,
                "frame_index": np.int64,
                "point_index": np.int64,
                "x_m": np.float32,
                "y_m": np.float32,
                "z_m": np.float32,
                "intensity": np.float32,
                "ring": np.int16,
                "point_time_s": np.float64,
            },
        )
        carry = None
        for chunk in reader:
            progress.update(raw.tell())
            if carry is not None:
                chunk = pd.concat((carry, chunk), ignore_index=True)
                carry = None
            last_frame = chunk["frame_index"].iloc[-1]
            complete = chunk["frame_index"] != last_frame
            ready = chunk.loc[complete]
            carry = chunk.loc[~complete].copy()
            for _frame_index, frame in ready.groupby("frame_index", sort=False):
                yield make_cloud(frame, audit, counters, frame_summaries, build_message)
        if carry is not None and not carry.empty:
            for _frame_index, frame in carry.groupby("frame_index", sort=False):
                yield make_cloud(frame, audit, counters, frame_summaries, build_message)
        progress.finish()
    finally:
        source.close()
        if not raw.closed:
            raw.close()


def write_summary(path, dataset, paths, audits, counters, frames, dry_run):
    stream_summaries = {name: audit.summary() for name, audit in audits.items()}
    all_first = [item["first_ns"] for item in stream_summaries.values() if item["first_ns"] is not None]
    all_last = [item["last_ns"] for item in stream_summaries.values() if item["last_ns"] is not None]
    lidar_samples = []
    if frames:
        for index in sorted({0, len(frames) // 2, len(frames) - 1}):
            lidar_samples.append(frames[index])
    document = {
        "dataset": str(dataset),
        "dry_run": dry_run,
        "inputs": {name: str(value) for name, value in paths.items()},
        "topics": OUTPUT_TYPES,
        "streams": stream_summaries,
        "global": {
            "first_ns": min(all_first) if all_first else None,
            "last_ns": max(all_last) if all_last else None,
            "duration_s": ((max(all_last) - min(all_first)) / NSEC) if all_first else 0.0,
        },
        "counters": dict(sorted(counters.items())),
        "lidar_samples": lidar_samples,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return document


def convert(args):
    dataset = args.dataset.expanduser().resolve()
    gnss_solution = args.gnss_solution.expanduser().resolve() if args.gnss_solution else None
    paths = locate_inputs(dataset, gnss_solution, args.skip_gnss)
    audits = {name: TimestampAudit(name) for name in ("lidar", "imu", "camera", "gnss")}
    counters = {
        "imu_rows": 0,
        "imu_angular_velocity_nonzero": 0,
        "imu_linear_acceleration_nonzero": 0,
        "camera_jpeg_marker_valid": 0,
        "lidar_input_rows": 0,
        "lidar_valid_points": 0,
        "lidar_filtered_points": 0,
        "gnss_rows": 0,
        "gnss_valid": 0,
        "gnss_rejected_malformed": 0,
        "gnss_rejected_covariance_or_finite": 0,
        "gnss_rejected_quality_or_satellites": 0,
    }
    frames = []
    progress_prefix = args.progress_label or ("PREFLIGHT" if args.dry_run else "BUILD")
    iterators = {
        "lidar": lidar_events(
            paths["lidar"], audits["lidar"], counters, frames, not args.dry_run,
            "{} LIDAR".format(progress_prefix),
        ),
        "imu": imu_events(paths["imu"], audits["imu"], counters),
        "camera": camera_events(paths["camera"], audits["camera"], counters),
    }
    if not args.skip_gnss:
        iterators["gnss"] = gnss_events(paths["gnss"], audits["gnss"], counters)

    output = args.output.expanduser().resolve() if args.output else None
    partial = Path(str(output) + ".partial") if output else None
    if not args.dry_run:
        if output is None:
            raise ValueError("--output is required unless --dry-run is used")
        if output.exists() and not args.overwrite:
            raise FileExistsError("output exists; pass --overwrite: {}".format(output))
        if partial.exists() and not args.overwrite:
            raise FileExistsError("partial output exists; pass --overwrite: {}".format(partial))
        if partial.exists():
            partial.unlink()
        output.parent.mkdir(parents=True, exist_ok=True)

    heap = []
    for source_id, (sensor, iterator) in enumerate(iterators.items()):
        try:
            timestamp_ns, payload = next(iterator)
        except StopIteration:
            continue
        heapq.heappush(heap, (timestamp_ns, PRIORITY[sensor], source_id, sensor, payload, iterator))
    if not heap:
        raise RuntimeError("dataset contains no valid sensor events")

    bag = None
    last_global_ns = None
    written = {name: 0 for name in iterators}
    event_progress = (
        ProgressReporter("{} EVENTS".format(progress_prefix), args.expected_events, "events")
        if args.expected_events else None
    )
    try:
        if not args.dry_run:
            bag = rosbag.Bag(str(partial), "w", compression=args.compression, chunk_threshold=4 * 1024 * 1024)
        while heap:
            timestamp_ns, _priority, source_id, sensor, payload, iterator = heapq.heappop(heap)
            if last_global_ns is not None and timestamp_ns < last_global_ns:
                raise RuntimeError("global timestamp merge produced an out-of-order event")
            last_global_ns = timestamp_ns
            if bag is not None:
                if sensor == "lidar":
                    msg = payload
                elif sensor == "imu":
                    msg = make_imu(payload)
                elif sensor == "camera":
                    msg = make_camera(timestamp_ns, payload)
                else:
                    msg = make_gnss(payload)
                if msg.header.stamp != stamp_from_ns(timestamp_ns):
                    raise RuntimeError("message header timestamp differs from source timestamp")
                bag.write(TOPICS[sensor], msg, t=msg.header.stamp)
            written[sensor] += 1
            total_written = sum(written.values())
            if event_progress:
                event_progress.update(total_written)
            elif total_written % 5000 == 0:
                print("processed {:,} events: {}".format(total_written, written), flush=True)
            try:
                next_timestamp_ns, next_payload = next(iterator)
            except StopIteration:
                continue
            heapq.heappush(
                heap,
                (next_timestamp_ns, PRIORITY[sensor], source_id, sensor, next_payload, iterator),
            )
    finally:
        if bag is not None:
            bag.close()
    if event_progress:
        event_progress.finish()

    for name, audit in audits.items():
        if name in iterators:
            audit.require_strict()
            if not audit.values:
                raise RuntimeError("{} stream contains no valid events".format(name))
    if counters["imu_angular_velocity_nonzero"] == 0 or counters["imu_linear_acceleration_nonzero"] == 0:
        raise RuntimeError("IMU parsed as all zero")
    summary = write_summary(args.summary_json, dataset, paths, audits, counters, frames, args.dry_run)
    summary["written"] = written
    with args.summary_json.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    if not args.dry_run:
        os.replace(str(partial), str(output))
        print("bag complete: {}".format(output), flush=True)
    print("counts: {}".format(written), flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--gnss-solution", type=Path)
    parser.add_argument("--skip-gnss", action="store_true", help="preflight only; omit GNSS")
    parser.add_argument("--dry-run", action="store_true", help="fully parse inputs without writing a bag")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    parser.add_argument("--summary-json", required=True, type=Path)
    parser.add_argument("--expected-events", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--progress-label", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.skip_gnss and not args.dry_run:
        parser.error("--skip-gnss is only valid with --dry-run")
    if not args.dry_run and args.output is None:
        parser.error("--output is required unless --dry-run is used")
    if args.expected_events < 0:
        parser.error("--expected-events must be non-negative")
    args.summary_json = args.summary_json.expanduser().resolve()
    return args


if __name__ == "__main__":
    convert(parse_args())
