#!/usr/bin/env python3
"""Build the petrochemical ROS1 bag with compressed left images and PPK GNSS."""

import argparse
import csv
import gzip
import heapq
import math
import os
import struct
import sys
from collections import Counter, defaultdict
from decimal import Decimal, InvalidOperation
from pathlib import Path

import rosbag
import rospy
import yaml
from gnss_serial_driver.msg import GnssPvtStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
from std_msgs.msg import Header


IMU_HEADER = (
    "timestamp_ns",
    "sequence",
    "frame_id",
    "angular_velocity_x_rad_s",
    "angular_velocity_y_rad_s",
    "angular_velocity_z_rad_s",
    "linear_acceleration_x_m_s2",
    "linear_acceleration_y_m_s2",
    "linear_acceleration_z_m_s2",
    "magnetic_field_x_t",
    "magnetic_field_y_t",
    "magnetic_field_z_t",
)
LIDAR_HEADER = (
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
PPK_HEADER = (
    "epoch_index",
    "gpst_iso",
    "utc_iso",
    "unix_utc_s",
    "unix_utc_ms",
    "gpst_week",
    "gpst_sow",
    "bds_week",
    "bds_sow",
    "segment_id",
    "time_gap_s",
    "gap_after_previous",
    "data_valid",
    "latitude_deg",
    "longitude_deg",
    "ellipsoidal_height_m",
    "ecef_x_m",
    "ecef_y_m",
    "ecef_z_m",
    "north_m",
    "east_m",
    "up_m",
    "quality_code",
    "quality_name",
    "satellites",
    "sd_n_m",
    "sd_e_m",
    "sd_u_m",
    "sd_en_m",
    "sd_nu_m",
    "sd_ue_m",
    "differential_age_s",
    "ar_ratio",
    "antenna_phase_center",
    "lever_arm_applied",
)
TOPICS = {
    "lidar": "/petrochemical/lidar",
    "imu": "/petrochemical/imu",
    "left": "/petrochemical/camera/left/image_raw/compressed",
    "gnss_pvt": "/gnss/pvt_local",
    "gnss_enu": "/gnss/enu_odom",
}
OUTPUT_TYPES = {
    TOPICS["lidar"]: "sensor_msgs/PointCloud2",
    TOPICS["imu"]: "sensor_msgs/Imu",
    TOPICS["left"]: "sensor_msgs/CompressedImage",
    TOPICS["gnss_pvt"]: "gnss_serial_driver/GnssPvtStamped",
    TOPICS["gnss_enu"]: "nav_msgs/Odometry",
}
PRIORITY = {"imu": 0, "gnss": 1, "left": 2, "lidar": 3}
GPS_EPOCH_UNIX_S = 315964800
GPS_UTC_LEAP_SECONDS = 18
LOCAL_ONLY_TIME_STATE = 1
POINT_STRUCT = struct.Struct("<ffffdH")
POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="offset_time", offset=16, datatype=PointField.FLOAT64, count=1),
    PointField(name="ring", offset=24, datatype=PointField.UINT16, count=1),
]
DEFAULT_WEIGHT_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "config"
    / "petrochemical_site"
    / "rtk_fixed_lag_backend.yaml"
)


def load_ppk_covariance_config(path):
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    config = document.get("ppk_covariance")
    if not isinstance(config, dict):
        raise ValueError(f"missing ppk_covariance mapping in {path}")
    try:
        reference_ratio = float(config["float_ar_reference_ratio"])
        maximum_scale = float(config["float_ar_sigma_scale_max"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid PPK covariance parameters in {path}") from exc
    if not math.isfinite(reference_ratio) or reference_ratio <= 0.0:
        raise ValueError("ppk_covariance/float_ar_reference_ratio must be positive")
    if not math.isfinite(maximum_scale) or maximum_scale < 1.0:
        raise ValueError("ppk_covariance/float_ar_sigma_scale_max must be >= 1")
    return {
        "float_ar_reference_ratio": reference_ratio,
        "float_ar_sigma_scale_max": maximum_scale,
    }


def ppk_ar_sigma_scale(quality_code, ar_ratio, config):
    if quality_code != 2:
        return 1.0
    if not math.isfinite(ar_ratio) or ar_ratio <= 0.0:
        return config["float_ar_sigma_scale_max"]
    return min(
        config["float_ar_sigma_scale_max"],
        max(1.0, math.sqrt(config["float_ar_reference_ratio"] / ar_ratio)),
    )


def ns_to_time(timestamp_ns):
    return rospy.Time(timestamp_ns // 1_000_000_000, timestamp_ns % 1_000_000_000)


def seconds_to_ns(value, label, allow_zero=True):
    try:
        decimal = Decimal(value)
    except InvalidOperation as exc:
        raise argparse.ArgumentTypeError(f"{label} must be a decimal number") from exc
    if decimal < 0 or (not allow_zero and decimal == 0):
        relation = "non-negative" if allow_zero else "positive"
        raise argparse.ArgumentTypeError(f"{label} must be {relation}")
    return int(decimal * Decimal(1_000_000_000))


def require_dataset_layout(dataset):
    paths = {
        "imu": dataset / "imu" / "imu.csv",
        "lidar": dataset / "lidar" / "points.csv.gz",
        "left": dataset / "camera" / "left",
        "ppk": dataset / "gnss" / "gnss_ppk_tool_bundle" / "gnss_ppk_tool" / "output" / "txt" / "rover_ppk_solution_full.txt",
        "stereo": dataset / "camera" / "stereo.yaml",
        "imu_calibration": dataset / "imu" / "imu.yaml",
        "extrinsics": dataset / "sensor_extrinsics.yaml",
    }
    missing = [str(path) for path in paths.values() if not path.exists()]
    if missing:
        raise FileNotFoundError("required dataset paths are missing:\n  " + "\n  ".join(missing))
    return paths


def note_timestamp(stats, sensor, timestamp_ns, previous_ns, large_gap_ns):
    if timestamp_ns == 0:
        stats[f"{sensor}_zero_timestamp"] += 1
    if previous_ns is not None:
        delta = timestamp_ns - previous_ns
        if delta == 0:
            stats[f"{sensor}_duplicate_timestamp"] += 1
        elif delta < 0:
            stats[f"{sensor}_nonmonotonic_timestamp"] += 1
        elif delta > large_gap_ns:
            stats[f"{sensor}_large_gap"] += 1
            stats[f"{sensor}_max_gap_ns"] = max(stats[f"{sensor}_max_gap_ns"], delta)


def imu_events(path, stats):
    previous_ns = None
    previous_sequence = None
    with path.open("r", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != IMU_HEADER:
            raise ValueError(f"unexpected IMU header in {path}: {reader.fieldnames}")
        for row in reader:
            stats["imu_rows"] += 1
            try:
                timestamp_ns = int(row["timestamp_ns"])
                sequence = int(row["sequence"])
                angular_velocity = tuple(float(row[name]) for name in IMU_HEADER[3:6])
                linear_acceleration = tuple(float(row[name]) for name in IMU_HEADER[6:9])
            except (TypeError, ValueError):
                stats["imu_malformed"] += 1
                continue
            values = angular_velocity + linear_acceleration
            if timestamp_ns <= 0 or not all(math.isfinite(value) for value in values):
                stats["imu_invalid"] += 1
                continue
            note_timestamp(stats, "imu", timestamp_ns, previous_ns, 10_000_000)
            if previous_sequence is not None and sequence != previous_sequence + 1:
                stats["imu_sequence_gap"] += 1
            previous_ns = timestamp_ns
            previous_sequence = sequence
            stats["imu_frame_ids_" + row["frame_id"]] += 1
            yield timestamp_ns, "imu", (sequence, row["frame_id"], angular_velocity, linear_acceleration)


def camera_events(directory, sensor, stats):
    files = []
    for path in directory.iterdir():
        if not path.is_file() or path.suffix.lower() != ".jpg":
            continue
        try:
            timestamp_ns = int(path.stem)
        except ValueError:
            stats[f"{sensor}_bad_filename"] += 1
            continue
        files.append((timestamp_ns, path))
    files.sort(key=lambda item: item[0])
    stats[f"{sensor}_files"] = len(files)
    previous_ns = None
    for timestamp_ns, path in files:
        if timestamp_ns <= 0:
            stats[f"{sensor}_zero_timestamp"] += 1
            continue
        note_timestamp(stats, sensor, timestamp_ns, previous_ns, 50_000_000)
        previous_ns = timestamp_ns
        yield timestamp_ns, sensor, path


def ppk_events(path, stats, covariance_config):
    previous_ns = None
    previous_epoch = None
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        if tuple(reader.fieldnames or ()) != PPK_HEADER:
            raise ValueError(f"unexpected PPK header in {path}: {reader.fieldnames}")
        for row in reader:
            stats["gnss_rows"] += 1
            try:
                epoch_index = int(row["epoch_index"])
                timestamp_ms = int(row["unix_utc_ms"])
                timestamp_ns = timestamp_ms * 1_000_000
                gpst_week = int(row["gpst_week"])
                gpst_sow = float(row["gpst_sow"])
                segment_id = int(row["segment_id"])
                quality_code = int(row["quality_code"])
                satellites = int(row["satellites"])
                latitude = float(row["latitude_deg"])
                longitude = float(row["longitude_deg"])
                altitude = float(row["ellipsoidal_height_m"])
                north = float(row["north_m"])
                east = float(row["east_m"])
                up = float(row["up_m"])
                sd_n = float(row["sd_n_m"])
                sd_e = float(row["sd_e_m"])
                sd_u = float(row["sd_u_m"])
                ar_ratio = float(row["ar_ratio"])
                covariance_terms = tuple(float(row[name]) for name in ("sd_en_m", "sd_nu_m", "sd_ue_m"))
                unix_s_ms = int(Decimal(row["unix_utc_s"]) * 1000)
                gpst_utc_ms = int(
                    (Decimal(GPS_EPOCH_UNIX_S + gpst_week * 604800 - GPS_UTC_LEAP_SECONDS)
                     + Decimal(row["gpst_sow"]))
                    * 1000
                )
            except (InvalidOperation, TypeError, ValueError):
                stats["gnss_malformed"] += 1
                continue
            values = (
                gpst_sow, latitude, longitude, altitude, north, east, up,
                sd_n, sd_e, sd_u, ar_ratio,
            ) + covariance_terms
            if (
                timestamp_ns <= 0
                or row["data_valid"] != "1"
                or quality_code not in (1, 2)
                or not 0 <= satellites <= 255
                or not all(math.isfinite(value) for value in values)
                or not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0)
                or min(sd_n, sd_e, sd_u) <= 0.0
                or unix_s_ms != timestamp_ms
                or gpst_utc_ms != timestamp_ms
                or row["antenna_phase_center"] != "1"
                or row["lever_arm_applied"] != "0"
            ):
                stats["gnss_invalid"] += 1
                continue
            note_timestamp(stats, "gnss", timestamp_ns, previous_ns, 500_000_000)
            if previous_epoch is not None and epoch_index != previous_epoch + 1:
                stats["gnss_epoch_gap"] += 1
            previous_ns = timestamp_ns
            previous_epoch = epoch_index
            stats["gnss_fixed" if quality_code == 1 else "gnss_float"] += 1
            ar_sigma_scale = ppk_ar_sigma_scale(
                quality_code, ar_ratio, covariance_config
            )
            if quality_code == 2:
                stats["gnss_float_ar_scale_milli_sum"] += int(round(ar_sigma_scale * 1000.0))
                stats["gnss_float_ar_scale_milli_max"] = max(
                    stats["gnss_float_ar_scale_milli_max"],
                    int(round(ar_sigma_scale * 1000.0)),
                )
            yield timestamp_ns, "gnss", (
                epoch_index,
                gpst_week,
                gpst_sow,
                segment_id,
                quality_code,
                satellites,
                latitude,
                longitude,
                altitude,
                east,
                north,
                up,
                sd_e,
                sd_n,
                sd_u,
                ar_ratio,
                ar_sigma_scale,
            )


def finish_lidar_frame(frame, build_payload, stats):
    if frame is None:
        return None
    if frame["valid_points"] == 0:
        stats["lidar_empty_frames"] += 1
        return None
    return (
        frame["timestamp_ns"],
        "lidar",
        (
            frame["frame_index"],
            frame["raw_points"],
            frame["valid_points"],
            frame["invalid_points"],
            frame["max_offset_ns"],
            frame["rings"],
            bytes(frame["data"]) if build_payload else None,
        ),
    )


def lidar_events(path, stats, build_payload):
    frame = None
    previous_frame_timestamp_ns = None
    previous_frame_index = None
    expected_point_index = 0
    with gzip.open(path, "rb") as stream:
        header = tuple((stream.readline()).decode("ascii").rstrip("\r\n").split(","))
        if header != LIDAR_HEADER:
            raise ValueError(f"unexpected LiDAR header in {path}: {header}")
        for raw_line in stream:
            columns = raw_line.rstrip(b"\r\n").split(b",")
            if len(columns) != len(LIDAR_HEADER):
                stats["lidar_malformed_rows"] += 1
                continue
            try:
                timestamp_ns = int(columns[0])
                frame_index = int(columns[1])
                point_index = int(columns[2])
            except ValueError:
                stats["lidar_malformed_rows"] += 1
                continue

            if frame is None or frame_index != frame["frame_index"]:
                completed = finish_lidar_frame(frame, build_payload, stats)
                if completed is not None:
                    yield completed
                note_timestamp(stats, "lidar", timestamp_ns, previous_frame_timestamp_ns, 250_000_000)
                if previous_frame_index is not None and frame_index != previous_frame_index + 1:
                    stats["lidar_frame_index_gap"] += 1
                previous_frame_timestamp_ns = timestamp_ns
                previous_frame_index = frame_index
                expected_point_index = 0
                frame = {
                    "timestamp_ns": timestamp_ns,
                    "frame_index": frame_index,
                    "raw_points": 0,
                    "valid_points": 0,
                    "invalid_points": 0,
                    "max_offset_ns": 0,
                    "rings": Counter(),
                    "data": bytearray(),
                }
            elif timestamp_ns != frame["timestamp_ns"]:
                stats["lidar_timestamp_changed_inside_frame"] += 1

            frame["raw_points"] += 1
            if point_index != expected_point_index:
                stats["lidar_point_index_gap"] += 1
            expected_point_index = point_index + 1
            try:
                x, y, z, intensity = (float(columns[index]) for index in range(3, 7))
                ring = int(columns[7])
                offset_time = float(columns[8])
            except ValueError:
                frame["invalid_points"] += 1
                continue
            if (
                not all(math.isfinite(value) for value in (x, y, z, intensity, offset_time))
                or offset_time < 0.0
                or not 0 <= ring <= 65535
            ):
                frame["invalid_points"] += 1
                continue
            frame["valid_points"] += 1
            frame["rings"][ring] += 1
            frame["max_offset_ns"] = max(frame["max_offset_ns"], int(offset_time * 1e9))
            if build_payload:
                frame["data"].extend(POINT_STRUCT.pack(x, y, z, intensity, offset_time, ring))

    completed = finish_lidar_frame(frame, build_payload, stats)
    if completed is not None:
        yield completed


def build_imu(payload, timestamp_ns, sequence):
    source_sequence, frame_id, angular_velocity, linear_acceleration = payload
    msg = Imu()
    msg.header = Header(seq=sequence, stamp=ns_to_time(timestamp_ns), frame_id=frame_id)
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = angular_velocity
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = linear_acceleration
    # The source sequence remains available in the CSV; ROS header.seq is contiguous after filtering.
    del source_sequence
    return msg


def build_lidar(payload, timestamp_ns, sequence):
    frame_index, _raw_count, point_count, _invalid_count, _max_offset_ns, _rings, data = payload
    msg = PointCloud2()
    msg.header = Header(seq=sequence, stamp=ns_to_time(timestamp_ns), frame_id="lidar0")
    msg.height = 1
    msg.width = point_count
    msg.fields = POINT_FIELDS
    msg.is_bigendian = False
    msg.point_step = POINT_STRUCT.size
    msg.row_step = msg.point_step * msg.width
    msg.data = data
    msg.is_dense = True
    del frame_index
    return msg


def build_compressed_image(path, timestamp_ns, sequence, stats, build_message):
    try:
        encoded = path.read_bytes()
    except OSError:
        encoded = b""
    if len(encoded) < 4 or not encoded.startswith(b"\xff\xd8") or not encoded.endswith(b"\xff\xd9"):
        stats["left_corrupted_image"] += 1
        return None
    stats["left_validated_images"] += 1
    if not build_message:
        return True
    msg = CompressedImage()
    msg.header = Header(seq=sequence, stamp=ns_to_time(timestamp_ns), frame_id="cam0")
    msg.format = "jpeg"
    msg.data = encoded
    return msg


def build_gnss(payload, timestamp_ns, sequence):
    (
        epoch_index,
        gpst_week,
        gpst_sow,
        _segment_id,
        quality_code,
        satellites,
        latitude,
        longitude,
        altitude,
        east,
        north,
        up,
        sd_e,
        sd_n,
        sd_u,
        _ar_ratio,
        ar_sigma_scale,
    ) = payload
    msg = GnssPvtStamped()
    msg.header = Header(seq=sequence, stamp=ns_to_time(timestamp_ns), frame_id="gnss_antenna")
    msg.session_id = 1
    msg.writer_epoch = 1
    msg.source_sequence = epoch_index
    msg.pvt.time.week = gpst_week
    msg.pvt.time.tow = gpst_sow
    msg.pvt.fix_type = 3
    msg.pvt.valid_fix = True
    msg.pvt.diff_soln = True
    msg.pvt.carr_soln = 2 if quality_code == 1 else 1
    msg.pvt.num_sv = satellites
    msg.pvt.latitude = latitude
    msg.pvt.longitude = longitude
    msg.pvt.altitude = altitude
    # The PPK file contains ellipsoidal height only; zero marks unavailable MSL height.
    msg.pvt.height_msl = 0.0
    # Keep the source PPK SDs visible to the adapter; the official ENU
    # Odometry covariance below carries the AR-ratio penalty exactly once.
    msg.pvt.h_acc = max(sd_n, sd_e)
    msg.pvt.v_acc = sd_u
    # PPK has no PDOP or velocity solution. Match the existing driver's explicit
    # unavailable sentinel; the petrochemical config keeps these fields ungated.
    msg.pvt.p_dop = 999.0
    msg.pvt.vel_n = 0.0
    msg.pvt.vel_e = 0.0
    msg.pvt.vel_d = 0.0
    msg.pvt.vel_acc = 999.0
    msg.utc_measurement_ns = timestamp_ns
    msg.local_measurement_ns = timestamp_ns
    msg.mapping_version = 0
    msg.time_state = LOCAL_ONLY_TIME_STATE
    msg.timestamp_source = GnssPvtStamped.REPLAY_PRESERVED
    msg.timestamp_uncertainty_ns = 0
    msg.utc_valid = True
    msg.local_measurement_time_valid = True
    msg.valid_for_fusion = True

    enu = Odometry()
    enu.header = Header(seq=sequence, stamp=ns_to_time(timestamp_ns), frame_id="map")
    enu.child_frame_id = "gnss_antenna"
    enu.pose.pose.position.x = east
    enu.pose.pose.position.y = north
    enu.pose.pose.position.z = up
    enu.pose.pose.orientation.w = 1.0
    effective_sd_e = sd_e * ar_sigma_scale
    effective_sd_n = sd_n * ar_sigma_scale
    effective_sd_u = sd_u * ar_sigma_scale
    enu.pose.covariance[0] = effective_sd_e * effective_sd_e
    enu.pose.covariance[7] = effective_sd_n * effective_sd_n
    enu.pose.covariance[14] = effective_sd_u * effective_sd_u
    enu.pose.covariance[21] = 1e6
    enu.pose.covariance[28] = 1e6
    enu.pose.covariance[35] = 1e6
    return msg, enu


def validate_bag(path):
    with rosbag.Bag(str(path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        actual_types = {topic: info.msg_type for topic, info in topic_info.items()}
        if actual_types != OUTPUT_TYPES:
            raise ValueError(f"unexpected output bag topics/types: {actual_types}")
        result = {
            topic: (info.msg_type, info.message_count, info.frequency)
            for topic, info in sorted(topic_info.items())
        }
        return bag.get_start_time(), bag.get_end_time(), result


def print_summary(stats, first_ns, last_ns, window_start_ns, window_end_ns, dry_run):
    mode = "DRY RUN" if dry_run else "CONVERSION"
    print(f"\n{mode} SUMMARY")
    print(f"window_start_ns: {window_start_ns}")
    print(f"window_end_ns:   {window_end_ns if window_end_ns is not None else 'dataset end'}")
    if first_ns is not None:
        print(f"first_output_ns: {first_ns}")
        print(f"last_output_ns:  {last_ns}")
        print(f"output_duration_s: {(last_ns - first_ns) / 1e9:.9f}")
    for sensor in ("lidar", "imu", "left", "gnss"):
        print(
            f"{sensor}: window_events={stats[f'{sensor}_window_events']} "
            f"emitted={stats[f'{sensor}_emitted']} skipped={stats[f'{sensor}_skipped']}"
        )
    anomaly_keys = sorted(
        key
        for key, value in stats.items()
        if value
        and any(token in key for token in ("invalid", "malformed", "gap", "duplicate", "nonmonotonic", "corrupted", "unexpected", "empty", "zero"))
    )
    print("anomalies:")
    if anomaly_keys:
        for key in anomaly_keys:
            print(f"  {key}: {stats[key]}")
    else:
        print("  none")
    if stats["lidar_frames"]:
        print(
            "lidar_parse: "
            f"frames={stats['lidar_frames']} raw_points={stats['lidar_raw_points']} "
            f"valid_points={stats['lidar_valid_points']} invalid_points={stats['lidar_invalid_points']} "
            f"max_offset_s={stats['lidar_max_offset_ns'] / 1e9:.9f}"
        )
    if stats["gnss_float"]:
        print(
            "gnss_float_ar_sigma_scale: "
            f"mean={stats['gnss_float_ar_scale_milli_sum'] / stats['gnss_float'] / 1000.0:.3f} "
            f"max={stats['gnss_float_ar_scale_milli_max'] / 1000.0:.3f}"
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path, help="dataset root")
    parser.add_argument("--ppk", type=Path, help="override rover_ppk_solution_full.txt path")
    parser.add_argument(
        "--weight-config",
        type=Path,
        default=DEFAULT_WEIGHT_CONFIG,
        help="petrochemical YAML containing ppk_covariance parameters",
    )
    parser.add_argument("--output", type=Path, help="output .bag (required unless --dry-run)")
    parser.add_argument("--start", default="0", help="seconds from the earliest sensor timestamp")
    parser.add_argument("--duration", help="duration in seconds")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing output/partial bag")
    parser.add_argument("--dry-run", action="store_true", help="parse and validate without writing a bag")
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    args = parser.parse_args(argv)
    if not args.dry_run and args.output is None:
        parser.error("--output is required unless --dry-run is used")
    args.start_ns = seconds_to_ns(args.start, "--start")
    args.duration_ns = seconds_to_ns(args.duration, "--duration", allow_zero=False) if args.duration else None
    return args


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    dataset = args.dataset.expanduser().resolve()
    paths = require_dataset_layout(dataset)
    weight_config_path = args.weight_config.expanduser().resolve()
    if not weight_config_path.is_file():
        raise FileNotFoundError(f"weight config does not exist: {weight_config_path}")
    covariance_config = load_ppk_covariance_config(weight_config_path)
    if args.ppk:
        paths["ppk"] = args.ppk.expanduser().resolve()
        if not paths["ppk"].is_file():
            raise FileNotFoundError(f"PPK result does not exist: {paths['ppk']}")
    stats = defaultdict(int)
    iterators = [
        imu_events(paths["imu"], stats),
        lidar_events(paths["lidar"], stats, build_payload=not args.dry_run),
        camera_events(paths["left"], "left", stats),
        ppk_events(paths["ppk"], stats, covariance_config),
    ]

    heap = []
    for source_id, iterator in enumerate(iterators):
        try:
            timestamp_ns, sensor, payload = next(iterator)
        except StopIteration:
            continue
        heapq.heappush(heap, (timestamp_ns, PRIORITY[sensor], source_id, sensor, payload, iterator))
    if not heap:
        raise RuntimeError("dataset contains no valid sensor events")

    dataset_start_ns = min(item[0] for item in heap)
    window_start_ns = dataset_start_ns + args.start_ns
    window_end_ns = window_start_ns + args.duration_ns if args.duration_ns is not None else None
    output = args.output.expanduser().resolve() if args.output else None
    partial = Path(str(output) + ".partial") if output else None
    if output and output.exists() and not args.overwrite:
        raise FileExistsError(f"output exists; pass --overwrite to replace it: {output}")
    if partial and partial.exists():
        if not args.overwrite:
            raise FileExistsError(f"partial output exists; pass --overwrite to replace it: {partial}")
        partial.unlink()
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)

    bag = None
    first_output_ns = None
    last_output_ns = None
    last_record_ns = None
    next_progress_ns = window_start_ns + 10_000_000_000
    sequences = Counter()
    try:
        if not args.dry_run:
            bag = rosbag.Bag(str(partial), "w", compression=args.compression)
        while heap:
            timestamp_ns, _priority, source_id, sensor, payload, iterator = heapq.heappop(heap)
            if window_end_ns is not None and timestamp_ns >= window_end_ns:
                break
            if timestamp_ns >= window_start_ns:
                stats[f"{sensor}_window_events"] += 1
                if sensor == "lidar":
                    stats["lidar_frames"] += 1
                    _frame_index, raw_count, valid_count, invalid_count, max_offset_ns, rings, _data = payload
                    stats["lidar_raw_points"] += raw_count
                    stats["lidar_valid_points"] += valid_count
                    stats["lidar_invalid_points"] += invalid_count
                    stats["lidar_max_offset_ns"] = max(stats["lidar_max_offset_ns"], max_offset_ns)
                    for ring, count in rings.items():
                        stats[f"lidar_ring_{ring}"] += count
                message = True
                if sensor == "left":
                    message = build_compressed_image(
                        payload, timestamp_ns, sequences[sensor], stats, build_message=not args.dry_run
                    )
                elif not args.dry_run and sensor == "gnss":
                    message = build_gnss(payload, timestamp_ns, sequences[sensor])
                elif not args.dry_run and sensor in ("imu", "lidar"):
                    if sensor == "imu":
                        message = build_imu(payload, timestamp_ns, sequences[sensor])
                    else:
                        message = build_lidar(payload, timestamp_ns, sequences[sensor])
                if message is None:
                    stats[f"{sensor}_skipped"] += 1
                else:
                    if last_record_ns is not None and timestamp_ns < last_record_ns:
                        raise RuntimeError("global timestamp merge produced an out-of-order message")
                    if bag is not None:
                        if sensor == "gnss":
                            pvt, enu = message
                            bag.write(TOPICS["gnss_pvt"], pvt, ns_to_time(timestamp_ns))
                            bag.write(TOPICS["gnss_enu"], enu, ns_to_time(timestamp_ns))
                        else:
                            bag.write(TOPICS[sensor], message, ns_to_time(timestamp_ns))
                    sequences[sensor] += 1
                    stats[f"{sensor}_emitted"] += 1
                    first_output_ns = timestamp_ns if first_output_ns is None else first_output_ns
                    last_output_ns = timestamp_ns
                    last_record_ns = timestamp_ns
                    if timestamp_ns >= next_progress_ns:
                        print(
                            f"progress: {(timestamp_ns - window_start_ns) / 1e9:.1f}s "
                            f"lidar={stats['lidar_emitted']} imu={stats['imu_emitted']} "
                            f"left={stats['left_emitted']} gnss={stats['gnss_emitted']}",
                            flush=True,
                        )
                        while next_progress_ns <= timestamp_ns:
                            next_progress_ns += 10_000_000_000
            try:
                next_timestamp_ns, next_sensor, next_payload = next(iterator)
                heapq.heappush(
                    heap,
                    (next_timestamp_ns, PRIORITY[next_sensor], source_id, next_sensor, next_payload, iterator),
                )
            except StopIteration:
                pass
    finally:
        for iterator in iterators:
            close = getattr(iterator, "close", None)
            if close is not None:
                close()
        if bag is not None:
            bag.close()

    if first_output_ns is None:
        raise RuntimeError("selected window contains no valid messages")
    print_summary(stats, first_output_ns, last_output_ns, window_start_ns, window_end_ns, args.dry_run)
    if not args.dry_run:
        bag_start, bag_end, topic_info = validate_bag(partial)
        os.replace(partial, output)
        print(f"\nbag: {output}")
        print(f"bag_start: {bag_start:.9f}")
        print(f"bag_end: {bag_end:.9f}")
        print(f"bag_duration: {bag_end - bag_start:.9f}")
        for topic, (msg_type, count, frequency) in topic_info.items():
            print(f"{topic}: type={msg_type} count={count} frequency={frequency:.6f}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
