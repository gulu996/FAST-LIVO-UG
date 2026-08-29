#!/usr/bin/env python3
"""Audit a Tiaozhanbei dataset, run formal offline GNSS, build and verify its bag."""

import argparse
import csv
import getpass
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rosbag
from sensor_msgs.msg import PointField

import tiaozhanbei_dataset_to_rosbag as engine


HERE = Path(__file__).resolve().parent
FAST_LIVO_ROOT = HERE.parents[1]
DEFAULT_TIAOZHANBEI_ROOT = Path(
    "/home/gulu/TiaoZhanBei/TIAOZHANBEI_FASTLIVO2_GNSS_CODE_20260826"
)
DEFAULT_REFERENCE_BAG = DEFAULT_TIAOZHANBEI_ROOT / "rosbags" / "competition.bag"
DEFAULT_DECODE_SCRIPT = DEFAULT_TIAOZHANBEI_ROOT / "scripts" / "decode_gnss.sh"
REPORT_FILES = (
    "dataset_audit.txt",
    "timestamp_audit.csv",
    "stereo_pair_audit.txt",
    "gnss_audit.txt",
    "bag_validation.txt",
    "build_manifest.txt",
)


def run(command, check=True):
    result = subprocess.run(
        [str(value) for value in command],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            "command failed (exit={}): {}\n{}".format(
                result.returncode,
                " ".join(str(value) for value in command if "-p" not in str(value)[:2]),
                result.stdout[-8000:],
            )
        )
    return result


def run_stream(command, check=True):
    """Run a long command while preserving output for reports and showing it live."""
    command = [str(value) for value in command]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    lines = []
    for line in process.stdout:
        lines.append(line)
        print(line, end="", flush=True)
    process.stdout.close()
    returncode = process.wait()
    result = subprocess.CompletedProcess(command, returncode, "".join(lines))
    if check and returncode != 0:
        raise RuntimeError(
            "command failed (exit={}): {}\n{}".format(
                returncode,
                " ".join(value for value in command if "-p" not in value[:2]),
                result.stdout[-8000:],
            )
        )
    return result


def phase(step, total, message):
    print("\n[STEP {}/{}] {}".format(step, total, message), flush=True)


def timestamp_summary(values):
    audit = engine.TimestampAudit("audit")
    for value in values:
        audit.add(int(value))
    return audit.summary()


def format_timestamp_summary(name, summary):
    return (
        "{}: count={} first_ns={} last_ns={} duration_s={:.9f} mean_hz={:.6f} "
        "median_dt_s={:.9f} p95_dt_s={:.9f} max_dt_s={:.9f} duplicate={} nonmonotonic={}".format(
            name,
            summary["count"],
            summary["first_ns"],
            summary["last_ns"],
            summary["duration_s"],
            summary["mean_hz"],
            summary["median_dt_s"],
            summary["p95_dt_s"],
            summary["max_dt_s"],
            summary["duplicate_timestamps"],
            summary["nonmonotonic_timestamps"],
        )
    )


def image_listing_from_paths(paths):
    by_key = defaultdict(list)
    timestamps = []
    for path in paths:
        key = Path(path).stem
        by_key[key].append(str(path))
        try:
            timestamps.append(int(key))
        except ValueError as exc:
            raise ValueError("image filename is not an integer timestamp: {}".format(path)) from exc
    timestamps.sort()
    return by_key, timestamps


def stereo_result(left_paths, right_paths):
    left, left_timestamps = image_listing_from_paths(left_paths)
    right, right_timestamps = image_listing_from_paths(right_paths)
    left_keys = set(left)
    right_keys = set(right)
    return {
        "left_count": sum(len(values) for values in left.values()),
        "right_count": sum(len(values) for values in right.values()),
        "matched_pairs": len(left_keys & right_keys),
        "left_only": sorted(left_keys - right_keys),
        "right_only": sorted(right_keys - left_keys),
        "duplicate_left": sorted(key for key, values in left.items() if len(values) > 1),
        "duplicate_right": sorted(key for key, values in right.items() if len(values) > 1),
        "left_timestamps": left_timestamps,
        "right_timestamps": right_timestamps,
    }


def classify_files(relative_sizes):
    groups = defaultdict(list)
    extensions = Counter()
    top = Counter()
    second = Counter()
    for relative, size in relative_sizes:
        parts = Path(relative).parts
        if parts:
            top[parts[0]] += 1
        if len(parts) >= 2:
            second["{}/{}".format(parts[0], parts[1])] += 1
        suffix = Path(relative).suffix.lower() or "[none]"
        extensions[suffix] += 1
        text = str(relative)
        if text.startswith("camera/left/") and suffix in (".jpg", ".jpeg"):
            group = "camera_left_jpg"
        elif text.startswith("camera/right/") and suffix in (".jpg", ".jpeg"):
            group = "camera_right_jpg"
        elif text.startswith("camera/"):
            group = "camera_meta"
        elif text.startswith("lidar/"):
            group = "lidar"
        elif text.startswith("imu/"):
            group = "imu"
        elif text.startswith("gnss/"):
            group = "gnss"
        else:
            group = "other"
        groups[group].append(int(size))
    return {
        "file_count": len(relative_sizes),
        "top": dict(sorted(top.items())),
        "second": dict(sorted(second.items())),
        "extensions": dict(sorted(extensions.items())),
        "groups": {
            name: {
                "count": len(sizes),
                "min_size": min(sizes),
                "max_size": max(sizes),
                "total_size": sum(sizes),
            }
            for name, sizes in sorted(groups.items())
        },
    }


def find_one(folder, pattern):
    matches = sorted(folder.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one {!r} in {}, found {}:\n  {}".format(
                pattern, folder, len(matches), "\n  ".join(str(path) for path in matches)
            )
        )
    return matches[0]


def rinex_base_ecef(path):
    with path.open("r", encoding="ascii", errors="replace") as stream:
        for line in stream:
            if "APPROX POSITION XYZ" in line:
                fields = line[:60].split()
                if len(fields) < 3:
                    break
                values = [float(value) for value in fields[:3]]
                radius = math.sqrt(sum(value * value for value in values))
                if all(math.isfinite(value) for value in values) and 5e6 < radius < 7e6:
                    return values
                break
            if "END OF HEADER" in line:
                break
    raise RuntimeError("valid APPROX POSITION XYZ was not found in {}".format(path))


def dataset_inventory(dataset):
    files = [path for path in dataset.rglob("*") if path.is_file()]
    relative_sizes = [(str(path.relative_to(dataset)), path.stat().st_size) for path in files]
    left_paths = sorted(
        path for path in (dataset / "camera" / "left").iterdir()
        if path.is_file() and path.suffix.lower() in (".jpg", ".jpeg")
    )
    right_paths = sorted(
        path for path in (dataset / "camera" / "right").iterdir()
        if path.is_file() and path.suffix.lower() in (".jpg", ".jpeg")
    )
    stereo = stereo_result(left_paths, right_paths)
    gnss_dir = dataset / "gnss"
    rover = find_one(gnss_dir, "ROV*.26O")
    base = find_one(gnss_dir, "BASE*.26O")
    navigation = find_one(gnss_dir, "ROV*.26C")
    lidar = [
        path for path in (dataset / "lidar" / "points.csv", dataset / "lidar" / "points.csv.gz")
        if path.is_file()
    ]
    if len(lidar) != 1:
        raise RuntimeError("expected exactly one points.csv or points.csv.gz, found {}".format(lidar))
    required = [
        dataset / "camera" / "stereo.yaml",
        dataset / "imu" / "imu.csv",
        dataset / "imu" / "imu.yaml",
        dataset / "sensor_extrinsics.yaml",
    ]
    missing = [path for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing dataset inputs: {}".format(missing))

    import cv2

    jpeg_decode_samples = []
    jpeg_marker_valid = {}
    # ponytail: decode first/middle/last per side; all left frames still get JPEG
    # marker validation in the streaming converter. Decode all frames if sampled
    # corruption is ever observed in a future dataset.
    jpeg_progress = engine.ProgressReporter(
        "JPEG AUDIT", len(left_paths) + len(right_paths), "files"
    )
    checked_jpegs = 0
    for side, paths in (("left", left_paths), ("right", right_paths)):
        if not paths:
            raise RuntimeError("camera/{} contains no JPEG files".format(side))
        invalid_markers = []
        for path in paths:
            if not engine.jpeg_has_markers(path):
                invalid_markers.append(path)
            checked_jpegs += 1
            jpeg_progress.update(checked_jpegs)
        if invalid_markers:
            raise RuntimeError(
                "camera/{} has incomplete JPEG markers; first samples: {}".format(
                    side, invalid_markers[:20]
                )
            )
        jpeg_marker_valid[side] = len(paths)
        for index in sorted({0, len(paths) // 2, len(paths) - 1}):
            image = cv2.imread(str(paths[index]), cv2.IMREAD_UNCHANGED)
            if image is None or image.size == 0:
                raise RuntimeError("OpenCV failed to decode {}".format(paths[index]))
            jpeg_decode_samples.append(
                {"side": side, "file": paths[index].name, "shape": list(image.shape)}
            )
    jpeg_progress.finish()
    return {
        "root": str(dataset),
        "files": classify_files(relative_sizes),
        "stereo": stereo,
        "camera_left_stats": timestamp_summary(stereo["left_timestamps"]),
        "camera_right_stats": timestamp_summary(stereo["right_timestamps"]),
        "rover": str(rover),
        "base": str(base),
        "navigation": str(navigation),
        "base_ecef": rinex_base_ecef(base),
        "lidar": str(lidar[0]),
        "jpeg_decode_samples": jpeg_decode_samples,
        "jpeg_marker_valid": jpeg_marker_valid,
    }


def zip_inventory(path):
    with zipfile.ZipFile(str(path)) as archive:
        entries = archive.infolist()
    files = [entry for entry in entries if not entry.is_dir()]
    def decoded_name(entry):
        # Some Baidu ZIP writers store UTF-8 or GBK bytes without the ZIP UTF-8 flag.
        try:
            raw = entry.filename.encode("cp437")
        except UnicodeEncodeError:
            return entry.filename
        for encoding in ("utf-8", "gb18030"):
            try:
                return raw.decode(encoding)
            except UnicodeDecodeError:
                pass
        return entry.filename

    decoded = [(entry, decoded_name(entry)) for entry in files]
    roots = {Path(name).parts[0] for _entry, name in decoded if Path(name).parts}
    common_root = next(iter(roots)) if len(roots) == 1 else None
    relative_sizes = []
    left_names = []
    right_names = []
    encrypted = 0
    for entry, name in decoded:
        parts = Path(name).parts
        if common_root and parts and parts[0] == common_root:
            parts = parts[1:]
        relative = Path(*parts)
        relative_sizes.append((str(relative), entry.file_size))
        text = str(relative)
        if text.startswith("camera/left/") and relative.suffix.lower() in (".jpg", ".jpeg"):
            left_names.append(relative.name)
        if text.startswith("camera/right/") and relative.suffix.lower() in (".jpg", ".jpeg"):
            right_names.append(relative.name)
        if entry.flag_bits & 1:
            encrypted += 1
    stereo = stereo_result(left_names, right_names)
    names = [str(relative) for relative, _size in relative_sizes]
    return {
        "path": str(path),
        "physical_size": path.stat().st_size,
        "entry_count": len(entries),
        "directory_count": len(entries) - len(files),
        "common_root": common_root,
        "encrypted_files": encrypted,
        "content_list_encrypted": False,
        "content_validated": encrypted == 0,
        "files": classify_files(relative_sizes),
        "stereo": stereo,
        "camera_left_stats": timestamp_summary(stereo["left_timestamps"]),
        "camera_right_stats": timestamp_summary(stereo["right_timestamps"]),
        "rinex_names": sorted(name for name in names if Path(name).suffix.lower() in (".26o", ".26c")),
        "lidar_names": sorted(name for name in names if name.startswith("lidar/")),
        "imu_names": sorted(name for name in names if name.startswith("imu/")),
        "baiduyun_cfg": sorted(name for name in names if name.endswith(".baiduyun.uploading.cfg")),
    }


def msg_stamp_ns(msg):
    return int(msg.header.stamp.secs) * engine.NSEC + int(msg.header.stamp.nsecs)


def time_ns(value):
    return int(value.secs) * engine.NSEC + int(value.nsecs)


def sha256_file(path, progress_label=None):
    digest = hashlib.sha256()
    progress = (
        engine.ProgressReporter(progress_label, path.stat().st_size, "bytes")
        if progress_label else None
    )
    completed = 0
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
            completed += len(block)
            if progress:
                progress.update(completed)
    if progress:
        progress.finish()
    return digest.hexdigest()


def reference_schema(path):
    if not path.is_file():
        raise FileNotFoundError("reference bag does not exist: {}".format(path))
    info_text = run(["rosbag", "info", str(path)]).stdout
    with rosbag.Bag(str(path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        actual_types = {topic: item.msg_type for topic, item in topic_info.items()}
        if actual_types != engine.OUTPUT_TYPES:
            raise RuntimeError(
                "reference bag schema differs from the audited converter schema:\nreference={}\nexpected={}".format(
                    actual_types, engine.OUTPUT_TYPES
                )
            )
        topics = {}
        for topic in sorted(actual_types):
            _topic, msg, event_time = next(bag.read_messages(topics=[topic]))
            record = {
                "type": actual_types[topic],
                "count": int(topic_info[topic].message_count),
                "frequency": float(topic_info[topic].frequency or 0.0),
                "frame_id": msg.header.frame_id,
                "header_stamp_ns": msg_stamp_ns(msg),
                "event_stamp_ns": time_ns(event_time),
            }
            if topic == engine.TOPICS["lidar"]:
                record.update(
                    {
                        "fields": [
                            {"name": field.name, "offset": field.offset, "datatype": field.datatype, "count": field.count}
                            for field in msg.fields
                        ],
                        "point_step": msg.point_step,
                        "first_width": msg.width,
                    }
                )
            elif topic == engine.TOPICS["camera"]:
                record.update({"format": msg.format, "first_data_size": len(msg.data)})
            elif topic == engine.TOPICS["gnss"]:
                record.update(
                    {
                        "position_convention": "x=East,y=North,z=Up",
                        "first_covariance": list(msg.pose.covariance),
                    }
                )
            topics[topic] = record
        return {
            "path": str(path),
            "sha256": sha256_file(path, "REFERENCE SHA256"),
            "start_s": bag.get_start_time(),
            "end_s": bag.get_end_time(),
            "duration_s": bag.get_end_time() - bag.get_start_time(),
            "topics": topics,
            "rosbag_info": info_text,
        }


def stereo_text(label, stereo):
    lines = [
        "{}:".format(label),
        "left_count={}".format(stereo["left_count"]),
        "right_count={}".format(stereo["right_count"]),
        "matched_pairs={}".format(stereo["matched_pairs"]),
        "left_only={}".format(len(stereo["left_only"])),
        "right_only={}".format(len(stereo["right_only"])),
        "duplicate_left={}".format(len(stereo["duplicate_left"])),
        "duplicate_right={}".format(len(stereo["duplicate_right"])),
        "LEFT_ONLY_SAMPLES:",
    ]
    lines.extend(stereo["left_only"][:20] or ["(none)"])
    lines.append("RIGHT_ONLY_SAMPLES:")
    lines.extend(stereo["right_only"][:20] or ["(none)"])
    lines.append(
        "LEFT_RIGHT_COUNT_MATCH={}".format(
            "YES" if stereo["left_count"] == stereo["right_count"] else "NO"
        )
    )
    return "\n".join(lines)


def write_stereo_report(path, reference, zip_audit=None):
    sections = [stereo_text("REFERENCE", reference["stereo"])]
    if zip_audit:
        sections.append(stereo_text("1.ZIP", zip_audit["stereo"]))
        blocked = zip_audit["encrypted_files"] > 0 and not zip_audit["content_validated"]
        pair_ok = not any(
            (
                zip_audit["stereo"]["left_only"],
                zip_audit["stereo"]["right_only"],
                zip_audit["stereo"]["duplicate_left"],
                zip_audit["stereo"]["duplicate_right"],
            )
        )
        status = "PASSWORD_BLOCKED" if blocked else ("PASS" if pair_ok else "FAIL")
    else:
        item = reference["stereo"]
        status = "PASS" if not any((item["left_only"], item["right_only"], item["duplicate_left"], item["duplicate_right"])) else "FAIL"
    sections.append("STEREO_PAIR_STATUS = {}".format(status))
    path.write_text("\n\n".join(sections) + "\n", encoding="utf-8")


def format_groups(inventory):
    lines = []
    for name, values in inventory["files"]["groups"].items():
        lines.append(
            "{} count={} min_size={} max_size={} total_size={}".format(
                name, values["count"], values["min_size"], values["max_size"], values["total_size"]
            )
        )
    return lines


def write_dataset_audit(path, reference, source_summary=None, zip_audit=None):
    lines = [
        "DATASET FORMAT AUDIT",
        "reference_root={}".format(reference["root"]),
        "reference_file_count={}".format(reference["files"]["file_count"]),
        "reference_top={}".format(json.dumps(reference["files"]["top"], ensure_ascii=False, sort_keys=True)),
        "reference_second={}".format(json.dumps(reference["files"]["second"], ensure_ascii=False, sort_keys=True)),
        "reference_extensions={}".format(json.dumps(reference["files"]["extensions"], ensure_ascii=False, sort_keys=True)),
    ]
    lines.extend("reference_" + value for value in format_groups(reference))
    lines.extend(
        [
            format_timestamp_summary("reference_camera_left", reference["camera_left_stats"]),
            format_timestamp_summary("reference_camera_right", reference["camera_right_stats"]),
            "camera_timestamp_source=integer filename stem, nanoseconds since Unix UTC",
            "imu_timestamp_source=imu/imu.csv timestamp_ns",
            "lidar_timestamp_source=lidar/points.csv[.gz] frame_timestamp_ns",
            "lidar_point_time_source=point_time_s seconds relative to scan start",
            "base_rinex={}".format(reference["base"]),
            "BASE_ECEF={}".format(" ".join("{:.4f}".format(value) for value in reference["base_ecef"])),
            "jpeg_decode_samples={}".format(json.dumps(reference["jpeg_decode_samples"], sort_keys=True)),
            "jpeg_marker_valid={}".format(json.dumps(reference["jpeg_marker_valid"], sort_keys=True)),
        ]
    )
    if source_summary:
        lines.append("CONTENT_PREFLIGHT=PASS")
        for sensor, summary in source_summary["streams"].items():
            lines.append(format_timestamp_summary("source_" + sensor, summary))
        lines.append("source_counters={}".format(json.dumps(source_summary["counters"], sort_keys=True)))
        lines.append("lidar_samples={}".format(json.dumps(source_summary["lidar_samples"], sort_keys=True)))
    if zip_audit:
        content_note = (
            "ZIP_CONTENT_VALIDATED_AFTER_EXTRACTION=YES"
            if zip_audit["content_validated"]
            else "IMAGE_CONTENT_NOT_VALIDATED_DUE_TO_PASSWORD"
        )
        lines.extend(
            [
                "",
                "1.ZIP",
                "path={}".format(zip_audit["path"]),
                "physical_size={}".format(zip_audit["physical_size"]),
                "entries={}".format(zip_audit["entry_count"]),
                "files={}".format(zip_audit["files"]["file_count"]),
                "directories={}".format(zip_audit["directory_count"]),
                "common_root={}".format(zip_audit["common_root"]),
                "encrypted_files={}".format(zip_audit["encrypted_files"]),
                "ZIP_CONTENT_LIST_ENCRYPTED=NO",
                content_note,
                "zip_top={}".format(json.dumps(zip_audit["files"]["top"], ensure_ascii=False, sort_keys=True)),
                "zip_second={}".format(json.dumps(zip_audit["files"]["second"], ensure_ascii=False, sort_keys=True)),
                "zip_extensions={}".format(json.dumps(zip_audit["files"]["extensions"], ensure_ascii=False, sort_keys=True)),
            ]
        )
        lines.extend("zip_" + value for value in format_groups(zip_audit))
        lines.extend(
            [
                format_timestamp_summary("zip_camera_left", zip_audit["camera_left_stats"]),
                format_timestamp_summary("zip_camera_right", zip_audit["camera_right_stats"]),
                "zip_rinex_names={}".format(json.dumps(zip_audit["rinex_names"], ensure_ascii=False)),
                "zip_lidar_names={}".format(json.dumps(zip_audit["lidar_names"], ensure_ascii=False)),
                "zip_imu_names={}".format(json.dumps(zip_audit["imu_names"], ensure_ascii=False)),
                "baiduyun_uploading_cfg_count={}".format(len(zip_audit["baiduyun_cfg"])),
                "FORMAT_STATUS={}".format(
                    "COMPATIBLE_WITH_DIFFERENCES"
                    if zip_audit["content_validated"]
                    else "PARTIAL_AUDIT_WAITING_FOR_ZIP_PASSWORD"
                ),
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_format_markdown(path, reference, zip_audit, schema):
    stereo = zip_audit["stereo"]
    content_status = "PARTIAL_AUDIT_WAITING_FOR_ZIP_PASSWORD" if zip_audit["encrypted_files"] else "COMPATIBLE_WITH_DIFFERENCES"
    if zip_audit["content_validated"]:
        opening = "`1.zip` 已成功解压并通过 Camera/LiDAR/IMU/RINEX 内容预检；其结构与参考数据兼容。"
        image_note = "ZIP_CONTENT_VALIDATED_AFTER_EXTRACTION"
        stereo_status = "PASS"
    else:
        opening = "`1.zip` 与“融合定位”的目录角色、文件扩展名和命名语义兼容，但 ZIP 内部文件使用 AES 加密，当前只能验证中央目录元数据，不能验证 JPEG 解码、CSV 表头、时间单位、RINEX 头或标定内容。"
        image_note = "IMAGE_CONTENT_NOT_VALIDATED_DUE_TO_PASSWORD"
        stereo_status = "PASSWORD_BLOCKED（文件名级配对为 PASS，内容级检查等待密码）"
    text = """# Dataset format audit

## DATASET FORMAT AUDIT

{opening}

- FORMAT_STATUS = `{status}`
- ZIP_CONTENT_LIST_ENCRYPTED = `NO`
- {image_note}
- 参考数据：left = {ref_left}, right = {ref_right}, matched = {ref_match}
- 1.zip：left = {zip_left}, right = {zip_right}, matched = {zip_match}
- 1.zip：left_only = {left_only}, right_only = {right_only}, duplicate_left = {dup_left}, duplicate_right = {dup_right}
- STEREO_PAIR_STATUS = `{stereo_status}`

## 结构差异

- 参考数据包含 GNSS 离线结果和辅助工具等派生文件；`1.zip` 只有原始 RINEX 三件套。
- 参考 LiDAR 是 `points.csv`，`1.zip` 中央目录也是 `points.csv`；两者未压缩大小分别为 {ref_lidar_size} 和 {zip_lidar_size} 字节。
- 参考相机每侧 {ref_left} 帧；`1.zip` 每侧 {zip_left} 帧。
- `1.zip/camera/left` 另含 {cfg_count} 个 `.baiduyun.uploading.cfg` 临时元数据文件，转换器只接受 `.jpg/.jpeg`，不会将这些文件当图像。

## REFERENCE_BAG_SCHEMA

参考 bag：`{bag}`，SHA-256 `{bag_sha}`。

| Topic | Type | Count | frame_id |
|---|---|---:|---|
{schema_rows}

参考 bag 只包含左目压缩 JPEG；LiDAR 是 `sensor_msgs/PointCloud2`，字段为 `x,y,z,intensity,time,ring`，其中 `time` 为 FLOAT64 秒制帧内相对时间；GNSS 是 ENU `geometry_msgs/PoseWithCovarianceStamped`。
""".format(
        status=content_status,
        opening=opening,
        image_note=image_note,
        stereo_status=stereo_status,
        ref_left=reference["stereo"]["left_count"],
        ref_right=reference["stereo"]["right_count"],
        ref_match=reference["stereo"]["matched_pairs"],
        zip_left=stereo["left_count"],
        zip_right=stereo["right_count"],
        zip_match=stereo["matched_pairs"],
        left_only=len(stereo["left_only"]),
        right_only=len(stereo["right_only"]),
        dup_left=len(stereo["duplicate_left"]),
        dup_right=len(stereo["duplicate_right"]),
        ref_lidar_size=reference["files"]["groups"]["lidar"]["max_size"],
        zip_lidar_size=zip_audit["files"]["groups"]["lidar"]["max_size"],
        cfg_count=len(zip_audit["baiduyun_cfg"]),
        bag=schema["path"],
        bag_sha=schema["sha256"],
        schema_rows="\n".join(
            "| `{}` | `{}` | {} | `{}` |".format(topic, item["type"], item["count"], item["frame_id"])
            for topic, item in sorted(schema["topics"].items())
        ),
    )
    path.write_text(text, encoding="utf-8")


def decode_check(decode_script, dataset):
    result = run([str(decode_script), "offline", "--dataset", str(dataset), "--check-inputs"])
    try:
        return json.loads(result.stdout), result.stdout
    except json.JSONDecodeError as exc:
        raise RuntimeError("decode_gnss.sh --check-inputs did not return JSON:\n{}".format(result.stdout)) from exc


def validate_gnss_provenance(csv_path, summary_path, dataset, inventory, source_summary):
    if not csv_path.is_file() or not summary_path.is_file():
        raise FileNotFoundError("GNSS reuse requires both {} and {}".format(csv_path, summary_path))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    inputs = summary.get("inputs", {})
    expected_paths = {
        "dataset": Path(dataset),
        "rover_obs": Path(inventory["rover"]),
        "base_obs": Path(inventory["base"]),
        "navigation": Path(inventory["navigation"]),
    }
    for name, expected in expected_paths.items():
        actual = inputs.get(name)
        if actual is None or Path(actual).resolve() != expected.resolve():
            raise RuntimeError("GNSS provenance mismatch for {}: {} != {}".format(name, actual, expected))
    base = [float(value) for value in inputs.get("base_ecef_m", [])]
    if len(base) != 3 or not np.allclose(base, inventory["base_ecef"], rtol=0.0, atol=1e-4):
        raise RuntimeError("GNSS summary BASE ECEF does not match current BASE RINEX")
    total = valid = 0
    first = last = None
    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [name for name in engine.GNSS_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            raise RuntimeError("GNSS CSV missing columns: {}".format(missing))
        for row in reader:
            total += 1
            try:
                timestamp_ns = int(row["timestamp_ns"])
                position = [float(row[name]) for name in ("east_m", "north_m", "up_m")]
                covariance = [
                    float(row[name])
                    for name in ("cov_ee", "cov_en", "cov_eu", "cov_nn", "cov_nu", "cov_uu")
                ]
                usable = int(row["quality"]) <= 2 and int(row["num_satellites"]) >= 6
            except (TypeError, ValueError):
                continue
            if (
                usable
                and all(math.isfinite(value) for value in position + covariance)
                and min(covariance[0], covariance[3], covariance[5]) > 0.0
            ):
                valid += 1
                first = timestamp_ns if first is None else min(first, timestamp_ns)
                last = timestamp_ns if last is None else max(last, timestamp_ns)
    if valid == 0:
        raise RuntimeError("GNSS solution contains no valid epochs")
    sensor_first = source_summary["global"]["first_ns"]
    sensor_last = source_summary["global"]["last_ns"]
    if last < sensor_first or first > sensor_last:
        raise RuntimeError("GNSS time range does not overlap the current dataset")
    return {
        "csv": str(csv_path),
        "summary": str(summary_path),
        "rows": total,
        "valid_rows": valid,
        "first_ns": first,
        "last_ns": last,
        "base_ecef": base,
        "provenance": inputs,
    }


def prepare_gnss(args, dataset, inventory, source_summary, report_dir):
    output_csv = report_dir / "gnss_solution_offline.csv"
    output_pos = report_dir / "gnss_solution_offline.pos"
    output_summary = report_dir / "gnss_solution_offline_summary.json"
    if args.reuse_gnss:
        candidates = [
            (output_csv, output_summary),
            (dataset / "gnss" / "gnss_solution_offline.csv", dataset / "gnss" / "gnss_solution_offline_summary.json"),
        ]
        errors = []
        for csv_path, summary_path in candidates:
            try:
                result = validate_gnss_provenance(csv_path, summary_path, dataset, inventory, source_summary)
                result["mode"] = "reused"
                return result, "reused validated GNSS solution\n"
            except (FileNotFoundError, RuntimeError) as exc:
                errors.append(str(exc))
        raise RuntimeError("no reusable GNSS solution passed provenance checks:\n" + "\n".join(errors))
    result = run_stream(
        [
            str(args.decode_script),
            "offline",
            "--dataset",
            str(dataset),
            "--raw-output",
            str(output_pos),
            "--output",
            str(output_csv),
            "--summary",
            str(output_summary),
        ]
    )
    audit = validate_gnss_provenance(output_csv, output_summary, dataset, inventory, source_summary)
    audit["mode"] = "fresh decode_gnss.sh offline"
    return audit, result.stdout


def pointcloud_sample(msg):
    fields = {field.name: field for field in msg.fields}
    time_field = fields.get("time")
    if time_field is None or time_field.datatype != PointField.FLOAT64:
        return {"error": "missing time FLOAT64"}
    count = int(msg.width * msg.height)
    values = np.ndarray(
        shape=(count,), dtype="<f8", buffer=msg.data, offset=time_field.offset, strides=(msg.point_step,)
    )
    return {
        "stamp_ns": msg_stamp_ns(msg),
        "frame_id": msg.header.frame_id,
        "point_count": count,
        "fields": [field.name for field in msg.fields],
        "point_time_min_s": float(values.min()) if count else None,
        "point_time_max_s": float(values.max()) if count else None,
        "all_point_time_zero": bool(count and np.all(values == 0.0)),
    }


def validate_bag(output, reference, source_summary, camera_timestamps):
    info_text = run(["rosbag", "info", str(output)]).stdout
    topic_for_sensor = engine.TOPICS
    expected_counts = {
        topic_for_sensor[name]: int(source_summary["streams"][name]["count"])
        for name in ("lidar", "imu", "camera", "gnss")
    }
    audits = {topic: engine.TimestampAudit(topic) for topic in engine.OUTPUT_TYPES}
    global_nonmonotonic = 0
    header_event_mismatch = 0
    camera_filename_mismatch = 0
    camera_index = 0
    image_marker_fail = 0
    imu_angular_nonzero = 0
    imu_linear_nonzero = 0
    gnss_covariance_invalid = 0
    gnss_min = [float("inf")] * 3
    gnss_max = [float("-inf")] * 3
    lidar_bad_fields = 0
    lidar_samples = []
    previous_global = None
    lidar_target_indices = {
        0,
        expected_counts[topic_for_sensor["lidar"]] // 2,
        expected_counts[topic_for_sensor["lidar"]] - 1,
    }
    lidar_index = 0
    validation_progress = engine.ProgressReporter(
        "BAG VALIDATION", sum(expected_counts.values()), "messages"
    )
    validated_messages = 0
    with rosbag.Bag(str(output), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        actual_types = {topic: item.msg_type for topic, item in topic_info.items()}
        reference_types = {topic: item["type"] for topic, item in reference["topics"].items()}
        if actual_types != reference_types:
            raise RuntimeError("output topics/types differ from REFERENCE_BAG_SCHEMA")
        for topic, expected in expected_counts.items():
            actual = int(topic_info[topic].message_count)
            if actual != expected:
                raise RuntimeError("message count mismatch for {}: {} != {}".format(topic, actual, expected))
        for topic, msg, event_time in bag.read_messages():
            validated_messages += 1
            validation_progress.update(validated_messages)
            event_ns = time_ns(event_time)
            header_ns = msg_stamp_ns(msg)
            audits[topic].add(event_ns)
            if previous_global is not None and event_ns < previous_global:
                global_nonmonotonic += 1
            previous_global = event_ns
            if event_ns != header_ns:
                header_event_mismatch += 1
            if topic == topic_for_sensor["camera"]:
                if camera_index >= len(camera_timestamps) or header_ns != camera_timestamps[camera_index]:
                    camera_filename_mismatch += 1
                camera_index += 1
                if bytes(msg.data[:2]) != b"\xff\xd8" or bytes(msg.data[-2:]) != b"\xff\xd9":
                    image_marker_fail += 1
            elif topic == topic_for_sensor["imu"]:
                if any(abs(value) > 0.0 for value in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
                    imu_angular_nonzero += 1
                if any(abs(value) > 0.0 for value in (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)):
                    imu_linear_nonzero += 1
            elif topic == topic_for_sensor["gnss"]:
                covariance = msg.pose.covariance
                if not all(math.isfinite(covariance[index]) and covariance[index] > 0.0 for index in (0, 7, 14)):
                    gnss_covariance_invalid += 1
                position = msg.pose.pose.position
                for index, value in enumerate((position.x, position.y, position.z)):
                    gnss_min[index] = min(gnss_min[index], value)
                    gnss_max[index] = max(gnss_max[index], value)
            elif topic == topic_for_sensor["lidar"]:
                expected_fields = [item["name"] for item in reference["topics"][topic]["fields"]]
                if [field.name for field in msg.fields] != expected_fields:
                    lidar_bad_fields += 1
                if lidar_index in lidar_target_indices:
                    lidar_samples.append(pointcloud_sample(msg))
                lidar_index += 1
        start_s = bag.get_start_time()
        end_s = bag.get_end_time()
    validation_progress.finish()
    for audit in audits.values():
        audit.require_strict()
    failures = {
        "global_nonmonotonic": global_nonmonotonic,
        "header_event_mismatch": header_event_mismatch,
        "camera_filename_mismatch": camera_filename_mismatch,
        "image_marker_fail": image_marker_fail,
        "gnss_covariance_invalid": gnss_covariance_invalid,
        "lidar_bad_fields": lidar_bad_fields,
        "lidar_all_point_time_zero_samples": sum(
            1 for sample in lidar_samples if sample.get("all_point_time_zero")
        ),
    }
    if imu_angular_nonzero == 0 or imu_linear_nonzero == 0:
        failures["imu_all_zero"] = 1
    if any(failures.values()):
        raise RuntimeError("bag validation failures: {}".format(failures))
    return {
        "rosbag_info": info_text,
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": end_s - start_s,
        "topics": {topic: audit.summary() for topic, audit in audits.items()},
        "counts": expected_counts,
        "failures": failures,
        "imu_angular_nonzero": imu_angular_nonzero,
        "imu_linear_nonzero": imu_linear_nonzero,
        "gnss_enu_min": gnss_min,
        "gnss_enu_max": gnss_max,
        "lidar_samples": lidar_samples,
    }


def write_timestamp_csv(path, source_summary, bag_validation=None):
    fieldnames = [
        "scope", "stream", "count", "first_ns", "last_ns", "duration_s", "mean_hz",
        "median_dt_s", "p95_dt_s", "max_dt_s", "duplicate_timestamps", "nonmonotonic_timestamps",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for name, summary in source_summary["streams"].items():
            writer.writerow({"scope": "source", "stream": name, **summary})
        if bag_validation:
            for name, summary in bag_validation["topics"].items():
                writer.writerow({"scope": "bag", "stream": name, **summary})


def write_reference_schema(path, schema):
    lines = [
        "REFERENCE_BAG_SCHEMA",
        "reference_bag={}".format(schema["path"]),
        "reference_bag_sha256={}".format(schema["sha256"]),
        "start_s={:.9f}".format(schema["start_s"]),
        "end_s={:.9f}".format(schema["end_s"]),
        "duration_s={:.9f}".format(schema["duration_s"]),
    ]
    for topic, item in sorted(schema["topics"].items()):
        lines.append(
            "topic={} type={} count={} frequency={} frame_id={} header_event_equal={}".format(
                topic,
                item["type"],
                item["count"],
                item["frequency"],
                item["frame_id"],
                item["header_stamp_ns"] == item["event_stamp_ns"],
            )
        )
        if "fields" in item:
            lines.append("point_fields={}".format(json.dumps(item["fields"], sort_keys=True)))
        if "format" in item:
            lines.append("image_format={}".format(item["format"]))
        if "position_convention" in item:
            lines.append("gnss_coordinate_convention={}".format(item["position_convention"]))
            lines.append("gnss_first_covariance={}".format(item["first_covariance"]))
    lines.extend(["", schema["rosbag_info"]])
    path.write_text("\n".join(lines), encoding="utf-8")


def git_head_and_dirty():
    git_dir = FAST_LIVO_ROOT / ".git"
    head = "unavailable"
    if (git_dir / "HEAD").is_file():
        text = (git_dir / "HEAD").read_text(encoding="ascii").strip()
        if text.startswith("ref: "):
            ref_path = git_dir / text[5:]
            if ref_path.is_file():
                head = ref_path.read_text(encoding="ascii").strip()
        else:
            head = text
    status = run(["git", "status", "--short"], check=False)
    return head, bool(status.stdout.strip())


def write_manifest(path, args, dataset, output, inventory, schema, source_summary, gnss=None, validation=None):
    head, dirty = git_head_and_dirty()
    lines = [
        "dataset_path={}".format(dataset),
        "output_bag={}".format(output),
        "build_time={}".format(datetime.now(timezone.utc).astimezone().isoformat()),
        "git_HEAD={}".format(head),
        "git_dirty={}".format("YES" if dirty else "NO"),
        "reference_bag={}".format(schema["path"]),
        "reference_bag_sha256={}".format(schema["sha256"]),
        "camera_left_count={}".format(inventory["stereo"]["left_count"]),
        "camera_right_count={}".format(inventory["stereo"]["right_count"]),
        "stereo_matched_pairs={}".format(inventory["stereo"]["matched_pairs"]),
        "lidar_count={}".format(source_summary["streams"]["lidar"]["count"]),
        "imu_count={}".format(source_summary["streams"]["imu"]["count"]),
        "gnss_count={}".format((gnss or {}).get("valid_rows", 0)),
        "first_stamp={}".format(source_summary["global"]["first_ns"]),
        "last_stamp={}".format(source_summary["global"]["last_ns"]),
        "duration={}".format(source_summary["global"]["duration_s"]),
        "BASE_ECEF={}".format(" ".join("{:.4f}".format(value) for value in inventory["base_ecef"])),
        "GNSS_solution_source={}".format((gnss or {}).get("mode", "not run in dry-run")),
        "raw_timestamps_modified=NO",
        "global_merge=timestamp heap merge",
        "dry_run={}".format("YES" if args.dry_run else "NO"),
    ]
    if validation:
        lines.extend(
            [
                "bag_first_stamp_s={:.9f}".format(validation["start_s"]),
                "bag_last_stamp_s={:.9f}".format(validation["end_s"]),
                "bag_duration_s={:.9f}".format(validation["duration_s"]),
                "bag_sha256={}".format(sha256_file(output, "OUTPUT SHA256")),
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def extract_zip(zip_path, password, keep_temp):
    temp_root = Path(tempfile.mkdtemp(prefix="tiaozhanbei_zip_"))
    command = ["7z", "x", "-y", "-o{}".format(temp_root)]
    if password is not None:
        command.append("-p{}".format(password))
    command.append(str(zip_path))
    result = run_stream(command, check=False)
    if result.returncode != 0:
        if not keep_temp:
            shutil.rmtree(str(temp_root))
        raise RuntimeError("ZIP extraction failed with exit {} (password not logged)".format(result.returncode))
    candidates = []
    for camera in temp_root.rglob("camera"):
        root = camera.parent
        if (
            (root / "camera" / "left").is_dir()
            and (root / "camera" / "right").is_dir()
            and (root / "imu" / "imu.csv").is_file()
            and (root / "gnss").is_dir()
            and ((root / "lidar" / "points.csv").is_file() or (root / "lidar" / "points.csv.gz").is_file())
        ):
            candidates.append(root)
    candidates = sorted(set(candidates))
    if len(candidates) != 1:
        if not keep_temp:
            shutil.rmtree(str(temp_root))
        raise RuntimeError("expected one dataset root after ZIP extraction, found {}".format(candidates))
    return temp_root, candidates[0]


def password_value(args, required):
    if args.password is not None:
        if args.password == "__PROMPT__":
            return getpass.getpass("ZIP password: ")
        return args.password
    value = os.environ.get("TIAOZHANBEI_ZIP_PASSWORD")
    if value:
        return value
    if required:
        return getpass.getpass("ZIP password: ")
    return None


def build(args):
    requested_dataset = args.dataset.expanduser().resolve()
    if (requested_dataset / "融合定位").is_dir():
        requested_dataset = requested_dataset / "融合定位"
    report_dir = (
        args.report_dir.expanduser().resolve()
        if args.report_dir else requested_dataset / "bag_build_report"
    )
    report_dir.mkdir(parents=True, exist_ok=True)
    for name in REPORT_FILES:
        path = report_dir / name
        if not path.exists():
            path.touch()
    phase(1, 7, "解析数据集路径和 ZIP 输入")
    zip_audit = zip_inventory(args.zip.expanduser().resolve()) if args.zip else None
    temporary = None
    dataset = requested_dataset
    if args.zip:
        password = password_value(args, required=not args.dry_run)
        if password is not None or zip_audit["encrypted_files"] == 0:
            temporary, dataset = extract_zip(args.zip.expanduser().resolve(), password, args.keep_temp)
            zip_audit["content_validated"] = True
        elif args.dry_run:
            if not requested_dataset.is_dir():
                raise FileNotFoundError("reference dataset for ZIP comparison does not exist: {}".format(requested_dataset))
        else:
            raise RuntimeError("encrypted ZIP requires --password prompt or TIAOZHANBEI_ZIP_PASSWORD")
    output = args.output.expanduser().resolve() if args.output else requested_dataset / "competition.bag"
    try:
        if not args.dry_run and output.exists() and not args.overwrite:
            raise FileExistsError("output bag exists; pass --overwrite: {}".format(output))
        phase(2, 7, "审计目录、双目配对和 JPEG 完整性")
        inventory = dataset_inventory(dataset)
        phase(3, 7, "检查参考 bag schema 并计算 SHA256")
        reference = reference_schema(args.reference_bag)
        write_reference_schema(report_dir / "bag_validation.txt", reference)
        write_stereo_report(report_dir / "stereo_pair_audit.txt", inventory, zip_audit)
        if zip_audit and temporary is None:
            write_dataset_audit(report_dir / "dataset_audit.txt", inventory, zip_audit=zip_audit)
            write_format_markdown(report_dir / "dataset_format_audit.md", inventory, zip_audit, reference)
            empty_source = {
                "streams": {name: engine.TimestampAudit(name).summary() for name in ("lidar", "imu", "camera", "gnss")},
                "global": {"first_ns": None, "last_ns": None, "duration_s": 0.0},
            }
            write_timestamp_csv(report_dir / "timestamp_audit.csv", empty_source)
            write_manifest(report_dir / "build_manifest.txt", args, dataset, output, inventory, reference, empty_source)
            print("PARTIAL_DATASET_AUDIT_WAITING_FOR_ZIP_PASSWORD")
            print("report_dir={}".format(report_dir))
            return 0
        stereo = inventory["stereo"]
        stereo_bad = any(
            (stereo["left_only"], stereo["right_only"], stereo["duplicate_left"], stereo["duplicate_right"])
        )
        if stereo_bad and not args.allow_stereo_mismatch:
            raise RuntimeError(
                "stereo mismatch: left={} right={} left_only={} right_only={}".format(
                    stereo["left_count"], stereo["right_count"], len(stereo["left_only"]), len(stereo["right_only"])
                )
            )
        if stereo_bad:
            with (report_dir / "stereo_pair_audit.txt").open("a", encoding="utf-8") as stream:
                stream.write("STRONG_WARN=continuing only because --allow-stereo-mismatch was supplied\n")
        phase(4, 7, "完整预检 Camera / LiDAR / IMU 时间戳和数据格式")
        check_json, check_text = decode_check(args.decode_script, dataset)
        if not np.allclose(check_json["base_ecef_m"], inventory["base_ecef"], rtol=0.0, atol=1e-4):
            raise RuntimeError("decode_gnss.sh BASE ECEF differs from current RINEX header")
        preflight_json = report_dir / "source_preflight.json"
        engine_command = [
            sys.executable,
            str(HERE / "tiaozhanbei_dataset_to_rosbag.py"),
            "--dataset",
            str(dataset),
            "--dry-run",
            "--skip-gnss",
            "--progress-label",
            "PREFLIGHT",
            "--summary-json",
            str(preflight_json),
        ]
        run_stream(engine_command)
        source_summary = json.loads(preflight_json.read_text(encoding="utf-8"))
        write_dataset_audit(report_dir / "dataset_audit.txt", inventory, source_summary, zip_audit)
        if zip_audit:
            write_format_markdown(report_dir / "dataset_format_audit.md", inventory, zip_audit, reference)
        write_timestamp_csv(report_dir / "timestamp_audit.csv", source_summary)
        (report_dir / "gnss_audit.txt").write_text(
            "GNSS_INPUT_CHECK=PASS\n{}\nBASE_ECEF={}\nGNSS_DECODE=SKIPPED_IN_DRY_RUN\n".format(
                check_text.rstrip(), " ".join(str(value) for value in inventory["base_ecef"])
            ),
            encoding="utf-8",
        )
        if args.dry_run:
            write_manifest(report_dir / "build_manifest.txt", args, dataset, output, inventory, reference, source_summary)
            with (report_dir / "bag_validation.txt").open("a", encoding="utf-8") as stream:
                stream.write("\nDRY_RUN=PASS\nNo GNSS decode was run and no bag was written.\n")
            print("DRY_RUN=PASS")
            print("report_dir={}".format(report_dir))
            return 0
        phase(5, 7, "运行正式离线 GNSS 解算并校验来源")
        gnss, decode_text = prepare_gnss(args, dataset, inventory, source_summary, report_dir)
        (report_dir / "gnss_audit.txt").write_text(
            "GNSS_INPUT_CHECK=PASS\n{}\nGNSS_DECODE_OUTPUT:\n{}\nGNSS_AUDIT={}\n".format(
                check_text.rstrip(), decode_text.rstrip(), json.dumps(gnss, ensure_ascii=False, sort_keys=True)
            ),
            encoding="utf-8",
        )
        expected_events = sum(
            int(source_summary["streams"][name]["count"])
            for name in ("lidar", "imu", "camera")
        ) + int(gnss["valid_rows"])
        phase(6, 7, "按原始时间戳合并并写入 ROS bag")
        build_summary_path = report_dir / "source_build.json"
        command = [
            sys.executable,
            str(HERE / "tiaozhanbei_dataset_to_rosbag.py"),
            "--dataset",
            str(dataset),
            "--output",
            str(output),
            "--gnss-solution",
            gnss["csv"],
            "--compression",
            args.compression,
            "--expected-events",
            str(expected_events),
            "--progress-label",
            "BUILD",
            "--summary-json",
            str(build_summary_path),
        ]
        if args.overwrite:
            command.append("--overwrite")
        build_result = run_stream(command)
        final_source = json.loads(build_summary_path.read_text(encoding="utf-8"))
        phase(7, 7, "逐消息校验 bag 并计算输出 SHA256")
        validation = validate_bag(output, reference, final_source, inventory["stereo"]["left_timestamps"])
        write_timestamp_csv(report_dir / "timestamp_audit.csv", final_source, validation)
        with (report_dir / "bag_validation.txt").open("a", encoding="utf-8") as stream:
            stream.write("\nOUTPUT_BAG_VALIDATION=PASS\n")
            stream.write(validation["rosbag_info"])
            stream.write("\nvalidation={}\n".format(json.dumps(validation, ensure_ascii=False, sort_keys=True)))
            stream.write("\nbuilder_output:\n{}".format(build_result.stdout))
        write_manifest(
            report_dir / "build_manifest.txt", args, dataset, output, inventory, reference, final_source, gnss, validation
        )
        print("PASS_BAG_BUILDER")
        print("output_bag={}".format(output))
        print("report_dir={}".format(report_dir))
        return 0
    except Exception as exc:
        with (report_dir / "bag_validation.txt").open("a", encoding="utf-8") as stream:
            stream.write("\nFAIL_BAG_BUILDER\n{}\n".format(exc))
        print("FAIL_BAG_BUILDER: {}".format(exc), file=sys.stderr)
        print("diagnostics={}".format(report_dir), file=sys.stderr)
        return 1
    finally:
        if temporary is not None:
            if args.keep_temp:
                print("kept_extracted_dataset={}".format(temporary))
            else:
                shutil.rmtree(str(temporary))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Password handling:
  Prefer TIAOZHANBEI_ZIP_PASSWORD or use --password with no value for a hidden prompt.
  Passwords are never written to reports or manifests.
""",
    )
    parser.add_argument("--dataset", required=True, type=Path, help="dataset root (or output/report root with --zip)")
    parser.add_argument("--output", type=Path, help="output bag; default DATASET/competition.bag")
    parser.add_argument("--dry-run", action="store_true", help="audit inputs without GNSS solve or bag write")
    parser.add_argument("--reuse-gnss", action="store_true", help="reuse only a provenance-validated offline solution")
    parser.add_argument("--allow-stereo-mismatch", action="store_true")
    parser.add_argument("--overwrite", action="store_true", help="allow replacement of an existing output bag")
    parser.add_argument("--zip", type=Path, help="encrypted or plain dataset ZIP")
    parser.add_argument("--password", nargs="?", const="__PROMPT__", help=argparse.SUPPRESS)
    parser.add_argument("--keep-temp", action="store_true", help="retain ZIP extraction directory")
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    parser.add_argument("--report-dir", type=Path, help="override DATASET/bag_build_report")
    parser.add_argument("--reference-bag", type=Path, default=DEFAULT_REFERENCE_BAG)
    parser.add_argument("--decode-script", type=Path, default=DEFAULT_DECODE_SCRIPT)
    args = parser.parse_args(argv)
    args.reference_bag = args.reference_bag.expanduser().resolve()
    args.decode_script = args.decode_script.expanduser().resolve()
    if not args.reference_bag.is_file():
        parser.error("reference bag does not exist: {}".format(args.reference_bag))
    if not args.decode_script.is_file():
        parser.error("decode script does not exist: {}".format(args.decode_script))
    return args


if __name__ == "__main__":
    sys.exit(build(parse_args()))
