# Tiaozhanbei dataset ROS bag builder

This directory contains the dataset-only tooling for the audited Tiaozhanbei ROS1 schema. It does not change FAST-LIVO2 sensor processing or fusion parameters.

## One-command use

```bash
DATA_DIR="/media/gulu/xxx/新数据集"
tools/build_rosbag.sh --dataset "$DATA_DIR"
```

The default output is `$DATA_DIR/competition.bag`. Use `--output /path/to/output.bag` to place it elsewhere. Existing bags are rejected unless `--overwrite` is explicit.

Useful preflight:

```bash
tools/build_rosbag.sh --dataset "$DATA_DIR" --dry-run
```

Dry-run discovers and fully parses Camera/LiDAR/IMU, checks unique RINEX inputs and the reference bag schema, but does not run RTKLIB and does not write a bag.

The command prints each stage immediately. Long operations show a text progress bar,
elapsed time, current rate, and ETA. LiDAR CSV reading estimates progress from file
bytes; bag writing and validation use the audited message count, so those percentages
are exact.

The supported options are:

- `--dataset PATH`
- `--output BAG`
- `--dry-run`
- `--reuse-gnss`
- `--allow-stereo-mismatch`
- `--overwrite`
- `--zip ZIP`
- `--keep-temp`
- `--compression none|bz2|lz4`
- `--report-dir DIR`

For an encrypted ZIP, set `TIAOZHANBEI_ZIP_PASSWORD` in the environment or pass `--password` with no value to receive a hidden prompt. The password is not written to logs, reports, examples, or manifests.

## Data and time contract

- Camera timestamps are integer nanoseconds from JPEG filename stems.
- IMU timestamps are the original `imu.csv/timestamp_ns` values.
- LiDAR scan timestamps are the original `frame_timestamp_ns` values. PointCloud2 `time` is FLOAT64 seconds relative to scan start, copied from `point_time_s`; it is never set to zero or converted to an absolute point timestamp.
- GNSS comes only from `decode_gnss.sh offline`. BASE ECEF is read from the current BASE RINEX `APPROX POSITION XYZ`; ENU and covariance come from the resulting formal CSV.
- Streams retain their native rates and timestamps. The writer performs a heap merge and uses `bag.write(..., t=msg.header.stamp)`; it does not resample, interpolate, or snap one sensor to another.

The audited reference bag contains only the left compressed JPEG topic. Both raw stereo directories are still required and paired before building, but the right camera is not added to the bag.

On this workstation the convenience entrypoint is also installed at `/home/gulu/TiaoZhanBei/TIAOZHANBEI_FASTLIVO2_GNSS_CODE_20260826/scripts/build_rosbag.sh`; it delegates to `tools/build_rosbag.sh`, the single maintained entrypoint.

Every run writes diagnostics under `$DATA_DIR/bag_build_report/` unless `--report-dir` is supplied. A full build fails if topic/type/count/timestamp, JPEG, IMU, LiDAR point-time, GNSS covariance, or reference-schema validation fails.
