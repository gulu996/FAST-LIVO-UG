# Public tunnel recording and LIVO replay

**Purpose:** provide a public, self-collected input for the frontend path, and
separate its results from the private competition 804 s recording. This page
does not claim ATE, ground truth or independent reproduction.

## Dataset identity

The maintainer shares a tunnel recording. It is a separate recording from the
private competition bag and is not stored in this Git repository.

| Item | Value |
|---|---|
| Share link | <https://pan.baidu.com/s/1vArBc4Gn5unoczOqP880RQ?pwd=mbpr> |
| Extraction code | `mbpr` |
| Share folder | `/采集数据集/隧道` |
| File | `suidao2_resized.bag` |
| Size | 16,334,415,367 bytes |
| SHA-256 (maintainer copy) | `9289318857b0ca009dab4570ac6f51915de1681fe60b2abc0b481c20d6711169` |

Link check performed on **2026-10-28**: the share page returned HTTP 200, the
extraction code was accepted by Baidu's verify endpoint, and the share listing
reported the file name and size above. The file was **not** downloaded again in
this documentation pass. The SHA-256 is from the maintainer's local copy whose
size exactly matches the share listing. Recompute it after download; a mismatch
means the share content changed or the download is incomplete.

The maintainer states that the recording is self-collected and publicly
shareable. This repository does not redistribute the bag. Users are responsible
for checking privacy, participant and site constraints before further
redistribution.

## Bag metadata

`rosbag info` on the local maintainer copy:

- Duration: **355 s** (5:55); 78,063 messages.
- `/livox/lidar`: 3,549 `livox_ros_driver2/CustomMsg` scans.
- `/livox/imu`: 70,963 `sensor_msgs/Imu` samples.
- `/left_camera/image`: 3,551 raw `sensor_msgs/Image` frames, 1300x1080 `rgb8`.
- Bag clock starts at 510.32 s; replay with `--clock` and do not treat the
  epoch-shaped header values as a wall-clock date.

The recording contains no GNSS or UWB stream. The tunnel replay is a frontend
LIO/LIVO replay; do not start the GNSS adapter or fixed-lag backend nodes.

## Code and configuration

The 0.542 m result below was produced with the maintainer-local commit
`5be5538b922ab3a621cd902ecb60b2658aeb3685` and this configuration:

- `config/mid360.yaml` (MID-360/Livox profile), with `common/img_en: 1`;
- `config/camera_pinhole_mid360.yaml` (2600x2160 model, `scale: 0.5`);
- `/diagnostics/livo_scan_contract_en: true`;
- `/pcd_save/pcd_save_en: false`;
- external GNSS/UWB factors not started.

The tracked `config/mid360.yaml` has `img_en: 0`; the runtime override to `1` is
required for the visual path. The run command is in the
[root README](../../README.md#public-tunnel-dataset-and-reproduction). Build
instructions are in the [build guide](../BUILD_AND_CAPTURE.md).

**Publication gap:** `5be5538` is one commit ahead of `origin/main`
(`02343a3`) and was not pushed in this documentation pass. A fresh clone of the
public branch does not contain it. The 0.542 m result is therefore tied to the
maintainer-local revision; it is not yet reproducible from the public branch
alone.

## Maintainer-run result

| Item | Value |
|---|---|
| Code | local `5be5538` |
| LIO commits | 3542 |
| First/last position distance | **0.542 m** |
| Maximum adjacent-scan speed | 2.02 m/s |
| Consecutive scans > 5 m/s | 0 |
| 233–260 s reverse segment | 0.473 m |
| Input | `suidao2_resized.bag`, full 355 s |

First/last distance is a revisit-consistency diagnostic, **not ATE**. The
maximum scan-difference speed is a stability diagnostic, not physical truth.
The accepted visual count and exact trajectory can vary with scheduling and
threading; the counts in this table are not accuracy scores.

## Rejected candidates from the same audit

These candidates were tested during the 2026-10-28 audit and are **not** part of
`5be5538`. They are recorded here so they are not mistaken for fixed features.

| Candidate | Public tunnel result | Status |
|---|---|---|
| A2 LiDAR query covariance, query-only | first/last 2.159 m; late 9.69 m reverse segment | rejected |
| A2 query + stored-map covariance | first/last 15.316 m; 2 s displacement 6.74 m | rejected |
| A6 scalar `Exp` small-angle | first/last 0.5475 m; competition 72.76 m | rejected |
| A5/A1 + A6 combination | first/last 0.535 m; 3.008 m reverse in 233–260 s | rejected |

The internal audit report with raw evidence is maintainer-local and is not
linked as a public artifact. The tunnel dataset identity above is independent
of those rejected candidates.

## What this page does and does not establish

- It establishes a public input with a checked share link and a maintainer-run
  replay result.
- It does **not** establish independent reproduction, ATE, absolute accuracy or
  physical correctness of all deskew/occlusion modes.
- It does **not** grant rights to the private competition bag, the GNSS/UWB
  recordings, site maps or images.
- The repository's license review is incomplete; see
  [third-party terms](../../THIRD_PARTY_LICENSES.md) and
  [release readiness](../RELEASE_READINESS.md).
