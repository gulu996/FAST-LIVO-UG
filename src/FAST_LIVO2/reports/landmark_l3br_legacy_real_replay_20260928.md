# L3-BR legacy same-ID board real replay — 2026-09-28

This is an isolated legacy compatibility experiment. It does not validate the
formal unique-child-ID Landmark Board or enable production correction.

## 1. Bag audit

| Dataset | `rosbag info` duration | Topics | Image schema |
| --- | --- | --- | --- |
| `/home/gulu/data/qr_detect/20m.bag` | 1:59 (119 s) | `/left_camera/image` 596, `/livox/imu` 23847, `/livox/lidar` 597 | 596/596 frames 1280x1024 `rgb8`, step 3840 |
| `/home/gulu/data/qr_detect/40m.bag` | 1:59 (119 s) | `/left_camera/image` 596, `/livox/imu` 23870, `/livox/lidar` 596 | 596/596 frames 1280x1024 `rgb8`, step 3840 |

LiDAR uses `livox_ros_driver2/CustomMsg`. Bag record time is near 1970 uptime;
sensor header time is near 1590192xxx s. The replay and analysis use header
time. Names 20m and 40m are not exact reference ranges or ground truth.

## 2. Calibration audit

The dedicated 1280x1024 camera YAML copies the commented historical block in
`config/camera_pinhole_mid360.yaml`: fx=1273.525961550808,
fy=1277.522948544942, cx=612.4210788294631,
cy=492.3065791444537, distortion=(-0.11113915388551408,
0.1861858287554102,-0.0007605139024049476,-0.001935749448109755).
Candidate A copies the first commented old LiDAR/camera extrinsics in
`config/mid360.yaml`; B copies the second. A was replayed. Offline A/B
comparisons used the same accepted PnP poses, exact first raw-scan timestamp,
and raw LIO trajectory, not PnP RMSE. For ID 1 the A/B outbound-to-return
board-position differences were 0.445/7.786 m (20m) and 0.344/6.156 m (40m).
Candidate A is selected for this compatibility replay, not certified as a
physical calibration.

## 3. Legacy Board geometry

Historical `git show HEAD:src/FAST_LIVO2/config/mid360.yaml` gives board
width=0.8 m, height=0.6 m, marker size=0.16 m, marker-center half-spacing
x=0.28 m, y=0.18 m. Old `src/vio.cpp` places the four centers at
(-0.28,+0.18), (+0.28,+0.18), (-0.28,-0.18), (+0.28,-0.18) m.
This supersedes the approximate verbal 0.6x0.8 m description for replay.
However, ID 3 has persistent 4-marker joint-PnP RMSE near 9 px against the
8 px quality gate. A one-frame diagnostic varying only marker size yielded
about 2 px at 0.18–0.19 m, suggesting historical nominal geometry may differ
from the physical board. It was **not** fitted into the replay config. Physical
geometry therefore remains unresolved.

## 4. Marker manifest

An all-596-image ArUco `DICT_6X6_250` scan of each bag found IDs 1, 2, 3;
no ID 4. The historical configured roster is 1, 2, 3, 4 and the final isolated
YAML preserves it. The completed runs started with IDs 1, 2, 3; the all-frame
manifest confirms this omitted no detected ID in these bags. See
`Log/legacy_qr_marker_manifest.txt` for per-ID counts and episodes.

## 5. Episode manifest

All-frame detector episodes split at gaps >2 s (absolute image-header time):

| Bag/ID | Outbound/first | Return/second | Gap |
| --- | --- | --- | --- |
| 20m / 1 | 1590192050.6–2066.8 | 1590192113.6–2116.2 | 46.8 s |
| 20m / 3 | 1590192030.4–2044.2 | 1590192136.0–2140.2 | 91.8 s |
| 40m / 1 | 1590192442.8–2455.8 | 1590192489.4–2491.4 | 33.6 s |

20m ID 2 has episodes 2067.8–2070.6 and 2077.0–2089.8 separated by
6.4 s; the motion meaning of that short gap is not established. 40m ID 2 and
ID 3 have one detected episode each. Return detection for ID 3 in 20m is real,
but no ID 3 factor is admitted.

## 6. Legacy association

The adapter enumerates 24 four-marker, 24 three-marker, 12 two-marker, and
4 one-marker slot assignments. Each uses the existing joint-corner PnP,
pose validation, and covariance calculation. The first valid unambiguous
4/3-marker result becomes the standard `LandmarkObservation`; 2/1-marker
results stay diagnostic-only. Synthetic 4/3/ambiguity/2/1/formal-isolation
tests pass. Real admitted 3-marker observations: two in 20m, one in 40m.
These are assignment-unambiguous, but their two IPPE pose candidate RMSEs are
close (gaps about 0.02–0.03 px), so real planar-pose uncertainty is not fully
calibrated.

## 7. Continuous visibility behavior

| Bag | Backend accepted/processed | Sparse keyposes | Visual factors | Suppressed/duplicate factor attempts | Queue peak |
| --- | ---: | ---: | ---: | ---: | ---: |
| 20m | 24/24 | 14 | 14 | 10/10 | 1 |
| 40m | 21/21 | 13 | 13 | 8/8 | 1 |

Both queues drained to zero; overflow, out-of-order and backend duplicate
submissions were all zero. One visual factor is allowed per sparse
keypose/board/episode. No factor flooding was observed.

## 8. Partial visibility

Counts are actual frontend-processed frames (stride=4), in order 4/3/2/1
visible markers; a dash means the board had no return episode.

| Bag/ID | Outbound | Return | Admitted outbound/return |
| --- | --- | --- | --- |
| 20m / 1 | 12/2/0/2 | 1/0/2/0 | 11/1 |
| 20m / 2 | 11/2/1/1 | — | 12/— |
| 20m / 3 | 15/0/2/0 | 0/1/4/0 | 0/0 |
| 40m / 1 | 9/2/4/0 | 2/0/0/0 | 9/2 |
| 40m / 2 | 13/3/1/2 | — | 10/— |
| 40m / 3 | 0/0/4/1 | — | 0/— |

The 20m ID 3 return contains one 3-marker PnP with RMSE 12.30 px and four
2-marker frames. They correctly produce no factor.

## 9. Persistent re-observation

ID 1 is queued in two accepted episodes in each bag: 20m relative
27.900–35.900 s and 83.900 s; 40m relative 75.900–83.100 s and
117.500–118.300 s. The unchanged backend indexes `PersistentLandmark` by
physical ID and the graph uses `Symbol('l', id)`, so both episodes address
the same `l1`, rather than creating a new return landmark. Each final graph
has two landmark variables (`l1`, `l2`).

## 10. Real shadow graph

20m: 14 ISAM updates, 13 motion factors, 14 visual factors, 1 gauge prior.
40m: 13 ISAM updates, 12 motion factors, 13 visual factors, 1 gauge prior.
Both report `graph_degraded=0`, zero factor rejects, zero ISAM exceptions,
and conservative (uncalibrated) motion covariance. Under candidate A, raw
LIO-plus-PnP ID 1 outbound/return position means differ by 0.445 m (20m)
and 0.344 m (40m); these are internal consistency diagnostics only. The
current graph does not export a pre/post-return objective or optimized factor
residual, so improvement of either is **NOT_MEASURED** and no absolute
accuracy claim is made.

## 11. 20m versus 40m

Both show real ID 1 cross-episode factor flow. 20m sees 4-marker ID 3
frequently but rejects it under the historical geometry; 40m sees ID 3 only
as 1/2-marker partial visibility. The final A-based raw cross-episode ID 1
position difference is 0.445 m versus 0.344 m, not a ground-truth ranking.

## 12. VIS-B execution contract

Per bag, 592 completed raw scans appear in `livo_scan_contract.csv`; the
first two are IMU initialization with no LIO transaction, and each of the
remaining 590 has exactly one LIO transaction. There are 590 committed,
zero rejected LIO updates, 296 lifecycle-counted map insertions, and no scan
with >1 insertion. The first insertion is initial `BuildVoxelMap`, explaining
why `lio_frame_transaction.csv` marks 295 ordinary update insertions.
Image-triggered insertion is zero; all state times are monotonic. Both
589-row output trajectories are finite and strictly time-increasing. The
five/four LiDAR messages not present as completed scans are startup/tail
buffering, not partial-cloud LIO transactions.

## 13. Production isolation

Only `global_backend` is allowed in the legacy identity mode. The route
submits standard observations to the persistent shadow backend; it does not
call landmark ESIKF update. `latestCorrection()` returns false. No new
`map_to_odom` publisher was added; the existing fixed-lag backend remains
the owner of that production output, and it was not launched here. GNSS/UWB
fusion was disabled in the isolated replay overlay.

## 14. Build and self-tests

`catkin_make --pkg fast_livo -j4` passed. All 24 CMake-declared self-test
executables exited zero, including legacy association, formal frontend,
shadow graph, LIO transaction, VIS-B scan transaction, and VIO transaction.
`git diff --check` passed. The runtime needed process-local
`LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0` on this host; no global
library or algorithm change was made.

## 15. Git diff and evidence

This task adds `landmark_legacy_adapter.h/.cpp`, its self-test, a dedicated
camera YAML, mapping overlay, and launch. It modifies `CMakeLists.txt`,
`include/vio.h`, and `src/vio.cpp` for a default-off branch and isolated CSV.
Other dirty files, including existing L1–L3B work, predated this task and
were preserved. No reset, clean, stash, commit, or push occurred.

Retained ignored artifacts: `Log/legacy_qr_20m_full/2026-09-28-185531/`,
`Log/legacy_qr_40m_full/2026-09-28-190217/`; each contains effective
parameters, config snapshots, full replay logs, association and frontend
CSVs, VIS-B CSV, backend final summary, and raw trajectory. The all-frame
marker manifest is `Log/legacy_qr_marker_manifest.txt`.

## 16. Formal limitation and status

The two bags are `LEGACY_SAME_ID_REAL_REPLAY`, not formal unique-child-ID
positive samples. Historical geometry is sourced, but ID 3's physical
geometry mismatch and uncalibrated planar-pose/covariance behavior block
full L3-BR acceptance and formal L3C readiness.

```text
LANDMARK_L3BR_LEGACY_REAL_REPLAY_COMPLETE = NO
LEGACY_BOARD_MODE_READY = YES
FORMAL_UNIQUE_ID_MODE_PRESERVED = YES
LEGACY_BOARD_GEOMETRY_READY = NO
LEGACY_CAMERA_INTRINSIC_READY = YES
LEGACY_CAMERA_RESOLUTION = 1280x1024
EXTRINSIC_SELECTED = A
LEGACY_MARKER_ID_MANIFEST_READY = YES
REAL_REOBSERVATION_EPISODE_DETECTED = YES
LEGACY_4MARKER_ASSOCIATION = PASS
LEGACY_3MARKER_ASSOCIATION = PASS
LEGACY_2MARKER_FACTOR_ENABLED = NO
VISUAL_FACTOR_FLOODING = NO
PERSISTENT_LANDMARK_REOBSERVATION = PASS
REAL_VISUAL_FACTOR_FLOW = PASS
REAL_SHADOW_ISAM_UPDATE = PASS
REAL_CROSS_EPISODE_LANDMARK_CONSTRAINT = PASS
VIS_B_EXECUTION_CONTRACT_PRESERVED = YES
LANDMARK_BACKEND_SHADOW_ONLY = YES
ESIKF_MODIFIED_BY_GLOBAL_BACKEND = NO
NEW_MAP_TO_ODOM_PUBLISHER_CREATED = NO
PRODUCTION_CORRECTION_OWNER = EXISTING_FIXED_LAG_BACKEND
FORMAL_UNIQUE_ID_BOARD_VALIDATION = NOT_RUN
READY_FOR_FORMAL_L3C = NO
```
