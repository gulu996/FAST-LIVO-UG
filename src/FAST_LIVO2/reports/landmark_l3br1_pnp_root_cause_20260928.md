# L3-BR1 legacy ID3 PnP root-cause audit — 2026-09-28

This is a diagnostic audit of the isolated legacy same-ID replay, not formal
unique-child-ID Board validation. No localization algorithm, physical geometry,
8 px reprojection gate, or production correction path was changed.

## 1. Known physical geometry

The user's production drawing confirms all four old boards are 0.80 x 0.60 m,
with 0.16 x 0.16 m markers and center coordinates (±0.28, ±0.18) m. These are
physical inputs, not parameters to fit from the bag. The 0.18–0.19 m
one-frame what-if from L3-BR is only a diagnostic clue and is **not** adopted.

## 2. 90-degree placement analysis

Rotating a rigid board by about 90 degrees changes `T_camera_board`, not its
board-frame width/height, X/Y center spacing, or marker size. Swapping
half-spacing X/Y would describe a different object and is not done.

## 3. Object-point audit

`Log/legacy_qr_pnp_audit_20260928/legacy_object_points.csv` contains all 16
actual legacy-adapter PnP object points. By slot, TL/TR/BR/BL are:

| Slot | Center (m) | TL | TR | BR | BL |
| --- | --- | --- | --- | --- | --- |
| 0 | (-.28,+.18) | (-.36,+.26) | (-.20,+.26) | (-.20,+.10) | (-.36,+.10) |
| 1 | (+.28,+.18) | (+.20,+.26) | (+.36,+.26) | (+.36,+.10) | (+.20,+.10) |
| 2 | (-.28,-.18) | (-.36,-.10) | (-.20,-.10) | (-.20,-.26) | (-.36,-.26) |
| 3 | (+.28,-.18) | (+.20,-.10) | (+.36,-.10) | (+.36,-.26) | (+.20,-.26) |

All Z coordinates are zero. `LegacySameIdBoardAdapter::configure()` and
`LandmarkFrontend::markerCornersInLandmark()` generate exactly these points.

## 4. ArUco corner-order and internal orientation

The added self-test draws the actual DICT_6X6_250 ID 7 using OpenCV 4.2,
projects four copies onto a slightly tilted synthetic board, rotates the
whole image by 0/90/180/270 degrees, re-runs `detectMarkers()`, and verifies
that each decoded ID and semantic corner index follows the exact pixel
rotation. The adapter's 24-assignment joint PnP is accepted with <2 px RMSE
for every orientation. The object convention is marker canonical TL/TR/BR/BL.
An exactly fronto-parallel artificial board exposed an IPPE degeneracy in the
test setup; a tilted board was used. On real ID3 frames, ITERATIVE vs IPPE
median RMSE differs by only 0.013 px, so that degeneracy does not explain ID3.

Within real ID3 frames, the four detected marker canonical-X directions
have median angular spread 0.47 degrees (range 0.10–3.34). The median angle
between their average canonical-X direction and Board-X, inferred from the
four selected centers, is 0.0 degrees (range -0.12–0.83). ID1/ID2 medians
are 0.39/1.10 degrees. There is no evidence of an individual 90/180-degree
marker print-orientation mismatch. The physical drawing's print orientation
was not separately inspected.

## 5. Per-corner residual analysis

Lossless RGB bag frames for all **39** processed complete-board cases in
`/home/gulu/data/qr_detect/20m.bag` were extracted to
`Log/legacy_qr_pnp_audit_20260928/frames/`. The diagnostic executable uses
the linked OpenCV 4.2 detector, original subpixel settings, configured K/D,
and the unchanged adapter. It exactly reproduces all 39 runtime RMSE values
and all 39 selected assignments (maximum difference 0 px). Full 624-corner
observed/projected coordinates and vectors are in `legacy_corner_residuals.csv`;
39 annotated full-resolution overlays are in the same audit directory.

| ID | Complete frames | RMSE median/range (px) | Mean corner error by slot 0/1/2/3 (px) | Mean corner error by index 0/1/2/3 (px) |
| --- | ---: | --- | --- | --- |
| 1 | 13 | 1.531 / 0.790–2.144 | 1.484 / 1.472 / 1.235 / 1.322 | 1.335 / 1.398 / 1.318 / 1.463 |
| 2 | 11 | 1.317 / 0.836–1.673 | 1.192 / 1.279 / 1.107 / 1.198 | 1.077 / 1.314 / 1.305 / 1.081 |
| 3 | 15 | 9.289 / 8.713–15.342 | 9.388 / 9.148 / 9.651 / 9.403 | 9.267 / 9.467 / 9.312 / 9.544 |

Every ID3 full-board frame remains above the 8 px gate; none of ID1/2 does.
The ID3 error is distributed across all four markers and all four corner
indices, not a single mislabeled corner or one bad marker. Red projected vs
green detected overlay corners show a systematic footprint discrepancy.

To separate marker footprint from board-center placement without refitting
physical geometry, the *undistorted observed* four marker centers were used
to define a planar homography. It predicts each configured 0.16 m marker's
corner location. The observed/predicted edge-length ratios are:

| ID | Median across frames | Four-slot horizontal range of medians | Four-slot vertical range of medians |
| --- | ---: | ---: | ---: |
| 1 | 0.950 | 0.946–0.962 | 0.952–0.965 |
| 2 | 0.950 | 0.948–0.953 | 0.958–0.964 |
| 3 | **1.168** | **1.151–1.155** | **1.180–1.182** |

The ID3 per-frame median ratio ranges only 1.162–1.171. The center-anchored
homography's corner RMSE remains 10.053 px median, so a different PnP pose
alone cannot remove the discrepancy. This ratio is an image-space diagnostic,
**not** a replacement marker size or proof that the confirmed 0.16 m physical
measurement is wrong. It also does not alone identify print border, mounting,
physical center placement, or calibration as the underlying mechanism.

## 6. Assignment ambiguity

`legacy_assignment_candidates.csv` contains all 24 permutations per frame.
ID3's best-vs-next assignment RMSE gap is 84.338 px median (81.332–146.762),
and its ratio is 10.166 median. ID1/2 gaps are 36.697/38.842 px median.
Assignment ambiguity is not the source of ID3's residual.

## 7. Image position, viewing geometry, and quality

`legacy_frame_summary.csv` records each frame's principal-point radius,
view angle, PnP distance, hull area, marker side, board-ROI Laplacian variance,
and saturation fraction. ID3 medians: radius 249 px, angle 11.99 degrees,
distance 2.485 m, marker side 97.1 px, blur proxy 481, saturation 0.771.
ID1/2 radii are farther out (312/306 px); thus image-edge distortion is not
an adequate ID3-only explanation. A nearby ID1 control at relative 83.9 s is
closer (1.823 m), larger (114.8 px side), more oblique (16.93 degrees), and
has lower ROI Laplacian variance (273), yet RMSE is 1.81 px. Blur/size/angle
therefore do not support a 9 px corner-localization error. The ROI blur metric
is only a proxy, not a calibrated corner uncertainty. Corner subpixel
refinement is enabled in both runtime and probe.

## 8. Distortion-domain audit

The bag image is raw `rgb8`, 1280x1024; ROS `cv_bridge` converts it to BGR,
then VIO converts to gray and calls `detectMarkers()` with no resize or
undistortion (`scale=1`). PnP and `projectPoints()` both use the configured
old-camera K and radtan D exactly once. Thus `PNP_IMAGE_DOMAIN=RAW_DISTORTED`
and `PNP_DISTORTION_MODEL_APPLIED=YES` are internally consistent.

As a diagnostic only, PnP on the same corners gave ID3 median RMSE 9.289 px
with raw+K/D, 9.294 with raw+K/zero-D, and 9.410 with undistorted+K/zero-D.
ID1/2 remain near 1–1.5 px. This does not re-calibrate K/D; it rules out an
obvious double/omitted-distortion-domain failure as the 9 px cause. Camera–
LiDAR extrinsic Candidate A is downstream of camera-frame reprojection and
cannot account for the joint-PnP residual.

## 9. ID3 cause and uncertainty

**Confirmed failure mechanism:** ID3's detected four-marker footprint is
systematically incompatible with the fixed center-anchored planar 0.16 m
corner model, by about 16.8% in the dimensionless edge-length comparison.
**Underlying physical or calibration origin is unresolved.** This evidence
does not justify claiming that the known physical marker is actually larger,
or that a particular print border, marker mounting, warping, or camera
calibration error is confirmed. A ruler/print-boundary inspection of the
actual ID3 board, or an independent calibrated close-range image, is needed
to distinguish those possibilities. Uniform four-slot/two-axis scale and
stable cross-frame pattern do not positively identify non-planarity;
`PHYSICAL_PLANARITY_SUSPECTED=UNRESOLVED`.

The 40m bag has no complete four-marker ID3 observation, so it cannot provide
a like-for-like ID3 full-board comparison. No 40m result is repurposed as
positive geometry evidence.

## 10. Code changes

Only a diagnostic executable (`src/landmark_legacy_pnp_probe.cpp` and its
`CMakeLists.txt` target) and the 0/90/180/270-degree synthetic check in the
existing legacy self-test were added. The diagnostic executable exports
object points, per-corner residuals, per-frame metrics, 24 assignments, and
overlays. **NO ALGORITHM CHANGE.** No legacy geometry, reprojection gate,
formal unique-ID path, ESIKF, or graph correction logic was modified.

## 11. Replay and checks

There is no software fix to rerun. The already completed full 20m and 40m
L3-BR replays remain the runtime baseline; this audit reprocessed only the
39 selected lossless 20m frames, exactly matching their runtime PnP/assignment
results. The prior full replay reported the VIS-B scan transaction contract
passing and the backend shadow-only. This turn does not claim a new full-bag
runtime validation. The unchanged 8 px gate continues rejecting all 15
processed complete ID3 frames.

`catkin_make --pkg fast_livo -j4` passed. The legacy adapter, formal frontend,
measurement, architecture, persistent backend, shadow graph, VIS-B scan
transaction, and VIO transaction self-tests all exited zero.
`git diff --check` passed. Existing uncommitted L1–L3-BR work was preserved;
no reset, clean, stash, commit, or push was performed.

## 12. Formal limitation and final status

Legacy same-ID boards are not the project's formal unique-ID combined ArUco
Board. No real formal-board PnP accuracy, covariance calibration, partial
visibility, or production correction is validated here. Legacy mode remains
default-off and the backend remains shadow-only.

```text
LANDMARK_L3BR1_PNP_ROOT_CAUSE_AUDIT_COMPLETE = YES
LEGACY_PHYSICAL_GEOMETRY_CONFIRMED = YES
LEGACY_BOARD_WIDTH_M = 0.8
LEGACY_BOARD_HEIGHT_M = 0.6
LEGACY_MARKER_SIZE_M = 0.16
LEGACY_HALF_SPACING_X_M = 0.28
LEGACY_HALF_SPACING_Y_M = 0.18
BOARD_90DEG_PLACEMENT_REQUIRES_GEOMETRY_SWAP = NO
LEGACY_OBJECT_POINTS_MATCH_PHYSICAL_DESIGN = YES
ARUCO_90DEG_CORNER_ORDER_AUDIT = PASS
PNP_IMAGE_DOMAIN = RAW_DISTORTED
PNP_DISTORTION_MODEL_APPLIED = YES
PNP_DISTORTION_DOMAIN_AUDIT = PASS
IMAGE_CORNER_QUALITY_ROOT_CAUSE = NOT_SUPPORTED
PHYSICAL_PLANARITY_SUSPECTED = UNRESOLVED
ID3_PRIMARY_ROOT_CAUSE = UNKNOWN_UNDERLYING_ORIGIN; CONFIRMED_CENTER_ANCHORED_MARKER_FOOTPRINT_INCONSISTENCY
ID3_SECONDARY_ROOT_CAUSE = NONE_CONFIRMED
LEGACY_SOFTWARE_BUG_FOUND = NO
LEGACY_SOFTWARE_BUG_FIXED = NOT_APPLICABLE
REPROJECTION_GATE_RELAXED = NO
FORMAL_UNIQUE_ID_MODE_PRESERVED = YES
VIS_B_EXECUTION_CONTRACT_PRESERVED = YES (prior full replays; no algorithm change)
LANDMARK_BACKEND_SHADOW_ONLY = YES
ESIKF_MODIFIED_BY_GLOBAL_BACKEND = NO
NEW_MAP_TO_ODOM_PUBLISHER_CREATED = NO
PRODUCTION_CORRECTION_OWNER = EXISTING_FIXED_LAG_BACKEND
READY_TO_RERUN_L3BR = NO
```
