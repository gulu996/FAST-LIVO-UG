# P4-A LiDAR Frontend Geometry / Correspondence / Map Causality Isolation

Date: 2026-09-18

Checkout: `/home/gulu/catkin_ws/src/FAST_LIVO2`

Mode: diagnostic / shadow / counterfactual only
Ground truth used: **false**

## Decision

`LIDAR_FRONTEND_ROOT_CAUSE_READY = NO`

Primary classification: **ROOT-F — state-map correlation remains dominant but cannot yet be isolated**.

Secondary candidate: **ROOT-E — recent-map feedback / historical map correlation**, supported only by a small one-step recent-plane-exclusion sensitivity. It is not established as the cause because the failure frozen-map runs do not share the production initial condition and the exclusion evaluator is conservative plane-level filtering rather than a refitted point-level map replay.

Negative evidence:

- ROOT-A is not supported as the primary cause: point times are monotonic, the U2 fixed-velocity deskew cloud perturbation in the failure window is small, and U2 association overlap is not anomalous relative to the two normal datasets.
- ROOT-C is not supported as the primary cause: failure-window plane churn is low and voxel association overlap stays high.
- ROOT-D is present as ordinary structural anisotropy, but not isolated as failure-specific: normals are dominated by X/Z planes and the Schur-conditioned weak eigenvalue drops relative to 890–900 s, yet entropy and rank do not collapse uniquely in 900–906 s.
- ROOT-B cannot be tested independently in the current source: S0 already is IMU propagation from the previous corrected state; S1 and S2 reduce to the same propagation route, and no non-feedback G2 seed exists synchronously.

No production frontend change is recommended from P4-A. The next phase should first build a **same-run forked state+map replay from one serialized pre-failure snapshot**. It must clone the state, covariance, IMU/deskew history and voxel map, then run evolving/frozen/recent-point variants from the identical snapshot. That is a causality harness, not a threshold or production heuristic.

## What was added

All new behavior is behind `p4_frontend/enable=false` by default.

- Accepted-correspondence provenance: raw/deskewed/world point, stable point/voxel/plane identity, plane geometry, signed residual, `R_i`, normalized residual, Jacobian, support frames/times, plane drift and association change.
- Per-iteration churn, signed-residual, normal-distribution, raw/Schur translation information, age and recent-voxel-reuse summaries.
- Map insertion provenance and a default-off `frozen_map` counterfactual.
- U2 fixed-previous-velocity translation deskew shadow while keeping the production IMU rotation and bias treatment.
- Offline C0/Cfinal, recent-plane exclusion and trajectory/window evaluator.
- One assert-based self-test with more than eight independent checks.

Primary artifacts:

- `include/p4_frontend_diagnostics.h`
- `src/p4_frontend_diagnostics_self_test.cpp`
- `tools/evaluate_p4_frontend_causality.py`
- `reports/p4a_lidar_frontend_causality_metrics_20260918.json`

## Source contract: current checkout, not paper inference

| Stage | Current source behavior | State/map/ordering dependency |
| --- | --- | --- |
| Raw Livox point and time | `offset_time / 1e6` is stored in `PointType.curvature` in milliseconds; surface points are sorted by `curvature` (`src/preprocess.cpp:173-184`, `src/preprocess.cpp:387-395`). | Raw acquisition order is normalized by timestamp sort. |
| Scan boundaries | Scan end is header time plus last point curvature/1000 (`src/LIVMapper.cpp:3688`, `src/LIVMapper.cpp:3949`). | Depends on the actual last per-point offset. |
| IMU deskew seed | `UndistortPcl` receives `_state`, preserves its velocity/position/bg/ba/gravity, and initializes the scan trajectory from `state_inout.rot_end/pos_end/vel_end` (`src/IMU_Processing.cpp:244-260`, `293`, `348-352`). | This is the previous corrected production state, so an abnormal accepted velocity/bias does enter later scans. |
| Scan-internal propagation | Consecutive IMU samples are averaged; bg and ba are removed; gravity is added; rotation, position and velocity are integrated and saved in `IMUpose` (`src/IMU_Processing.cpp:379-482`). | Uses rotation, position, velocity, gyro bias, accel bias and gravity. |
| Backward compensation | Each point is transformed from its interpolated/extrapolated IMU pose into scan-end coordinates (`src/IMU_Processing.cpp:548-612`). | Uses the scan-internal trajectory and final propagated scan-end pose. |
| Downsample and prior | The undistorted body cloud is voxel-filtered; the propagated state transforms it to world before initial-map handling (`src/LIVMapper.cpp:2614-2674`). | Production deterministic feature sort remains unchanged and off unless already configured. |
| IEKF iteration | Every iteration transforms all downsampled points using the current iterate and calls `BuildResidualListOMP` (`src/voxel_map.cpp:836-868`). | Correspondences are rematched each iteration. Production OpenMP/order strategy is unchanged. |
| Plane match | Residual builder queries the current voxel/octree and returns the selected plane and signed point-to-plane distance. The diagnostic path records its stable root voxel plus plane pointer/ID. | Depends on the historical voxel map and current iterate. |
| H and R | Translation Jacobian is the plane normal. `R_i = 0.001 + J_plane P_plane J_plane^T + n^T P_point n` (`src/voxel_map.cpp:910-963`). | Uses plane covariance and propagated point covariance. |
| Normal equation/update | The original ESIKF normal-equation and covariance path remains intact (`src/voxel_map.cpp:975-1024`). | No P4 diagnostic value gates or scales the production update. |
| Candidate transaction | Invalid/rejected candidates restore the prior and rebuild associations on the restored pose (`src/voxel_map.cpp:1611-1633`). | REJECT does not enter the map. |
| Initial map | The first downsampled world scan builds the octree (`src/voxel_map.cpp:1655-1721`). | The first frame seeds historical geometry. |
| Later insertion | After StateEstimation/transaction, accepted points are transformed with the final state and passed to `UpdateVoxelMap` (`src/LIVMapper.cpp:2678-2689`, `2846-2902`). | The current scan cannot affect its own current solve, but an accepted scan can affect subsequent frames. |
| Sliding | Normal map sliding runs after insertion logic. P4 frozen mode also suppresses sliding so the snapshot remains immutable (`src/LIVMapper.cpp:2913-2919`). | No mutation after the configured diagnostic freeze time. |

## Data and replay evidence

| Dataset / variant | Window | Runner result | P4 rows / notes |
| --- | ---: | --- | --- |
| stadtgarten_seq1 F0 observe | 0–930 s | PASS, 7 child exits | 18,571 frames, 80,718 iterations, 39,728 detailed correspondences, 133.05 MB |
| stadtgarten_seq1 F1 frozen at 890 s | 0–930 s | PASS, 7 child exits | 752 analyzed frames, 38,049 detailed correspondences, 42.38 MB |
| stadtgarten_seq2 observe | 0–600 s | PASS, 7 child exits | 11,995 frames, 48,738 iterations, 15,926 detailed correspondences, 72.00 MB |
| stadtgarten_seq2 frozen at 560 s | 0–600 s | FAIL at final conversion | Algorithm children exited normally, but LIO had zero correspondences from early replay and backend TUM contained no rows; this run is not a frozen-map causal comparison. |
| construction_seq2 observe | +1 / 598 s | PASS, 7 child exits | 11,939 frames, 47,736 iterations, 29,051 detailed correspondences, 83.16 MB |
| construction_seq2 frozen at 560 s | +1 / 598 s | PASS, 7 child exits | 954 analyzed frames, 4,270 iterations, 37,041 detailed correspondences, 39.33 MB |

All variant selection and analysis used estimator/LiDAR/IMU/map data only. GT, GNSS error and reference trajectory error were not read.

### Replay repeatability boundary

The requested production threading strategy was deliberately not changed. As a consequence, independent runs are not interchangeable counterfactual branches:

- seq1 F0/F1 exact-timestamp position difference was already 10.027 m median in 880–890 s, before the 890 s freeze. It reached 20.138 m median in 900–906 s. F1's 4.677 m/s median pose-derived speed in 900–906 s therefore cannot be attributed to map freezing.
- seq2 observe/frozen differed by 51.78 m median in the first 25 s and the frozen run had no valid map/correspondences. Its configured freeze was still 560 s, proving this was pre-counterfactual divergence.
- construction observe/frozen was the only usable pair: identical to sub-micrometre scale in the first 100 s and 5.42 mm median apart in 550–560 s. After freeze, freeze-relative position separation became 0.127 m median, 0.208 m p95 and 0.295 m max over 38 s. The frozen branch's conditional weak eigenvalue fell from 234,901 before freeze to 9,851 after freeze, while observe remained about 124,878. Long frozen-map registration therefore perturbs a normal sequence materially; it is not a failure-specific remedy.

## Failure-window findings

Medians use the final production IEKF iteration unless stated otherwise.

| Metric | 880–890 | 890–900 | 900–906 | 906–930 | Interpretation |
| --- | ---: | ---: | ---: | ---: | --- |
| changed-plane ratio | 0.352% | 0.414% | 0.483% | 0.605% | No churn spike in failure. |
| voxel-ID Jaccard | 1.000 | 0.998 | 0.997 | 0.997 | Associations remain stable. |
| residual sign-flip ratio | 0.923% | 1.323% | 1.219% | 1.427% | No failure-specific sign switching. |
| weighted signed residual | +0.648 mm | +0.752 mm | +0.170 mm | +0.094 mm | Magnitude decreases in failure. |
| weak signed projection | -0.038 mm | -0.058 mm | +0.038 mm | +0.026 mm | Only 55% of failure frames are positive; no persistent same-sign push. |
| normal directional entropy | 0.828 | 0.828 | 0.835 | 0.850 | No entropy collapse. |
| octant coverage | 1.0 | 1.0 | 1.0 | 1.0 | All octants represented. |
| conditional translation min eigenvalue | 33,406 | 60,714 | 35,694 | 45,307 | Weaker than immediately before, but comparable to 880–890. |
| support age `<0.5 s` | 0.133% | 2.899% | 0.202% | 0.080% | Failure planes are not dominated by new points. |
| support age `5–20 s` | 63.37% | 38.71% | 67.17% | 7.38% | Failure primarily uses mature local history. |
| support age `>20 s` | 32.09% | 22.41% | 28.75% | 89.85% | Map ages naturally with motion/window. |
| recently updated root voxel, 1 frame | 46.44% | 53.75% | 50.34% | 46.02% | High, but not elevated at failure. |
| recently updated root voxel, 5 frames | 92.71% | 93.50% | 91.10% | 91.22% | High root reuse is normal for this map lifecycle. |

Detailed sampled failure correspondences had dominant absolute normal axes Z=56.0%, X=36.0%, Y=8.0%; all eight signed octants occurred. This is a structural anisotropy, not a rank-6 guarantee of isotropic translation information. Before failure the split was Z=60.1%, X=36.8%, Y=3.0%, so the failure does not introduce a new single-plane cluster.

No sampled correspondence used a support frame at or after its current frame. In the 900–906 detailed sample, 1.89% of retained support entries originated after the 900 s fault onset; after 906 s this rose to 5.39%. Thus accepted bad-state feedback exists, but it is not the dominant support population during onset.

## Counterfactuals

### Frozen map

The implementation is immutable after the diagnostic freeze: construction's updated-root count stayed at 98,123 and map rows reported `inserted=0,p4_frozen_map_counterfactual`. However:

- seq1 failure F0/F1 is causally invalid because the branches were already 10 m apart before freeze.
- seq2 is causally invalid because the frozen-labelled independent run failed long before its 560 s freeze.
- construction is comparable and shows that a 38 s freeze causes 0.127 m median relative separation and severe geometry-information loss on a normal sequence.

Therefore frozen map does not isolate ROOT-E in this experiment.

### Recent-point exclusion

The evaluator conservatively removes an entire plane if any retained support point lies inside the exclusion window. It does not refit the plane from remaining points.

For 12 sampled seq1 failure frames, final-correspondence weak-axis pose-only solve changed from +0.175 mm to:

- +0.191 mm excluding the last 1 frame;
- +1.086 mm excluding the last 3 frames;
- +0.936 mm excluding the last 5 frames;
- +1.713 mm excluding supports younger than 0.5 s;
- +1.587 mm excluding supports younger than 1 s.

All variants remained rank 6. Normal sampled changes were smaller but nonzero: roughly 0.2–0.3 mm relative to Cfinal on seq2/construction. This is the only mechanism-specific sensitivity observed, so ROOT-E remains a secondary candidate. It is insufficient for a production decision because it is a one-step plane-level shadow, not a point-level map refit or trajectory replay.

### Fixed correspondence

Across the same 12 failure samples:

- C0: rank 6, 572.5 correspondences median, translation solve 8.63 mm median, weak component -1.30 mm median.
- Cfinal: rank 6, 576 correspondences median, translation solve 7.76 mm median, weak component +0.175 mm median.

The association identities themselves are stable, but the linearized weak component changes across relinearization. The current evaluator solves C0 and Cfinal once from the recorded linearization; it does **not** execute a full nonlinear all-iterations-fixed-C0 trajectory branch. Consequently it rules out violent switching but cannot fully distinguish a nonlinear basin from a biased fixed optimum.

### Seed sensitivity

- S0 is the production IMU-propagated pose.
- S1 would duplicate S0.
- S2, “previous final pose + IMU increment,” is the same current source path because the previous final state seeds `UndistortPcl` and IMU propagation.
- S3 is unavailable without adding a new synchronized non-feedback state source; P3 G2 output is not such a synchronous registration seed.

No GT seed was used. Seed-basin causality therefore remains unisolated.

### Deskew sensitivity

U0 uses the previous corrected position/rotation/velocity/bg/ba/gravity and the current scan's IMU trajectory. Therefore abnormal accepted velocity/bias can affect later scans.

U1 is skipped: the production code already uses IMU scan-internal propagation, while necessarily starting from the previous corrected state. There is no separate “IMU-only without LiDAR hidden-state feedback” source in this architecture.

U2 keeps the production rotation/bias treatment but fixes translation to the previous seed velocity across the scan:

- seq1 failure point-shape delta: 0.076 mm median / 1.324 mm p95 mean-per-point; 0.112 mm median / 2.148 mm p95 maximum-per-frame.
- failure association plane Jaccard: 0.711 median; seq2 normal: 0.708; construction normal: 0.657.
- failure U2 weak solve: -1.45 mm median; the sampled normal values were -0.39 mm and +0.33 mm.

The shape and association sensitivity are not failure-specific, so U2 does not support ROOT-A as primary.

### Timestamp and IMU coverage

- Point-relative timestamps were monotonic in every logged frame; non-monotonic count was zero across all successful observe runs.
- The strict `imu_end >= propagation_end` flag was false for nearly all frames. In seq1 failure the end gap was 2.55 ms median, 4.47 ms p95 and 4.66 ms max; adjacent windows had similar 2–3 ms medians.
- Current source extends the last averaged IMU pair to the requested propagation end. This is a short tail extrapolation, not a point-order failure, and it is not unique to 900–906 s.

## Answers to the required 15 questions

1. **Does deskew use abnormal velocity/bias?** Yes. The previous corrected velocity, bg and ba seed the next scan; gravity, position and rotation are also used.
2. **Abnormal churn at 900–906?** No. Changed-plane median is 0.483%, voxel Jaccard 0.997 and sign-flip 1.219%.
3. **Persistent weak-direction signed bias?** No strong evidence. Median is +0.038 mm and only 55% of frames are positive.
4. **Structurally weak normal distribution?** Yes, X/Z dominate and the conditional minimum eigenvalue is lower than 890–900, but it is not a unique failure collapse.
5. **How new are referenced map points?** Median 0.202% are younger than 0.5 s; 67.17% are 5–20 s and 28.75% are older than 20 s.
6. **Does recent-map reuse rise abnormally?** No. One-/five-frame root reuse is 50.3%/91.1%, slightly below 890–900.
7. **Does frozen map change failure?** The independent seq1 branch changes dramatically, but the branch was already 10 m apart before freeze, so no causal answer is valid.
8. **Does recent exclusion change failure?** Yes, by up to about 1.5 mm additional weak-axis one-step correction versus Cfinal, but only in a conservative sampled plane-level shadow.
9. **Does fixed correspondence change failure?** C0 and Cfinal single solves differ in weak component, but identity churn is low and a full fixed-C0 nonlinear replay was not performed.
10. **Do seed changes enter a different basin?** Not answered: S0/S1/S2 are not independent in this source, and S3 is unavailable without a new feedback path.
11. **Does the deskew variant change failure?** It changes sampled associations, but no more than normal data; raw shape perturbation is small. No failure-specific deskew cause is shown.
12. **Do normal datasets behave the same?** They share stable correspondences, high root-voxel reuse and U2 association sensitivity. A valid construction freeze materially degrades geometry over 38 s; seq2 frozen is pre-freeze-invalid.
13. **Most likely primary root?** ROOT-F primary, ROOT-E secondary candidate.
14. **Production-fix threshold reached?** No.
15. **Next layer, not threshold?** Build a same-snapshot forked state+map counterfactual replay layer; do not tune residual, speed, iteration or map-age thresholds.

## Performance and scope

- A 60 s sampled diagnostic gate processed the same 1,195 frames as the non-detailed gate, passed all seven child-exit/window checks, took 96.1 s wall time at replay rate 0.8, peaked at 766 MiB RSS and wrote 19.64 MB of P4 CSV.
- Full observe runs peaked at roughly 1.61–2.14 GiB RSS and wrote 72–133 MB. Detailed provenance is therefore intentionally windowed/strided and remains default-off.
- The stadtgarten and construction datasets satisfy the requested current-schema failure/normal coverage. No independently confirmed tunnel/indoor current-schema dataset was added; that remains a coverage limitation.
- Ground truth remained excluded from design, variant selection and causal classification.
- No production heuristic, sensor noise, replay rate, ESIKF math, transaction, sequential detector, P2-D/P3-A/VIO/GNSS/UWB behavior or production threading strategy was changed.

## Final validation

- `catkin_make --pkg fast_livo -j2`: PASS.
- All 16 discovered C++ `*self_test` executables: PASS, including `p4_frontend_diagnostics_self_test`, transaction, degeneracy, motion-consistency, VIO, GNSS, UWB, point-time, preprocess and voxel-filter coverage.
- `tools/evaluate_p4_frontend_causality.py --self-test`: PASS.
- All five P4 YAML overrides parsed successfully.
- `roslaunch --nodes` with the seq1 observe override expanded successfully to the expected five nodes.
- `git diff --check`: P4 files are clean; the command remains nonzero only because the pre-existing user-owned `tools/make_competition_submission/petrochemical_stage6b_schedule.json` contains trailing whitespace. That unrelated schedule was not modified.
- Metrics SHA-256: `876ec2cf0a8fc51e236b02aeac9e00e7ae8931219b48d1654887871d73279c21`.
- No commit and no push were performed.
