# Shadow Graph Re-observation Effect Diagnostics — 2026-10-02

Both complete real replays measured reduced ID1 cross-episode visual residuals
inside the actual Shadow Graph. The 40m event also caused a **2.883 m KeyPose
change**, which is explicitly flagged as a large shadow correction.

## 1. Current graph diagnostic gap

L3-BR previously proved that outbound and return observations addressed the
same persistent `Symbol('l',1)` and entered ISAM2. It did not record a common
visual factor's residual or the optimized variables immediately before and
after its update, so the consistency effect was `NOT_MEASURED`.

The datasets used this turn are `/home/gulu/data/qr_detect/20m.bag` (rosbag
duration 119.227516 s) and `/home/gulu/data/qr_detect/40m.bag` (119.351045 s).
Each contains 596 `/left_camera/image` messages; LiDAR is
`livox_ros_driver2/CustomMsg` on `/livox/lidar`, IMU is `/livox/imu`.
Both were played completely at rate 1.0, in the requested 20m-then-40m order,
using the same Candidate A extrinsic and effective graph/frontend parameters.
The checked `aruco_landmarks`, `extrin_calib`, `vio`, `lio`, `common`, and
`diagnostics` parameter blocks are equal between runs.

## 2. Diagnostic implementation

The hook is in `SparseLandmarkShadowGraph::update()`, immediately around the
existing factor insertion and **one** normal `isam_.update(factors, values)`.
No separate replay graph or extra ISAM update was created. Events require a
previously successful formal factor for the same physical landmark and a
different episode ID. Within-episode factors and deduped observations do not
generate new event rows. The gap uses the prior episode's last observation
delivered to the backend, including suppressed valid observations.

The actual frame convention is:

```text
K_i = T_W_body       (raw FAST-LIVO IMU/body sparse pose)
L_j = T_W_landmark   (self-mapped Pose3 random variable)
Z   = T_body_camera * T_camera_landmark
Z_hat = K_i.inverse() * L_j
E = Z.inverse() * Z_hat
r = Pose3::Logmap(E) = [omega_x, omega_y, omega_z, rho_x, rho_y, rho_z]
```

Rotation components are radians; translation Lie-algebra components are
metres. The translation residual reported here is `||rho||`, not necessarily
the Euclidean norm of `E.translation()` for nonzero rotation. This build's
GTSAM has `GTSAM_POSE3_EXPMAP`; every recorded Logmap vector is checked against
the actual `BetweenFactor<Pose3>::unwhitenedError()`. The same factor's
`whitenedError()` and `error()` are read before and after. For its existing
Gaussian model, `q = ||whitenedError||² = rᵀΣ⁻¹r` and `factor_error = q/2`;
both equalities were independently checked using the exported full covariance.
This is graph measurement error, **not** the legacy fixed-landmark NIS.

The visual right-local covariance is unchanged:
`Σ_right = diag(R_lc,R_lc) Σ_camera diag(R_lc,R_lc)ᵀ`. Its full 6x6 matrix,
measurement Z, before/after K/L translation and quaternion, residual vectors,
weighted errors, pose deltas, covariance source, and pure ISAM update latency
are saved in `landmark_reobservation_diagnostics.csv`.

**Pre-state detail:** the backend creates a new return KeyPose in this normal
update. It has no earlier ISAM estimate. For such a pose, pre diagnostics use
the unchanged raw pose already prepared in `new_values`, together with the
existing Landmark's current ISAM estimate. CSV marks this explicitly as
`NEW_RAW_VALUES_SEED`; an already-existing KeyPose uses `CURRENT_ISAM_ESTIMATE`.
No initial value was changed to obtain these numbers. KeyPose deltas describe
the actual batch update, which inserts its normal raw-motion factor along
with the return visual factor. They do not isolate the visual factor through
a counterfactual motion-only optimization.

Graph costs have distinct names and meanings:

- `pre_existing_graph_error`: old factors at the pre values.
- `pre_existing_plus_candidate_error`: old factors plus the candidate visual
  factor, excluding any new motion factor.
- `pre_augmented_graph_error`: old factors plus **all** pending normal factors,
  including motion and candidate visual factors, at the pre values.
- `post_full_graph_error`: that same augmented factor set at the post values.

Only the last two form a same-factor-set total-cost comparison. Diagnostics
catch their own failures independently of factor admission and ISAM failure
handling. Records contain no images or point clouds and are written after
worker drain at normal shutdown. Storage grows with episode transitions;
long-running deployments can replace retained records with streaming.

## 3. Synthetic test

The existing shadow-graph self-test now creates K0 observing L7, a three-edge
motion chain with translation/yaw drift, and K3 observing L7 in a new episode.
It verifies the same factor's pre/post residual and weighted error, Gaussian
cost identity, Logmap/factor chart equality, both variable movements, and CSV
event count. It also covers a current-ISAM KeyPose, 30 deduped observations,
and an additional accepted factor on a new keypose within the same episode.

| Metric | Pre | Post |
| --- | ---: | ---: |
| Translation residual (m) | 0.305068 | 0.00384450 |
| Rotation residual (deg) | 1.71887 | 0.0492422 |

K3 moves 0.292279 m; L7 moves 0.00389962 m. The test fails if L7 is frozen.
`LANDMARK_REOBSERVATION_RANDOM_VARIABLE_BEHAVIOR=PASS`.

## 4. 20m ID1

Actual return event: relative timestamp 83.899724 s, observation 24, sparse
keypose 14, landmark 1, episode 1 → 3. The backend-valid observation gap is
47.999928 s. The previously reported 46.8 s gap was based on all detector
frames, rather than the backend's stride/gate-admitted observations.

| Metric | Pre | Post |
| --- | ---: | ---: |
| Translation residual (m) | 1.337028 | 0.00550173 |
| Rotation residual (deg) | 13.652088 | 0.0547338 |
| Whitened residual norm | 2058.813704 | 8.406549 |
| Factor error (`q/2`) | 2119356.933892 | 35.335034 |

K14 changes 0.953098 m / 13.651034 deg. L1 changes 1.378889e-7 m /
5.142687e-5 deg. ISAM update latency is 0.349524 ms.
`SHADOW_REOBSERVATION_LARGE_CORRECTION=NO`.

Graph costs: pre existing 3796.801127; pending motion 0.060445; existing plus
candidate 2123153.735019; pre augmented 2123153.795464; post full 9680.458888.
The old-factor-only and post-full numbers must not be directly compared.
The same candidate and same augmented factor-set costs decrease.

## 5. 40m ID1

Actual return event: relative timestamp 117.500253 s, observation 20, sparse
keypose 12, landmark 1, episode 2 → 3. The backend-valid gap is 34.400145 s;
the earlier all-detector-frame gap was 33.6 s.

| Metric | Pre | Post |
| --- | ---: | ---: |
| Translation residual (m) | 3.206182 | 0.0247761 |
| Rotation residual (deg) | 11.312315 | 0.131012 |
| Whitened residual norm | 5808.020493 | 45.148683 |
| Factor error (`q/2`) | 16866551.024658 | 1019.201766 |

K12 changes **2.883315 m** / 11.312617 deg. L1 changes 3.394320e-7 m /
1.848751e-5 deg. ISAM update latency is 0.100263 ms.
`SHADOW_REOBSERVATION_LARGE_CORRECTION=YES`: the diagnostic threshold is
≥2 m translation or ≥30 deg rotation for either K or L. This threshold
only labels the event; it changes no factor weight, gate, or production state.

Graph costs: pre existing 758.877158; pending motion 1.144201; existing plus
candidate 16867309.901816; pre augmented 16867311.046017; post full 1778.141673.
The same candidate and same augmented factor-set costs decrease. Tiny real
Landmark changes reflect the existing gauge/outbound constraints, not a
constant Landmark variable; the synthetic test explicitly verifies mobility.

Both cases support **improved internal cross-episode landmark consistency**.
They do not measure absolute localization accuracy. The nonzero post residual,
particularly the 40m weighted cost, is retained; no weight was increased to
force zero residual. Conservative motion uncertainty and real visual
covariance remain uncalibrated.

## 6. Factor sparsity

| Count | 20m | 40m |
| --- | ---: | ---: |
| Raw frontend board observations | 56 | 41 |
| Backend submitted / accepted / processed | 24 / 24 / 24 | 21 / 21 / 21 |
| Sparse KeyPose variables | 14 | 13 |
| Landmark Pose3 variables | 2 | 2 |
| Motion factors | 13 | 12 |
| Visual factors | 14 | 13 |
| Suppressed visual observations | 10 | 8 |
| Graph duplicate factor attempts | 10 | 8 |
| Cross-episode diagnostic rows | 1 | 1 |

Backend duplicate **submissions** are zero in each run; the graph duplicate
counts refer to valid observations suppressed by its existing sparse-factor
policy. There is one gauge prior per run and no Landmark prior/freeze.
Both return episodes address `l1`; ordinary continuous factors, including
the later 40m return observation, produce no extra long-interval event.
`VISUAL_FACTOR_FLOODING=NO`.

## 7. Numerical health

Both completed runs: zero ISAM exceptions, zero diagnostic failures, zero
graph factor rejects, `graph_degraded=0`, queue peak 1, queue drained to 0,
overflow/out-of-order/backend duplicates 0, worker start/stop count 1/1,
drain/join completed. All required pre/post numerical CSV fields are finite,
their covariance matrices are positive definite, and both raw output
trajectories have 589 finite, strictly time-increasing poses. The existing
`scans_pos.json` files were inspected: despite their suffix, their contents
are timestamp/XYZ/quaternion text. Neither completed replay had a mapping
crash, FATAL, or deadlock. Existing inactive legacy diagnostics can still use
NaN placeholders; these are not NaN estimates or re-observation measurements.

Initial setup attempts never played a bag: a copied launch snapshot caused
ROS filename ambiguity, then a TIME_WAIT port check and inherited image-off
setting stopped startup. Their evidence is retained in the task log directory.
The runner now uses an absolute launch path, and the dedicated legacy overlay
explicitly enables image input. Completed-run results above exclude those
startup-only attempts.

## 8. VIS-B

| Runtime count | 20m | 40m |
| --- | ---: | ---: |
| Completed raw scans | 592 | 592 |
| IMU initialization scans (no measurement required) | 2 | 2 |
| LIO attempted / committed / rejected | 590 / 590 / 0 | 590 / 590 / 0 |
| Lifecycle map insertions | 296 | 296 |
| Ordinary update insertions in LIO transaction CSV | 295 | 295 |
| Image received / synced / processed | 596 / 590 / 589 | 596 / 590 / 589 |
| Input buffer overflow | 0 | 0 |

Each measurement-required completed scan has exactly one LIO transaction;
every transaction timestamp matches its raw scan end within 1e-6 s. Every
scan has ≤1 insertion and monotonic state time. The initial BuildVoxelMap
insertion explains the lifecycle/ordinary insertion count difference.
Missing startup/tail messages are not partial-cloud LIO updates. The unchanged
map-insertion code remains in `handleLIO()`; no image path gained an insertion.

```text
SCAN_TRANSACTION_CONTRACT = PASS
PARTIAL_CLOUD_LIO_REINTRODUCED = NO
IMAGE_TRIGGERED_MAP_INSERTION = NO
STATE_TIMESTAMP_MONOTONIC = PASS
VIS_B_EXECUTION_CONTRACT_PRESERVED = YES
```

## 9. Production isolation

`latestCorrection()` still returns false and an invalid correction. No
ESIKF update, voxel-map logic, GNSS/UWB route, fixed-lag backend, or publisher
was changed. The existing fixed-lag backend remains the production correction
owner. New pose values and the 40m large correction stay inside the Shadow
Graph. The frontend, physical legacy geometry, Candidate A, and 8 px gate are
preserved; ID3 is still rejected under the documented unresolved footprint
issue. These are `LEGACY_SAME_ID_REAL_REPLAY` datasets, not formal unique-ID
Board validation.

## 10. Git diff, checks, and evidence

The initial worktree was clean on `main`, HEAD `2cd38f2`. Changes are limited
to the graph diagnostic struct/history/computation/CSV writer, the backend's
read-only writer and counters, VIO shutdown CSV output, the existing graph
self-test, the dedicated legacy image-enable overlay, this report, and
`reports/run_landmark_reobservation_replay.py`. No reset/clean/restore/checkout/
stash/commit/push was performed. Production `config/mid360.yaml` was preserved.

`catkin_make --pkg fast_livo -j4` passed. All eight required related tests
passed: frontend, measurement, architecture, persistent backend, shadow graph,
legacy adapter, VIS-B scan transaction, and VIO transaction. `git diff --check`
passed. Replay needed only process-local `LD_PRELOAD` for this host's libusb
ordering; no global library configuration changed.

Retained evidence:

- `Log/landmark_reobservation_20261002/analysis.json`
- `Log/landmark_reobservation_20261002/20m/2026-10-02-191811/landmark_reobservation_diagnostics.csv`
- `Log/landmark_reobservation_20261002/40m/2026-10-02-192152/landmark_reobservation_diagnostics.csv`
- Each run's normal frontend, scan/LIO transaction, runtime counters, final
  backend summary, trajectory, full ROS logs, bag info, effective parameters,
  and source configuration snapshots.
- Task-root build/self-test logs, `source.diff`, and `runtime_sha256.txt`.

For a fresh replay, source `/home/gulu/catkin_ws/devel/setup.bash` and run
`python3 reports/run_landmark_reobservation_replay.py 20m NEW_OUTPUT_ROOT`.
After checking it, run the same command with `40m` and a different fresh root.
The runner refuses to overwrite earlier evidence and stops only owned ROS
process groups.

```text
LANDMARK_REOBSERVATION_DIAGNOSTICS_READY = YES
REAL_20M_REOBSERVATION_EFFECT_MEASURED = YES
REAL_40M_REOBSERVATION_EFFECT_MEASURED = YES
REAL_REOBSERVATION_PRE_POST_RESIDUAL_READY = YES
REAL_REOBSERVATION_KEYPOSE_DELTA_READY = YES
REAL_REOBSERVATION_LANDMARK_DELTA_READY = YES
LANDMARK_REOBSERVATION_RANDOM_VARIABLE_BEHAVIOR = PASS
REAL_CROSS_EPISODE_CONSISTENCY_CHANGE = IMPROVED
SHADOW_REOBSERVATION_LARGE_CORRECTION_20M = NO
SHADOW_REOBSERVATION_LARGE_CORRECTION_40M = YES
VISUAL_FACTOR_FLOODING = NO
ISAM_EXCEPTION_COUNT = 0
VIS_B_EXECUTION_CONTRACT_PRESERVED = YES
LANDMARK_BACKEND_SHADOW_ONLY = YES
ESIKF_MODIFIED_BY_GLOBAL_BACKEND = NO
NEW_MAP_TO_ODOM_PUBLISHER_CREATED = NO
PRODUCTION_CORRECTION_OWNER = EXISTING_FIXED_LAG_BACKEND
FORMAL_UNIQUE_ID_BOARD_VALIDATION = NOT_RUN
READY_FOR_FORMAL_L3C = YES
```

Here readiness means the software, actual legacy cross-episode shadow behavior,
and diagnostics are ready to await the formal unique-ID physical Board. It
does not grant production correction ownership or substitute the legacy bags
for a formal L3-C experiment. The flagged 40m large correction remains an
explicit observation to investigate with calibrated formal-board data.
