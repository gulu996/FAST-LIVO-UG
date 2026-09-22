# P3-A full-state fixed-lag shadow prototype

Date: 2026-09-18  
Checkout: `/home/gulu/catkin_ws/src/FAST_LIVO2`  
Base commit: `f35e2aefa8a76fc3c75a31e4ed00273b4de0b6da`  
Decision: `FULLSTATE_SMOOTHER_ARCH_READY = NO`  
Failure class: **NO-B** -- the extracted LiDAR geometry / correspondence / map
path carries the failure; explicit IMU temporal state does not remove it.

## 1. Executive result

G2 is a real `X(k), V(k), B(k)` fixed-lag smoother using a LiDAR-only final
point-to-plane normal equation, raw IMU preintegration, and bias random walk.
It is shadow-only: no optimized state is consumed by FAST-LIVO, the voxel map,
IMU propagation, VIO, GNSS, or UWB. It does not contain a raw-LIVO relative
pose factor, so the formal prototype does not double count the frontend IMU.

The graph and time semantics passed their tests and all three replays were
finite and stable. The decisive result is nevertheless negative. In
`stadtgarten_seq1` 900--906 s, G2 speed is almost identical to G0:

| signal | median | p95 | p99 | max |
|---|---:|---:|---:|---:|
| G0 production speed (m/s) | 4.65873 | 5.19393 | 5.22374 | 5.24217 |
| G2 graph speed (m/s) | 4.66639 | 5.19883 | 5.22133 | 5.24657 |
| absolute G2-G0 speed difference (m/s) | 0.01127 | 0.03198 | 0.04198 | 0.04280 |
| G2 IMU normalized error | 0.34867 | 2.19772 | 4.21680 | 4.38948 |
| G2 LiDAR normalized error | 6.15074 | 23.16375 | 31.13110 | 31.95796 |

The hidden-state velocity build-up therefore did **not** disappear or change
materially. The rising LiDAR-factor conflict is propagated through a sequence
of pose constraints; the IMU factor can distribute the correction over
`X/V/B`, but it cannot veto sustained inconsistent scan-to-map geometry.
No further heuristic was added.

## 2. Existing backend audit

The pre-existing RTK backend is:

```text
/backend/livo_odom_raw
  -> motion/time keyframe trigger (0.8 m, 8 deg, or 1.0 s)
  -> raw-pose relative measurement
  -> BetweenFactor<Pose3>(X[k-1], X[k]) with configured diagonal sigmas
  -> IncrementalFixedLagSmoother, lag 20 s

/gnss_fusion/enu_odom -> GnssPositionArmFactor(X[k])
UWB ranges (when enabled) -> UwbRangeFactor(X[k])
optional gravity consistency -> Pose3AttitudeFactor(X[k])

estimate X[k]
  -> optimized odometry/path/TUM
  -> map_to_odom = optimized_pose * raw_pose^-1
```

Source audit findings:

1. It uses `gtsam::IncrementalFixedLagSmoother`.
2. Its only state key is `X(k): Pose3`; it has no velocity or bias key.
3. Its LIVO factor is `BetweenFactor<Pose3>` measured as
   `raw_pose[i].between(raw_pose[j])`. Noise is configured diagonal, not
   derived from point-to-plane geometry (`0.01 rad`, `0.05 m` in the audited
   RTK-SLAM config; a special lateral/vertical value exists for UWB mode).
4. Nodes are motion/time keyframes, not every LiDAR epoch.
5. The configured lag is 20 s.
6. GNSS is a lever-arm-aware unary position factor; UWB is a unary anchor
   range factor; the optional attitude term preserves frontend up direction.
7. Initialization is a Pose3 prior after the backend's initial map/odom
   alignment.
8. It publishes optimized odometry/path, `map_to_odom`, online/final TUM files,
   status, and measurement diagnostics.
9. It does not subscribe to raw IMU.
10. It had no IMU preintegration infrastructure.

The machine actually builds against GTSAM `4.2a9` (`4.2.0` numeric), not the
historical 4.0.3 assumption. The node resolves `/usr/local/lib/libgtsam.so.4`
and the locally installed matching `libgtsam_unstable.so.4`.

## 3. Double-counting answer

### Q1: does the raw LIVO Between factor contain IMU information?

Yes. Each raw LIVO pose is the result of IMU propagation followed by the
LiDAR IEKF update. A relative transform computed from two such poses is not an
independent LiDAR measurement.

### Q2: what is counted twice?

Adding both that relative transform and raw-IMU preintegration for the same
interval reuses the same IMU samples through two likelihood paths. Treating
those paths as conditionally independent multiplies correlated evidence,
understates uncertainty, and can overconstrain pose, velocity, attitude, and
bias. The missing cross-covariance cannot be repaired by choosing a larger
fixed diagonal pose sigma.

### Q3: which LiDAR-only source was selected?

Candidate L1 was selected: the final point-to-plane normal equation before the
current scan mutates the voxel map.

- L1 already exists inside the actual scan-to-map solve and exposes the
  measured anisotropy without a second registration implementation.
- L2 would duplicate scan-to-map association and introduce a second set of
  lifecycle and tuning choices.
- No existing L3 interface was found that exported an independently computed,
  IMU-free relative LiDAR constraint.

G1 (`raw LIVO Between + raw IMU`) was not implemented because it is knowingly
inconsistent and is not needed to decide G2. The executable contract test
explicitly rejects `RAW_LIVO_BETWEEN` as a factor source.

## 4. G2 architecture

```text
raw /livox/imu -------------------------+
  strict interval buffer                |
  boundary interpolation                v
  (t[k], t[k+1]]                 ImuFactor(X,V,B)
                                      + Bias BetweenFactor

final FAST-LIVO point/plane H,R,r       |
  before current map insertion          |
  Lambda = H' R^-1 H                    v
  eta    = H' R^-1 (-r)     LinearContainerFactor(X[k])
                                      |
                                      v
                         IncrementalFixedLagSmoother(20 s)
                         X(k), V(k), B(k), about 20 Hz
                                      |
                         shadow odom/TUM/state/event CSV only
```

State is `X(k): Pose3`, `V(k): Vector3`, and
`B(k): imuBias::ConstantBias`. Exposure, VIO, GNSS, UWB, and a per-frame
gravity state are absent.

The first graph node reads production pose, velocity, gyro bias, accel bias,
gravity, and accelerometer scale once. Gravity then remains a fixed global
preintegration parameter. Later initial values come from the previous graph
estimate plus preintegration prediction; production `V/B` never overwrite the
graph.

The chosen `ImuFactor + BetweenFactor<ConstantBias>` form makes bias evolution
explicit. Frontend configured discrete variances are `acc_cov=0.5` and
`gyr_cov=0.3`. Since the frontend propagates `variance * dt^2` while GTSAM
expects continuous covariance density, each interval uses
`q = configured_variance * median(dt)`. Bias variance is the frontend's
existing `1e-4` per discrete step, giving sigma
`sqrt(1e-4 * sum(dt_i^2))`. Integration covariance is the fixed numerical
`1e-8`; none of these values was tuned against seq1.

## 5. LiDAR factor and weak directions

At the accepted final LIO iteration, the frontend freezes:

```text
R_i = 0.001 + plane_covariance_i + n_i' body_point_covariance_i n_i
Lambda_fast = H' R^-1 H
eta_fast = H' R^-1 measurement
```

This normal equation contains point/plane measurement geometry and does not
include the ESIKF state prior/covariance. FAST-LIVO increments are ordered as
right-body rotation plus world translation, whereas GTSAM Pose3 local
coordinates use right-body rotation plus body translation. With
`J = diag(I, R_world_body)`, the factor uses
`Lambda_gtsam = J' Lambda_fast J` and `eta_gtsam = J' eta_fast`.

PSD handling is an eigendecomposition, not a heuristic information floor:

- a materially negative eigenvalue below `-1e-9 * lambda_max` rejects the
  factor;
- tiny negative roundoff is clipped to zero;
- rank uses `lambda > 1e-12 * lambda_max`;
- `eta` is projected into the retained range;
- zero/weak eigenvalues remain zero/weak.

The self-test constructs a rank-five factor and confirms the missing direction
stays below `1e-10`. Real-run minimum/maximum eigenvalue ratios had medians
`0.00287`, `0.00313`, and `0.00395`; the graph therefore retained strong
anisotropy rather than replacing it with isotropic pose noise.

The submap is frozen only during current factor evaluation and publication:
the message is emitted after state estimation but before current voxel-map
insertion. The older submap was still built using historical ESIKF estimates,
so state-map correlation remains unmodelled. No landmark/plane-state graph was
added.

## 6. Time semantics and tests

Each accepted LiDAR measurement epoch creates one graph node. For `k -> k+1`,
the buffer integrates exactly `(t_k, t_{k+1}]`. Samples bracketing both
boundaries are linearly interpolated, interior measurements are trapezoidally
integrated, the duration must close numerically, and consumed samples cannot
belong to the next interval. Duplicate and non-monotonic IMU timestamps are
rejected; an unbracketed boundary or a gap above 0.03 s rejects the node.

`fullstate_shadow_backend_self_test` covers:

- boundary interpolation, duration closure, unique sample ownership;
- duplicate/non-monotonic timestamp rejection;
- preintegration bias correction;
- L1 identity/finite/PSD/rank and no-information-floor behavior;
- raw-LIVO no-double-counting contract rejection;
- `X/V/B` graph initialization and fixed-lag marginalization.

The evaluator has a separate assert-based self-test. All 15 current compiled
`*self_test` executables returned zero after the final changes, including the
existing VIO, LIO, GNSS, UWB, RTK backend, preprocess, and voxel tests.

## 7. Replay provenance

No ground truth or reference trajectory was read. Each `input_window.txt` and
`runtime.json` records `ground_truth_used=false`. GNSS/UWB are absent from G2.

| dataset | requested window | shadow rows / span | runner / child audit / SHA256 |
|---|---|---:|---|
| stadtgarten_seq1 | 0--930 s | 18,594 / 929.663 s | PASS / 7 PASS / PASS |
| stadtgarten_seq2 | 0--600 s | 11,995 / 599.704 s | PASS / 7 PASS / PASS |
| construction_seq2 | +1.047924 s, duration 598 s | 11,954 / 597.647 s | PASS / 7 PASS / PASS |

All three ran at 0.8x bag rate and produced approximately 20 graph nodes/s.
All event CSVs contain only their header. Final counters are zero for duplicate
IMU stamps, non-monotonic IMU stamps, missing intervals, and factorization
failures.

## 8. Failure-segment result

For seq1 900--930 s, G2/G0 speed median is `0.89902/0.88966 m/s`, p95 is
`5.03943/5.03131`, p99 is `5.19884/5.19395`, and both peak near `5.24 m/s`.
The 900--906 s pose-increment differences are also tiny: median absolute
translation-increment difference is `0.000461 m` and rotation-increment
difference is `4.53e-5 rad`.

At the same time, LiDAR normalized residual rises from a whole-run median of
`0.591` to `6.151` (p95 `23.164`) in 900--906 s. IMU normalized residual rises
from `0.0298` to `0.3487` (p95 `2.198`). The graph remains finite and keeps
marginalizing normally. This is evidence of sustained disagreement between
LiDAR geometry and IMU temporal consistency, not a timestamp loss or solver
reset. Because G2 follows G0, the explicit temporal model does not block the
build-up.

The primary classification is NO-B: return to the point-to-plane geometry,
map correlation, correspondence lifecycle, or scan matching. NO-A is rejected
because G2 has no raw-LIVO pose factor. NO-C is not supported by the zero time
errors, successful interval tests, and healthy normal-run IMU residuals. NO-D
is not the failure cause because marginalization/covariance remain stable,
although tail latency needs optimization before deployment.

## 9. Normal-run distortion

| metric (median / p95 / p99 / max) | stadtgarten_seq2 | construction_seq2 |
|---|---:|---:|
| G2 speed (m/s) | 0.495 / 1.534 / 1.698 / 1.924 | 0.716 / 1.531 / 1.667 / 1.867 |
| G0 speed (m/s) | 0.504 / 1.551 / 1.707 / 1.964 | 0.718 / 1.548 / 1.704 / 1.903 |
| abs speed difference (m/s) | 0.0133 / 0.0618 / 0.0919 / 0.2336 | 0.0121 / 0.0550 / 0.0857 / 0.1399 |
| G2 translation increment (m) | 0.0252 / 0.0773 / 0.0860 / 0.1019 | 0.0353 / 0.0777 / 0.0852 / 0.1016 |
| G0 translation increment (m) | 0.0247 / 0.0781 / 0.0872 / 0.1091 | 0.0351 / 0.0783 / 0.0864 / 0.1026 |
| G2 rotation increment (rad) | 0.0130 / 0.0595 / 0.0891 / 0.1461 | 0.0144 / 0.0552 / 0.0791 / 0.1502 |
| G0 rotation increment (rad) | 0.0131 / 0.0595 / 0.0890 / 0.1461 | 0.0144 / 0.0552 / 0.0791 / 0.1504 |
| IMU normalized residual | 0.0189 / 0.1238 / 0.2345 / 1.3409 | 0.0306 / 0.1918 / 0.3834 / 1.9485 |
| LiDAR normalized residual | 0.451 / 2.593 / 4.807 / 23.093 | 0.636 / 3.078 / 5.083 / 20.310 |

Fixed diagnostic subsets (not parameter tuning) show normal p95 absolute speed
differences of `0.0786/0.0652 m/s` during >=1 m/s^2 acceleration,
`0.0749/0.0660` during >=15 deg/s turns, `0.0292/0.0301` while G0 speed is
<=0.1 m/s, and `0.0796/0.0668` when the LiDAR eigenvalue ratio is <=1e-3
(seq2/construction respectively). There is no new gross normal instability.

## 10. Bias behavior

The production G0 comparison comes from the earlier same-day P2 shadow files.
Their timestamps match the three P3 state CSVs exactly, row for row. G0 bias
propagation is constant between updates, so its logged bias correction is its
per-frame bias step.

| dataset | G2 bg step p95 / max | G0 bg step p95 / max | G2 ba step p95 / max | G0 ba step p95 / max |
|---|---:|---:|---:|---:|
| seq1 | 1.77e-4 / 7.60e-4 | 1.49e-4 / 6.53e-4 | 1.17e-3 / 4.60e-2 | 1.09e-3 / 3.67e-3 |
| seq2 | 1.63e-4 / 1.10e-3 | 1.24e-4 / 4.22e-4 | 1.15e-3 / 3.36e-2 | 1.03e-3 / 2.48e-3 |
| construction | 1.94e-4 / 1.29e-3 | 1.60e-4 / 6.62e-4 | 1.34e-3 / 2.96e-2 | 1.16e-3 / 3.36e-3 |

The G2 accel-bias maxima occur only in its startup transient at approximately
0.95, 1.25, and 0.60 s. After 5 s, combined bias-step maxima are 0.00751,
0.00659, and 0.00709, while medians remain `3.9e-4`--`4.8e-4`. In seq1
900--906 s, G2 and G0 bias p95 values are comparable (`bg 3.59e-4 vs
3.07e-4`, `ba 2.30e-3 vs 2.59e-3`). Thus no late sudden bias jump explains
the velocity failure. The startup accel-bias transient should be fixed before
any production use, but it is not the seq1 failure mechanism.

## 11. Smoother health and runtime

| dataset | update ms median / p95 / p99 / max | RSS MB median / max | active X nodes / factors | marginalized X nodes |
|---|---:|---:|---:|---:|
| seq1 | 25.38 / 39.97 / 54.61 / 126.89 | 89.70 / 102.11 | 401 / 1202 | 18,193 |
| seq2 | 25.56 / 42.50 / 57.04 / 132.47 | 94.82 / 102.07 | 400 / 1199 | 11,595 |
| construction | 25.28 / 40.47 / 53.61 / 78.60 | 85.48 / 93.89 | 400 / 1199 | 11,554 |

All logged 15D block marginal covariance minimum eigenvalues are positive.
Whole-run median covariance conditions are `9.66e3`, `1.07e4`, and `1.30e4`;
maxima are `4.23e4`, `3.76e4`, and `3.60e4`. No graph reset or factorization
failure occurred. At 20 Hz the median and p95 update are within 50 ms, but p99
and maxima exceed the per-node real-time budget. This is acceptable for
continued prototype work, not yet acceptable as a production guarantee.

## 12. Answers to the required final questions

1. **Original backend:** Pose3-only `IncrementalFixedLagSmoother`, raw LIVO
   relative Between factors, optional GNSS/UWB/attitude unary factors, 20 s
   motion/time-keyframed window, no raw IMU or preintegration.
2. **Double count:** raw LIVO poses already contain the same IMU evidence;
   adding independent raw-IMU factors multiplies correlated likelihoods and
   loses the required cross-covariance.
3. **LiDAR-only factor:** L1, the final point-to-plane `Lambda/eta`, frozen
   before current-map mutation and represented as a GTSAM linear container
   Hessian factor on `X(k)`.
4. **Weak direction retained:** yes; no positive information floor is added,
   and rank-deficient directions remain zero.
5. **Graph state:** `X(k): Pose3`, `V(k): Vector3`,
   `B(k): ConstantBias`.
6. **Gravity:** read once from completed frontend initialization and fixed for
   all preintegration; no per-frame gravity variable.
7. **IMU boundary:** exactly `(t_k, t_{k+1}]`, with interpolated boundaries,
   trapezoidal segments, gap/ordering checks, and unique ownership.
8. **Window/rate:** fixed 20 s; one node per LIO epoch, approximately 20 Hz,
   yielding 400--401 active pose nodes.
9. **seq1 900--906:** velocity build-up remains essentially identical to G0;
   G2 max is 5.247 m/s versus 5.242 m/s for G0.
10. **Normal stability:** no gross instability in seq2 or construction;
    pose increments and speed distributions remain close to G0.
11. **Bias:** normal and failure-window p95 behavior is broadly comparable to
    G0; G2 has a larger short startup accel-bias transient but no late jump
    explaining the failure.
12. **Residual/covariance:** finite, positive marginal covariance, stable
    marginalization, zero factorization errors; LiDAR/IMU residual conflict
    rises strongly in the failure window.
13. **Runtime:** median/p95 are useful for continued desktop prototyping, but
    p99/max exceed 50 ms and require optimization before feedback/deployment.
14. **Map correlation:** the current factor is scan-to-map against a frozen
    evaluation-time submap, but that submap was historically built from ESIKF
    states; this correlation and correspondence selection are not modelled.
15. **Enter P3-B:** no. First diagnose/fix the frontend geometry/map/
    correspondence path and the startup bias transient; do not add GNSS/UWB/
    visual factors or feed graph state back yet.

## 13. Reproduction artifacts

- `tools/evaluate_fullstate_shadow.py`: standard-library-only evaluator and
  self-test.
- `reports/p3a_fullstate_shadow_metrics_20260918.json`: complete internal
  median/p95/p99/max tables and segment subsets.
- Each run directory contains the frozen configs, commands, input window,
  child-exit audit, runtime metadata, state/event/TUM outputs, and verified
  `SHA256SUMS`.

No commit or push was performed.

`FULLSTATE_SMOOTHER_ARCH_READY = NO`
