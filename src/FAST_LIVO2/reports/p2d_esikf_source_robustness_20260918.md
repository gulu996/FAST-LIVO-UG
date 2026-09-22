# P2-D: LIO ESIKF weak-geometry hidden-state source robustness

Date: 2026-09-18  
Checkout: `/home/gulu/catkin_ws/src/FAST_LIVO2`  
Commit base: `f35e2aefa8a76fc3c75a31e4ed00273b4de0b6da`  
Decision: `ESIKF_SOURCE_FIX_READY = NO`

## Executive result

The failure mechanism is confirmed at source level, but none of A/B/C has a
selective Pareto advantage over healthy motion.

- In `stadtgarten_seq1` 900--906 s, the weakest algebraic pose-information
  direction contributes a cumulative velocity-correction norm of 4.47289 m/s
  versus 4.76868 m/s for the complete raw cumulative correction proxy. Its
  median rotation/translation axis fractions are 0.023/1.000 and its absolute
  cosine with the Schur-conditioned weak translation direction is 0.99985
  (p10 0.99914). The failure is therefore dominated by the weak translation
  information direction, not by a hidden covariance explosion.
- A can be made covariance-consistent by localizing pose--hidden prior cross
  covariance while preserving both marginals. It suppresses the confirmed
  failure, but it attenuates healthy corrections at almost the same relative
  rate. The parameter-free weak-direction confidence is not failure-selective:
  the stronger A weight for direction 0 has median 0.279 in failure 900--906,
  but only 0.0705 in healthy `stadtgarten_seq2` and 0.201 in healthy
  `construction_seq2` (smaller means stronger attenuation).
- B prior-metric damping has the same non-selective tradeoff and lacks a valid
  covariance interpretation unless the numerical damping is falsely treated
  as additional physical prior information. Scaling only the innovation term
  is strongly non-monotonic because it breaks the observed innovation versus
  relinearization cancellation.
- C is statistically associated with the failure but is not a label:
  `max_iter_healthy` occurs in 26.7% and 19.3% of the two normal datasets,
  36.8% of the failure dataset before 880 s, 54.2% in 900--930 s, and 75.8%
  in 900--906 s. Confidence scaling suppresses healthy corrections at roughly
  the same rate as the failure proxy and has no defined covariance update.

No production estimator, covariance update, map insertion rule, VIO warp fix,
sequential gate, sensor noise, GNSS/UWB path, runner, or replay rate was changed.

## 1. Exact current source contract

### 1.1 State and IMU propagation

The error-state ordering is:

`[theta(0:2), position(3:5), exposure(6), velocity(7:9), bg(10:12), ba(13:15), gravity(16:18)]`.

Nominal propagation subtracts gyro and accelerometer bias, updates rotation,
then integrates world acceleration `R * a + gravity` into position and
velocity. Bias, gravity, and exposure nominal values are random-walk/constant
states during this propagation.

The covariance update is exactly:

`P <- F_x P F_x^T + Q`.

The cross-block creation paths visible in `F_x` are:

- `theta <- bg` through `F_x(0:2,10:12) = -I dt`;
- `position <- velocity` through `F_x(3:5,7:9) = I dt`;
- `velocity <- theta` through `F_x(7:9,0:2) = -R [a]x dt`;
- `velocity <- ba` through `F_x(7:9,13:15) = -R dt`;
- `velocity <- gravity` through `F_x(7:9,16:18) = I dt`.

This propagation creates the pose--velocity/bias/gravity cross covariance by
ordinary covariance transport. It is not a bug and was not zeroed.

### 1.2 LiDAR residual, Jacobian, and covariance

Every IEKF iteration transforms the current downsampled scan, rebuilds the
point-to-plane residual list, and constructs one row per accepted plane:

`r_i = n_i^T p_i^w + d_i`, `z_i = -r_i`.

The direct state Jacobian is pose-only:

`H_i = [ ( [p_i^l]x R_k^T n_i )^T, n_i^T ]`.

The scalar variance used by the current source is:

`R_i = 0.001 + J_nq plane_var J_nq^T + n_i^T var_point n_i`.

The normal equation is:

`Lambda = H^T R^-1 H`, `eta = H^T R^-1 z`.

Current/frozen quantities are important:

- current every iteration: pose, transformed points, correspondences,
  residuals, `H`, `R`, `Lambda`, and `eta`;
- fixed for the LiDAR frame: `state_propagat`, its 19x19 propagated covariance
  `Pminus`, body point covariances, and the precomputed LiDAR cross matrices;
- `Pminus` remains fixed across all IEKF iterations because `StatesGroup +=`
  changes the nominal state but not covariance. Covariance is assigned only at
  the final accepted linearization.

### 1.3 IEKF iteration

With the 6x6 `Lambda` embedded in the top-left of a 19x19 zero matrix, the
source computes:

`K1 = (Lambda_full + Pminus^-1)^-1`

`G = K1(:,0:5) Lambda`

`vec = state_propagat - state_iterate`

`innovation = K1(:,0:5) eta`

`relinearization = vec - G vec_pose`

`solution = innovation + relinearization`.

The candidate nominal state is updated by `state += solution`, after the
pre-existing pose-step scale if the rotation or translation increment exceeds
its existing safety bound. The hidden-state path is therefore not a separate
measurement Jacobian. It is the off-diagonal block of `K1(:,pose)`, created by
`Pminus` cross covariance.

For an unclipped iteration, the candidate relative to the propagated prior is
exactly:

`delta_prior = K1(:,pose) (eta - Lambda vec_pose)`.

For a final source step scale `s`, the exact applied form is:

`delta_applied = s K1(:,pose)(eta - Lambda vec_pose) + (1-s)(-vec)`.

The shadow logger records the second term as a non-directional carry. Across
all 42,543 valid current-schema frames, the accumulated
innovation+relinearization velocity closure has a maximum of `1.34e-15`.

### 1.4 Covariance and transaction semantics

At the stop iteration the current source computes:

`Pplus = (I - G) Pminus`.

It uses the final iteration's `G` and the same fixed `Pminus`. The transaction
validator checks finite state/covariance, relative asymmetry, eigenvalues of
the symmetrized covariance, rotation validity, correspondence count, cost,
residual, and total pose increment. It does not project to PSD or replace the
candidate covariance with its symmetrized form. Deterministic state snapping
would symmetrize later, but `state_snap_en` is false in these production-mode
dataset runs.

Both `CONVERGED` and `MAX_ITER_HEALTHY` commit the candidate and the same
posterior covariance when those structural checks pass. Map insertion is then
controlled by the existing commit/schedule/map-guard path; P2-D did not alter
it.

## 2. Exact pose-direction contribution

For the symmetric algebraic pose information:

`Lambda = U diag(lambda_i) U^T`.

The velocity part of the prior-relative final target decomposes exactly as:

`delta_v = sum_i K_vp u_i [u_i^T (eta - Lambda vec_pose)]`.

When the existing final step scale is active, every directional term is
multiplied by `s` and the logged carry `(1-s)(-vec_v)` is added. This is an
algebraic normal-equation decomposition. Because raw rotation and translation
coordinates have different units, it is not itself a physical observability
metric; physical weak-translation interpretation continues to use the
Schur-conditioned 3x3 translation information.

For `stadtgarten_seq1` 900--906 s:

| direction | cumulative velocity contribution norm | median rotation fraction | median translation fraction |
|---:|---:|---:|---:|
| 0 (minimum information) | 4.47289 | 0.023 | 1.000 |
| 1 | 0.260734 | 0.166 | 0.986 |
| 2 | 0.208861 | 0.126 | 0.992 |
| 5 | 0.0350609 | 0.992 | 0.127 |
| 4 | 0.0320853 | 0.993 | 0.121 |
| 3 | 0.0288511 | 0.993 | 0.114 |

Direction 0 versus the Schur weak translation direction has absolute cosine
median/p10/p90 of `0.99985 / 0.99914 / 0.99998`. This answers the directional
source question without inventing an observability label from a mixed-unit
6x6 condition number.

## 3. Counterfactual definitions

### A: covariance-localized hidden transfer

Partition the propagated covariance into pose and hidden blocks:

`P = [[A, B^T], [B, D]]`, where `A=P_pose,pose`.

Whiten pose by its prior and diagonalize the dimensionless measurement
information:

`J = A^(1/2) Lambda A^(1/2) = V diag(j_i) V^T`.

The no-bag-threshold confidence family is:

`w_i = (j_i / (1+j_i))^alpha`, for `alpha = 0, 0.25, 0.5, 1`.

`alpha=0` is identity. Define `W=V diag(w_i)V^T` and replace only the cross
block:

`B_local = B A^(-1/2) W A^(1/2)`.

Keep `A` and `D` unchanged. Since `0 <= W^2 <= I`, the hidden conditional
Schur complement becomes no smaller, so the localized prior remains PSD. The
posterior is recomputed as:

`Pplus_local = (P_local^-1 + Lambda_full)^-1`.

This is covariance-consistent for the localized Gaussian model and preserves
the pose posterior mean/covariance because the measurement observes pose only
and `A` is unchanged. Across all three runs, A stronger had positive prior and
posterior minimum eigenvalues; maximum posterior asymmetry was `3.96e-18` and
maximum pose-posterior covariance difference from raw was `3.44e-17`.

The problem is not numerical consistency but selectivity: healthy weak
geometry is attenuated at least as strongly as the confirmed failure.

### B: damping/trust candidates

The normal-equation shadow uses:

`K_lambda = (Lambda_full + (1+lambda)Pminus^-1)^-1`,

with `lambda = 0, 0.25, 1, 4`. This can have a PSD posterior only if it is
interpreted as replacing the physical prior with `Pminus/(1+lambda)`. As an LM
or trust-region numerical device, that interpretation is false: damping should
control the nonlinear iteration, not manufacture information. Keeping the raw
covariance after damping leaves the mean and covariance based on different
objectives; committing the damped covariance is overconfident.

Applying damping to all 19 dimensions also changes pose and every hidden state.
Applying it only to the measurement-induced mean component has no corresponding
Gaussian posterior. The recorded iteration counterfactual confirms the danger:
scaling accumulated innovation to 0.75 suppresses the failure proxy by 80.8%,
but scaling it further to 0.50 suppresses only 38.3%, because the unchanged
relinearization term stops cancelling it. Normal p95 velocity distortion grows
to `0.121--0.131 m/s` for the 0.50 case.

### C: max-iteration confidence

Current rates are:

| dataset/window | max_iter_healthy | total | rate |
|---|---:|---:|---:|
| stadtgarten_seq2 0--600 normal | 3,207 | 11,995 | 26.7% |
| construction_seq2 +1--599 normal | 2,307 | 11,954 | 19.3% |
| stadtgarten_seq1 0--880 | 6,480 | 17,596 | 36.8% |
| stadtgarten_seq1 900--930 | 324 | 598 | 54.2% |
| stadtgarten_seq1 900--906 | 91 | 120 | 75.8% |

In 900--906 s, converged frames still contribute a cumulative velocity
correction norm of 1.09513 m/s and max-iteration frames 3.68818 m/s. The label
is associated with the window but neither necessary nor rare in healthy data.
Scaling only max-iteration frame means to 0.75/0.50 suppresses the failure proxy
by 19.3%/38.6%, while normal p95 velocity distortion is about
`0.0145--0.0157 / 0.0291--0.0315 m/s`. The distortion-to-raw-p95 ratio is again
approximately the suppression rate, and covariance semantics are undefined.

## 4. Pareto result

`normal distortion` below is the worst of the two independently classified
normal datasets. The parenthesized percentage is p95 distortion divided by the
same dataset's raw p95 correction, not a trajectory error.

| method | 900--906 cumulative suppression | peak suppression | worst normal velocity p95 distortion | parameters | covariance consistency | implementation risk |
|---|---:|---:|---:|---:|---|---|
| A mild (`alpha=.25`) | 24.9% | 22.1% | 0.01884 m/s (25.2%) | 1 | localized Gaussian is consistent | medium |
| A medium (`alpha=.5`) | 43.5% | 39.2% | 0.03300 m/s (42.8%) | 1 | localized Gaussian is consistent | medium |
| A stronger (`alpha=1`) | 67.7% | 62.6% | 0.05154 m/s (65.4%) | 1 | localized Gaussian is consistent | medium |
| B mild (`lambda=.25`) | 14.5% | 13.5% | 0.01110 m/s (13.9%) | 1 | only as a fictitious stronger prior | high |
| B medium (`lambda=1`) | 40.3% | 38.3% | 0.03145 m/s (38.7%) | 1 | only as a fictitious stronger prior | high |
| B stronger (`lambda=4`) | 72.8% | 70.7% | 0.05952 m/s (70.7%) | 1 | only as a fictitious stronger prior | high |
| B innovation x0.75 | 80.8% | 67.0% | 0.06563 m/s (81.1%) | 1 | undefined mean-only scaling | high |
| B innovation x0.50 | 38.3% | 41.8% | 0.13126 m/s (162%) | 1 | undefined; cancellation is broken | high |
| C max-iter x0.75 | 19.3% | 25.0% | 0.01573 m/s (19.5%) | 1 | undefined mean-only scaling | high |
| C max-iter x0.50 | 38.6% | 40.6% | 0.03147 m/s (39.0%) | 1 | undefined mean-only scaling | high |

A is the only candidate with a defensible covariance construction, but its
failure suppression and normal distortion lie on essentially the same line.
Smaller `alpha` merely moves toward identity; fitting another function or
threshold to separate this one failure would be the prohibited bag-specific
heuristic escalation.

Normal p95 block distortions for A mild were:

| normal dataset | pose | exposure | velocity | bg | ba | gravity |
|---|---:|---:|---:|---:|---:|---:|
| stadtgarten_seq2 | 9.29e-7 | 1.34e-4 | 1.8785e-2 | 2.77e-6 | 2.58e-4 | 3.73e-5 |
| construction_seq2 | 2.39e-6 | 2.12e-4 | 1.8837e-2 | 3.21e-6 | 2.44e-4 | 3.60e-5 |

The evaluator separately checked normal weak-geometry, turn, acceleration,
start/stop, low-speed, and max-iteration quantile subsets. No method acquired a
failure-only advantage there; full distributions are in
`source_robustness_summary.json`.

## 5. Numerical health and cost

- All three accepted runs had zero LIO rejection and zero buffer overflow:
  18,594/18,594, 11,995/11,995, and 11,954/11,954 committed LIO frames.
- Exact direction velocity closure maxima were at numerical precision; the
  accumulated innovation+relinearization closure maximum was `1.34e-15`.
- Every A/B counterfactual row was finite and PSD in the evaluated schema.
- Eight A/B variants together cost median `0.391--0.396 ms/frame`, p95
  `0.514--0.525 ms/frame`, and p99 `0.618--0.629 ms/frame`. A single production
  variant would cost less, but that was not measured and is not claimed.
- A currently performs a 6x6 prior eigensolve, a 6x6 dimensionless-information
  eigensolve, and 19x19 prior/posterior solves per strength. It could be reduced
  with block/Woodbury algebra only if a production prototype were justified.

## 6. Dataset and inference limits

- Failure evaluation: `stadtgarten_seq1`, 0--930 s, confirmed failure 900--930 s.
- Independent normal evaluation: `stadtgarten_seq2`, 0--600 s;
  `construction_seq2`, +1--599 s. The construction offset is the established
  workaround for the first 7-point empty-map bootstrap boundary, without code
  or runner changes.
- No independently confirmed current-schema indoor or tunnel normal dataset was
  available, so none was silently relabeled.
- No GT position error, GNSS position error, or external reference trajectory
  error was read for design or selection.
- These are same-final-linearization counterfactuals. They do not include the
  next-frame propagation/map/correspondence feedback of a real modified
  estimator and are not formal trajectory A/B results.

## 7. Decision and next architecture

`ESIKF_SOURCE_FIX_READY = NO`

No A/B/C production prototype is recommended. A has the correct directional
mechanism and a consistent covariance construction, but lacks selectivity and
materially changes normal hidden-state corrections. B requires false prior
semantics or an inconsistent covariance. C is common in healthy data and does
not explain all abnormal correction.

The architecture stop-loss condition is met for this round: further tuning of
A attenuation, B damping, or C confidence would add an empirical heuristic to
the current 19D ESIKF without demonstrated Pareto separation. Stop patching the
current update and move the next prototype to:

`FAST-LIVO geometry frontend + IMU preintegration + fixed-lag full-state smoother`.

The smoother prototype should expose pose/velocity/bias/gravity factors over a
time window, retain robust LiDAR factors in their measured directions, and use
marginalization rather than single-frame pose--hidden covariance transfer as
the only temporal coupling. Ground truth remains evaluation-only.

## 8. Validation and artifacts

Validation performed:

- `catkin_make --pkg fast_livo -j2`: PASS;
- `lio_motion_consistency_self_test`: PASS;
- `evaluate_lio_source_robustness.py --self-test`: PASS;
- final-schema `stadtgarten_seq2 --prefix 60`: PASS;
- `stadtgarten_seq1 --prefix 930`: PASS;
- `stadtgarten_seq2 --prefix 600`: PASS;
- `construction_seq2 --smoke 598`: PASS;
- all three long runs: child-exit audit PASS, trajectory window validation PASS,
  manifest produced, zero buffer overflow;
- task-scoped `git diff --check`: PASS.

Primary machine-readable artifacts:

- `/home/gulu/TiaoZhanBei/eval_20260830/results/city1_recovery_20260910/reports/lio_source_robustness_20260918/source_robustness_summary.json`
- `/home/gulu/TiaoZhanBei/eval_20260830/results/city1_recovery_20260910/reports/lio_source_robustness_20260918/pareto.csv`
- `/home/gulu/TiaoZhanBei/eval_20260830/results/city1_recovery_20260910/reports/lio_source_robustness_20260918/REPORT.md`

SHA-256:

- `source_robustness_summary.json`: `92138b678fa729850cf53adfc2991bf09bdb9af3f90e866df81f4da22bce368d`
- `pareto.csv`: `9b9186298fc4b92e4ad92c964cef710033b6a81424382d4b5070841b03d908d2`
- generated `REPORT.md`: `f140346d09ac5ba10e6a154f4cc10358e934428d0cb4d79f6f8b8b238ba5b63b`

The tracked research implementation is shadow-only:

- `include/lio_motion_consistency.h`: exact direction decomposition and A/B
  covariance counterfactuals;
- `src/lio_motion_consistency_self_test.cpp`: closure, identity, pose-preserving,
  ordering, PSD, and damping checks;
- `src/voxel_map.cpp`: passes the final source step scale into diagnostics;
- `src/LIVMapper.cpp`: writes the current 759-column shadow schema;
- `tools/evaluate_lio_source_robustness.py`: internal-only cross-dataset evaluator.

