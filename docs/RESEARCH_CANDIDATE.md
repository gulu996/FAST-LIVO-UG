# Research candidate: contracts and validation scope

This candidate keeps the P12 estimator and GNSS parameters, selectively imports
P8-C UWB transport/queue protections, and retains the public tree's licenses,
optional-hardware exclusions and documentation. No new filter/weight tuning or
P17–P20 map-correlation estimator is included.

## Build and software checks

Use the workspace-root build from the main README; it builds the native targets
below. Source only ROS and this workspace, not a previous candidate's overlay.
GTSAM 4.2 **and gtsam_unstable** must resolve to compatible libraries (see the
existing build guide). Keep the new workspace's `devel/lib` first when choosing
locally installed runtime libraries.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
for test in lio_plane_map_self_test lio_update_transaction_self_test   imu_transaction_self_test livo_scan_transaction_self_test   vio_update_transaction_self_test p8_contract_self_test   p8_lifecycle_self_test causal_readout_self_test   rtk_fixed_lag_backend_self_test uwb_manager_self_test; do
  rosrun fast_livo "$test" || exit 1
done
# Explicit synthetic fixture; output is not a real ranging experiment.
rosrun fast_livo p8_uwb_contract_self_test /tmp/research_uwb_synthetic.csv
```

The no-measurement check compares all active graph poses and marginal covariances
with UWB off/on, including GNSS and gravity-consistency inputs. Synthetic range
geometry and ROS transport checks do not certify real UWB accuracy. Tests and
resource/identity logs for the integration live in the external experiment
archive, not in the source repository.

## RTK-SLAM startup and output ownership

The dataset entry below retains the existing RTK-SLAM camera, extrinsics, time
settings and backend preset, adds causal output, and disables costly production
fingerprints. It does not manufacture GNSS from the bag. The GNSS input must be
an independently validated ENU `PoseWithCovarianceStamped` stream with the fixed
origin and quality contract in `gnss_adapter.yaml`. Never label arbitrary Pose
or host-time ranges as Fixed/device measurements.

```bash
roslaunch fast_livo research_candidate.launch   run_dir:=/tmp/fast_livo_research_run   gnss_pose_topic:=/rtk_slam/gnss/enu_pose
```

A new output directory should be used for each experiment. The launch starts
mapper, image decoder, adapter, backend and causal readout; it does not start a
bag, external GNSS conversion, UWB device or trusted EOF controller. Use an
isolated ROS master and load the correct sequence's recorded input contract
before playback. The RTK-SLAM data disk was unavailable during integration; this RTK entry has
software/launch validation. A verified competition copy was replayed separately
with its own existing frontend sensor configuration.

- Raw `/backend/livo_odom_raw` and its atomic `/quality` envelope belong to the
  frontend. GNSS/UWB optimized results are never written into frontend state/P.
- Native optimized odometry and Final may be sparse. `/rtk_backend/causal_fused`
  carries explicit validity, session, correction/version and availability;
  `/rtk_backend/causal_fused_odom` publishes only valid projected poses.
- Causal output is the latest available correction applied to Raw, not a fresh
  GTSAM optimization at every raw timestamp. Do not interpret pose revisions'
  difference quotients as physical speed or unknown fused covariance as known.
- `BackendStreamControl` accepts PAUSE/RESUME/EOF only with matching frontend
  session and drained output sequence. A trusted controller must verify all
  expected inputs/outputs before EOF; bag process exit or wall silence alone
  is insufficient. TIMEOUT/session failure does not auto-recover.

The RTK-SLAM gravity preset requires `gravity_align_en=true` for the frontend
and `raw_odom_gravity_aligned=true`. It preserves the observed vertical while
leaving yaw/translation free; its sigma is the existing 0.05 rad tilt prior.
Do not enable it for an arbitrary unaligned odometry frame. Retain 20 s lag,
3 m GNSS residual gate, existing NIS/noise/origin/lever arms and PSD protection.

## UWB experimental input

UWB is disabled by the research launch and both default backend presets. An
explicit separate experiment must first qualify RF measurement timestamps,
clock mapping/uncertainty, session/tag/anchor/round identity and freshness,
independently surveyed anchors, tag-to-IMU lever arm and range bias/noise.
Host receive time and preserved replay time are not device measurement proof.

When no ranges are present, UWB's enable flag does not select different LIVO
BetweenFactor or initial-prior noise. Old UWB-specific noise fields remain
parse-compatible but no longer select these graph noises. Pending measurements
are bounded before initialization and every rejection is accounted for. The
new provenance rules can reject old logs/fixtures; missing metadata must not be
invented. Real synchronized UWB and 1–4 anchor benefits remain unverified.

## Actual experiment status and stopping boundary

This integrated build completed one 30 s competition qualification followed by
one complete approximately 804 s S1 replay at 1×, with the unchanged archived
camera/extrinsics/timing/map settings and GNSS/UWB disabled. Input SHA-256:
`844b06a2c9c3b518a51491227c4a9d5df02cf37fc5bb27252035b8a48053cda5`.
It received 4031 LiDAR, 160793 IMU and 24115 images;
24094 images were processed, with zero buffer overflow. All 4029 LIO
transactions committed. VIO accepted 3581 / 24094 attempts. First/last
distance was **0.951613 m**; mapper RSS peaked at 1767.5 MiB.
This is an endpoint diagnostic, not ATE or a physical truth claim. The 21-image
difference is accounted for: 11 older-than-state drops, 6 no-LiDAR-feature
processing rejections and 4 terminal buffered images, matching the archived
reference event identities/order. It is not silent image-buffer overflow.

The qualified `suidao2_resized.bag` was unavailable: its complete regression,
endpoint distance and 233–260 s reverse-projection check are NOT_RUN. The other
61 GB `suidao2.bag` was not substituted. Restore and hash-check the qualified
input before completing that regression; no current result is borrowed from P9.

Software checks: 11 native targets, 33 driver tests, ROS metadata transport, two
mapper publication/rollback fixtures and launch parameter validation passed.
No-range UWB off/on matches every active pose/covariance bit-for-bit, with and
without GNSS/gravity factors. A declared synthetic 30 s ROS stream delivered
301 quality messages, 60 UWB factors and 300 valid causal poses with explicit EOF.
Two initial mapper fixtures lacked camera parameters in the harness; the archived
B2.5 parameter setup fixed that harness issue without changing production code.

Historical P12 Seq2 Raw/Fused 1.083/0.642 m (16 holdouts) and Seq1
35.815/37.275 m (33 holdouts) use fixed early three-point SE(3), nanosecond time,
≤200 ms interpolation and base-center lever arm once. Causal values are first
legal delayed availability, not zero-latency or Final-backfilled estimates.
Keep the P10 good Seq1 path and P11/P12 failures; no cherry-picked reruns.

Seq1's sensitivity is unresolved despite deterministic VIO and fully committed
LIO. Continuous output during GNSS loss is not an error bound. These limitations
permit committing an explicit research candidate, not certifying a stable
complete GNSS/UWB navigation system.
