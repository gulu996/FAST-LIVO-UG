# Petrochemical competition submission chain

This directory converts an already completed FAST-LIVO2 run into the official
person-ground submission files. It never changes or reruns Visual, LiDAR, GNSS,
UWB, EKF, or the fixed-lag backend.

## Frozen reference definition

- Formal trajectory: `rtk_optimized_online.tum`. Final is diagnostic only.
- BASE origin: the BASE RINEX header `APPROX POSITION XYZ`; no post-processed or
  truth-fitted BASE is used.
- Official confirmation for this dataset makes `d_base_ENU=[0,0,0]`, with source
  `OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE`.
- `person_ground_projection` is GNSS antenna phase center to the person's ground
  projection. The two body-frame vectors are:

```text
body -> GNSS antenna          = [-0.094,-0.076,+0.136] m
GNSS antenna -> person ground = [+0.274,+0.086,-1.328] m
body -> person ground         = [+0.180,+0.010,-1.192] m
```

The formal equation is:

```text
p_person_ENU = p_body_ENU + R_ENU_body * [0.180,0.010,-1.192] + d_base_ENU
(N,E,U)      = (ENU.y, ENU.x, ENU.z)
```

The body lever is rotated per epoch. `d_base_ENU` is a fixed E,N,U translation
and is not rotated. ENU to NEU is performed exactly once.

## Sampling policy

- P01 is before backend initialization. It uses `livo_raw_online.tum` transformed
  by the run's actual `ALIGNMENT_SUCCESS` yaw/translation from `rtk_backend.log`,
  then applies the person-ground lever. Its source is `ALIGNED_RAW_FALLBACK`.
- P02 through P10 use `SINGLE_TIMESTAMP`: exact Online position interpolation and
  quaternion SLERP, followed by the per-epoch person-ground transform.
- Dynamic outputs use Online at inclusive 10 Hz with the same interpolation,
  SLERP, and person-ground conversion.
- Nearest-neighbor sampling, timestamp rounding, extrapolation, Final selection,
  truth fitting, and trajectory-overlap lever estimation are forbidden.

Competition epochs are not hard-coded in Python. They come from
`petrochemical_stage6b_schedule.json`, supplied with `--schedule-file`. Replace
that JSON when a new official attachment defines different point times or dynamic
windows; do not edit algorithm code. The current schedule also records the
independent UTC/BDT sanity pair:

```text
Unix UTC 1785900626.0 <-> BDS week 1074, SOW 271830.0
```

The converter uses the historical leap-second table; for this 2026 dataset
`BDT-UTC=4 s`.

## G4 Stage 6B command

```bash
REPO=/home/gulu/catkin_ws/src/FAST_LIVO2
RUN=/home/gulu/data/2026-08-28-095108
OUT="$RUN/submission_stage6b"
BASE_RINEX='/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/融合定位/gnss/BASE217d.26O'
REF_DIR='/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/参考结果'
SCRIPT="$REPO/tools/make_competition_submission/make_competition_submission_v2.py"
SCHEDULE="$REPO/tools/make_competition_submission/petrochemical_stage6b_schedule.json"

# Stop before generation if any frozen G4 input changed.
sha256sum -c "$RUN/g4_sha256.txt"

python3 "$SCRIPT" "$RUN" \
  --out-dir "$OUT" \
  --schedule-file "$SCHEDULE" \
  --reference-mode person_ground \
  --saved-reference-lever-body-m 0 0 0 \
  --body-to-person-m 0.180 0.010 -1.192 \
  --base-rinex "$BASE_RINEX" \
  --base-offset-m 0 0 0 \
  --base-offset-source OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE \
  --reference-dir "$REF_DIR"
```

The optional reference directory is opened only after the four formal files are
generated, strictly validated, and hashed. It cannot influence transforms,
sampling, or source selection.

## Standalone validation and hashes

```bash
python3 "$SCRIPT" --schedule-file "$SCHEDULE" --validate-only "$OUT"
(cd "$OUT" && sha256sum -c submission_stage6b_sha256.txt)
python3 "$REPO/tools/make_competition_submission/make_competition_submission_v2_self_test.py"
```

Strict validation rejects BOM, CR/CRLF, non-ASCII bytes, blank lines, trailing
spaces, extra columns, incorrect precision, NaN/Inf, duplicate/non-monotonic
epochs, wrong week/SOW conversion, non-10-Hz steps, and wrong row counts.

Outputs:

- `position_points.txt`: exactly 10 lines, `P01..P10,N,E,U`, four decimals,
  no header.
- `position_dynamic_01/02/03.txt`: exactly 201 lines each,
  `BDS_WEEK,BDS_SOW,N,E,U`; SOW one decimal and NEU four decimals.
- `submission_stage6b_sha256.txt`: SHA-256 of the four formal position files.
- `submission_manifest.txt`: baseline, source, reference, schedule, alignment,
  and provenance.
- `submission_validation_report.md`: format and read-only score validation.

## Competition-day path-configurable chain

No tool requires the G4 absolute path. Set paths for the delivered dataset and
schedule at runtime:

```bash
BUNDLE=/path/to/TIAOZHANBEI_FASTLIVO2_GNSS_CODE
DATASET=/path/to/official_dataset
BAG=/path/to/output/competition.bag
SCHEDULE=/path/to/official_submission_schedule.json
BASE_RINEX=/path/to/official_dataset/gnss/BASE.26O

# 1. Decode GNSS offline. Select the profile required by the event rules.
"$BUNDLE/scripts/decode_gnss.sh" offline --dataset "$DATASET"

# 2. Package raw sensors and decoded GNSS.
"$BUNDLE/scripts/build_dataset_rosbag.sh" offline \
  --dataset "$DATASET" --output "$BAG" --compression lz4

# 3. Run this workspace's petrochemical launch. Set pcd_save/save_path in the
#    dataset-specific config to the event result root before launching.
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
roslaunch fast_livo mapping_petrochemical_site.launch \
  use_sim_time:=true rviz:=false

# In a second terminal, play exactly once at 1x.
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
rosbag play --clock -d 2 "$BAG" \
  /tiaozhanbei/lidar:=/petrochemical/lidar \
  /tiaozhanbei/imu:=/petrochemical/imu \
  /tiaozhanbei/camera/left/image/compressed:=/petrochemical/camera/left/image_raw/compressed

# 4. Freeze the run directory created under the configured save_path.
RESULT_DIR=/path/to/result_root/YYYY-MM-DD-HHMMSS

# 5. Generate formal person-ground output with the official BASE definition.
python3 "$SCRIPT" "$RESULT_DIR" --out-dir "$RESULT_DIR/submission" \
  --schedule-file "$SCHEDULE" --reference-mode person_ground \
  --saved-reference-lever-body-m 0 0 0 \
  --body-to-person-m 0.180 0.010 -1.192 \
  --base-rinex "$BASE_RINEX" --base-offset-m 0 0 0 \
  --base-offset-source OFFICIAL_CONFIRMATION_SAME_AS_RINEX_BASE

# 6. Validate format and locked hashes.
python3 "$SCRIPT" --schedule-file "$SCHEDULE" \
  --validate-only "$RESULT_DIR/submission"
(cd "$RESULT_DIR/submission" && sha256sum -c submission_stage6b_sha256.txt)
```

The bundled `run_gnss_livo.sh` creates a different locked pose-graph artifact and
is not silently substituted for this fixed-lag Online trajectory. The `result_dir`,
output, schedule, RINEX, dataset, bag, and optional reference
directory are all CLI inputs. The current G4 path is only a reproducibility
example, not a code dependency. Never use official position truth to fit BASE
offsets, alignment, levers, timestamps, parameters, or source selection.
