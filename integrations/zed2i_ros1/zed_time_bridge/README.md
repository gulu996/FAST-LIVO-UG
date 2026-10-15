# zed_time_bridge

ROS1 L2-B **SHADOW_ONLY** package. It does not modify any image Header, publish
Image messages, control the camera, write Livox shared memory, or feed FAST-LIVO2.
Default collection uses only host clocks and the existing read-only IMU anchor.
`valid=true` means the configured software mapping gates passed. It is **not** a
physical synchronization or exposure-accuracy claim. Extra compensation defaults
to zero, meaning uncalibrated rather than zero physical error.

## Model and existing dependency

`M_Z = inverse_clock_fit(Z)` and
`D = A_D + round(alpha_D * (M_Z-A_M)) - compensation_ns`.
Positive compensation moves output earlier. Epoch timestamps, subtraction,
fit coefficients and final rounding use checked integer/rational arithmetic;
floating ppm values are diagnostics only. Fits use centered differences.

The protocol dependency is the actual existing module
`src/sensor_time_bridge/src/sensor_time_bridge/imu_anchor.py` in this repository.
The recorder loads its `LivoxImuAnchorReader` with process-local import paths;
neither its source nor mmap format is copied or changed. On another checkout set
`SENSOR_TIME_BRIDGE_PYTHON_ROOT` to the directory containing `sensor_time_bridge`.
`record --anchor-module-root` is also supported. No pip/APT installation needed.

MONOTONIC is mandatory; MONOTONIC_RAW is not a compatible replacement. Current
writer age/sequence progress and historical target-to-anchor distance are separate
gates. Clock discontinuities and anchor/session faults invalidate affected models;
an unexplained ZED regression requires a new explicit camera session. A hidden SDK
restart that preserves all observable counters/stamps cannot be proven by headers.
Dual-eye metadata uses one immutable result per `(session_id, raw_stamp_ns)`.
CameraInfo duplicates reuse the result; IMU has an independent order check.
An already saved result is never rewritten after a later fault.

See [integration setup](../README.md) for dependency paths and isolated build.
Commands below run from this package directory.

## Run (new output directory required every time)

```bash
# No ROS subscriptions, no image workload; reads clocks and anchor only.
python3 -B scripts/zed_time_audit.py record \
  --outdir /tmp/zed_clock_run_001 --duration 30 --config config/shadow.json

# Replay ONLY this run's captured inputs, never live clocks/anchors.
python3 -B scripts/zed_time_audit.py replay \
  --input-dir /tmp/zed_clock_run_001 --outdir /tmp/zed_clock_replay_001

# Existing L1 CSV: metadata/negative-delay audit only. No historic anchor means invalid.
python3 -B scripts/zed_time_audit.py l1-audit \
  --input-dir "$ZED_EVIDENCE_DIR/baseline" \
  --outdir /tmp/zed_l1_metadata_001 --load-label historical_baseline

# Optional later, in the selected ROS master/environment: only the small raw IMU.
python3 -B scripts/zed_time_audit.py record --outdir /tmp/zed_imu_run_001 \
  --duration 30 --imu-topic /zed2i/zed_node/imu/data_raw --load-label imu_metadata
```

The optional CameraInfo path requires **both** `--camera-info` and
`--accept-camera-info-workload`: subscribing can activate the official wrapper's
lazy image work. It is not needed for this software acceptance. No Image
subscription or nodelet-loading code exists. `launch/shadow.launch` starts only
the clock collector and needs an explicit fresh `outdir`; it never launches ZED.
The optional ROS path detects caller/sequence regressions, but does not restart
the publisher or change its settings.

Each output directory has a manifest (software/config/dependency hashes and
provenance), ordered `records.jsonl` (all clockpairs, raw anchors, events, results),
flattened `events.csv`, summary and checksums. Absolute nanoseconds are decimal
integers. JSONL is the canonical replay input; original raw fields are retained.
Replay compares complete results including invalid reasons and frozen versions.
Outputs refuse to overwrite an existing directory. A changed sidecar fails hash
verification. Checksums establish internal consistency, not authenticated origin.

Live anchor history is never mixed into historical L1 data. Historical L1 files
lack bracketed clockpairs and Livox anchors, so they cannot produce valid device
time. Capture a fresh sidecar alongside a future simultaneous recording.

## Software checks

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/src:${SENSOR_TIME_BRIDGE_PYTHON_ROOT:?set the existing protocol source directory}${PYTHONPATH:+:$PYTHONPATH}" \
python3 -B -m unittest discover -s test -v
```

Tests inject clock jumps/restarts in memory and create simulated anchor files
inside temporary directories only. They do not change the real clock, shared
file, nodes or firmware. Numeric fixture accuracy is distinct from physical error.

## Dual IMU tool (B4 preparation)

`scripts/correlate_imus.py --help` documents three CSV inputs. Select one continuous
mapping session and its valid interior; keep the original full sidecar unmodified.
Both gyro CSVs contain `raw_stamp_ns,session_id,units,angular_velocity_x/y/z`.
MID's raw stamp is device time; the ZED CSV is joined to the captured mapping by
session/raw stamp. Use `rad/s` or explicit `deg/s`. Declare a common stationary
interval in **MID-domain integer ns**, not the ZED raw epoch.

```bash
python3 -B scripts/correlate_imus.py --zed-csv zed_imu.csv --mid-csv mid_imu.csv \
  --mapping-csv selected_mapping.csv --bias-start-ns START_NS --bias-end-ns END_NS \
  --provenance recorded --output new_imu_analysis.json
```

The sign is `corr(Z(t),MID(t+delta))`: add delta to ZED mapped time; equivalent
mapper parameter is `compensation_ns=-delta_ns`. The tool never applies it.
Native-rate centered FIR, limited interpolation, independent windows, subwindow
bootstrap, drift fitting and held-out windows are included. Static/periodic motion,
large gaps, excessive jitter and invalid/cross-session mappings are rejected.
The CI has a half-grid lower bound (2.5 ms at 200 Hz); do not infer nanosecond
physical accuracy from precise arithmetic. It estimates relative effective gyro
timing, including unknown sensor/filter delays, **not image exposure timing**.
Without inputs it outputs `HARDWARE_PENDING`; a computed estimate also does not
automatically become `HARDWARE_VERIFIED`.

## Remaining physical work

No finite bound is established for Livox sample→SDK callback delay, SDK IMAGE/IMU
clock estimation, PPS/RMC second association, exposure reference or rolling-shutter
row timing. Ready/type and configured uncertainty are not calibration evidence.
After separate hardware authorization: rigidly mount both devices, verify actual
PPS/GPRMC and packet clock source, record motion plus sidecar, then run multiwindow
dual-IMU checks. Controlled image-load A/B and external LED/photodiode/scope exposure
experiments remain B5/B6 pending. Do not inherit the old camera's +0.1 s offset.
