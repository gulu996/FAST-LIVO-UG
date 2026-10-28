# Frontend and external-factor modes

These instructions target the fixed-lag path introduced at `dd35f77` and
unchanged by the later frontend revisions through the current public algorithm
tree `9fe7e510`. Do not use removed frontend GNSS/UWB selectors. GNSS-only and
UWB-only mean external-factor selection, not replacement of LIVO.

## Explicit startup without launch parameter overwrite

Start `roscore` in terminal 1. In terminal 2 load all parameters **before** starting
any estimator. This MID-360 example uses the actual current consumers:

```bash
source ~/fast_livo_ug_ws/devel/setup.bash
PKG=$(rospack find fast_livo)
rosparam set /use_sim_time true             # false for live capture
rosparam load "$PKG/config/mid360.yaml"
rosparam load "$PKG/config/gnss_adapter_mid360.yaml"
rosparam load "$PKG/config/rtk_fixed_lag_backend_mid360.yaml"
rosparam load "$PKG/config/camera_pinhole_mid360.yaml" /laserMapping
rosparam set /pcd_save/save_path "$HOME/fast_livo_runs"
rosparam set /pcd_save/pcd_save_en false
rosparam set /fast_livo/run_log_directory __PENDING__
```

Select one mode (example: GNSS only):

```bash
rosparam set /gnss_fusion/enable true
rosparam set /rtk_backend/enable true
rosparam set /rtk_backend/uwb_factor_backend_en false
```

| Mode | GNSS enable | Backend enable | UWB factor enable |
|---|---|---|---|
| Frontend only | false | false | false |
| GNSS only | true | true | false |
| UWB only | false | true | true |
| GNSS + UWB | true | true | true |

Then, in separate sourced terminals:

```bash
rosrun fast_livo fastlivo_mapping __name:=laserMapping _use_gnss_fusion_enable:=true
```

```bash
rosrun fast_livo gnss_adapter_node __name:=gnss_adapter _use_gnss_fusion_enable:=true
```

```bash
rosrun fast_livo rtk_fixed_lag_backend_node __name:=rtk_fixed_lag_backend _use_gnss_fusion_enable:=true
```

The adapter is unnecessary when GNSS is disabled; the backend is unnecessary
when both external factors are off. Start playback/drivers only after subscribers
are ready. Do not subsequently launch a mapping launch that reloads defaults.
For the petrochemical profile replace the four configuration loads by
`petrochemical_site/mapping.yaml`, `calibration.yaml`, `gnss_adapter.yaml`,
`rtk_fixed_lag_backend.yaml`, and load `camera_left.yaml` into `/laserMapping`.
Start its compressed decoder separately as in the
[launch source](../src/FAST_LIVO2/launch/mapping_petrochemical_site.launch).

## GNSS requirements

The MID-360 adapter reads `/gnss/pvt_local` (`GnssPvtStamped`) in `stamped_local`
mode. It checks time/quality, selects `first_fixed` or `average_fixed` origin,
and publishes ENU odometry plus status. Configuration stores uncertainty in
standard-deviation units before conversion to covariance where applicable.
Inspect [`gnss_adapter_mid360.yaml`](../src/FAST_LIVO2/config/gnss_adapter_mid360.yaml)
and the actual receiver fields; do not substitute arrival time for measurement time.

The backend waits for sufficient raw-odometry/GNSS alignment geometry. The antenna
lever arm is **body/IMU origin to GNSS antenna**, expressed in body coordinates.
Measure it for your rig. Merely enabling the node does not establish an ENU transform.
The adapter already has different profiles for other formats/datasets; do not
feed an arbitrary `NavSatFix` into the MID-360 stamped-local endpoint.

## UWB requirements

For external stamped ranges, before starting mapper/backend:

```bash
rosparam set /uwb/input_mode external_topic
rosparam set /uwb/external_topic /uwb/ranges
```

The actual type is
[`uwb_serial_driver/UwbRangeArray`](../src/uwb_serial_driver/msg/UwbRangeArray.msg),
including session/round identity, timestamp provenance/uncertainty and per-anchor
validity. The mapper's range bridge forwards eligible measurements to
`/uwb/backend_ranges`. The fixed-lag backend creates range factors; the bridge
does not directly update the frontend state/P.

Alternatively `uwb/input_mode: internal` uses configured `uwb/source` (`serial`
or `file`), serial settings or `uwb/replay_file`. The existing
[`run_uwb_backend_replay.sh`](../src/FAST_LIVO2/tools/run_uwb_backend_replay.sh)
is a historical machine-specific harness with absolute paths and scenario overrides,
not a portable reproduction command. Do not run it unchanged on another host.

Before enabling factors, load your **surveyed and time-validated** UWB overlay:
`rosparam load /path/to/your_uwb_calibration.yaml`. It must set `uwb/anchors` and
`uwb/anchor_frame_align_en: false` only after establishing the correct frame.
The stored MID-360 profile uses `anchor_frame_align_en: true` for the retired path;
it is not ready to enable backend UWB without actual offline alignment.
Do not flip this flag alone to bypass the frame requirement.

UWB-only uses the configured backend initial/local map reference. With GNSS also
enabled, anchors must be in the backend's GNSS-aligned map frame, not an arbitrary
UWB local frame. Two anchor ranges cannot determine a general six-degree-of-freedom
frame transform. Timing, surveyed positions, range bias and antenna reference
points are dataset/rig-specific. Unverified example coordinates are not calibration.

## What has been tested

- Separate real-recorded GNSS and UWB 90 s backend software tests.
- A synthetic known common-frame/time graph receiving both factor types.
- Transaction publication/rollback and malformed-input checks.

See [GNSS](experiments/gnss.md), [UWB](experiments/uwb.md) and
[joint interfaces](experiments/joint-backend.md). These are not real simultaneous
field precision certification. The static UWB device audit still requires firmware
confirmation of measurement freshness, timing, antenna endpoints and delay calibration.
