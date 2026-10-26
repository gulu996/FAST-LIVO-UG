# Independent GNSS factor regression

**Purpose:** verify real recorded GNSS passes the time/quality/origin adapter and
enters the independent fixed-lag graph without frontend state/P feedback.
**Version:** `dd35f77fa0c1ff4728eace5330d7d9905ce36534`; this backend remains in
`d9497af`, but the historical sensor experiment was not rerun there.

Input: a separate local `804` recording's raw LIVO trajectory and serialized
`GnssPvtStamped` observations, limited to 90 s. This is **not** the petrochemical
`competition.bag` frontend accuracy test. No public data link/permission or ground
truth has been established.

| Measurement | Recorded count |
|---|---:|
| Raw poses | 898 |
| GNSS messages | 1349 |
| Origin initializations | 1 |
| Accepted adapter outputs | 439 |
| GNSS factors | 197 |
| Optimizations | 395 |
| Published TFs | 198 |

Initialized/aligned were true; halted was false. This is processing/factor evidence,
not absolute positioning accuracy. RTK status is not an independent truth trajectory.

**Actual invocation with private paths expressed as variables:**

```bash
rosrun fast_livo backend_real_input_self_test gnss \
  "$RAW_LIVO_TUM" "$GNSS_90S_BIN" 0 "$OUTPUT_JSON"
```

Requires a separate ROS master, `mid360.yaml`, `gnss_adapter_mid360.yaml`,
`rtk_fixed_lag_backend_mid360.yaml`, and camera parameters in `/laserMapping`.
Set `/real_backend_short/use_gnss_fusion_enable=true`, `/gnss_fusion/enable=true`,
and disable backend file logs for the fixture. Exact recorded setup is
`legacy_removal_20261023_130945/scripts/formal_native_runner.py` under the archive
identified by the [manifest](evidence_manifest.json). Its payload format is
length-prefixed ROS serialization, not an arbitrary GNSS CSV.

**Public fixture commands (no private inputs):**

```bash
rosrun fast_livo gnss_adapter_self_test
rosrun fast_livo gnss_fusion_policy_self_test
rosrun fast_livo rtk_fixed_lag_backend_self_test
```

Source resides in `src/FAST_LIVO2/src/` with those names and `.cpp` suffixes.
These test quality/time policies, lever-arm/alignment and fixed-lag behavior;
they do not re-create the real 90 s data. A public recorded reproduction still
needs cleared data, calibration, a timestamp reference and the saved parameter set.
