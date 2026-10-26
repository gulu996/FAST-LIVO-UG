# Independent UWB range-factor regression

**Purpose:** test real ranges entering the fixed-lag backend, not frontend ESIKF.
**Version:** `dd35f77fa0c1ff4728eace5330d7d9905ce36534`; distinct from the competition
LIVO data and from real simultaneous GNSS/UWB validation.

Input: local `0709` range text and its start-of-run unfused LIVO trajectory,
90 s, using `config/experiments/uwb_0709_calibrated.yaml` on top of the MID-360
backend profile. The historical trajectory filename `GT.json` actually contains
TUM text and is **not** ground truth. No public distribution permission is known.

| Measurement | Count |
|---|---:|
| Raw poses | 900 |
| UWB factors | 141 |
| Optimizations | 319 |
| Published TFs | 178 |

Initialized/aligned true; halted false. One earlier test used the wrong trajectory
segment and produced zero factors; its failure remains in the local archive.
The corrected run changed the selected input, not estimator thresholds.

**Actual command shape:**

```bash
rosrun fast_livo backend_real_input_self_test uwb \
  "$RAW_0709_TUM" unused 1590192198.700268745 "$OUTPUT_JSON"
```

Before running, a private master must have the saved MID-360/backend and
`uwb_0709_calibrated.yaml` parameters, GNSS disabled, backend/UWB factors enabled,
`uwb/anchor_frame_align_en=false`, and `uwb/replay_file` pointing to the matching
real text. The last numeric argument is that dataset's time reference, not a
universal constant. Exact setup is the archived `formal_native_runner.py`.
Do not apply its anchor coordinates or time offset to another site.

**Public fixture commands:**

```bash
rosrun fast_livo uwb_manager_self_test
rosrun fast_livo rtk_fixed_lag_backend_self_test
```

The second includes the range-factor analytic Jacobian check. Both are synthetic
software checks. Actual device measurements still require confirmation of antenna
reference points/delay calibration, clock mapping and stale-result behavior.
`distance[n]` identifies a target anchor in the examined text. Repeated diagnostic
`ok=1` with a new host receive time does not prove a fresh range. Different anchor
links must not be mixed to estimate one link's noise. No absolute range/position
accuracy claim is made without surveyed truth and firmware semantics.
