# Joint GNSS/UWB graph and transaction publication

**Purpose:** verify that a single real GTSAM fixed-lag graph can accept both
factor types on the same pose key and that a rejected frontend query publishes
no speculative raw odometry. **Data:** synthetic, explicitly common frame/time.
No field dataset, SDK or private bag is required for the source fixture.

Historical passing version: `dd35f77fa0c1ff4728eace5330d7d9905ce36534`.
The fixture is still tracked in current source at
[`backend_odometry_commit_self_test.cpp`](../../src/FAST_LIVO2/src/backend_odometry_commit_self_test.cpp).
Historical result: one GNSS factor, one UWB factor, three actual optimization
updates. Duplicate/stale/NaN/ordering negative cases were rejected. Conditional
accepted publication happened once; rejected visual query publication was blocked.

## Reproduction

Use a dedicated master with **no hardware or live mapper**. In terminal 1:

```bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11331
roscore -p 11331
```

In terminal 2, from the workspace root:

```bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11331
PKG=$(rospack find fast_livo)
OUT=$(mktemp -d /tmp/fast-livo-joint.XXXXXX)
rosparam load "$PKG/config/mid360.yaml"
rosparam load "$PKG/config/gnss_adapter_mid360.yaml"
rosparam load "$PKG/config/rtk_fixed_lag_backend_mid360.yaml"
rosparam load "$PKG/config/camera_pinhole_mid360.yaml" /laserMapping
rosparam set /laserMapping/save_path "$OUT"
rosparam set /pcd_save/save_path "$OUT"
rosparam set /pcd_save/pcd_save_en false
rosparam set /rtk_backend/save_results false
rosparam set /rtk_backend/save_text_log false
rosparam set /fusion_ownership_fixture/use_gnss_fusion_enable true
rosparam set /gnss_fusion/enable true
rosrun fast_livo backend_odometry_commit_self_test "$OUT"
cat "$OUT/counterexample.json"
```

The fixture internally changes test parameters and creates its own graph. It
checks external UWB ingress over actual ROS callbacks as well as publication.
Stop only this test master after completion. No long replay is involved.

Do not reinterpret the legacy output field `full_compatibility_pass:false` as
failing the scoped assertions: the source deliberately denies complete field
compatibility certification. Inspect exit status and the scoped fields, including
`rejected_query_publication_blocked` and `conditional_committed_publication_once`.
Nor should passing these fields be renamed “real GNSS+UWB positioning accuracy.”

Remaining requirements for a real joint test: simultaneous sensor recordings,
common time mapping, surveyed anchor/antenna coordinates, backend map alignment,
independent truth, documented loss/quality conditions and a frozen evaluator.
