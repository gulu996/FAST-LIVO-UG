# Competition recording: ~804 s frontend regression

**Purpose:** test input completion, bounded motion, state/P health, IMU/time
transactions and map lifecycle under full-duration LiDAR/IMU/camera operation.
Dataset: local `competition.bag`, WLR-720 / INS355 / ZED 2i profile.
No public download or redistribution permission has been verified.

**Version/configuration:** isolated `VISIBLE` candidate whose algorithm sources
were integrated as `d9497af`; `config/petrochemical_site/{mapping,calibration,camera_left}.yaml`.
External GNSS/UWB and landmark corrections were off; gravity alignment was off.
The tracked generic launch is not identical to the archived diagnostic launch.
That launch selected a traced queue-8 decoder, per-run output, compact diagnostics
and explicit external-off settings. Read its parameter record before exact replay.

| Run | LIO commits | VIO attempts | Accepted VIO | Max scan speed (m/s) | First/last distance (m) |
|---|---:|---:|---:|---:|---:|
| Pure LIO | 4029 | 0 | 0 | 1.250006 | 1.231824 |
| Full LIVO | 4029 | 24100 | 3467 | 1.255960 | 1.043327 |
| DROP2 LIVO | 4029 | 24098 | 3467 | 1.255960 | 1.043327 |

Last completed LIO: approximately 803.830079 s. LIVO received 4031 LiDAR,
160793 IMU (including final IMU), and 24115 images; DROP2 intentionally withheld
two images from mapper. No writer drops/errors were reported. All completed
transaction inputs, state/P and map gates passed. Full callback payload hashes
and TCP-send completion were not observed; detailed per-step F/Q matrices were
not independently recorded over the whole run.

## Actual execution form

Archive root identifier:
`VIO_GRID_ASSOCIATION_REPAIR_20261026_184637/variants/VISIBLE` (`VISIBLE` below).
Paths below substitute variables for private mount names, not executable files
added to this repo:

```bash
roslaunch "$VISIBLE/runs/NO_DROP804/run.launch" rviz:=false use_sim_time:=true \
  output_root:="$VISIBLE/runs/NO_DROP804/native"
rosbag play --clock -r 1 -d 2 "$COMPETITION_BAG" --topics \
  /tiaozhanbei/lidar /tiaozhanbei/imu /tiaozhanbei/camera/left/image/compressed
```

Pure LIO used the deterministic native `build/a_driver`, supplied through
`runs/A804/input.fifo`, with arguments `capture 4029 1785915346.3774276`;
`runs/A804/commands.json` records the full producer and driver invocation.
It was **not** an online LIVO run with images merely hidden from RViz.
The harness, ABI-specific binaries and data are maintainer-local. Do not run
archived launch files against another ABI or overwrite their output directories.

A normal public launch recipe is in the [README](../../README.md); it permits
operation on compatible data but does not recreate diagnostic state/P comparisons.
To make this exact test public, release permission-cleared input, calibration,
parameter manifests, portable decoder/driver/comparator sources and EOF gates.

**Integration scope:** formal build/native tests and new 20 s ROS comparison were
exact on recorded core fields. The three long runs were on the isolated candidate;
there was no additional formal-installed-binary 804 s run in that integration.
No ground-truth trajectory or ATE is available. Hardware: RTX 2060 laptop,
Ubuntu 20.04, ROS Noetic. No GPU speed or Jetson benchmark is inferred.
