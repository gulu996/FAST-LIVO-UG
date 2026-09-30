# Vision Pro current-frame point cloud, phase 1

## Source audit

ROS1 Noetic, catkin. `/cloud_registered` is advertised in `LIVMapper::initializeSubscribersAndPublishers` and published in `publish_frame_world`. `handleLIO` first creates `laserCloudWorld` by `RGBpointBodyToWorld`, using current pose and LiDAR extrinsics. Its `frame_id` is `camera_init`, the SLAM world frame. The source PCL type `PointXYZINormal` has FLOAT32 x/y/z/intensity/normal_x/normal_y/normal_z/curvature. In the default image branch `/cloud_registered` instead carries RGB points and can accumulate scans when `pub_scan_num > 1`. `/cloud_effected` is selected map correspondences; `/mapping/globalMap` is historical map data.

Opt-in `/cloud_registered_frame` publishes that already transformed `laserCloudWorld` before the RGB branch. One message is one LIO scan, with `camera_init` and the scan's `last_lio_update_time`. Expected cadence is the completed LIO scan rate; measure it on the target bag/sensor. No history is added. The code default is disabled; `config/mid360.yaml` enables the source topic for the MID360 launch profile.

## VPPC v1

Each WebSocket **binary** message is one `POINT_CLOUD_FRAME`. Fixed header: 36 bytes. Integers are unsigned little-endian; points are IEEE-754 float32 little-endian. Then UTF-8 `frame_id`, then packed XYZI points, 16 bytes each.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | ASCII `VPPC` |
| 4 | 2 | version = 1 |
| 6 | 2 | type = 1 (`POINT_CLOUD_FRAME`) |
| 8 | 8 | Gateway input sequence; gaps indicate drops |
| 16 | 8 | ROS header timestamp in nanoseconds |
| 24 | 4 | filtered point count |
| 28 | 2 | point stride = 16 |
| 30 | 2 | UTF-8 frame ID byte length |
| 32 | 4 | payload length = count × 16 |
| 36 | variable | frame ID, no terminator |
| next | count × 16 | x, y, z, intensity float32 per point |

The Gateway validates PointCloud2 layout and removes nonfinite points. Payload is bounded at 64 MiB. `max_range_m` measures from the `camera_init` origin because PointCloud2 has no sensor origin; leave it zero for unchanged geometry. Voxel filtering keeps the first point per voxel, also disabled by default. One client is supported. The async sender holds at most one in-flight write and one pending frame; a newer scan replaces the pending frame. No old frame is replayed to a new client.

## Run

```bash
catkin_make -C /home/gulu/catkin_ws -j4
source /home/gulu/catkin_ws/devel/setup.bash
```

Start `roscore`. `config/mid360.yaml` already sets `publish/visionpro_frame_en: true`, so launch the mapper in a sourced terminal:

```bash
roslaunch fast_livo mapping_mid360.launch
```

Start the Gateway and receiver in separate sourced terminals:

```bash
roslaunch fast_livo visionpro_pointcloud_gateway.launch
python3 /home/gulu/catkin_ws/src/FAST_LIVO2/tools/visionpro_pointcloud_receiver.py ws://127.0.0.1:8765/
```

The Gateway may start before the mapper and waits for its topic. `Ctrl+C` stops it independently. `listen_address` and `port` are in its dedicated YAML. The default bind address exposes an unauthenticated display link on all interfaces; use a trusted network.
