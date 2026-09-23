# Petrochemical-site offline dataset

This entrypoint is isolated from the MID-360 launch/configuration. The compact
bag stores the original left JPEG bytes and PPK GNSS; a launch-local
`image_transport republish` node decodes the image for FAST-LIVO2.

## Verified source contract

- Camera: left/right baseline JPEG, 1280x720, timestamp in each filename as Unix ns.
- IMU: `imu/imu.csv`, Unix-ns timestamp, angular velocity in rad/s and acceleration in m/s^2.
- LiDAR: `lidar/points.csv.gz`, 16-ring rotating scan, approximately 5 Hz. Each row has a
  frame Unix-ns timestamp and a point offset in seconds. Manufacturer/model is not stated.
- GNSS: `rover_ppk_solution_full.txt`, UTC Unix ms, RTKLIB Fixed/Float quality,
  WGS-84 ellipsoidal height, and BASE-origin N/E/U with position standard deviations.
- The first IMU samples already contain rotation; no synthetic stationary prefix is added.

The LiDAR converter uses a generic `sensor_msgs/PointCloud2` schema:
`x(float32), y(float32), z(float32), intensity(float32), offset_time(float64 seconds), ring(uint16)`.
It does not create Livox-only `tag` or `reflectivity` fields.

The compact camera topic is
`/petrochemical/camera/left/image_raw/compressed` (`sensor_msgs/CompressedImage`).
The original JPEG file bytes are copied directly with `format: jpeg`; the right
camera is intentionally omitted. PPK LLA/quality is written as the already-supported
`gnss_serial_driver/GnssPvtStamped` on `/gnss/pvt_local`; its BASE-RINEX-origin
`east_m,north_m,up_m` is written as `nav_msgs/Odometry` on `/gnss/enu_odom`.
For Float epochs, that Odometry covariance applies the AR-ratio penalty from
`ppk_covariance` in `rtk_fixed_lag_backend.yaml`; the backend then applies its
dataset-only Float/satellite scale and post-outage recovery ramp. The PVT
`h_acc/v_acc` fields remain the source PPK SD values for quality diagnostics.
During post-outage Float-only recovery, PositionFactors are additionally capped
at 1 Hz; normal factor rate resumes when accepted Fixed factors are stable.
The reference truth files are never read by the converter or written to the bag.
The PPK ENU origin is the BASE RINEX `APPROX POSITION XYZ`. That RINEX has blank
marker/antenna metadata and zero `ANTENNA: DELTA H/E/N`; the dataset does not
provide an APC/ARP/marker-to-official-BASE height or offset. Applying only the
known rover body-to-person lever would therefore be incomplete. The backend
graph, ROS optimized odometry, and saved optimized TUM files currently remain at
the body/IMU origin until the missing BASE geometry is supplied.

## 1. Dry run

```bash
source /home/gulu/catkin_ws/devel/setup.bash
python3 tools/dataset_conversion/convert_petrochemical_to_bag.py \
  --dataset "/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/融合定位" \
  --start 0 --duration 30 --dry-run
```

## 2. Convert the first continuous 30 seconds

The converter always parses the original LiDAR and IMU files, copies JPEG bytes
without decoding/re-encoding, defaults to LZ4, and atomically replaces
`<output>.partial` only after the new bag passes its topic/type check.

```bash
source /home/gulu/catkin_ws/devel/setup.bash
python3 tools/dataset_conversion/convert_petrochemical_to_bag.py \
  --dataset "/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/融合定位" \
  --output /tmp/petrochemical_compressed_gnss_30s.bag \
  --start 0 --duration 30
```

Use `--overwrite` only when intentionally replacing an existing output.

## 3. Inspect

```bash
rosbag info /tmp/petrochemical_compressed_gnss_30s.bag
```

## 4. Start FAST-LIVO2

```bash
source /home/gulu/catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}
roslaunch fast_livo mapping_petrochemical_site.launch rviz:=false
```

The system-library prefix avoids the incompatible libusb under `/opt/MVS/lib/64`
on this workstation while retaining the ROS and workspace library paths.

## 5. Play

FAST-LIVO2 synchronizes these inputs from their message header stamps, so simulated
time is not required for the first test:

```bash
rosbag play -d 2 /tmp/petrochemical_compressed_gnss_30s.bag
```

If another consumer explicitly needs simulated ROS time, enable both sides together:

```bash
roslaunch fast_livo mapping_petrochemical_site.launch rviz:=false use_sim_time:=true
rosbag play --clock -d 2 /tmp/petrochemical_compressed_gnss_30s.bag
```

The launch starts the JPEG decoder, `fastlivo_mapping`, GNSS adapter, and fixed-lag
backend. UWB remains disabled. PPK Float is accepted only by this dataset's
adapter/backend configuration; MID-360 retains its existing quality policy.
Its default runtime-output directory is `/tmp/fast_livo_petrochemical`; pass
`cmd_name:=/some/writable/path` when logs or maps need to be retained elsewhere.

## Competition bag image topic

`competition.bag` records the left camera on
`/tiaozhanbei/camera/left/image/compressed`, not the compact-dataset topic above.
Use the dataset-specific one-command frontend launch; it starts the required
decoder and keeps the generic mapper subscribed to a raw image topic:

```bash
source /home/gulu/catkin_ws/devel/setup.bash
roslaunch fast_livo mapping_competition.launch rviz:=false use_sim_time:=true
```

The compressed input base and decoded output remain launch arguments:
`compressed_image_base_topic` and `decoded_image_topic`. No competition camera
topic is hard-coded in the mapper source.

## Full conversion

Run this only after the 30-second replay succeeds and after checking free disk space:

```bash
source /home/gulu/catkin_ws/devel/setup.bash
python3 tools/dataset_conversion/convert_petrochemical_to_bag.py \
  --dataset "/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/融合定位" \
  --output "/media/gulu/一只沙糖桔/BaiduNetdiskDownload/测试数据/petrochemical_compressed_gnss.bag"
```
