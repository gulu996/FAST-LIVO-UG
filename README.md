## 编译仓库

```
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_CONVENIENCE_LIBRARIES=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_PYTHON=OFF

cmake --build build -j2
sudo cmake --install build
sudo ldconfig
```

```
cd ~/catkin_ws/src/livox_ros_driver2
./build.sh -j2
```

## 采集数据

`bag_name` 为空或省略时输出 `YYYYMMDD_HHMMSS.bag`；指定
`bag_name:=mission_01` 或 `bag_name:=mission_01.bag` 时均严格输出
`mission_01.bag`。已有同名 bag 或 `.active` 文件时拒绝覆盖。

### 仅uwb

```
roslaunch sensor_recording_bringup record_all.launch \
  enable_time_bridge:=true \
  enable_livox:=true \
  enable_camera:=true \
  enable_gnss:=false \
  enable_gnss_adapter:=false \
  enable_uwb:=true \
  uwb_port:=/dev/ttyUSB1 \
  uwb_dtr:=true \
  uwb_rts:=false \
  mcu_input_mode:=simulation \
  sim_start_without_gnss_s:=86400 \
  output_dir:=/home/jetson/bags \
  bag_name:=livo_camera_uwb \
  record_profile:=both
```

### 仅gnss

```
roslaunch sensor_recording_bringup record_all.launch \
  enable_time_bridge:=true \
  enable_livox:=true \
  enable_camera:=true \
  enable_gnss:=true \
  enable_gnss_adapter:=true \
  enable_uwb:=false \
  gnss_port:=/dev/ttyUSB0 \
  mcu_input_mode:=simulation \
  sim_start_without_gnss_s:=86400 \
  output_dir:=/home/jetson/bags \
  bag_name:=livo_camera_gnss \
  record_profile:=both
```
