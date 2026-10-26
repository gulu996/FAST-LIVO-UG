# Build and sensor capture

Scope: the source tree at `d9497af`, Ubuntu 20.04 / ROS 1 Noetic. Commands were
checked against CMake, package manifests and launch files; no clean-machine
installation or algorithm rebuild was performed for this documentation change.
Resolve [redistribution issues](../THIRD_PARTY_LICENSES.md) before shipping binaries.

## Core dependencies

Install ROS Noetic first using the [ROS installation instructions](https://wiki.ros.org/noetic/Installation/Ubuntu).
Use a fresh terminal with no unrelated catkin overlays.

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install build-essential cmake git libeigen3-dev libpcl-dev libopencv-dev \
  libboost-all-dev libusb-1.0-0-dev python3-rosdep python3-catkin-tools \
  python3-serial python3-numpy python3-nose \
  ros-noetic-pcl-ros ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-compressed-image-transport ros-noetic-eigen-conversions \
  ros-noetic-cmake-modules ros-noetic-tf ros-noetic-rosbag ros-noetic-rviz \
  ros-noetic-rostest ros-noetic-geometry-msgs ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs ros-noetic-visualization-msgs
```

The core uses PCL, Eigen, OpenCV, Boost, ROS messages and optional OpenMP/mimalloc.
Use the bundled `rpg_vikit`; do not clone a second package with the same names.
The frontend ships its legacy Livox message headers in `include/livox_ros_driver/`.
Do not silently substitute another wire format: verify bag message MD5/type first.

### Sophus (old non-templated API)

The upstream installation pins `a621ff`; a recent templated Sophus release is
not a drop-in replacement.

```bash
mkdir -p ~/fast_livo_deps
cd ~/fast_livo_deps
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
sudo cmake --install build
sudo ldconfig
```

### GTSAM 4.2 and unstable fixed-lag component

```bash
cd ~/fast_livo_deps
git clone --branch 4.2 --depth 1 https://github.com/borglab/gtsam.git
cd gtsam
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_CONVENIENCE_LIBRARIES=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_PYTHON=OFF
cmake --build build -j2
sudo cmake --install build
sudo ldconfig
ls /usr/local/include/gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h
ls /usr/local/lib/libgtsam*.so*
```

`find_package(GTSAM REQUIRED)` and the unstable header/library are mandatory.
For a nonstandard installation, pass `GTSAM_DIR` and `GTSAM_UNSTABLE_ROOT` to
catkin/CMake and arrange the runtime library search path consistently. Do not
mix header/library versions from different GTSAM builds.

### Build without optional proprietary camera SDK or LiDAR driver

From the repository/workspace root:

```bash
cd ~/fast_livo_ug_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='sensor_time_msgs;sensor_time_bridge;gnss_comm;gnss_serial_driver;uwb_serial_driver;vikit_common;vikit_ros;fast_livo;sensor_recording_bringup'
source devel/setup.bash
rospack find fast_livo
roslaunch --files fast_livo mapping_mid360.launch
roslaunch --files fast_livo mapping_petrochemical_site.launch
```

The whitelist builds the frontend/backend and sensor message/time support, but
not the hardware camera or Livox driver. It does not make all recording launch
options usable without their drivers. `src/FAST-Calib` is not tracked in the
current local commit; do not assume it is provided by a clone.

### Optional drivers and complete workspace

The existing `mvs_ros_driver` needs a separately installed Hikrobot **MVS SDK**:
`/opt/MVS/include` and `/opt/MVS/lib/64` on x86-64, or `/opt/MVS/lib/aarch64` on ARM.
Obtain it from the vendor and accept its terms yourself; it is not installed by
these instructions. Its wrapper's redistribution license is unresolved.

`livox_ros_driver2` needs [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
installed separately and the ROS1 CMake selection. After installing the matching
SDKs and configuring the sensor network:

```bash
cd ~/fast_livo_ug_ws
catkin_make -j2 -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES=''
source devel/setup.bash
```

This is the full-workspace path, not a claim that SDK installation was verified
on a clean host in this docs task. ROS2 and Jetson performance are not certified.
If the MVS environment shadows system `libusb`, inspect `ldd` on the actual node;
fix the process-local search path instead of replacing system libraries.

## Recording (independent from fusion)

[`record_all.launch`](../src/sensor_recording_bringup/launch/record_all.launch)
starts selected capture drivers and the session recorder. It does not prove that
a backend accepted factors. Serial devices, trigger wiring, sensor profiles,
network settings, timestamp bridges and file permissions must be configured for
your hardware. Use serial-group/udev permissions rather than world-writable devices.

Example GNSS recording with real MCU input:

```bash
roslaunch sensor_recording_bringup record_all.launch \
  enable_time_bridge:=true enable_livox:=true enable_camera:=true \
  enable_gnss:=true enable_gnss_adapter:=true enable_uwb:=false \
  mcu_input_mode:=serial mcu_port:=/dev/sensor_mcu \
  gnss_port:=/dev/ttyUSB0 output_dir:="$HOME/bags" \
  bag_name:=livo_gnss record_profile:=both
```

For UWB-only capture use `enable_gnss:=false enable_gnss_adapter:=false
 enable_uwb:=true uwb_port:=/dev/ttyUSB1 uwb_dtr:=true uwb_rts:=false` in place
of the GNSS enable/port options. For both, enable both sets and use **different**
serial ports. Use the GNSS-assisted or non-GNSS clock procedure actually verified
for your hardware; a recorder successfully starting is not a clock-quality test.

The old README's `mcu_input_mode:=simulation sim_start_without_gnss_s:=86400`
is a **software test mode**, not physical trigger/synchronization validation.
Keep that distinction when capturing without GNSS. The separate local ZED worktree
and Jetson capture project are not included or modified by this release.

Inspect `rosbag info`, topic cadence, time-domain metadata, session finalization
and file completeness before using a capture. Stale UWB diagnostic values can
repeat with new host receive times; device `ok=1` alone is not freshness evidence.

## Troubleshooting

- Package not found: source this workspace's `devel/setup.bash` and inspect
  `ROS_PACKAGE_PATH` / `CMAKE_PREFIX_PATH` / `rospack find fast_livo`.
- Duplicate launch: move your experimental source copies out of the ROS package;
  do not delete the official launch. `roslaunch --files` is a non-hardware check.
- Missing unstable GTSAM: rebuild that dependency with `GTSAM_BUILD_UNSTABLE=ON`.
- No images: match `common/img_topic`, raw/compressed type and decoder input base.
- No optimized output: check adapter quality/origin readiness and backend alignment;
  frontend `camera_init` output is not GNSS-aligned by default.
- Repeatability: use a fresh mapper and the same inputs at `-r 1`; validate callback
  and committed-event identities. Wall-clock differences alone are not failures.
