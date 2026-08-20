# FAST-LIVO-UG

**FAST-LIVO2 + GNSS/RTK + UWB Multi-Sensor Fusion Localization and Mapping**

FAST-LIVO-UG is a multi-sensor localization and mapping system developed on top of [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2). The system extends the original LiDAR-Inertial-Visual odometry framework with **GNSS/RTK absolute positioning** and **UWB ranging constraints**, and uses a **GTSAM fixed-lag factor graph backend** for trajectory optimization.

The current system supports:

* LiDAR + IMU + Camera based FAST-LIVO2 odometry;
* GNSS/RTK absolute positioning;
* UWB range measurements;
* GNSS and UWB unified time synchronization;
* GTSAM `IncrementalFixedLagSmoother` backend;
* LiDAR / Camera / IMU / GNSS / UWB synchronized data acquisition;
* Real-time operation on Ubuntu 20.04 / ROS Noetic;
* Livox MID-360 and industrial camera based hardware platform.

---

## 1. System Architecture

The main processing pipeline is:

```text
                         ┌──────────────┐
                         │    Camera    │
                         └──────┬───────┘
                                │
┌──────────────┐         ┌──────▼───────┐
│ Livox LiDAR  ├────────►│              │
└──────────────┘         │  FAST-LIVO2  │
                         │   LIVO Front │
┌──────────────┐         │              │
│     IMU      ├────────►│              │
└──────────────┘         └──────┬───────┘
                                │ Raw LIVO Pose
                                ▼
                      ┌──────────────────────┐
                      │ Fixed-Lag Factor     │
GNSS / RTK ──────────►│ Graph Backend       │
                      │                      │
UWB Range ───────────►│ GTSAM                │
                      └──────────┬───────────┘
                                 │
                                 ▼
                      Optimized Trajectory
```

FAST-LIVO2 provides high-frequency relative motion estimation, while GNSS/RTK and UWB provide absolute or range-based constraints to suppress long-term drift.

The UWB module does **not** directly modify the FAST-LIVO2 front-end ESIKF state. Raw UWB ranges are added to the same fixed-lag optimization backend as GNSS constraints.

---

# 2. Repository Structure

The main directory structure is:

```text
FAST-LIVO-UG/
├── scripts/
│
└── src/
    ├── FAST-Calib/
    │
    ├── FAST_LIVO2/
    │   ├── config/
    │   ├── include/
    │   ├── launch/
    │   ├── msg/
    │   ├── rviz_cfg/
    │   ├── scripts/
    │   ├── src/
    │   └── tools/
    │
    ├── gnss_comm/
    ├── gnss_serial_driver/
    ├── livox_ros_driver2/
    ├── mvs_ros_driver/
    ├── rpg_vikit/
    │
    ├── sensor_recording_bringup/
    ├── sensor_time_bridge/
    ├── sensor_time_msgs/
    │
    ├── stm32_timersync-open/
    └── uwb_serial_driver/
```

The ROS package name of the main localization module is:

```text
fast_livo
```

The main MID-360 launch file is:

```text
src/FAST_LIVO2/launch/mapping_mid360.launch
```

---

# 3. Environment

The project is primarily developed and tested under:

```text
Ubuntu 20.04
ROS Noetic
C++14
```

Recommended hardware:

```text
Desktop:
    x86_64 Ubuntu 20.04

Embedded:
    NVIDIA Jetson Orin Nano 8GB

LiDAR:
    Livox MID-360

Camera:
    Industrial global-shutter camera

Absolute positioning:
    GNSS / RTK
    UWB
```

---

# 4. Dependencies

## 4.1 Install ROS Noetic

Install ROS Noetic according to the official ROS documentation.

After installation:

```bash
source /opt/ros/noetic/setup.bash
```

It is recommended to add this line to `~/.bashrc`:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 4.2 Install Basic Dependencies

```bash
sudo apt update

sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-all-dev \
    libusb-1.0-0-dev \
    python3-catkin-tools \
    python3-rosdep
```

Install common ROS packages:

```bash
sudo apt install -y \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-eigen-conversions \
    ros-noetic-tf \
    ros-noetic-nav-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs
```

---

# 5. Install Sophus

FAST-LIVO2 uses the non-templated / double-only version of Sophus.

```bash
cd ~

git clone https://github.com/strasdat/Sophus.git

cd Sophus

git checkout a621ff

mkdir -p build
cd build

cmake ..

make -j$(nproc)

sudo make install

sudo ldconfig
```

---

# 6. Install GTSAM

The GNSS/UWB fixed-lag backend depends on:

```text
GTSAM 4.2
gtsam_unstable
IncrementalFixedLagSmoother
```

Therefore, **GTSAM must be compiled with `GTSAM_BUILD_UNSTABLE=ON`**.

```bash
cd ~

git clone https://github.com/borglab/gtsam.git

cd gtsam

git checkout 4.2
```

Configure:

```bash
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
```

Compile:

```bash
cmake --build build -j2
```

Install:

```bash
sudo cmake --install build
sudo ldconfig
```

Check installation:

```bash
ls /usr/local/include/gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h

ls /usr/local/lib | grep gtsam
```

The following libraries should be available:

```text
libgtsam.so
libgtsam_unstable.so
```

> The fixed-lag backend cannot be compiled if `gtsam_unstable` is missing.

---

# 7. Clone FAST-LIVO-UG

For a new installation, the repository can be used directly as the catkin workspace.

```bash
cd ~

git clone https://github.com/gulu996/FAST-LIVO-UG.git catkin_ws

cd ~/catkin_ws
```

The resulting directory should be:

```text
~/catkin_ws/
├── src/
├── scripts/
└── ...
```

---

# 8. Install ROS Dependencies

Initialize `rosdep` if it has not previously been initialized:

```bash
sudo rosdep init
rosdep update
```

Then:

```bash
cd ~/catkin_ws

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

Some custom packages, including GNSS, UWB and time synchronization packages, are already included in this repository and therefore do not need to be cloned separately.

---

# 9. Compile Livox ROS Driver

Enter:

```bash
cd ~/catkin_ws/src/livox_ros_driver2
```

Compile:

```bash
./build.sh -j2
```

For machines with sufficient memory, the number of parallel jobs can be increased.

---

# 10. Compile FAST-LIVO-UG

Return to the workspace:

```bash
cd ~/catkin_ws
```

Compile:

```bash
catkin_make -j2
```

On a desktop with sufficient memory:

```bash
catkin_make -j$(nproc)
```

After compilation:

```bash
source ~/catkin_ws/devel/setup.bash
```

Add the workspace to `~/.bashrc`:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 10.1 Check Compilation

Check whether the main ROS package can be found:

```bash
rospack find fast_livo
```

Expected output is similar to:

```text
/home/<USER>/catkin_ws/src/FAST_LIVO2
```

Check the main executable:

```bash
roscd fast_livo
```

If GTSAM compilation fails with an error similar to:

```text
IncrementalFixedLagSmoother requires the GTSAM unstable component
```

check:

```bash
ls /usr/local/include/gtsam_unstable
ls /usr/local/lib/libgtsam_unstable*
```

and make sure GTSAM was compiled with:

```text
-DGTSAM_BUILD_UNSTABLE=ON
```

---

# 11. MID-360 Configuration

The main configuration file for the current hardware platform is:

```text
src/FAST_LIVO2/config/mid360.yaml
```

The default FAST-LIVO2 topics are:

```yaml
common:
    img_topic: "/left_camera/image"
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"
```

Before running your own sensors, check that the actual ROS topics are consistent with these settings.

Use:

```bash
rostopic list
```

to inspect available topics.

Typical sensor topics are:

```text
/livox/lidar
/livox/imu
/left_camera/image
```

---

# 12. Start Mapping

The recommended entry point for the MID-360 system is:

```bash
roslaunch fast_livo mapping_mid360.launch
```

This launch file starts:

```text
FAST-LIVO2 front-end
        +
GNSS adapter
        +
RTK / UWB fixed-lag backend
        +
RViz
```

The loaded configuration files include:

```text
config/mid360.yaml
config/gnss_adapter_mid360.yaml
config/rtk_fixed_lag_backend_mid360.yaml
config/camera_pinhole_mid360.yaml
```

---

## 12.1 Start Without RViz

For Jetson or remote operation:

```bash
roslaunch fast_livo mapping_mid360.launch rviz:=false
```

This can significantly reduce unnecessary GUI resource usage.

---

# 13. Replay a rosbag Dataset

It is recommended to **start the mapping node before playing the bag**.

Terminal 1:

```bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch fast_livo mapping_mid360.launch
```

Wait until the ROS nodes have completed initialization.

Then open Terminal 2:

```bash
cd ~/catkin_ws
source devel/setup.bash
```

Play the dataset:

```bash
rosbag play -d 2 /path/to/dataset.bag
```

`-d 2` gives the subscribers approximately two seconds to finish establishing connections before publishing sensor messages.

For this project, using this delay is recommended when performing repeatability experiments.

---

## 13.1 Replay at 2× Speed

For long datasets:

```bash
rosbag play -d 2 -r 2 /path/to/dataset.bag
```

where:

```text
-d 2
```

means a 2-second startup delay, and

```text
-r 2
```

means 2× playback speed.

For example:

```bash
rosbag play -d 2 -r 2 /media/gulu/Data/数据集/804.bag
```

---

## 13.2 Replay at Normal Speed

```bash
rosbag play -d 2 -r 1 /path/to/dataset.bag
```

or simply:

```bash
rosbag play -d 2 /path/to/dataset.bag
```

---

## 13.3 Pause During Playback

Press:

```text
SPACE
```

in the `rosbag play` terminal to pause or resume playback.

This is useful for long datasets or when inspecting GNSS outages.

---

## 13.4 Inspect a Dataset Before Playback

```bash
rosbag info /path/to/dataset.bag
```

Check that at least the required sensor topics exist.

For the default MID-360 configuration:

```text
/livox/lidar
/livox/imu
/left_camera/image
```

Depending on the experiment, GNSS and/or UWB topics should also be present.

---

# 14. Real-Time Sensor Data Acquisition

The repository contains a unified sensor acquisition package:

```text
sensor_recording_bringup
```

The main recording launch file is:

```text
sensor_recording_bringup/launch/record_all.launch
```

It can start sensor drivers, the time synchronization chain and rosbag recording in one command.

Supported sensors include:

```text
Livox LiDAR
IMU
Camera
GNSS
UWB
STM32 time bridge
```

---

# 15. Record LiDAR + IMU + Camera + UWB

Example:

```bash
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

The recorded file will be:

```text
/home/jetson/bags/livo_camera_uwb.bag
```

---

# 16. Record LiDAR + IMU + Camera + GNSS

Example:

```bash
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

The recorded file will be:

```text
/home/jetson/bags/livo_camera_gnss.bag
```

---

# 17. Record GNSS + UWB Simultaneously

To enable both GNSS and UWB:

```bash
roslaunch sensor_recording_bringup record_all.launch \
  enable_time_bridge:=true \
  enable_livox:=true \
  enable_camera:=true \
  enable_gnss:=true \
  enable_gnss_adapter:=true \
  enable_uwb:=true \
  gnss_port:=/dev/ttyUSB0 \
  uwb_port:=/dev/ttyUSB1 \
  uwb_dtr:=true \
  uwb_rts:=false \
  mcu_input_mode:=simulation \
  sim_start_without_gnss_s:=86400 \
  output_dir:=/home/jetson/bags \
  bag_name:=livo_camera_gnss_uwb \
  record_profile:=both
```

---

# 18. Recording Parameters

The main parameters of `record_all.launch` are:

| Parameter             |           Default | Description                 |
| --------------------- | ----------------: | --------------------------- |
| `enable_time_bridge`  |            `true` | Enable time synchronization |
| `enable_livox`        |            `true` | Enable Livox LiDAR/IMU      |
| `enable_camera`       |            `true` | Enable camera               |
| `enable_gnss`         |            `true` | Enable GNSS serial driver   |
| `enable_gnss_adapter` |            `true` | Enable GNSS adapter         |
| `enable_uwb`          |           `false` | Enable UWB                  |
| `mcu_port`            | `/dev/sensor_mcu` | STM32 serial device         |
| `mcu_baud`            |          `115200` | STM32 baud rate             |
| `gnss_port`           |    `/dev/ttyUSB0` | GNSS serial device          |
| `gnss_baud`           |          `921600` | GNSS baud rate              |
| `uwb_port`            |    `/dev/ttyUSB0` | UWB serial device           |
| `uwb_baud`            |          `115200` | UWB baud rate               |
| `output_dir`          |            `/tmp` | rosbag output directory     |
| `bag_name`            |             empty | Output bag name             |
| `record_profile`      |            `both` | Recording profile           |

If `bag_name` is omitted, the system automatically generates a name similar to:

```text
YYYYMMDD_HHMMSS.bag
```

For example:

```text
20260820_231500.bag
```

If:

```bash
bag_name:=mission_01
```

or:

```bash
bag_name:=mission_01.bag
```

is specified, the output will be:

```text
mission_01.bag
```

The recording program refuses to overwrite an existing `.bag` or `.active` file with the same name.

---

# 19. Check Serial Devices Before Recording

Check available devices:

```bash
ls -l /dev/ttyUSB*
```

If a persistent STM32 device name is configured:

```bash
ls -l /dev/sensor_mcu
```

You can also inspect USB devices using:

```bash
dmesg | tail -n 30
```

Typical configuration:

```text
STM32:
    /dev/sensor_mcu
    115200

GNSS:
    /dev/ttyUSB0
    921600

UWB:
    /dev/ttyUSB1
    115200
```

Actual device names depend on the computer and udev configuration.

---

# 20. Check Sensor Topics

Before starting mapping or recording, use:

```bash
rostopic list
```

Check LiDAR:

```bash
rostopic hz /livox/lidar
```

Check IMU:

```bash
rostopic hz /livox/imu
```

Check camera:

```bash
rostopic hz /left_camera/image
```

The sensor frequencies should remain stable during operation.

---

# 21. Recommended Dataset Replay Procedure

For reproducible experiments, the recommended procedure is:

### Terminal 1 — restart ROS

```bash
roscore
```

### Terminal 2 — start localization

```bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch fast_livo mapping_mid360.launch
```

### Terminal 3 — replay dataset

```bash
rosbag play -d 2 -r 2 /path/to/dataset.bag
```

For repeatability testing, completely terminate the previous mapping process before starting the next run.

---

# 22. Main Configuration Files

The most important configuration files are:

```text
src/FAST_LIVO2/config/
├── mid360.yaml
├── camera_pinhole_mid360.yaml
├── gnss_adapter_mid360.yaml
└── rtk_fixed_lag_backend_mid360.yaml
```

Their responsibilities are roughly:

### `mid360.yaml`

FAST-LIVO2 front-end configuration:

```text
LiDAR topic
IMU topic
Camera topic
LiDAR preprocessing
LIO parameters
VIO parameters
Extrinsic calibration
Time offset
Voxel map
Visual map
Degeneracy handling
GNSS/UWB switches
```

### `camera_pinhole_mid360.yaml`

Camera intrinsic parameters.

### `gnss_adapter_mid360.yaml`

GNSS input, quality control, ENU origin initialization and GNSS data adaptation.

### `rtk_fixed_lag_backend_mid360.yaml`

Fixed-lag factor graph configuration, including GNSS/UWB constraints and backend optimization parameters.

---

# 23. Sensor Calibration

Before using a new LiDAR-camera hardware platform, the camera intrinsics and LiDAR-camera extrinsics must be calibrated.

The repository contains:

```text
src/FAST-Calib
```

for LiDAR-camera calibration.

After calibration, update:

```text
src/FAST_LIVO2/config/mid360.yaml
src/FAST_LIVO2/config/camera_pinhole_mid360.yaml
```

Do not directly use the provided calibration values on a different physical sensor assembly.

---

# 24. Useful ROS Commands

List nodes:

```bash
rosnode list
```

List topics:

```bash
rostopic list
```

Check topic frequency:

```bash
rostopic hz /livox/lidar
```

Display topic content:

```bash
rostopic echo /livox/imu
```

Inspect node connections:

```bash
rqt_graph
```

Check rosbag:

```bash
rosbag info dataset.bag
```

---

# 25. Common Problems

## 25.1 `roslaunch` Cannot Find `fast_livo`

Error:

```text
RLException: [mapping_mid360.launch] is neither a launch file...
```

Run:

```bash
source ~/catkin_ws/devel/setup.bash
```

Then check:

```bash
rospack find fast_livo
```

---

## 25.2 GTSAM Unstable Missing

Error similar to:

```text
IncrementalFixedLagSmoother requires the GTSAM unstable component
```

Recompile GTSAM using:

```text
-DGTSAM_BUILD_UNSTABLE=ON
```

and run:

```bash
sudo ldconfig
```

---

## 25.3 Cannot Open GNSS/UWB Serial Port

Check:

```bash
ls -l /dev/ttyUSB*
```

Temporary permission fix:

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

A persistent udev rule is recommended for actual deployment.

---

## 25.4 No LiDAR Data

Check:

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
```

If no topic exists, check Livox network configuration and `livox_ros_driver2`.

---

## 25.5 No Camera Image

Check:

```bash
rostopic hz /left_camera/image
```

and:

```bash
rqt_image_view
```

Make sure the topic name matches:

```yaml
img_topic: "/left_camera/image"
```

in `mid360.yaml`.

---

## 25.6 Different Results Between Repeated rosbag Runs

For offline experiments, do not start `rosbag play` immediately after the mapping launch command.

Recommended:

```bash
rosbag play -d 2 /path/to/dataset.bag
```

For long datasets:

```bash
rosbag play -d 2 -r 2 /path/to/dataset.bag
```

Also completely restart the mapping process between repeated experiments.

---

# 26. Quick Start

For an already configured computer, the complete workflow can be reduced to the following.

## Build

```bash
cd ~/catkin_ws
catkin_make -j2
source devel/setup.bash
```

## Start Mapping

```bash
roslaunch fast_livo mapping_mid360.launch
```

## Play Dataset

```bash
rosbag play -d 2 -r 2 /path/to/dataset.bag
```

## Record GNSS Dataset

```bash
roslaunch sensor_recording_bringup record_all.launch \
  enable_gnss:=true \
  enable_gnss_adapter:=true \
  enable_uwb:=false \
  gnss_port:=/dev/ttyUSB0 \
  output_dir:=/home/jetson/bags \
  bag_name:=test_gnss \
  record_profile:=both
```

## Record UWB Dataset

```bash
roslaunch sensor_recording_bringup record_all.launch \
  enable_gnss:=false \
  enable_gnss_adapter:=false \
  enable_uwb:=true \
  uwb_port:=/dev/ttyUSB1 \
  output_dir:=/home/jetson/bags \
  bag_name:=test_uwb \
  record_profile:=both
```

---

# 27. Acknowledgements

This project is developed based on several open-source projects, including:

* FAST-LIVO2
* FAST-LIVO
* FAST-Calib
* Livox ROS Driver 2
* GTSAM
* RPG Vikit
* gnss_comm

We thank the authors and contributors of these projects for their excellent work.

---

# 28. License

The FAST-LIVO2 component follows its original GPLv2 license.

Please check the license of each third-party dependency before redistribution or commercial use.

---

# 29. Project

Repository:

```text
https://github.com/gulu996/FAST-LIVO-UG
```

Current system:

```text
FAST-LIVO2
   +
GNSS / RTK
   +
UWB
   +
GTSAM Fixed-Lag Factor Graph
   +
Multi-Sensor Time Synchronization
```
