# ZED 2i ROS1 桌面集成与旁路时间映射

适用：Ubuntu 20.04、ROS1 Noetic、RTX 2060 Max-Q 6GB、CUDA Toolkit 12.1.1、ZED SDK 4.1.4。
这是已完成的 L1 / L2-B 桌面开发成果整理，**不是 ZED↔MID-360 物理同步或图像曝光同步验收**。
L1 有历史真实相机证据；L2-B 是 SHADOW_ONLY 软件验证。当前不向 FAST-LIVO2 注入新时间戳。

本目录在仓库根目录的 `integrations/`，位于原 catkin `src/` 之外，不改变原构建、驱动、固件或标定。
原始数据及本机报告不随代码提交。[技术总结](docs/technical_summary.md)记录验证边界，
[整理验证](docs/curation_validation.md)记录本次副本测试。包的完整接口见 [zed_time_bridge/README.md](zed_time_bridge/README.md)。

## 固定依赖与第三方归属

- Stereolabs 官方 [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)，固定提交
  `6e5ab7d2a109eb8e43b8035363e34334ca48eaa4`，MIT；manifest 仍标 3.5.0，不能把它称为 Wrapper 4.1.4。
- 其 `zed-ros-interfaces` 子模块固定提交 `12f55da885dd3f0f68c168d0bccd33c4e892328d`。
  两者均不复制到本目录，也不作为用户原创代码。请遵守各上游许可。
- ZED SDK 4.1.4 为独立安装的专有依赖；CUDA Toolkit 12.1.1 与兼容 NVIDIA 驱动须预先就绪。
  历史测试驱动为 570.133.20。代码不安装 SDK、不升级驱动/CUDA，不改变系统时钟或 BLAS。
- ROS/catkin、roscpp、rospy、sensor_msgs、rosbag、nodelet、image_transport、tf2、dynamic_reconfigure、
  diagnostic_updater、xacro、robot_state_publisher 等，以固定上游 package.xml 为完整依赖清单。
- L1 预览需要 rqt_gui/rqt_image_view、系统 OpenCV；图像审计保存 PNG 需要 cv_bridge/python3-opencv。
- L2-B 需要 Python 3.8 标准库及 NumPy（双 IMU 分析）；复用本仓库现有
  `src/sensor_time_bridge/src/sensor_time_bridge/imu_anchor.py`，不复制协议或修改 Writer。
  无需为纯 Python 检查重建原 catkin 工作空间。基础代码审计来自仓库提交
  `b9dfc0ab47ac08edcd44c101d37dc49612c0417c`。

## 路径与独立构建

以下命令供明确选择的新目录使用，不在已有运行工作空间覆盖文件。
先在本仓库任意子目录执行，后续终端使用相同变量：

```bash
export REPO_ROOT="$(git rev-parse --show-toplevel)"
export ZED_INTEGRATION="$REPO_ROOT/integrations/zed2i_ros1"
export SENSOR_TIME_BRIDGE_PYTHON_ROOT="$REPO_ROOT/src/sensor_time_bridge/src"
export ZED_WS="$HOME/zed_desktop_ws"                # 独立、新建的 Wrapper 工作空间
export ZED_EVIDENCE_DIR="$ZED_WS/evidence"         # 数据仅在本机
export ROS_MASTER_URI=http://localhost:11331       # 可覆盖；勿与其他实验混用
export PYTHONDONTWRITEBYTECODE=1
```

仅首次创建全新的官方 Wrapper 工作空间（已有目录时先核对，勿重复克隆或覆盖）：

```bash
mkdir -p "$ZED_WS/src"
git clone https://github.com/stereolabs/zed-ros-wrapper.git "$ZED_WS/src/zed-ros-wrapper"
git -C "$ZED_WS/src/zed-ros-wrapper" checkout --detach 6e5ab7d2a109eb8e43b8035363e34334ca48eaa4
git -C "$ZED_WS/src/zed-ros-wrapper" submodule update --init --recursive
git -C "$ZED_WS/src/zed-ros-wrapper" rev-parse HEAD
git -C "$ZED_WS/src/zed-ros-wrapper" submodule status
(
  unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
  source /opt/ros/noetic/setup.bash
  cd "$ZED_WS"
  catkin_make -j2 -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.1
)
```

核对两个 SHA 与上文一致。依赖缺失应先检查上游 manifest/本机安装状态，不能用自动升级来绕过兼容性。
未包含 SDK 或 CUDA 安装器；软件许可由使用者独立确认。

L2-B 可单独构建，不依赖相机、不重编 Wrapper：

```bash
export ZED_SHADOW_BUILD="$(mktemp -d -t zed-shadow-build.XXXXXX)"
(
  unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
  source /opt/ros/noetic/setup.bash
  cmake -S "$ZED_INTEGRATION/zed_time_bridge" -B "$ZED_SHADOW_BUILD/build" \
    -DCATKIN_DEVEL_PREFIX="$ZED_SHADOW_BUILD/devel" \
    -DCMAKE_INSTALL_PREFIX="$ZED_SHADOW_BUILD/install" \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=ON \
    -DSENSOR_TIME_BRIDGE_PYTHON_ROOT="$SENSOR_TIME_BRIDGE_PYTHON_ROOT"
  cmake --build "$ZED_SHADOW_BUILD/build" -j2
  (cd "$ZED_SHADOW_BUILD/build" && ctest --output-on-failure -V)
)
# 现有 36 项 unittest；不要求真实 MID、ZED、ROS master 或更改系统时间。
python3 -B "$ZED_INTEGRATION/test/check_integration.py"
```

普通 catkin 打包可将 `zed_time_bridge` 链接进另一个新 workspace/src，并使既有
sensor_time_bridge Python 包可导入；上述单包构建是本次实际复测入口。
运行时仍需导出 `SENSOR_TIME_BRIDGE_PYTHON_ROOT`，或已有正确的已安装协议模块位于 PYTHONPATH。
CMake 的测试路径变量不会永久修改 shell 环境。

## L1：采集、预览、录包和回放

先检查 `rosnode list`、相机占用及 USB3。**已有相机节点时不要重复启动、重启或抢占设备**。
在允许的新采集窗口运行：

```bash
bash "$ZED_INTEGRATION/l1/run_camera.sh"
# 多相机时显式传入自己的序列号，不把真实序列号写入仓库：
# bash "$ZED_INTEGRATION/l1/run_camera.sh" serial_number:=YOUR_SERIAL
bash "$ZED_INTEGRATION/l1/run_preview.sh"
# 另一个终端；新绝对路径，不覆盖已有 bag。固定 5 秒、2 GiB buffer。
bash "$ZED_INTEGRATION/l1/record_bag.sh" "$ZED_EVIDENCE_DIR/new_capture.bag"
```

默认 serial_number=0 使用官方 camera ID 选择逻辑；多设备须显式选择。
配置保留 HD1080/30Hz、NATIVE、flip='OFF'、self_calib=false、depth=NONE、位置跟踪/建图/目标检测关闭，
IMU sensors_timestamp_sync=false、轮询上限400Hz。图像维持当前原始安装方向，不修改工厂标定。
脚本仅清理自身进程的库/ROS overlay 环境，默认 master 可由 ROS_MASTER_URI 覆盖。
预览只终止它自己启动的两个 GUI；当前相机节点不由预览脚本管理。

主话题前缀 `/zed2i/zed_node`：

| 后缀 | 类型 | 用途 |
|---|---|---|
| `/left/image_rect_color`, `/right/image_rect_color` | sensor_msgs/Image | 矫正双目，原始 SDK 时间 |
| `/left/camera_info`, `/right/camera_info` | sensor_msgs/CameraInfo | 矫正内参，同图像 stamp，可重复 |
| `/imu/data_raw`, `/imu/data` | sensor_msgs/Imu | 独立 IMU 采样时间 |

审计工具显式传新输出目录；已有目录拒绝覆盖。L1 图像审计、预览及录包都会引入全图像负载；
两路 BGRA8 1080P30 约497.664 MB/s。不要把这些工具默认叠加到 L2 旁路采集。

```bash
source "$ZED_WS/devel/setup.bash"
python3 -B "$ZED_INTEGRATION/l1/ros_audit.py" --self-test
python3 -B "$ZED_INTEGRATION/l1/bag_audit.py" --self-test
# 显式相机测试窗口，保留原 stamp；可选 --hash-payload 有额外CPU成本。
python3 -B "$ZED_INTEGRATION/l1/ros_audit.py" --duration 35 --imu-hz 400 \
  --outdir "$ZED_EVIDENCE_DIR/new_capture_audit"
python3 -B "$ZED_INTEGRATION/l1/bag_audit.py" "$ZED_EVIDENCE_DIR/new_capture.bag" \
  --imu-hz 400 --outdir "$ZED_EVIDENCE_DIR/new_bag_audit"
```

回放使用独立 master，例如另开终端 `ROS_MASTER_URI=http://localhost:11333 roscore -p 11333`。
在相同独立 master 上先启动带 `--hash-payload --duration 25` 的 ros_audit，输出新目录；等待订阅就绪，
再 `rosbag play --rate 1.0 --delay 1 new_capture.bag`。25秒覆盖上述5秒bag，长bag应相应延长审计窗口。
把 bag_audit 的 `bag_summary.json` 和回放 `summary.json` 交给现有 `compare_summaries`，仅对六个必需话题：

```python
from bag_audit import compare_summaries  # 将本目录 l1 加到 PYTHONPATH
import json
original = json.load(open('new_bag_audit/bag_summary.json'))
replayed = json.load(open('new_replay_audit/summary.json'))
original['topics'] = {k: v for k, v in original['topics'].items() if k in replayed['topics']}
assert compare_summaries(original, replayed)['all_topics_equal']
```

频率来自原 Header 差值；重放 wall−Header 包含 bag 年龄，不能用于硬件延迟测量。
原一次性 run_replay_check.py 的固定路径/文件名不保留，上述已有两个工具覆盖其实际功能。

## L2-B：默认无 ROS 订阅的旁路诊断

```bash
python3 -B "$ZED_INTEGRATION/zed_time_bridge/scripts/zed_time_audit.py" record \
  --outdir "$ZED_EVIDENCE_DIR/new_shadow" --duration 30 \
  --config "$ZED_INTEGRATION/zed_time_bridge/config/shadow.json"
python3 -B "$ZED_INTEGRATION/zed_time_bridge/scripts/zed_time_audit.py" replay \
  --input-dir "$ZED_EVIDENCE_DIR/new_shadow" --outdir "$ZED_EVIDENCE_DIR/new_shadow_replay"
```

默认只读主机钟和 `$HOME/timeshare_imu`，缺失锚点时报告 invalid，不伪造设备时间。
可显式 `--anchor`；`record --anchor-module-root` 可覆盖协议源码位置。
只有显式 `--imu-topic` 才订阅 IMU；CameraInfo 还要求 `--camera-info --accept-camera-info-workload`。
没有 Image 订阅/重发布，也不改变原 Header。保存整数原戳、时钟夹测、锚点、会话、失败原因及映射值。
离线重放使用记录时的 clock/anchor，不能把历史bag配当前共享文件。

## 本地证据与未来路线

原工作空间以 `ZED_ORIGINAL_WS` 表示：其中 `evidence/` 保存 L1 bags、CSV、图像、构建/回放及FAST-LIVO2回归；
`l2a_time_audit_evidence_20261015/` 保存源码审计；`l2b_evidence_20261015/` 保存合成、时钟和重放记录。
根目录三个 `zed_*report.md`/`zed_l2a_time_audit.md` 为完整本机报告。
L0 的 SVO 证据在本机 SDK 测试目录，未移动或重写。具体本机绝对路径只列入本地提交报告。

Jetson 若选择 zed-open-capture，应在独立目录/分支管理采集适配与版本依赖；不能把当前桌面 SDK 路线
直接当作 ARM/Jetson 已验证实现。必须重新审计图像与 IMU 时间戳语义、时钟域和同次双目关系，
再决定是否复用纯整数映射核心。不可照搬 SDK IMAGE 的入主机内存语义或已有性能数字。
本次没有实现 Open Capture、硬件曝光校准或 FAST-LIVO2 消费端改动。
