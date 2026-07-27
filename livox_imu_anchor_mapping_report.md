# MID-360 IMU 共享时间锚点映射实施报告

日期：2026-07-27  
开发工作空间：`/home/gulu/catkin_ws`  
部署工作空间：`/home/jetson/catkin_ws`  
状态：开发电脑编译与软件测试完成；Jetson 和真实传感器尚未验证

## 1. 执行摘要

本次新增了独立的 `$HOME/timeshare_imu` 共享时间锚点，并将 UWB、GNSS 的完整串口记录接收时刻连续映射到 MID-360 IMU 设备时间域：

```text
mapped_stamp_ns =
    imu_device_stamp_ns
    + (sensor_receive_monotonic_ns - imu_host_monotonic_ns)
    - configured_time_offset_ns
```

结论：

- Livox 驱动在 MID-360 SDK IMU 回调入口为同一 IMU 包记录 `CLOCK_MONOTONIC`，经现有 IMU 队列携带到发布线程后写入 64 字节共享协议。
- `/livox/imu` 和 `/livox/lidar` 原有时间戳没有改变。
- Camera 继续使用原 `$HOME/timeshare`；协议、驱动和时间戳行为没有改变。
- UWB `distance_round5` 的 ID、五槽和零值语义没有改变；`/uwb/ranges` 默认采用第一条语法有效的正式 `distance` 行上下文，旧 `first_nonzero` 行为可显式配置。
- GNSS 在完整行形成时记录接收时刻，仅让校验通过且位置有效的 PVT 进入映射后的融合链路。
- `mcu_input_mode=simulation` 只保留会话和录包门控职责，不再决定 anchor 模式下 UWB/GNSS 的 `header.stamp`。
- 映射失败、锚点未就绪或锚点超过 100 ms 时不回退系统时间或 simulation LOCAL 时间。
- ROS 消息定义及 MD5 均未改变。
- Release 全工作空间编译通过；最终测试汇总为 153 项、0 error、0 failure、0 skipped。

这是一套软件时间域统一方案，不是硬件级同步。它把主机完整数据接收时刻投影到 MID-360 时间轴，仍包含设备内部处理、网络/SDK、串口缓存和调度延迟。

## 2. 修改前时间架构

实际审计结果：

```text
STM32 PB5/RMC
  └─> MID-360 legacy 设备时间
       ├─> /livox/lidar header.stamp
       ├─> /livox/imu header.stamp
       └─> 原 $HOME/timeshare
            └─> Camera legacy header.stamp

sensor_time_bridge simulation LOCAL
  ├─> UWB host_local
  └─> GNSS host_local / UTC inverse mapping
```

因此修改前 LiDAR/IMU/Camera 和 UWB/GNSS 可能属于不同 epoch。

关键代码证据：

- SDK IMU 包入口：`src/livox_ros_driver2/src/comm/pub_handler.cpp:97`
- MID-360 包时间解析：同文件 `GetEthPacketTimestamp()`，第 262 行；PTP/GPS 直接返回设备包内时间，无同步类型则走既有系统时间分支。
- ROS IMU 构造和发布：`src/livox_ros_driver2/src/lddc.cpp:1242`
- GNSS 完整换行帧入口：`src/gnss_serial_driver/scripts/gnss_serial_node.py:304`
- GNSS 校验：`src/gnss_serial_driver/src/gnss_serial_driver/parser.py:89`
- GNSS ENU 适配器复制输入 Header：`src/gnss_serial_driver/src/gnss_adapter_node.cpp:83`
- FAST 外部 GNSS 已拒绝 `header.stamp==0`：`src/FAST_LIVO2/src/gnss_manager.cpp:1517`
- UWB 轮次时间策略：`src/uwb_serial_driver/src/uwb_serial_driver/parser.py` 的 `DistanceRoundAssembler`；默认 `first_distance_line`，兼容策略为 `first_nonzero`。

## 3. 修改后的实际数据链

### 3.1 Livox IMU 锚点

```text
Livox SDK IMU Ethernet packet
  data->time_type + data->timestamp
  │  pub_handler.cpp:97,111
  ├─> GetEthPacketTimestamp()
  │     └─> imu_data.time_stamp（MID-360 设备纳秒）
  └─> MonotonicNowNs()
        └─> imu_data.host_monotonic_ns（CLOCK_MONOTONIC）
              │
              └─> LidarImuDataQueue::Push()
                    lidar_imu_data_queue.cpp:35
                    │
                    └─> Lddc::PublishImuData()
                          lddc.cpp:1242
                          ├─> 原 sensor_msgs/Imu 发布，stamp 不变
                          └─> UpdateImuAnchor()
                                lddc.cpp:398,1255
                                └─> ImuTimeAnchorWriter
                                      └─> $HOME/timeshare_imu
```

`host_monotonic_ns` 在 SDK 回调边界记录，而不是在 ROS 消息发布完成后记录。队列排队和发布处理不会替换它。

### 3.2 UWB

```text
完整非空串口行
  └─> time.monotonic_ns()
        uwb_serial_node.py:324-328
        ├─> LivoxImuTimeMapper
        │     └─> /uwb/raw.header.stamp
        └─> DistanceRoundAssembler
              ├─> UWBDBG/TWR 仅作为协议边界或诊断
              ├─> 默认保留第一条正式 distance 的 RawSerialFrame 上下文
              └─> 完整 5 行且该上下文映射有效
                    └─> /uwb/ranges
```

映射失败时 `/uwb/raw` 仍发布，`header.stamp=0`、`timestamp_source=INVALID`；完整测距轮次不发布 `/uwb/ranges`，且不把时间失败计作解析错误。

### 3.3 GNSS

```text
完整串口行/文件记录
  └─> time.monotonic_ns()
        gnss_serial_node.py:304-309
        ├─> LivoxImuTimeMapper
        │     └─> /gnss/raw
        └─> GnssParser
              ├─> checksum_valid
              └─> position_valid
                    └─> /gnss/pvt_local
                          └─> gnss_adapter_node
                                原样复制 Header
                                └─> /gnss/enu_odom
                                      └─> FAST external_topic
```

GNSS 设备 UTC、GPS week/TOW 兼容字段和解算质量没有被时间映射替换。当前仓库的实际主融合输入是 `GnssPvtStamped` 和 ENU Odometry，没有 NavSatFix 发布链。

`/ublox_driver/receiver_pvt` 的兼容类型 `gnss_comm/GnssPVTSolnMsg` 本身没有 Header；该话题继续发布原兼容消息，无法在不改变第三方消息 MD5 的前提下携带 ROS 时间戳。映射后的融合时间由 `/gnss/pvt_local` 携带。

## 4. 新共享协议

默认文件：`$(env HOME)/timeshare_imu`  
旧相机文件：`$(env HOME)/timeshare`，保持完全独立。

小端格式：`<IIQQQQQIIQ`

| offset | size | type | field |
|---:|---:|---|---|
| 0 | 4 | uint32 | magic = `0x31414D49`，内存字节为 `IMA1` |
| 4 | 4 | uint32 | version = 1 |
| 8 | 8 | uint64 | writer_epoch |
| 16 | 8 | uint64 | write_sequence / seqlock |
| 24 | 8 | uint64 | imu_stamp_ns |
| 32 | 8 | uint64 | host_monotonic_ns |
| 40 | 8 | uint64 | update_monotonic_ns |
| 48 | 4 | uint32 | clock_source |
| 52 | 4 | uint32 | ready |
| 56 | 8 | uint64 | uncertainty_ns |

实现位置：

- C++ 协议与静态 offset/size 断言：`src/livox_ros_driver2/include/livox_ros_driver2/imu_time_anchor.h:25,131-149`
- C++ 原子 seqlock：`src/livox_ros_driver2/src/imu_time_anchor.cpp:26-108`
- Python `struct.calcsize()==64`、reader 和 mapper：`src/sensor_time_bridge/src/sensor_time_bridge/imu_anchor.py`
- 跨语言 golden bytes：C++ 和 Python 测试均覆盖。

writer 先将 sequence 写为奇数，写完所有字段后用 release store 提交偶数 sequence。Python reader读取两个完整 64 字节快照，要求 sequence 相同、为偶数且两份字节完全一致。

## 5. CLOCK_MONOTONIC 一致性

Anchor 模式严格使用：

- C++：`clock_gettime(CLOCK_MONOTONIC, ...)`
- Python：`time.monotonic_ns()`

新协议的 `host_monotonic_ns`、`update_monotonic_ns` 和 UWB/GNSS anchor 模式接收时刻处于同一个 Linux 内核时钟域。

没有使用 `update_monotonic_ns` 做映射；它只用于诊断。最终 `header.stamp` 是 MID-360 设备时间，不是 Jetson 日期时间。

保留的独立 `host_local` 兼容模式仍按原 Stage A/B1 接口使用 `CLOCK_MONOTONIC_RAW`，因为现有 `HostMonotonicToLocal` 服务定义在该域；`livox_imu_anchor` 分支不会进入该代码。`host_receive_stamp=rospy.Time.now()` 只保留原始诊断 wall-clock 字段，不参与最终 Header。

旧相机 timeshare 的 writer epoch 中原有 `CLOCK_REALTIME` 也未修改；它不属于新 IMU anchor 协议或映射公式。

## 6. Livox writer 行为

实现：

- 初始化与参数：`src/livox_ros_driver2/src/lddc.cpp:278-342`
- 创建/重开：`Lddc::InitializeImuAnchorWriter()`，第 344 行
- 每包观察：`Lddc::UpdateImuAnchor()`，第 398 行
- writer 协议实现：`src/livox_ros_driver2/src/imu_time_anchor.cpp`
- shutdown 失效：`Lddc::ShutdownImuAnchorWriter()`，第 379 行

行为：

- 绝对路径校验；
- `open(O_CREAT|O_RDWR, 0660)`；
- `ftruncate(..., 64)`；
- `mmap(MAP_SHARED)`；
- 每次进程启动生成新的非零 epoch；
- 初始写 `ready=0` 和空锚点；
- 默认连续 3 个有效样本后 `ready=1`；
- 设备时间为零、host monotonic 为零、设备时间重复/倒退、时间步长超过 0.1 s、时钟源变化或不支持的时钟源都会重新收敛；
- 当前只接受 Livox SDK 的 PTP/GPS 时间类型。`NoSync` 被拒绝，因为现有 `GetEthPacketTimestamp()` 在该类型下生成主机系统时间，不能当 MID-360 设备锚点；
- 进程退出时用 seqlock 写 `ready=0` 和空锚点；
- 打开失败会明确输出 ERROR 并限频重试；
- `[IMU_ANCHOR]` 初始化、ready 变化和 20 s 周期统计均限频。

当设备静默停止发包但 Livox 进程仍存活时，共享文件中的 `ready` 不会由独立 watchdog 立即翻转；读者按 `abs(sensor_monotonic-anchor_monotonic)>100 ms` 强制拒绝，因此不会继续产生融合时间。重连后若时间跳变、倒退或时钟源变化，writer 会重新经历 3 样本 ready 门控。真实设备断连回调语义仍需在 Jetson 实测确认。

## 7. Reader、生命周期与 stale 规则

公共模块位于 `sensor_time_bridge`，UWB 和 GNSS 均依赖这一份实现，没有重复的协议/转换代码。

reader 检查：

- 文件存在且长度为 64；
- magic/version；
- 稳定偶数 sequence；
- writer epoch 非零；
- ready；
- IMU/host 时间非零；
- PTP/GPS clock source；
- inode、fd 长度和路径长度。

文件不存在、尚未 ready、mmap 失败、文件被替换或长度改变时不会永久缓存失败；后续调用会自动重开。epoch 变化会返回一次 `epoch_changed`，清除 reader 的旧 epoch 认知，再等待新稳定快照。

mapper 规则：

- delta 使用 Python 有符号整数；
- 允许小范围负 delta；
- `abs(delta)>20 ms`：仍映射，但增加 warning 计数并限频报警；
- `abs(delta)>100 ms`：`stale_anchor`，不生成有效 stamp；
- 结果必须处于 ROS1 uint32 秒可表示范围且大于零；
- 每个 writer epoch 内的输出映射严格递增，否则丢弃；
- 不回退到 `now()`、simulation LOCAL 或 bag receive time。

reader 第一次发现新 writer epoch 时仍返回一次 `epoch_changed`。下一次读取稳定新 epoch 后，mapper 清空上一 epoch 的单调状态，因此允许新时间轴从较小值重新开始；同一 epoch 内的重复或倒退仍拒绝。不会在新旧 epoch 之间伪造 1 ns 连续性。

在 `livox_imu_anchor` 模式下，现有消息的 `session_id` 表示 Livox IMU anchor mapping session，并与 `writer_epoch` 取相同值；它不是 `sensor_time_bridge` 的 MCU/simulation LOCAL session。这样 UWB 与 GNSS 在读取同一 `timeshare_imu` writer 时具有相同会话标识，Livox writer 进程重启后两个字段同步变化。`header.frame_id=livox_imu_legacy_time` 用于标识时间域。

不能复制 `/sensor_time/status.session_id`：该字段属于 MCU/simulation LOCAL 时间会话，而 anchor 模式的 Header 属于 MID-360 legacy IMU 时间域；混用会把两个无映射关系的会话错误描述为同一时间域。

## 8. UWB 时间戳选择规则

实现位置：

- 模式配置：`src/uwb_serial_driver/scripts/uwb_serial_node.py:46-75`
- 时间选择：同文件 `_timestamp_for_line()`，第 165 行
- 完整行接收时刻：`_handle_line()`，第 324 行
- 融合轮次门控：`_publish_ranges()`，第 421 行

规则：

- standalone `uwb_serial.launch` 默认 `host_local`，保持兼容；
- `sensor_recording_bringup` 默认传入 `livox_imu_anchor`；
- `replay_mode=preserve` 且记录携带原时间戳时，原时间戳优先，不重新映射；
- anchor 模式中所有原始非空行继续发布；
- UWBDBG/TWR 不成为测距时间；
- 默认 `round_timestamp_policy=first_distance_line`，无论第一个槽位距离是否为零都保存其上下文；
- 可显式选择 `round_timestamp_policy=first_nonzero` 兼容旧行为；
- UWBDBG/TWR、后续非零行、第 5 条行或真正 publish 时刻都不会覆盖已选择上下文；
- 所有可发布的 RangeArray 时间元数据都复制自同一选择上下文；
- 时间映射失败只增加 `timestamp_mapping_drop_count`，不增加 `parse_error_count`；
- `UwbStatus.detail` 和 `[UWB_TIME]` 周期日志包含全部 anchor 计数。

## 9. GNSS 时间戳选择规则

实现位置：

- 模式配置：`src/gnss_serial_driver/scripts/gnss_serial_node.py:52-81`
- 时间选择：同文件 `_timestamp_for_frame()`，第 161 行
- 完整行接收：`_handle_line()`，第 304 行
- 校验及 PVT 门控：第 350、362 行
- PVT Header 构造：`_publish_pvt()`，第 400 行
- 适配器 Header 保留：`src/gnss_serial_driver/src/gnss_adapter_node.cpp:83`

规则：

- standalone `gnss_serial.launch` 默认 `host_local`；
- bringup 默认 `livox_imu_anchor`；
- 每个完整行先记录 host monotonic，再解析；
- 校验失败帧保留 raw，但不生成融合有效 PVT；
- anchor 失败时 raw/status 保留，PVT 兼容话题和 `/gnss/pvt_local` 不发布有效观测；
- 映射成功后 `/gnss/pvt_local` 携带接收映射时间；
- GNSS adapter 原样复制 Header 到 `/gnss/enu_odom`；
- FAST external_topic 已有 `valid_for_fusion` 和 stamp-zero 双门控，本次未改；
- GNSS UTC、位置、质量、RTK 状态和融合权重均未修改；
- `GnssPvtStamped` 使用已有最接近的 `HOST_RECEIVE_LOCAL` 常量表示“由主机接收时刻产生”，`status.detail` 明确记录 `timestamp_mode=livox_imu_anchor`，没有增加消息常量或改变 MD5。

## 10. simulation 模式职责

bringup 的职责现在明确分开：

```text
mcu_input_mode=simulation
  └─> sensor_time 会话 + 录包门控

uwb_timestamp_mode=livox_imu_anchor
gnss_timestamp_mode=livox_imu_anchor
  └─> UWB/GNSS header.stamp
```

启动时可观察：

```text
[SENSOR_TIME] mcu_input_mode=simulation recorder_gate_only=true
[UWB_TIME] mode=livox_imu_anchor path=...
[GNSS_TIME] mode=livox_imu_anchor path=...
```

已有 software simulation 测试仍显式使用兼容 `host_local`，Stage A/B1 行为保持通过。

## 11. launch 参数与默认值

Livox：

```text
imu_anchor_enable=true
imu_timeshare_path=$(env HOME)/timeshare_imu
imu_anchor_min_ready_samples=3
imu_anchor_uncertainty_ns=10000000
imu_anchor_max_stamp_step_s=0.1
```

UWB/GNSS reader：

```text
timestamp_mode=livox_imu_anchor        # bringup 默认
round_timestamp_policy=first_distance_line
imu_anchor_warn_age_s=0.020
imu_anchor_max_age_s=0.100
time_offset_s=0.0
```

`roslaunch --dump-params sensor_recording_bringup sensors_only.launch ...` 实测展开：

- Livox、UWB、GNSS 均为 `/home/gulu/timeshare_imu`；
- UWB/GNSS 均为 `timestamp_mode=livox_imu_anchor`；
- 两者 offset 均为 `0.0`；
- writer ready 样本数为 3、基础不确定度为 10 ms、最大 stamp step 为 0.1 s。

## 12. 依赖结构

新增依赖：

```text
sensor_time_bridge
  ↑                 ↑
uwb_serial_driver   gnss_serial_driver
```

`sensor_time_bridge` 不依赖 UWB/GNSS，UWB/GNSS 之间也没有依赖。`catkin_topological_order` 实测顺序为：

```text
sensor_time_msgs
sensor_time_bridge
gnss_serial_driver
uwb_serial_driver
sensor_recording_bringup
...
fast_livo
```

无循环依赖，FAST 不读取共享文件。

## 13. 修改文件清单

### Livox

- `src/livox_ros_driver2/CMakeLists.txt`
- `src/livox_ros_driver2/launch_ROS1/msg_MID360.launch`
- `src/livox_ros_driver2/src/comm/lidar_imu_data_queue.h`
- `src/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp`
- `src/livox_ros_driver2/src/comm/pub_handler.cpp`
- `src/livox_ros_driver2/src/lddc.h`
- `src/livox_ros_driver2/src/lddc.cpp`
- 新增 `src/livox_ros_driver2/include/livox_ros_driver2/imu_time_anchor.h`
- 新增 `src/livox_ros_driver2/src/imu_time_anchor.cpp`
- 新增 `src/livox_ros_driver2/test/imu_time_anchor_test.cpp`

### 公共 Python 模块

- `src/sensor_time_bridge/CMakeLists.txt`
- `src/sensor_time_bridge/package.xml`
- `src/sensor_time_bridge/src/bridge_node.cpp`
- 新增 `src/sensor_time_bridge/setup.py`
- 新增 `src/sensor_time_bridge/src/sensor_time_bridge/__init__.py`
- 新增 `src/sensor_time_bridge/src/sensor_time_bridge/imu_anchor.py`
- 新增 `src/sensor_time_bridge/test/test_livox_imu_anchor.py`

### UWB

- `src/uwb_serial_driver/CMakeLists.txt`
- `src/uwb_serial_driver/package.xml`
- `src/uwb_serial_driver/config/default.yaml`
- `src/uwb_serial_driver/launch/uwb_serial.launch`
- `src/uwb_serial_driver/scripts/uwb_serial_node.py`
- 新增 `src/uwb_serial_driver/test/test_uwb_imu_anchor.py`

### GNSS

- `src/gnss_serial_driver/CMakeLists.txt`
- `src/gnss_serial_driver/package.xml`
- `src/gnss_serial_driver/config/default.yaml`
- `src/gnss_serial_driver/launch/gnss_serial.launch`
- `src/gnss_serial_driver/scripts/gnss_serial_node.py`
- 新增 `src/gnss_serial_driver/test/test_gnss_imu_anchor.py`

### Bringup/集成测试

- `src/sensor_recording_bringup/CMakeLists.txt`
- `src/sensor_recording_bringup/package.xml`
- `src/sensor_recording_bringup/launch/sensors_only.launch`
- `src/sensor_recording_bringup/launch/record_all.launch`
- 新增 `src/sensor_recording_bringup/test/imu_anchor_mapping_integration.test`
- 新增 `src/sensor_recording_bringup/test/imu_anchor_mapping_integration_test.py`

没有修改或删除消息文件、相机文件、FAST-LIVO2 文件、STM32 文件或 bag。

## 14. 测试结果

### Release 全工作空间编译

执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/gulu/catkin_ws
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2
```

结果：通过。新增 `livox_ros_driver2_node`、anchor C++ 测试、Python 包及 UWB/GNSS 依赖均编译/配置成功。只出现既有 GTSAM deprecated 和主机 VTK 安装警告，没有构建失败。

### 全量 catkin 测试

执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
cd /home/gulu/catkin_ws
catkin_make run_tests -j2
catkin_test_results build/test_results --all
```

最终结果：

```text
Summary: 153 tests, 0 errors, 0 failures, 0 skipped
```

包含：

- C++ 64 字节布局、offset、golden bytes；
- C++ 跨进程 seqlock 无撕裂；
- writer ready、失效和 shutdown；
- Python bad magic/version/length、奇数/变化 sequence；
- Python 并发 reader/writer；
- 文件缺失后恢复、文件替换和长度变化；
- epoch change；
- 正/负 delta、20 ms warning、100 ms rejection；
- offset、ROS 时间范围和严格单调；
- UWB 默认首条正式 `distance` 行上下文及显式 `first_nonzero` 兼容策略；
- 单基站 1 号、单基站切换双基站、全零轮、不完整轮和上下文清理；
- GNSS 校验和设备 UTC 保留；
- Stage A/B1 原测试；
- software simulation；
- UWB distance_round5 原集成测试；
- 新的 UWB/GNSS 共享 2020 epoch、暂停 stale、writer epoch 重启自动恢复 ROS 集成测试。

### 相关 Python 测试

按四个直接相关测试文件执行。结果：

```text
test_parser.py:             22 passed
test_uwb_imu_anchor.py:      3 passed
test_livox_imu_anchor.py:   17 passed
test_gnss_imu_anchor.py:     3 passed
total:                      45 passed
```

### Launch 和静态检查

- 5 个修改 launch 和新 rostest XML 均通过 `xmllint --noout`；
- `roslaunch --dump-params` 成功；
- 修改 Python 文件通过 `py_compile`；
- `git diff --check` 通过；
- 集成测试临时文件 `/tmp/livox_imu_anchor_mapping_integration.bin` 已由测试清理；
- 没有写入或覆盖 bag。

## 15. ROS 消息 MD5

修改前后相同：

| message | MD5 |
|---|---|
| `sensor_time_msgs/RawSerialFrame` | `b3f0843fc91b42663f3ec3c977e3095b` |
| `uwb_serial_driver/UwbRange` | `43c4ece2df281f479060c8c1c4ea5dcb` |
| `uwb_serial_driver/UwbRangeArray` | `f70481b55f677d52d6e6c5da524486ec` |
| `uwb_serial_driver/UwbStatus` | `d9a50c3bfe71d71344174953cab378a7` |
| `gnss_comm/GnssPVTSolnMsg` | `d18171357d7a159f76d4d7c0b12fb631` |
| `gnss_serial_driver/GnssPvtStamped` | `bdfb1e39cf636dda7ac090d7feb65573` |
| `gnss_serial_driver/GnssStatus` | `89bd2929a7a8274c1f46b1f399c930ec` |

`git diff -- src/*/msg/*.msg` 为空。

## 16. 精度与风险边界

已解决的是 epoch 和连续软件时间轴关联，不是传感器测量时刻的硬件同步。

仍存在的误差来源：

- Livox `host_monotonic_ns` 是 SDK 回调入口时间，包含 MID-360 到 Jetson 的网络和 SDK 交付延迟；
- UWB/GNSS 是完整串口行形成时间，包含设备内部解算/测距和串口输出延迟；
- Linux 调度、USB 串口缓存和 Python 调度抖动；
- UWB/GNSS 当前固定 offset 为 0，尚未标定上述固定延迟；
- `uncertainty_ns=10 ms` 是保守的软件基础不确定度配置，不是实测硬件精度；
- 当前没有 STM32 事件串口，无法把真实 UWB/GNSS 测量事件直接锚定到 MID-360；
- 当前没有 GNSS PPS 输入或 UTC/PPS 映射，GNSS 设备 UTC 只作为数据字段保留。

因此 “Direct offline fusion readiness” 只表示消息处于可关联的软件时间域，不表示设备测量时刻已硬件同步。

## 17. Jetson 构建和启动命令

本轮没有在 Jetson 执行。部署后先重新编译：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jetson/catkin_ws
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2
source /home/jetson/catkin_ws/devel/setup.bash
```

建议为 GNSS/UWB 建立不同的稳定 udev 名称。以下命令明确使用两个不同设备；部署时替换为实际端口，禁止让两个驱动占用同一 tty：

```bash
source /opt/ros/noetic/setup.bash
source /home/jetson/catkin_ws/devel/setup.bash

roslaunch sensor_recording_bringup record_all.launch \
  enable_time_bridge:=true \
  enable_livox:=true \
  enable_camera:=true \
  enable_gnss:=true \
  enable_gnss_adapter:=true \
  enable_uwb:=true \
  mcu_input_mode:=simulation \
  gnss_port:=/dev/gnss \
  uwb_port:=/dev/uwb \
  imu_timeshare_path:="$HOME/timeshare_imu" \
  uwb_timestamp_mode:=livox_imu_anchor \
  gnss_timestamp_mode:=livox_imu_anchor \
  uwb_time_offset_s:=0.0 \
  gnss_time_offset_s:=0.0 \
  output_dir:=/home/jetson/bags/imu_anchor_test \
  bag_name:=livo_camera_uwb_gnss_imu_anchor \
  record_profile:=both
```

## 18. Jetson 验证步骤

共享文件：

```bash
ls -l "$HOME/timeshare"
ls -l "$HOME/timeshare_imu"
stat "$HOME/timeshare_imu"
```

必须确认：

- 两个文件路径不同；
- `timeshare_imu` 长度为 64；
- Livox 日志出现 GPS 或 PTP 设备时间类型；
- 日志出现 `[IMU_ANCHOR] ready=true`；
- 如果只出现 `NoSync`，不得继续把 anchor 输出当 MID-360 设备时间。

日志：

```text
[IMU_ANCHOR] ready=true ...
[UWB_TIME] mode=livox_imu_anchor ...
[GNSS_TIME] mode=livox_imu_anchor ...
[SENSOR_TIME] mcu_input_mode=simulation recorder_gate_only=true
```

频率：

```bash
rostopic hz /livox/imu
rostopic hz /livox/lidar
rostopic hz /left_camera/image
rostopic hz /uwb/ranges
rostopic hz /gnss/pvt_local
rostopic hz /gnss/enu_odom
```

时间戳：

```bash
rostopic echo -n 3 /livox/imu/header
rostopic echo -n 3 /livox/lidar/header
rostopic echo -n 3 /left_camera/image/header
rostopic echo -n 3 /uwb/ranges/header
rostopic echo -n 3 /gnss/pvt_local/header
rostopic echo -n 3 /gnss/enu_odom/header
```

检查所有可融合 Header：

- 非零；
- 严格递增；
- 位于同一 MID-360 legacy epoch；
- UWB/GNSS 不再落在 simulation 2000 epoch；
- LiDAR/IMU/Camera 原时间戳行为不变。

还应执行真实断连、Livox 节点重启、串口重连和长时间录包测试，并重新运行现有 bag 检查工具。

## 19. 后续硬件方案

本次没有实施：

- STM32 事件串口；
- UWB 真实测距事件序号/时间戳；
- GNSS PPS 输入；
- GNSS UTC/PPS 到 MID-360 的绝对时间映射；
- 固定延迟标定；
- PTP 网络同步；
- 对旧 bag 的修复。

若后续增加 STM32 事件串口和 GNSS PPS，应将“主机完整串口帧接收时刻”替换为可追踪的硬件测量事件，并保留本协议的 epoch、ready、seqlock 和 stale 防护思想。

## 20. 备份与回滚

备份目录：

```text
/home/gulu/catkin_ws/.codex_backups/livox_imu_anchor_mapping_20260727_124437
```

回滚脚本：

```text
/home/gulu/catkin_ws/.codex_backups/livox_imu_anchor_mapping_20260727_124437/restore.sh
```

脚本可执行且已通过 `bash -n`。本轮没有执行回滚。

执行回滚会恢复全部原文件，并删除本次新增源码、测试和本报告：

```bash
/home/gulu/catkin_ws/.codex_backups/livox_imu_anchor_mapping_20260727_124437/restore.sh
```

## 21. 最终边界确认

- PC 编译：已验证。
- PC 单元/ROS 集成测试：已验证。
- Jetson ARM64 编译：未验证。
- MID-360/UWB/GNSS/Camera 真实硬件：未验证。
- ROS 消息 MD5：已验证未变化。
- Camera `$HOME/timeshare`：未修改。
- LiDAR/IMU 原 Header：未修改。
- UWB distance_round5：仅修改轮次时间戳上下文策略和相关元数据/测试；ID、五槽、零值、重复基站及滤波语义未修改。
- FAST 状态估计、融合因子和权重：未修改。
- STM32：未修改。
- bag：未修改。
- Git commit/push：未执行。
