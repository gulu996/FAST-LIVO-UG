# 阶段 B1 实施报告

日期：2026-07-23  
工作空间：`/home/gulu/catkin_ws`

## 1. 执行结论

阶段 B1 的软件范围已经落地并通过全工作空间编译、Stage A 回归、Stage
B1 单元测试和无 FAST-LIVO2 的 ROS 联调测试：

- 新建 `sensor_time_msgs`、`sensor_time_bridge`、`gnss_serial_driver`、
  `uwb_serial_driver`、`sensor_recording_bringup` 五个包。
- LOCAL_SENSOR_TIME 固定使用
  `946684800000000000 + floor(local_tick * 1e9 / local_tick_hz)`，整数计算，
  不用 wall time 生成测量时间。
- host 端 MCU 协议采用 COBS、CRC32C、little-endian、固定头长度和有界队列。
- LOCAL↔UTC 使用 reference-pair 稳健仿射拟合，提供完整状态机、holdover、
  relock 连续性门禁、mapping version/history 和正反转换服务。
- GNSS/UWB 的串口所有权、解析、日志、回放和 ROS 发布已从 FAST 融合生命周期
  中独立出来。
- FAST-LIVO2 增加 `legacy_internal | external_topic | disabled` 输入边界；
  默认仍为 `legacy_internal`，融合公式未改。
- sensors-only/record-all bringup 不启动 FAST、后端或 RViz，录包按 MCU
  session 自动分卷，UTC/GNSS Fixed 不是录制门禁。
- STM32F103C8T6 的 medium-density 宏、startup 文件和资源确认文档已准备，
  但未切换 startup、未启用新中断、未编译 Keil、未烧录。

本轮没有修改相机或 Livox 的 Stage A 默认时间路径，没有修改
`img_time_offset: 0.1`，没有修改 ESIKF、视觉、GNSS/UWB 融合数学，没有覆盖
bag，也没有提交或推送 Git。

## 2. 实施范围与明确未实施项

已实施的是软件骨架、host 协议、LOCAL 主链、UTC 映射、独立采集驱动、
FAST 外部消费入口、录制框架和模拟测试。

以下项目故意保留到硬件 bring-up：

- 未指定 MCU 事件 UART、GNSS PPS GPIO、TIM 通道、USART2/3 引脚。
- 未把 `startup_stm32f10x_md.s` 切入 Keil 工程，原
  `startup_stm32f10x_hd.s` 引用仍保留。
- 未启用 TIM4、USART3 或 GNSS PPS 捕获中断。
- 未实现或烧录 MCU 端事件发送、合成 RMC/PPS 正式链。
- 未把 MID-360 切换到新的合成 RMC/PPS，也未声称 `time_type=2` 已实测。
- 未把相机默认 stamp 切到 MCU trigger ring；Stage A 相机路径保持不变。
- 未改变 Livox `LIDAR_BASE_TIME_LEGACY` 默认兼容模式。
- 未做真实 GNSS、UWB、相机、LiDAR 串口/GPIO/示波器测试。

因此，本报告证明的是 B1 软件链可编译、可模拟、可回放和可录制；不构成硬件
时间同步已经完成的声明。

## 3. 新包结构

```text
sensor_time_msgs
├── msg/{McuEvent,TimeMapping,TimeStatus,RawSerialFrame,UtcObservation}.msg
└── srv/{ConvertTime,HostMonotonicToLocal}.srv

sensor_time_bridge
├── include/sensor_time_bridge/{wire_protocol,local_time,time_mapping}.h
├── src/{wire_protocol,local_time,time_mapping,bridge_node,mcu_event_simulator}.cpp
└── test/sensor_time_core_test.cpp

gnss_serial_driver
├── msg/{GnssPvtStamped,GnssStatus}.msg
├── src/gnss_serial_driver/{parser,exclusive_serial,time_policy}.py
├── scripts/{gnss_serial_node,gnss_adapter_node}.py
├── launch/{gnss_serial,gnss_adapter}.launch
└── test/test_parser.py

uwb_serial_driver
├── msg/{UwbRange,UwbRangeArray,UwbStatus}.msg
├── src/uwb_serial_driver/{parser,exclusive_serial}.py
├── scripts/uwb_serial_node.py
├── launch/uwb_serial.launch
└── test/test_parser.py

sensor_recording_bringup
├── launch/{software_simulation,sensors_only,record_all}.launch
├── config/{topics_raw,topics_parsed}.yaml
├── scripts/{session_recorder,check_recording_ready}.py
├── src/sensor_recording_bringup/recorder_gate.py
├── data/{gnss_replay,uwb_replay}.txt
└── test/{test_recorder_gate,software_simulation_test}.py
```

## 4. 消息与服务语义

| 类型 | 关键语义 |
|---|---|
| `McuEvent` | `header.stamp == LOCAL_SENSOR_TIME`；带 session、boot、writer epoch、事件/源序号、tick、tick Hz、host monotonic 和 uncertainty |
| `TimeMapping` | reference pair、斜率 ppm、拟合跨度/残差、版本、状态和观测计数 |
| `TimeStatus` | LOCAL ready、UTC mapping 状态、gap/CRC/frame/duplicate/drop/queue 诊断 |
| `RawSerialFrame` | 同时保存 LOCAL header、host wall、host monotonic、时间来源和原始字节 |
| `UtcObservation` | 显式区分 hardware PPS、host serial association 和 preserved replay；host 串口关联不能锁定高精度 mapping |
| `GnssPvtStamped` | PVT、UTC 测量时间、逆映射 LOCAL 时间、mapping version、位置/时间/融合有效性分离 |
| `UwbRangeArray` | 一行一 round；共享 header/round sequence；标记 HOST_RECEIVE_LOCAL 或 replay 来源和非零 uncertainty |
| `ConvertTime` | LOCAL→UTC 和 UTC→LOCAL；mapping 不可用时 fail closed |
| `HostMonotonicToLocal` | `CLOCK_MONOTONIC_RAW`→LOCAL；明确只是主机接收时刻 |

`sensor_time_msgs` 只含接口，不含算法。

## 5. 依赖关系和实际数据流

```text
sensor_time_msgs
  ├── sensor_time_bridge
  ├── gnss_serial_driver ──> gnss_comm
  ├── uwb_serial_driver
  └── sensor_recording_bringup

gnss_serial_driver/GnssPvtStamped ──> FAST GnssManager external_topic
uwb_serial_driver/UwbRangeArray  ──> FAST UwbManager external_topic

sensor_time_bridge + GNSS/UWB + Livox + Camera
  └── sensor_recording_bringup ──> session-aware rosbag
```

软件时间流：

```text
MCU wire/simulator/file
  → incremental COBS/CRC32C decoder
  → boot/session/sequence/monotonic gates
  → integer tickToLocalNs()
  → /sensor_time/events
  → host CLOCK_MONOTONIC_RAW↔LOCAL fit

MCU GNSS_PPS_CAPTURE + hardware-associated UTC observation
  → robust LOCAL↔UTC candidate
  → LOCAL_ONLY/ACQUIRING/LOCKED/HOLDOVER/RELOCKING
  → /sensor_time/mapping + conversion service

GNSS UTC measurement
  → utcToLocal()
  → /gnss/pvt_local
  → GNSS adapter or FAST external consumer

UWB read() host monotonic
  → hostMonotonicToLocal()
  → /uwb/ranges (HOST_RECEIVE_LOCAL + uncertainty)
  → FAST external consumer
```

LOCAL 不随 GNSS 出现、消失或重锁而 step。重锁 candidate 若会相对旧
holdover mapping 产生超过 `max_relock_step_ns` 的 UTC 跳变会被拒绝。

## 6. LOCAL_SENSOR_TIME 和 wire protocol

核心实现：

- `/home/gulu/catkin_ws/src/sensor_time_bridge/src/local_time.cpp`
- `/home/gulu/catkin_ws/src/sensor_time_bridge/src/wire_protocol.cpp`
- `/home/gulu/catkin_ws/src/sensor_time_bridge/src/bridge_node.cpp`
- `/home/gulu/catkin_ws/src/sensor_time_bridge/src/time_mapping.cpp`

已实现：

- 固定 magic/version/type/header length，wire payload length 不被动态信任；
- COBS delimiter 重同步、CRC32C、little-endian；
- 半帧、坏帧、过长帧、sequence gap/duplicate/wrap；
- BOOT/session 切换、旧 boot 拒绝、writer epoch；
- 串口 `flock`、断线重连、decoder partial reset；
- queue capacity 受限，溢出丢最旧事件并累计 drop；
- `__int128` 安全整数 tick→ns 和溢出拒绝；
- STM32 16 位 CNT/high/UIF 稳定快照的 host 参考函数；
- host monotonic 稳健拟合及接收时间 uncertainty；
- rolling/Huber/inlier refit、PPS interval/单调/slope/residual 门禁；
- mapping history 有界保存，relock 原子替换；
- HOLDOVER uncertainty 随时间增长。

模拟器在 subscriber 建立后发送 latched BOOT，并保留 250 ms 建链窗口，避免
测试启动竞态。它支持指定 tick Hz、ppm、Camera/PPS 频率和 GNSS
可用/中断/恢复时间。

## 7. GNSS 迁移

| 原职责 | B1 位置 |
|---|---|
| 串口打开、波特率、DTR/RTS、独占锁、重连 | `gnss_serial_driver/exclusive_serial.py`、`gnss_serial_node.py` |
| raw line、raw/parsed log、file replay | `gnss_serial_node.py` |
| NMEA checksum、KSXT/GGA/RMC/GSA/GST/ZDA | `gnss_serial_driver/parser.py` |
| AGRICA、legacy JSON、日期/历元聚合 | `gnss_serial_driver/parser.py` |
| 位置质量分类 | `parser.py` |
| 时间质量、UTC→LOCAL、topic 单调门禁 | `time_policy.py`、`gnss_serial_node.py` |
| `/ublox_driver/receiver_pvt` 兼容发布 | `gnss_serial_node.py` |
| `/gnss/pvt_local`、`/gnss/status_v2` | 新消息和 `gnss_serial_node.py` |
| `/gnss/enu_odom` LOCAL 适配 | `gnss_adapter_node.py` |

时间行为：

- mapping 未锁时 raw、兼容 PVT 和 UTC observation 仍可发布；
- `/gnss/pvt_local.local_measurement_time_valid=false` 且
  `valid_for_fusion=false`；
- 串口 RMC/ZDA 默认标记 `HOST_SERIAL_ASSOCIATED`，bridge 不把它当硬件 PPS；
- mapping 可用后才把 GNSS UTC 逆映射为 LOCAL；
- RTK Fixed 与时间 mapping 有效性是两个独立条件；
- 转换结果按 topic 严格单调，倒序/重复测量时间 fail closed。

file replay 可循环用于仿真；`replay_has_pps_association` 必须由可信 corpus
显式打开，默认 false。

## 8. UWB 迁移

| 原职责 | B1 位置 |
|---|---|
| 串口、独占锁、重连 | `uwb_serial_driver/exclusive_serial.py`、`uwb_serial_node.py` |
| distance/debug/pairs/values parser | `uwb_serial_driver/parser.py` |
| scale、per-anchor bias、量程检查 | `parser.py` |
| repeated-range filter | `parser.py:RangeFilter` |
| raw/parsed 日志、file replay | `uwb_serial_node.py` |
| preserve/rebase replay | `uwb_serial_node.py` |
| ROS round/status 发布 | 新消息和 `uwb_serial_node.py` |

一条 pairs/values 输入行中的 anchors 是同一 round；独立输入行永不按时间窗口
猜测合并。真实串口没有设备时间时，header 是 read 返回时刻对应的
HOST_RECEIVE_LOCAL，且 uncertainty 非零。anchor 世界坐标、初始化、状态与
协方差、ESIKF range update、残差和更新日志仍留在 FAST-LIVO2。

## 9. FAST-LIVO2 输入模式

新增全局 ROS 参数：

```yaml
gnss:
  input_mode: legacy_internal   # 或 external_topic / disabled
  external_topic: /gnss/pvt_local

uwb:
  input_mode: legacy_internal   # 或 external_topic / disabled
  external_topic: /uwb/ranges
```

- `legacy_internal`：默认兼容路径；串口增加 advisory `flock`。
- `external_topic`：只订阅外部消息，不打开串口、不启动 read thread；UWB
  也不加载旧 replay file。消息通过原 `GnssMeasurement`/`UwbMeasurement`
  队列进入现有融合入口。
- `disabled`：无串口、无 replay、无 subscriber、无相关线程。
- 外部 GNSS 必须同时有有效 LOCAL 时间和 `valid_for_fusion`。
- 外部 UWB 必须有非零 LOCAL header 和非零 uncertainty。
- legacy 串口锁冲突会明确报错，线程不会静默竞争。

受影响文件只有：

```text
FAST_LIVO2/CMakeLists.txt
FAST_LIVO2/package.xml
FAST_LIVO2/include/{gnss_manager,uwb_manager}.h
FAST_LIVO2/src/{gnss_manager,uwb_manager}.cpp
FAST_LIVO2/src/{gnss_manager_self_test,uwb_manager_self_test}.cpp
```

没有改 FAST 的状态更新、协方差、GNSS 残差、UWB 残差、地图或视觉公式。
工作树中原先存在的 FAST 配置/launch 修改均保留且不是本轮改动。

## 10. sensors-only 和录包

`sensors_only.launch` 的默认节点清单经 `roslaunch --nodes` 验证为：

```text
/sensor_time_bridge
/livox_lidar_publisher2
/mvs_camera_trigger
/gnss_serial_driver
/gnss_adapter
/uwb_serial_driver
```

其中不存在 `/laserMapping`、FAST、RTK/backend 或 RViz。`record_all.launch`
只在此基础上增加 `/session_recorder`，并强制 `/use_sim_time=false`。

录制门禁只检查：

- `local_ready=true`；
- 非零 `session_id`；
- 用户启用的 required topics 已发布。

它不等待 UTC mapping、GNSS UTC valid 或 RTK Fixed。MCU session 变化时先向
当前 rosbag 进程发 SIGINT，再用包含新 session ID 的文件名启动新 bag。
退出路径有显式 `shutting_down` 门禁，避免关闭 bag 后被并发回调重新启动。

硬件录制命令示例：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
roslaunch sensor_recording_bringup record_all.launch \
  mcu_input_mode:=serial mcu_port:=/dev/sensor_mcu \
  gnss_source:=serial gnss_port:=/dev/gnss \
  uwb_source:=serial uwb_port:=/dev/uwb \
  output_dir:=/data/bags bag_prefix:=stageB1 record_profile:=both
```

纯软件、无 FAST、实际写 bag 的命令：

```bash
roslaunch sensor_recording_bringup software_simulation.launch \
  start_without_gnss_s:=999 start_recorder:=true recorder_dry_run:=false
```

原始 bag 不会被覆盖；输出文件名包含 session ID 和启动时间。

## 11. STM32F103C8T6 准备结果

工程检查结果：

- Keil device 是 `STM32F103C8`；
- 编译宏已改为 `USE_STDPERIPH_DRIVER,STM32F10X_MD`；
- 工程仍引用 `startup_stm32f10x_hd.s`；
- 新增 `CORE/startup_stm32f10x_md.s`，补齐 medium-density 的
  TIM4/I2C2/SPI2/USART3 等向量；
- 新增 `docs/stm32f103c8t6_resource_requirements.md`。

资源文档记录：

- PA1：当前相机触发；
- PB5：当前本地 PPS；
- PA9：当前 USART1 合成 RMC；
- MCU 事件串口：待定；
- GNSS PPS 捕获 GPIO/TIM：待定；
- GPIO 电平、极性、共地和电缆终点：必须实物确认。

因为当前 Linux 环境没有 Keil/armcc，本轮没有删除旧 startup、没有切换工程
引用、没有声称 clean build 或硬件有效。

## 12. 构建和测试结果

### 12.1 全量构建

执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
cd /home/gulu/catkin_ws
catkin_make -j2
```

结果：全工作空间 100% 构建成功，包括五个新包、Livox、相机、FAST-LIVO2
及 GNSS/UWB self-test binaries。`__int128` 产生 GCC pedantic warning，但这是
本设计要求的溢出安全整数路径；无编译错误。环境仍显示既有 VTK imported
target 缺失警告，但未影响目标生成。

### 12.2 Stage A 回归

```bash
catkin_make -j2 run_tests_livox_ros_driver2 run_tests_mvs_ros_driver
catkin_test_results build/test_results/livox_ros_driver2 --all
catkin_test_results build/test_results/mvs_ros_driver --all
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_analyze_sensor_timestamps.py
```

结果：

- Livox XML 汇总：12 tests，0 error，0 failure；
- MVS XML 汇总：6 tests，0 error，0 failure；
- Stage A 合计：18 tests，0 error，0 failure；
- 离线时间戳分析器：5 tests passed。

### 12.3 Stage B1 新增测试

命令：

```bash
catkin_make -j2 \
  run_tests_sensor_time_bridge \
  run_tests_gnss_serial_driver \
  run_tests_uwb_serial_driver \
  run_tests_sensor_recording_bringup
```

实际 testcase：

| 组 | 数量 | 结果 |
|---|---:|---|
| sensor_time core/wire/mapping | 12 | 全通过 |
| GNSS parser/time policy | 7 | 全通过 |
| UWB parser/filter | 5 | 全通过 |
| recorder gate | 2 | 全通过 |
| 无 FAST 软件 rostest | 1 | 通过 |
| 合计 | 27 | 0 failure |

测试覆盖 COBS golden、CRC32C、半帧、随机丢字节、坏 magic/version/type、
bounded frame/queue、duplicate/gap/wrap/boot、16 位 CNT/UIF 竞争、1 MHz
integer ns、大数溢出、session 单调、稳健 outlier、UTC↔LOCAL round trip、
1200 s 全状态序列、holdover uncertainty、relock 大 step 拒绝、mapping
history、host monotonic、GNSS 全质量级别和时间/位置质量分离、UWB
round/bias/range/repeat/replay。

`catkin_test_results build/test_results --all` 的 ROS XML 汇总为 58 tests，
0 error，0 failure；其中 gtest/rostest aggregate suite 会重复计数，故上表给出
新增源 testcase 的实际数量。

FAST 边界 self-test：

```bash
devel/lib/fast_livo/gnss_manager_self_test
devel/lib/fast_livo/uwb_manager_self_test
```

两者退出码均为 0；外部消息进入原队列、无效外部时间拒绝等断言通过。

消息兼容性：

```text
livox_ros_driver2/CustomMsg  e4d6829bdfe657cb6c21a746c86b21a6
gnss_comm/GnssPVTSolnMsg    d18171357d7a159f76d4d7c0b12fb631
```

与基线一致。

## 13. 软件模拟和 GNSS 始终无效测试

rostest 使用：

```text
start_without_gnss_s=999
start_recorder=true
recorder_dry_run=true
```

实测：

- bridge 收到 BOOT，LOCAL session 非零且 `local_ready=true`；
- `TimeStatus.time_state == LOCAL_ONLY`，UTC mapping 未锁；
- GNSS raw 持续发布；
- UWB ranges 持续发布，来源为 `REPLAY_REBASED`，uncertainty 非零；
- recorder 在 required topics
  `/sensor_time/events`、`/gnss/raw`、`/uwb/raw` 就绪后进入 RECORDING；
- `/sensor_time_bridge`、GNSS driver、UWB driver 存在；
- `/laserMapping` 不存在，无名称包含 backend 的节点；
- 无崩溃或阻塞。

这证明 GNSS UTC 永远无效不会阻止 LOCAL、GNSS raw、UWB 或 recorder。该自动
测试没有伪造 LiDAR/Camera 硬件，因此二者只完成 launch 静态检查，尚未做
运行时出数/录包实测。

## 14. 尚需确认的信息和风险

1. MCU 板级原理图、可用 UART 和输入捕获 GPIO。
2. PA1/PB5/PA9 的实物终点、电平、极性和是否反相。
3. HSE 晶体型号、负载与实际 ppm；理论配置不是实测频率。
4. GNSS 接收机型号、PPS 电平、TIM-TP/时间脉冲语义、UTC 与 PPS 对应序号。
5. UWB 固件文本格式、tag/anchor ID、量程单位、是否可增加 round ID/device time。
6. 相机 `nTriggerIndex` 在目标型号/触发模式下是否有效并单调。
7. MID-360 固件对 RMC/PPS、丢源、恢复、日期 rollover 和 `time_type` 的行为。
8. Advisory `flock` 只保护配合该协议的进程；不配合的进程仍可能绕过。
9. UWB 当前是主机接收时刻，不是射频测量时刻。
10. MCU reset 会使 LOCAL 数值回到合成 epoch；bag 已按 session 分卷，但所有
    下游消费者仍需在硬件集成阶段验证 session reset。
11. Camera/Livox 尚未切到 B1 LOCAL，当前不能宣称五类传感器已处于同一硬件
    时间轴。

## 15. 下一阶段硬件 bring-up

1. 用户确认 MCU UART、GNSS PPS timer/channel/GPIO、电平与连接器。
2. 在 Keil 中切换到 `startup_stm32f10x_md.s`，clean build，核对 map/vector；
   失败时保留旧文件便于对照，不烧录未验证镜像。
3. 实现 MCU 端 64 位 tick、事件 ring、BOOT/status、COBS/CRC32C UART；
   对照 host golden frame。
4. 示波器同时测 Camera trigger、LOCAL PPS、RMC TX、GNSS PPS，验证相位、
   极性、延迟和 ppm。
5. 接入 GNSS PPS + 明确 UTC epoch/sequence，标记为
   `HARDWARE_PPS_ASSOCIATED` 后验证 LOCK/HOLDOVER/RELOCK。
6. MID-360 接入 MCU 合成 LOCAL RMC/PPS，强制并诊断 `time_type=2`，
   验证 LiDAR/IMU header 与 `CustomMsg.timebase`。
7. 验证相机 `nTriggerIndex`，实现带 trigger sequence 的 ring 匹配；未锁定、
   gap、ring miss 时 fail closed。
8. 采集新 bag，验证五传感器单调性、session、mapping version、相对速率和
   nearest-neighbor 漂移；原 bag 保持只读。
9. 硬件链稳定后才评估 `img_time_offset` 是否归零及关闭 legacy 时间模式。

## 16. 修改文件范围

新增：

```text
src/sensor_time_msgs/**
src/sensor_time_bridge/**
src/gnss_serial_driver/**
src/uwb_serial_driver/**
src/sensor_recording_bringup/**
src/stm32_timersync-open/CORE/startup_stm32f10x_md.s
src/stm32_timersync-open/docs/stm32f103c8t6_resource_requirements.md
stageB1_implementation_report.md
```

修改：

```text
src/FAST_LIVO2/CMakeLists.txt
src/FAST_LIVO2/package.xml
src/FAST_LIVO2/include/gnss_manager.h
src/FAST_LIVO2/include/uwb_manager.h
src/FAST_LIVO2/src/gnss_manager.cpp
src/FAST_LIVO2/src/uwb_manager.cpp
src/FAST_LIVO2/src/gnss_manager_self_test.cpp
src/FAST_LIVO2/src/uwb_manager_self_test.cpp
src/stm32_timersync-open/USER/PWM.uvprojx
```

没有修改第三方消息定义、相机/Livox Stage A 源码或用户已有 FAST 配置。

## 17. 回滚

持久化备份：

```text
/home/gulu/catkin_ws/.codex_backups/stageB1_20260723_185657/
```

回滚脚本：

```text
/home/gulu/catkin_ws/.codex_backups/stageB1_20260723_185657/restore.sh
```

脚本已通过 `bash -n`。它恢复 9 个本轮修改前的已有文件，删除五个新增包、
新增 STM32 文档/startup 和本报告；不会触碰本轮开始前已存在的其他用户修改。
本轮未执行回滚脚本。
