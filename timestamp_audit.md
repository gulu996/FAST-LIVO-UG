# FAST-LIVO2 工作空间传感器时间戳端到端审计

- 审计日期：2026-07-23
- 审计范围：`/home/gulu/catkin_ws`，以 `src/` 下真实源码为准
- 旧 bag：`/home/gulu/Downloads/0709.bag`
- 方法：只读源码搜索、ROS 包/消息解析、MVS SDK 头文件检查、bag 全量逐消息扫描
- 排除项：`build/`、`devel/`、日志目录中的重复源码；编译产物仅用于确认实际链接和消息可解析性
- 本轮限制：除本报告外，没有修改源码、配置、launch、bag 或生成文件

> 结论强度标记：
>
> - **已证实（源码）**：可由当前源码和行号直接证明。
> - **已证实（bag）**：可由旧 bag 全量逐消息统计直接证明。
> - **高概率**：源码机制与实测数值吻合，但缺少历史二进制构建身份、设备运行状态或物理接线证据。
> - **未确认**：当前工作空间和 bag 不包含足够证据。

## 1. 执行摘要

### 1.1 最重要结论

1. **“相机 header 比 LiDAR 快约 3362 ppm”不是持续的相机时钟频率差。**  
   旧 bag 的 4465 帧相机消息中，只有第 0→1 个 header 间隔异常，为
   `1.399789095 s`；去掉这个启动异常后，相机 header 平均周期为
   `100.00003279 ms`，不是 `100.291 ms`。全段端点比值产生的
   `2990.85 ppm`，主要是把这次固定的约 `1.3 s` 跳变摊到了 446 s
   时长上。该单次异常解释了报告中 `1.335012172 s` 累计超前量的约
   **97.36%**。

2. **相机正常段根本没有独立于 LiDAR 的 header 时间轴。**  
   4465 个相机 header 中有 **4462 个**与 LiDAR `CustomMsg.header.stamp`
   **逐纳秒完全相等**；它们是相机索引 3…4464 对应 LiDAR 索引
   0…4461，连续、无跳号。两者对应区间向量的最大差为 `0 ns`。
   因此旧 bag 中相机 header 实际上是在读取 Livox 驱动写入的“最新
   LiDAR 点云 base time”，不是 MCU 自己产生的相机触发时间。

3. **`/home/gulu/timeshare` 的真实写入者不是 MCU 接收程序，而是
   `livox_ros_driver2`。**  
   `lddc.cpp` 在每次发布点云时把 `pkg.base_time` 写入
   `pointt->low`；相机取帧后读取这个单槽。`high` 在整个工作空间内
   没有写入，当前协议中没有有效含义。工作空间内没有发现把 MCU
   定时器值经 UART/GPIO 接收后写入该文件的 Jetson 程序。

4. **当前源码中存在一个能产生启动跳变的具体机制。**  
   Livox 点云轮询线程启动后先睡眠 3 s，而点云队列只能容纳 16 个
   10 Hz 包；随后线程会连续清空积压并快速覆写单槽。旧 bag 的相机
   第一次异常恰好跳过约 14 个 100 ms 时刻。源码机制、队列容量和
   `1.399789095 s` 数值高度吻合。由于没有旧 bag 对应二进制的构建
   ID，这一历史因果关系标为高概率；启动跳变本身和“端点 ppm 算法
   失真”则由 bag 直接证实。

5. **MCU 的整数定时器配置理论上是正确的。**  
   STM32F103：HSE 8 MHz、PLL ×9、SYSCLK 72 MHz、APB1 36 MHz；
   因 APB1 分频不为 1，TIM2/TIM3 输入仍为 72 MHz。`PSC=7199`，
   TIM2 `ARR=999` 得 10 Hz，TIM3 `ARR=9999` 得 1 Hz。源码没有漏掉
   STM32 APB 定时器 ×2，也没有把 10 Hz 错配成 9.97 Hz。

6. **正常晶振误差不足以解释 2990 ppm。**  
   若 `100.291 ms` 真由该定时器持续产生，则等效 HSE 约为
   `7.9768 MHz`，相对 8 MHz 低约 2900 ppm（约 23.2 kHz）。
   这是百分之 0.29 量级，不是正常工作的石英晶振常见“几十 ppm”
   量级。当前源码未包含晶振 BOM/容差，最终硬件频率仍应使用频率计
   或示波器实测；但旧 bag 已经不需要这种频率误差来解释。

7. **单槽共享协议不能证明图像—触发一一对应。**  
   它没有触发序号、相机帧号、版本、有效位、时钟域标识、原子协议、
   锁、seqlock 或新鲜度检查。即使旧 bag 正常段每帧恰好消费了连续
   LiDAR 时间，也仍不能判断它对应物理触发 N、N−1 还是 N+1。

8. **Livox 在这份 bag 中很可能使用 MCU 的 1 Hz + GPRMC 进行设备
   时间同步，而不只是采样相位同步。**  
   MCU 固定发送日期 `230520`；bag 中 LiDAR、IMU、相机转抄后的
   header 都落在 2020-05-23。驱动只有在 packet `time_type` 为 GPS
   或 PTP 时才原样使用设备 8 字节时间戳。bag 没有保存 `time_type`，
   物理接线也不在仓库内，因此 GPS/RMC 模式为高概率、不是绝对证明。

9. **LiDAR 与 IMU 使用同一套 packet timestamp 解析路径。**  
   约 116～127 ppm 是各自 header 相对 bag 记录时间的端点/回归差，
   包含主机接收、发布、bag 写入和不同采样语义；它不是二者拥有两个
   独立设备时钟的证据。

### 1.2 审计边界

- 当前没有连接硬件，无法实测 PA1/PB5 周期、相机 USB 返回延迟和 Livox
  的实时 `time_type`。
- 当前 `/home/gulu/timeshare` 不存在，无法检查运行时映射内容。
- 仓库没有物理接线图，也没有 Livox 设备持久配置导出。
- 旧 bag 的 connection header 可确认发布节点名，但没有历史可执行文件
  build ID；所以只对“当前源码能否产生该现象”作代码级判断。

## 2. 源码和节点清单

### 2.1 搜索方法与 ROS 包

在 `/home/gulu/catkin_ws` 下对题目所列关键词及大小写变体执行了递归
搜索，文件类型覆盖：

```text
*.cpp *.cc *.c *.h *.hpp *.py *.ino *.yaml *.launch
*.service *.sh CMakeLists.txt package.xml
```

搜索排除了 `build/`、`devel/`、`.git/` 和日志生成目录。`src/` 中发现
的 ROS 包包括：

```text
FAST_LIVO2
gnss_comm
livox_ros_driver2
mvs_ros_driver
rpg_vikit/*
```

MCU 工程 `src/stm32_timersync-open` 是 Keil 工程，不是 ROS 包。
正确 source：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
```

在该环境中，`rospack` 可定位上述包，且
`rosmsg show livox_ros_driver2/CustomMsg` 成功；本次 bag 分析不缺
自定义消息定义。

### 2.2 角色清单

| 角色 | 实际文件/节点 | 证据 |
|---|---|---|
| MCU 主时钟配置 | `src/stm32_timersync-open/USER/system_stm32f10x.c` | HSE/SYSCLK/APB 配置见 115、161–162、987–1056 行 |
| 相机触发生成者 | `src/stm32_timersync-open/USER/main.c` + `HARDWARE/TIMER/timer.c` | `TIM2_PWM_Init(999,7199)`：`main.c:22`；TIM2 CH2 PA1：`timer.c:157–206` |
| Livox 同步信号生成者 | 同上 | TIM3 CH2 PB5 1 Hz：`main.c:24`、`timer.c:216–260`；GPRMC UART：`timer.c:93–143` |
| MCU 软件时间生产者 | `HARDWARE/TIMER/timer.c` | hh:mm:ss 每 1 Hz 自增、固定日期 `230520`：122–143 行 |
| Livox 设备时间生产者 | Livox 设备 packet | 驱动读取 `timestamp[8]` 与 `time_type`：`src/livox_ros_driver2/src/comm/pub_handler.cpp:98–151,265–275` |
| 共享文件创建/写入者 | `/livox_lidar_publisher2`，`src/livox_ros_driver2/src/lddc.cpp` | 创建/mmap：187–218 行；写 `low`：281、310 行 |
| 共享文件读取者 | `/mvs_camera_trigger`，`src/mvs_ros_driver/src/grab_trigger.cpp` | 结构：19–23 行；映射：318–325 行；读取：233–244 行 |
| 备用、非编译读取者 | `src/mvs_ros_driver/beifen/grab_trigger.cpp` | 备份源码；当前 CMake 未引用 |
| 相机图像发布者 | `/mvs_camera_trigger` | `grab_trigger.cpp:204–278,316,471` |
| LiDAR CustomMsg 发布者 | `/livox_lidar_publisher2` | `lddc.cpp:289–317,455–476,722–743` |
| IMU 发布者 | `/livox_lidar_publisher2` | `lddc.cpp:603–620,758–795` |
| Livox packet 时间解析 | `src/livox_ros_driver2/src/comm/pub_handler.cpp` | `GetEthPacketTimestamp`：265–275 行 |
| FAST-LIVO2 消费节点 | `/laserMapping` | `src/FAST_LIVO2/src/LIVMapper.cpp:634–638,1926–1992,2075` |
| 硬件组合 launch | `src/FAST_LIVO2/launch/sensors.launch` | Livox 与相机同时启动：31–48 行 |
| 相机参数 | `src/mvs_ros_driver/config/left_camera_trigger.yaml` | Trigger、曝光、格式、10 Hz：8–31 行 |
| Livox 参数 | `src/FAST_LIVO2/launch/sensors.launch`、`src/livox_ros_driver2/config/MID360_config.json` | `xfer_format=1`、`publish_freq=10`；设备网络参数，无 PTP 配置 |
| MCU 工程/芯片声明 | `src/stm32_timersync-open/USER/PWM.uvprojx` | STM32F103C8：17 行 |
| 串口实现 | `src/stm32_timersync-open/SYSTEM/usart/usart.c` | 阻塞 `fputc`：42–47 行 |
| 将 MCU 时间写入 Jetson 的程序 | **未发现** | 全工作空间关键词与调用链搜索均无此生产者 |
| 自启动/systemd | **未发现相关单元** | 工作空间、`/etc/systemd/system`、`/lib/systemd/system`、用户 autostart 只读搜索无匹配 |

### 2.3 相机真实源码路径

用户描述的逻辑位于：

```text
/home/gulu/catkin_ws/src/mvs_ros_driver/src/grab_trigger.cpp
```

不是根据文件名推断：`src/mvs_ros_driver/CMakeLists.txt:42` 明确用该文件
构建 `grabImgWithTrigger`；launch
`src/mvs_ros_driver/launch/mvs_camera_trigger.launch:2–9` 启动该可执行
文件。`devel/lib/mvs_ros_driver/grabImgWithTrigger` 链接系统 MVS SDK，
仅作为辅助证据。

### 2.4 与未来 GNSS 接入有关的文件

`src/gnss_comm` 提供 GNSS 消息及转换工具；FAST-LIVO2 中也有 GNSS
配置/管理代码。这些文件当前不在相机—Livox header 生产链上。接入时
应特别审查：

```text
src/gnss_comm/msg/GnssTimeMsg.msg
src/gnss_comm/msg/GnssTimePulseInfoMsg.msg
src/FAST_LIVO2/config/*
src/FAST_LIVO2/src/*
```

不能因为包已存在就假定当前系统已经有 UTC/PPS 映射。

## 3. 实际时间链路图

### 3.1 相机链路：当前源码真正执行的路径

```text
STM32 HSE 8 MHz
  → PLL/APB/TIM2 72 MHz
  → TIM2_CH2/PA1 10 Hz 电平
  → 海康相机 LINE0 外触发并产生图像
  → USB/GigE SDK 队列
  → MV_CC_GetOneFrameTimeout()
  → 读取 /home/<getlogin()>/timeshare 的 pointt->low
  → double 秒 b/1e9
  → ros::Time(double)
  → sensor_msgs/Image.header.stamp
```

注意：图中的 `timeshare` 值并不来自前半段 STM32 链路，而来自下面的
Livox 点云发布链。代码把两条独立链在相机取帧时“取最新值”拼接起来。

| 箭头 | 输入 → 输出 | 单位/时钟来源 | 频率 | 文件与行 | 缓存/延迟 |
|---|---|---|---|---|---|
| HSE→系统时钟 | 8 MHz HSE → 72 MHz SYSCLK | 晶振/PLL | 连续 | `system_stm32f10x.c:115,987–1056` | PLL 相位延迟，不影响整数比 |
| 系统时钟→TIM2 | APB1 36 MHz → TIM2 72 MHz | 同一 MCU 晶振 | 连续 | `system_stm32f10x.c:1021–1028`；`timer.c:13` | APB1 分频时定时器 ×2 |
| TIM2→PA1 | PSC/ARR → PWM | MCU tick | 10 Hz | `main.c:22`；`timer.c:157–206` | 硬件输出，无软件队列 |
| PA1→相机 | LINE0 边沿 → 曝光/帧 | 相机硬件触发域 | 10 Hz 配置 | `grab_trigger.cpp:456–468`；相机 YAML 13、16、31 行 | 物理线/相机内部延迟未测 |
| 相机→SDK | 图像 → SDK 帧缓冲 | 相机设备/USB | 约 10 Hz | `grab_trigger.cpp:228–250` | SDK 默认 OneByOne，多缓冲，可能排队 |
| SDK→程序 | `stImageInfo`/像素 → 一帧 | 主机调用时间 | 每次循环 | `grab_trigger.cpp:230` | 最长等待 1000 ms |
| 共享槽→局部值 | `pointt->low` → `b` | **Livox base time，ns** | 点云发布约 10 Hz | `grab_trigger.cpp:233–244` | 单槽“最新值”，无序号；可覆盖 |
| ns→ROS 时间 | `b/1e9` → `ros::Time(double)` | Livox/GPS/PTP 或主机墙钟 | 每帧 | `grab_trigger.cpp:237–239` | double 舍入，约百 ns |
| 图像处理 | Bayer→RGB→resize | 无时间换算 | 每帧 | `grab_trigger.cpp:252–278` | 会推迟下一次取帧 |
| 发布 | `rcv_time` → Image header | 上述时间域 | 约 10 Hz | `grab_trigger.cpp:276–278,316` | ROS publisher 队列 1 |

若共享映射无效、`low==0` 或关闭 TriggerEnable，代码在
`grab_trigger.cpp:241–244` 回退 `ros::Time::now()`。这会无提示地切到
ROS/主机墙钟域。

### 3.2 `/timeshare` 的真实来源

```text
Livox packet timestamp[8]
  → GetEthPacketTimestamp()
  → PointXyzlt.time_stamp
  → 点云队列
  → LidarDataQueue::StorageRawPacket()
  → PointCloudPacket.base_time
  → PublishCustomPointcloud()
  → pointt->low = pkg.base_time
  → /home/<getlogin()>/timeshare
```

| 箭头 | 输入 → 输出 | 单位/时钟来源 | 频率 | 文件与行 | 缓存/延迟 |
|---|---|---|---|---|---|
| 设备→驱动 | `timestamp[8]`,`time_type` → `uint64_t` | GPS/PTP 设备时间；NoSync 时主机时间 | packet 频率 | `pub_handler.cpp:98–151,265–275` | 网络接收延迟 |
| packet→点 | packet stamp + `i*point_interval` | ns；`time_interval` 为 0.1 µs | 每点 | `pub_handler.cpp:142–144,366–419`；`/usr/local/include/livox_lidar_def.h:129–137` | 点包聚合 |
| 点→队列 | 点集合 → raw pointcloud queue | 同一 ns 域 | 发布目标 10 Hz | `src/livox_ros_driver2/src/lds.cpp:184–195` | 队列实际容量 16 |
| 队列→pkg | 首点时间 → `pkg.base_time` | ns | 10 Hz，启动可突发 | `lddc.cpp:188–236,289–317` | 轮询线程启动先睡 3 s |
| pkg→共享槽 | `pkg.base_time` → `pointt->low` | ns | 每点云一次 | `lddc.cpp:308–315` | 单槽覆写；写在 ROS publish 前 |

因此，“共享内存写在相机触发之前还是之后”在当前实现中没有固定答案：
它不是按相机触发写入，而是一个点云聚合完成并出队时写入，通常已经晚于
该点云 base time 对应的物理采样。

### 3.3 LiDAR CustomMsg 链路

```text
MCU TIM3_CH2/PB5 1 Hz + USART1 GPRMC
  → [物理接线/设备配置：仓库中未记录]
  → Livox 设备 time_type + timestamp[8]
  → GetEthPacketTimestamp()
  → PointXyzlt.time_stamp
  → pkg.base_time（点云第一点）
  → CustomMsg.timebase
  → ros::Time(pkg.base_time/1e9)
  → CustomMsg.header.stamp
```

| 箭头 | 输入 → 输出 | 单位/时钟来源 | 频率 | 文件与行 | 缓存/延迟 |
|---|---|---|---|---|---|
| MCU→同步输出 | TIM3 tick → PB5 1 Hz | MCU HSE 派生 | 1 Hz | `main.c:24`；`timer.c:216–260` | 硬件 PWM |
| MCU→RMC | hh/mm/ss + 固定 date → `$GPRMC` | MCU 软件秒计数 | 1 Hz | `timer.c:93–143` | 9600 baud 阻塞发送约数十 ms |
| 同步输入→设备时钟 | PPS/RMC → packet timestamp | **物理接线和 Livox 配置未入库** | 设备内部 | 无可审查源码 | 未确认 |
| packet→base time | 首点 ns → `pkg.base_time` | packet `time_type` 指定 | 约 10 Hz 点云 | `pub_handler.cpp:265–275,366–419`；`lddc.cpp:455–470` | 点云积累约 100 ms |
| base→消息 | base → `timebase`,`header.stamp` | 相同 ns 值 | 10 Hz | `lddc.cpp:455–476` | double 舍入；发布队列 |

`src/livox_ros_driver2/msg/CustomMsg.msg` 将 `timebase` 定义为第一点时间，
每点 `offset_time` 是相对 `timebase` 的 ns 偏移。FAST-LIVO2 在
`src/FAST_LIVO2/src/preprocess.cpp:126,176` 将偏移除以 `1e6` 变成 ms。

### 3.4 IMU 链路

```text
Livox IMU packet time_type + timestamp[8]
  → GetEthPacketTimestamp()
  → ImuData.time_stamp
  → ros::Time(time_stamp/1e9)
  → sensor_msgs/Imu.header.stamp
```

| 箭头 | 输入 → 输出 | 单位/时钟来源 | 频率 | 文件与行 | 缓存/延迟 |
|---|---|---|---|---|---|
| IMU packet→时间 | `timestamp[8]` → `imu_data.time_stamp` | 与 LiDAR 相同解析函数 | 约 200 Hz | `pub_handler.cpp:111–125,265–275` | 网络与回调延迟 |
| IMU 队列→ROS | `time_stamp` → `header.stamp` | ns→double 秒 | 约 200 Hz | `lddc.cpp:603–620,758–795` | IMU 队列有 mutex |

LiDAR 点和 IMU 都调用同一个 `GetEthPacketTimestamp()`。当前源码没有为
它们建立两个不同时间基准。

## 4. 各时钟域说明

| 时钟域 | 产生者 | 在代码中的表示 | 与其他域关系 |
|---|---|---|---|
| MCU 晶振/定时器域 | STM32F103 HSE/PLL | TIM2/TIM3 counter | PA1 10 Hz、PB5 1 Hz 和软件秒来自同一 HSE |
| MCU 伪 UTC 标签 | `timer.c` hh:mm:ss + `230520` | GPRMC 文本 | 无真实 GNSS 校时；日期固定，午夜后重复 |
| Livox 设备同步域 | Livox packet | `time_type`,`timestamp[8]` | GPS/PTP 时由设备同步；NoSync 时驱动改用主机墙钟 |
| 主机 ROS/墙钟域 | Linux/ROS | `ros::Time::now()`、C++ `system_clock` | 相机失败回退和 Livox NoSync 回退会进入此域 |
| 相机设备域 | 海康相机 | `nDevTimeStampHigh/Low` 等 | 当前代码取得但完全未使用 |
| bag 记录时间域 | rosbag recorder 主机 | connection record time | 受调度、队列、传输和磁盘写入影响，不等于设备采样时间 |
| GNSS/UTC 域 | 未来真实 GNSS | 当前链路不存在 | 必须用 PPS + 时间报文明确映射 |

`std::chrono::high_resolution_clock` 在当前 libstdc++ 9 实现中别名为
`system_clock`，duration 为 ns；证据：
`/usr/include/c++/9/chrono:828–842,888`。但这不是跨标准库可移植保证。

## 5. 单片机定时器计算

### 5.1 芯片与时钟树

- MCU：STM32F103C8，`USER/PWM.uvprojx:17`。
- `HSE_VALUE=8,000,000`：`USER/stm32f10x.h:115–120`。
- PLL：HSE ×9 → SYSCLK 72 MHz：
  `USER/system_stm32f10x.c:987–1056`。
- AHB：`HCLK=SYSCLK/1=72 MHz`：1021–1022 行。
- APB2：`PCLK2=HCLK/1=72 MHz`：1024–1025 行。
- APB1：`PCLK1=HCLK/2=36 MHz`：1027–1028 行。
- STM32F1 规则：APB prescaler 不为 1 时，定时器时钟为 2×PCLK；
  所以 TIM2/TIM3 为 72 MHz。项目自身注释也在
  `HARDWARE/TIMER/timer.c:13` 明确写出这一点。

Keil 工程 XML 中的 `CLOCK(12000000)` 是 IDE/debug CPU 描述字段，
不是固件运行时 SystemInit 的时钟源；实际 C 代码明确配置 8 MHz HSE
和 72 MHz PLL。

### 5.2 TIM2 相机触发

源码：

```text
USER/main.c:22          TIM2_PWM_Init(999,7199)
HARDWARE/TIMER/timer.c:157–206
```

计算：

```text
f_TIM2 = 72,000,000 Hz
f_tick = 72,000,000 / (7199 + 1) = 10,000 Hz
T_tick = 100 µs
T_TIM2 = (999 + 1) × 100 µs = 100 ms
f_TIM2 = 10 Hz
```

`TIM_SetCompare2(TIM2, arr/2)` 产生约 50% 占空比。TIM3 每秒中断时执行
`TIM_SetCounter(TIM2, TIM2->ARR/2)`（`timer.c:103`），会每秒重新对齐
TIM2 相位，但不会把稳定周期改为 100.291 ms。

### 5.3 TIM3 Livox 同步/PPS

源码：

```text
USER/main.c:24          TIM3_PWM_Init(9999,7199)
HARDWARE/TIMER/timer.c:216–260
```

计算：

```text
f_tick = 10,000 Hz
T_TIM3 = (9999 + 1) / 10,000 = 1 s
f_TIM3 = 1 Hz
```

### 5.4 计数、溢出和单位换算

- 当前固件没有可供 Jetson 读取的自由运行 ns/us 时间戳。
- 没有 tick→ns、tick→us 公式，也没有 32/64 位硬件 counter 溢出扩展。
- 软件时间只是 TIM3 每 1 s 把 hh:mm:ss 自增：
  `timer.c:122–136`。
- 时间戳不是“每帧固定加 100 ms 后写入共享内存”；固件根本不写
  `/timeshare`。
- 午夜后 hh:mm:ss 回到 00:00:00，但日期仍固定 `230520`，会产生
  24 小时回环。

### 5.5 发现的 MCU 实现问题

1. `chckNumChar[2]`（`timer.c:54`）用于
   `sprintf(chckNumChar,"%02X",checkNum)`（141 行），需要 3 字节含
   `\0`，每秒存在 1 字节越界写。
2. GPRMC 经纬度和日期是硬编码；它不是实际 UTC/GNSS。
3. 9600 baud 的 `printf` 在 TIM3 中断内阻塞发送：
   `SYSTEM/usart/usart.c:42–47`。约 70–80 字符的句子会占用约
   73–83 ms。硬件 PWM 仍运行，且 TIM2 相位复位发生在打印前，因此
   这不是 2990 ppm 的解释，但会带来软件中断延迟风险。
4. TIM3 OC 初始化结构在 `TIM_OC2Init` 前没有显式设置 `TIM_Pulse`
   （`timer.c:241–245`），随后 251 行再设置 compare；可能影响启动
   首脉冲，不影响长期频率。

### 5.6 与 100.291 ms 比较

若稳定输出周期真的为 100.291 ms，保持 PSC/ARR 不变，则：

```text
f_TIM_required ≈ 72 MHz × 100/100.291 ≈ 71.791 MHz
f_HSE_required ≈ 7.9768 MHz
误差 ≈ -2900 ppm
```

源码里的整数值不能产生这个比例。正常石英晶振的典型误差为几十 ppm
量级，不能合理解释约 2990 ppm；若板上并非预期晶振、晶振失效或测量
点错误则另当别论，必须实测。

更关键的是 bag 的稳定十帧跨度为：

```text
均值   0.999999907995 s
中位数 0.999968529 s
最小   0.999528169 s
最大   1.000469208 s
```

因此旧 bag 本身排除了“相机 header 稳定以 100.291 ms 运行”。

## 6. 共享内存协议分析

### 6.1 创建与写入

定义：

```cpp
struct time_stamp {
    int64_t high;
    int64_t low;
};
```

真实定义位于 `src/livox_ros_driver2/src/lddc.h:43–48`，相机侧重复定义
位于 `src/mvs_ros_driver/src/grab_trigger.cpp:19–23`。

Livox 驱动在 `src/livox_ros_driver2/src/lddc.cpp:187–218`：

1. 用 `getlogin()` 组装 `/home/<user>/timeshare`；
2. `open(O_CREAT|O_RDWR|O_TRUNC, 0666)`；
3. `lseek(fd, sizeof(time_stamp), SEEK_SET)` 后写 1 字节，使文件实际为
   17 字节，而映射长度为 16 字节；
4. `mmap` 16 字节。

在 CustomMsg 模式下：

```text
lddc.cpp:308  timestamp = pkg.base_time
lddc.cpp:310  pointt->low = timestamp
lddc.cpp:315  publish(msg)
```

因此：

- `low` 的真实含义：**Livox 点云第一点的 `pkg.base_time`，单位 ns**。
- `high` 的真实含义：**当前没有含义；工作空间内没有任何写入**。
- 写入频率：正常约 10 Hz，每发布点云一次；启动积压时可能突发。
- 写入时序：在 CustomMsg ROS publish 之前，但不是在相机触发前后。
- 每次相机触发写一次：否。

### 6.2 读取

相机启动时仅执行一次：

```text
grab_trigger.cpp:318–325
open(path, O_RDWR)
mmap(sizeof(time_stamp))
```

取帧成功后：

```text
grab_trigger.cpp:235  pointt != MAP_FAILED && pointt->low != 0
grab_trigger.cpp:237  int64_t b = pointt->low
```

没有重新打开、重试、序号检查或新鲜度检查。

### 6.3 已确认的协议缺陷

| 项目 | 结论 | 证据/影响 |
|---|---|---|
| 单槽 | 是 | 只有 16 字节结构；新值覆盖旧值 |
| 触发序号 | 无 | 结构只有 high/low |
| 帧号 | 无 | 相机 `nFrameNum` 未写入协议 |
| 版本/magic | 无 | 无法升级或验证布局 |
| 有效位/时钟域 | 无 | 非零值被直接当 ns |
| 原子/锁/seqlock | 无 | 普通跨进程 load/store |
| 内存屏障 | 无 | 未使用 C++ atomic 或平台 barrier |
| mmap 错误处理 | 不完整 | open/mmap 失败后仍可继续；writer 还会继续使用 fd |
| 文件尺寸 | 异常 | 通过 lseek+1 字节建成 17 B，不是明确 `ftruncate(16)` |
| fd 生命周期 | 泄漏 | mmap 后未关闭 fd |
| 旧值残留 | 可能 | 崩溃后文件保留；reader 不检查年龄 |
| writer 重启 | 有风险 | `O_TRUNC` 清零/重建，旧 mapping 与重启竞态未定义为协议 |
| 重复/倒退检测 | 无 | reader 不保存/比较前值 |

在常见 x86_64/aarch64 上，对齐的 64 位单次 load/store通常不会被硬件
撕裂；但这里没有跨进程原子协议，C++ 层也没有同步保证。更直接的问题是
reader 在条件和赋值处读取 `low` 两次，writer 可在二者之间更新。若未来
同时使用 `high` 和 `low`，普通的两个独立 store/load 更无法保证成对一致。

### 6.4 启动与残留

- `sensors.launch` 同时启动 Livox 和相机，没有建立“共享文件先就绪”的
  顺序。
- Livox 直到点云 polling 路径才创建文件；相机若先启动，首次 `open`
  失败后不会重试，整个进程持续回退 `ros::Time::now()`。
- `getlogin()` 在 systemd、无控制终端或容器中可能返回 null；两端都未
  正确处理。
- 当前审计时 `/home/gulu/timeshare` 不存在，不能验证在线内容。
- 若 writer 崩溃但 reader 仍运行，最后一个非零值可被无限重复使用。

### 6.5 单槽覆盖情形

以下竞态在当前协议中成立：

```text
图像 N 已经被相机曝光
  → 图像 N 尚在相机/USB/SDK 缓冲
  → Livox 点云 N+1 出队并覆盖 pointt->low
  → MV_CC_GetOneFrameTimeout 返回图像 N
  → 图像 N 被赋成最新 Livox N+1 的时间
```

它不会产生平滑的频率缩放，而会产生离散的一个或多个周期跳变、重复或
固定偏一帧。旧 bag 的启动跳变正是这种“槽位更新速度远高于图像消费”
机制可以产生的数值形态。

## 7. 相机帧—触发对应分析

### 7.1 MVS SDK 可用字段

当前 CMake 从 `/opt/MVS/include` 包含 SDK：
`src/mvs_ros_driver/CMakeLists.txt:32`。实际
`/opt/MVS/include/CameraParams.h:416–492` 的
`MV_FRAME_OUT_INFO_EX` 包含：

```text
nFrameNum             CameraParams.h:422
nDevTimeStampHigh     CameraParams.h:423
nDevTimeStampLow      CameraParams.h:424
nHostTimeStamp        CameraParams.h:426
nSecondCount          CameraParams.h:431
nCycleCount           CameraParams.h:432
nCycleOffset          CameraParams.h:433
nFrameCounter         CameraParams.h:444
nTriggerIndex         CameraParams.h:445
nLostPacket           CameraParams.h:456
```

所以当前 SDK 已经提供相机帧号、触发索引和设备/主机时间字段；不需要
先升级 SDK 才能观测它们。

### 7.2 当前代码实际使用情况

- `stImageInfo` 在 `grab_trigger.cpp:215` 声明。
- `nFrameNum` 只出现在 247–250 行的注释调试语句，未发布、未匹配。
- `nDevTimeStampHigh/Low`、`nHostTimeStamp`、`nTriggerIndex`、
  `nFrameCounter` 全部未使用。
- ROS `header.seq` 不是从相机 `nFrameNum` 显式赋值，因此 bag 的 ROS
  seq 连续不能证明相机设备没有丢帧。

### 7.3 SDK 缓冲与处理时序

`MV_CC_GetOneFrameTimeout` 在 `/opt/MVS/include/MvCameraControl.h:735`
声明。SDK 文档 717–733 行提示主动取帧时应用需控制调用速率。默认抓取
策略是 OneByOne（805–829 行）；代码没有显式设置 grab strategy 或
buffer node count。

成功取帧后的顺序：

```text
GetOneFrameTimeout
→ 读取 pointt->low 到局部 rcv_time
→ Bayer/RGB 像素转换
→ cv::resize
→ 构造 ROS Image
→ 写 header.stamp
→ publish
```

证据：`grab_trigger.cpp:230–278`。

这意味着：

- 下一次触发在像素转换/resize 期间覆盖槽位，**不会改变当前帧已拷到
  局部变量的 `rcv_time`**。
- 但转换/resize 会推迟下一次 `GetOneFrameTimeout`；若整体处理慢于
  100 ms，SDK OneByOne 队列中可保留旧帧，而共享槽已经前进。
- 图像约为 1080×1300、`rgb8`、4,212,000 B；10 Hz ROS payload 约
  42 MB/s。曝光配置 5 ms。USB 实际传输、转换和 resize 耗时没有在
  代码或 bag 中被记录，不能凭理论带宽断言一定低于 100 ms。
- `cv::resize` 即使 `image_scale=1` 仍执行（272 行），会增加无必要
  的处理成本；这是性能问题，不是 ppm 根因。

### 7.4 bag 能证明和不能证明的事

bag 能证明：

- 相机 ROS seq 0…4464 连续。
- header 无重复、无倒序。
- 启动后 4462 帧连续取得了 4462 个连续的 LiDAR base time；没有
  共享槽重复使用或跳过。

bag 不能证明：

- 相机设备 `nFrameNum` 是否丢帧。
- `nTriggerIndex` 是否与 MCU 脉冲一一对应。
- 相机索引 3 对应物理触发 0、1、2 还是其他编号。
- 是否存在稳定的一帧延迟；事实上按 bag 记录时间最近邻，相机 header
  通常比当时最新 LiDAR base time 落后约 100 ms。
- 启动前 3 帧的设备对应关系。

### 7.5 是否会“累计错配”

单槽错配不是每帧进行小量积分，因此通常不会产生线性累计误差；它表现为
离散的 `±N×100 ms` 跳变、重复时间、跳帧或固定偏 N 帧。若处理持续
跟不上，队列深度会不断增加，错配的帧数可以扩大；若 SDK 丢旧帧或取
最新帧，则会表现为帧号跳变。当前代码没有记录足够字段区分这些情形。

## 8. LiDAR 时间戳来源

### 8.1 packet 时间解析

`src/livox_ros_driver2/src/comm/pub_handler.cpp:265–275`：

```text
time_type == kTimestampTypeGps
  或 time_type == kTimestampTypePtp
    → 直接读取 packet timestamp[8]

否则（包括 NoSync）
    → high_resolution_clock::now().time_since_epoch().count()
```

因此当前代码有两种不同来源：

1. GPS/PTP：Livox 设备时间；
2. NoSync：主机 wall-clock-like 时间，发生在 packet callback。

代码通过 `data->time_type != kTimestampTypeNoSync` 设置一个静态全局
`is_timestamp_sync_`（105–109 行）。它没有把 `time_type` 放入 ROS
消息，也不是按设备/流独立保存；多设备或混合状态时可被最后一个 packet
覆盖。

### 8.2 `CustomMsg.timebase` 与 header

在 `src/livox_ros_driver2/src/lddc.cpp:455–476`：

```text
msg.timebase = pkg.base_time
msg.header.stamp = ros::Time(pkg.base_time / 1e9)
```

两者来自同一个整数。bag 实测 `timebase - header.toNSec()` 范围约
`-242…+245 ns`，中位数 `1 ns`，4463 条中 4447 条非零。这只是经过
double 秒造成的量化舍入，不能解释毫秒或 ppm 现象。

### 8.3 MCU 信号的含义

当前 MCU 源码产生：

- PA1：10 Hz，相机 Line0 的高概率物理来源；
- PB5：1 Hz；
- PA9/USART1：每秒一条 GPRMC。

仓库中没有 Livox“扫描触发”API调用，也没有证明 PB5 直接触发一帧点云。
Livox SDK 虽在 `/usr/local/include/livox_lidar_api.h:439–447` 提供
`SetLivoxLidarRmcSyncTime`，工作空间驱动没有调用它。

bag 时间落在 MCU 固定日期 2020-05-23；驱动 GPS/PTP 分支才保留设备
时间。因此最符合证据的解释是：

```text
PB5 1 Hz + UART GPRMC 用于 Livox 时间同步
```

也就是说它很可能在设置/校准设备时间，而不只是同步采样相位。但物理接线、
Livox 实际 `time_type` 和设备配置未记录，不能宣称完全确认。即使 PPS
同时影响采样相位，也不等于相机触发与 LiDAR 点云 base time 自动属于
同一事件编号。

### 8.4 LiDAR 与 IMU

- LiDAR 点：`pub_handler.cpp:142–144,366–419`。
- IMU：`pub_handler.cpp:111–125`。
- 二者调用相同 `GetEthPacketTimestamp()`。
- 点云 header 是一个约 100 ms 点云包的第一点时间；IMU header 是
  单个 IMU packet 时间。按 bag 记录时间做最近邻时，两者天然可能相差
  一个点云窗口，不能把这个差直接解释成时钟偏移。

约 116～127 ppm 的 header/bag 端点差主要反映设备/主机时间相对 bag
记录时间、消息语义及收包/调度抖动；二者回归斜率仅差约 4.76 ppm，
没有证据支持独立漂移。

## 9. bag 统计结果

### 9.1 文件与连接

原 bag 未被修改：

```text
路径       /home/gulu/Downloads/0709.bag
大小       20,538,142,798 bytes
总时长     446.416370 s
总消息数   98,175
压缩       none
```

全量顺序扫描耗时约 42.7 s。connection callerid：

```text
/left_camera/image  → /mvs_camera_trigger
/livox/lidar        → /livox_lidar_publisher2
/livox/imu          → /livox_lidar_publisher2
```

### 9.2 每话题汇总

| 话题 | 数量 | bag duration (s) | header duration (s) | 端点 header/bag | header 单调/重复/倒序 |
|---|---:|---:|---:|---:|---|
| `/left_camera/image` | 4,465 | 446.364923264 | 447.699935436 | +2990.8537 ppm | 单调；0 重复；0 倒序 |
| `/livox/lidar` | 4,463 | 446.143099744 | 446.199746371 | +126.9696 ppm | 单调；0 重复；0 倒序 |
| `/livox/imu` | 89,247 | 446.218989376 | 446.270712852 | +115.9150 ppm | 单调；0 重复；0 倒序 |

ROS seq：

```text
Camera 0…4464，全部 +1
LiDAR  18…4480，全部 +1
IMU    653…89899，全部 +1
```

这只证明 ROS 消息序列连续，不等价于相机设备帧号连续。

### 9.3 header 帧间隔分布

| 话题 | min | p1 | median | p99 | max |
|---|---:|---:|---:|---:|---:|
| Camera | 99.519968 ms | 99.839925 ms | 99.919796 ms | 100.440025 ms | **1399.789095 ms** |
| LiDAR | 99.519968 ms | — | 99.919796 ms | — | 100.488662 ms |
| IMU | 3.942967 ms | — | 4.970551 ms | — | 6.107092 ms |

Camera 唯一大于 150 ms 的区间：

```text
index 0 header = 1590192197.100239754
index 1 header = 1590192198.500028849
Δ                 1.399789095 s
```

之后没有 `<50 ms` 或 `>150 ms` 的 camera header 区间。

### 9.4 启动异常对“2990 ppm”的影响

全段平均：

```text
447.699935436 s / 4464 = 100.291204175 ms/frame
```

去掉第一个异常区间：

```text
(447.699935436 - 1.399789095) / 4463
= 100.00003279 ms/frame
```

相对于正常约 100 ms，第一个区间多出：

```text
1.399789095 - 0.1 = 1.299789095 s
```

占此前计算累计多走 `1.335012172 s` 的：

```text
1.299789095 / 1.335012172 = 97.36%
```

因此端点 duration 比值不适合在含启动跳变时估计时钟频率。应使用分段、
稳健回归或先检测 change point/outlier。

### 9.5 bag_time - header_stamp 线性趋势

对稳定段做普通最小二乘，`header = a + b×bag_time`：

| 话题/区间 | b−1 | 等价 `bag-header` 趋势 | 回归残差标准差 |
|---|---:|---:|---:|
| Camera，去掉 index 0 | +80.1337 ppm | −80.1337 µs/s | 3.709 ms |
| LiDAR | +121.484 ppm | −121.484 µs/s | 2.397 ms |
| IMU | +116.721 ppm | −116.721 µs/s | 0.424 ms |

Camera 全段回归为 +84.048 ppm，但最大残差约 1.2996 s，说明单一直线
无法描述启动跳变。LiDAR/IMU 的 bag-time 趋势接近，符合它们共享
设备 packet 时间域而 bag time 含主机接收/写入时序。

### 9.6 Camera 与 LiDAR 的真实关系

最强的实测证据不是拟合，而是整数 ns 精确匹配：

```text
Camera index 3…4464 的 4462 个 header
==
LiDAR index 0…4461 的 4462 个 header
```

- 匹配数：4462/4465 camera。
- 匹配索引每次均 +1，无跳过。
- 相应帧间隔向量最大差：`0 ns`。
- 这两个序列的仿射关系：正常段可直接写成 `t_camera = t_lidar`；
  不存在 3362 ppm 的相对速率差。

同一 header 的 bag 记录时序：

```text
camera_bag_time - lidar_bag_time
中位数 102.229104 ms
p1      87.0249 ms
p5      93.7589 ms
p95    115.4647 ms
p99    117.1281 ms
min     57.917 ms
max    128.858 ms
```

该延迟沿 bag 的斜率约 +41.3 µs/s，端点约增长 21.7 ms；这是发布相位、
USB/SDK/ROS/bag 调度的综合量，不是 header 时钟差。

按 **bag 记录时间** 给每幅图找最近 LiDAR 消息时，camera header 通常比
该 LiDAR header 早约 99.92 ms。这与 FAST-LIVO2 当前
`src/FAST_LIVO2/config/mid360.yaml:71` 的
`img_time_offset: 0.1` 数值吻合，但该配置是经验补偿，不构成物理触发
对应证据。

### 9.7 启动前三帧

```text
Camera[0] header = 2020-05-23 00:03:17.100239754 UTC
Camera[1] header = 2020-05-23 00:03:18.500028849 UTC
Camera[2] header = 2020-05-23 00:03:18.600408792 UTC
Camera[3] header = 2020-05-23 00:03:18.700268745 UTC
LiDAR[0]  header = 2020-05-23 00:03:18.700268745 UTC
```

第 3 帧开始逐一等于 LiDAR。MCU 固定 RMC 日期也是 2020-05-23，
是 GPS/RMC 同步链路的强旁证。

### 9.8 `CustomMsg.timebase` 一致性

每条 LiDAR 消息比较 `timebase` 与 `header.stamp.toNSec()`：

```text
范围   -242…+245 ns
中位数 1 ns
非零   4447 / 4463
```

原因是 `ros::Time(timestamp / 1e9)` 先转 double。推荐改用
`ros::Time stamp; stamp.fromNSec(timestamp);`，但这不是本次速率异常。

## 10. 已确认根因

1. **已确认（bag）：2990.85 ppm 是把一次启动阶跃误当成长期频率斜率。**  
   第一个 1.399789095 s 区间贡献了约 1.299789095 s 额外跨度；稳定段
   是约 100.000033 ms。

2. **已确认（源码+bag）：相机 header 不是相机/MCU 独立时钟，而是
   Livox 点云 base time 的单槽转抄。**  
   写入代码 `lddc.cpp:308–310`，读取代码
   `grab_trigger.cpp:233–239`；4462 个 exact match。

3. **已确认（源码）：协议没有图像—触发身份。**  
   只有最新 `low`，不使用 `high`，无 trigger/frame sequence。因而即使
   数值单调，也无法证明物理事件一一对应。

4. **已确认（源码）：相机和 Livox header 均用浮点秒构造 ROS Time。**  
   已实测最多约 245 ns 舍入；应修正但不是 ppm 根因。

5. **已确认（源码）：MCU 当前提供伪日期/软件秒，而不是实际 GNSS UTC。**  
   固定日期和坐标使绝对日期不可靠，也不能支撑未来统一 UTC。

## 11. 高概率但尚未确认的问题

1. **启动 jump 来自 Livox 3 s polling 延迟后的队列突发清空。**  
   `src/livox_ros_driver2/src/livox_ros_driver2.cpp:196–213` 的点云/IMU
   polling 线程先睡 3 s；队列由
   `src/livox_ros_driver2/src/comm/comm.cpp:42–47`、
   `src/livox_ros_driver2/src/ldq.cpp:39–49,128–130` 计算为 16；
   `lddc.cpp:221–235,291` 连续 drain。10 Hz 下跳过约 14 帧与
   1.399789095 s 完全同量级。缺少历史 build ID，所以保留“高概率”
   标签。

2. **Livox 运行在 GPS/RMC 同步而非仅相位同步。**  
   固定日期精确出现在 packet/header 域，且 GPS/PTP 分支才保留设备
   时间。仍需抓取 `time_type` 和核对物理接线/设备配置。

3. **图像与共享时间可能固定偏一帧。**  
   旧 bag 只能证明连续消费，不知道触发编号；约 100 ms 的发布时序差
   和现有 `img_time_offset=0.1` 都提示需要硬件级编号验证。

4. **相机先于 writer 启动会永久使用 `ros::Time::now()`。**  
   launch 并发启动、writer 延后创建、reader 不重试，代码路径明确；
   但旧 bag 正常段显然没有走该 fallback。

5. **崩溃/重启时会重复旧时间或混合时钟域。**  
   协议无新鲜度和 epoch，机制明确；本 bag 未出现重复。

6. **相机 SDK 队列可能在高负载时导致旧帧配新槽。**  
   默认 OneByOne 且未记录 buffer depth/设备帧号；本 bag 正常段未见
   槽位 skip，但不能排除固定偏移或设备丢帧。

7. **64 位普通跨进程读写缺少语言层同步保证。**  
   当前只用 `low` 时硬件撕裂概率低于逻辑覆盖风险；未来若使用
   high/low 成对时间则风险会显著增加。

## 12. 被排除的问题

| 候选原因 | 结论 | 依据 |
|---|---|---|
| STM32 APB1 定时器 ×2 被漏算 | 排除 | 源码明确 PCLK1/2，timer 注释和计算均按 TIM=72 MHz |
| PSC/ARR 少加 1 | 排除 | 7199+1、999+1 正好 10 Hz |
| 代码稳定地产生 100.291 ms | 排除 | 稳定 bag 均值 100.000033 ms，十帧均值约 1 s |
| 正常晶振容差导致 2990 ppm | 作为正常工况排除 | 所需约 −2900 ppm，远大于常见石英量级；仍建议硬件实测 |
| 相机 header 与 LiDAR header 持续相对漂移 3362 ppm | 排除 | 4462 个 ns 级完全相等，间隔向量完全相同 |
| bag 内 header 倒序/重复造成结果 | 排除 | 三个话题均 0 倒序、0 重复 |
| 正常段共享槽连续跳帧/重复 | 对该 bag 排除 | 对应索引连续 +1；启动前三帧除外 |
| LiDAR 与 IMU 分别使用不同驱动时钟 | 排除 | 同一个 `GetEthPacketTimestamp()` 代码路径 |
| double 舍入造成毫秒级漂移 | 排除 | 实测仅约 ±245 ns |

尚未排除、但已不再需要用来解释旧 bag 的项目：真实板载晶振是否偏离
标称值、相机设备时钟频率、物理线延迟和 Livox 内部同步算法。它们需要
硬件测量。

## 13. 推荐修复方案

### 13.1 优先级 P0：先让系统可观测

1. 相机每帧记录/发布：
   `nFrameNum`、`nFrameCounter`、`nTriggerIndex`、
   `nDevTimeStampHigh/Low`、`nHostTimeStamp`、SDK lost-frame 信息。
2. Livox 记录每设备/每 packet 的 `time_type`、同步状态、base time、
   点云队列深度和启动 drain 数。
3. 所有时间转换用整数 `fromNSec()`，禁止经 double 秒。
4. 触发模式下共享时间失效应明确报错/诊断；不要静默混入
   `ros::Time::now()`。

### 13.2 优先级 P1：建立事件一一对应

推荐的最小可靠协议是带序号的有界环形队列，而不是单槽：

```text
protocol magic/version/clock_id/epoch
write_seq
entries[] = {
  trigger_seq,
  trigger_time_ns,
  flags,
  integrity/sequence guard
}
```

- writer 用 release store/seqlock 提交完整 entry；
- reader 用 acquire load 验证 entry 前后序列一致；
- 记录 overflow、consumer lag、重复、倒退；
- 进程重启增加 epoch，旧数据不能误认为新触发。

但是当前系统没有 MCU→Jetson 的触发 timestamp/sequence 通道。只有把
MCU 的实际触发序号和计时值通过 UART/GPIO capture/其他可靠通道带到
Jetson，环形队列才真正代表相机触发。不能只把 Livox base time 搬进
一个更大的队列就称为触发对应。

相机侧匹配优先级：

1. 若海康 `nTriggerIndex` 与外部触发序号可靠，直接按序号匹配；
2. 否则用 `nFrameNum` + 相机设备时间建立严格单调映射，并用 MCU
   trigger sequence 做有界最近邻；
3. 任一序号缺失都显式标记 drop，不得取“最新值”掩盖。

### 13.3 优先级 P2：统一时钟轴

1. 使用真实 GNSS 接收机的 PPS + UTC/GPS 时间报文，不再生成固定日期
   GPRMC。
2. 明确统一基准是 UTC、GPS time 还是 TAI，并处理 GPS−UTC leap-second
   差；ROS header 通常需要 Unix/UTC 表示。
3. Livox 继续使用设备 GPS/PTP 时间，但必须确认和记录 `time_type`；
   NoSync 时发布健康状态，不能无提示换到主机时间。
4. 相机若不能直接接受 PTP/PPS UTC，则用相机设备 tick 与 MCU/PPS
   观测建立仿射映射：

   ```text
   t_unified = scale × t_camera_device + offset
   ```

   scale 应在线/分段估计并监控残差，而不是固定加 0.1 s。
5. 主机若参与时间生成，应由 chrony/PTP/PHC 明确驯服，并记录同步状态。

### 13.4 优先级 P3：修复启动和失效模式

1. 移除或重构 Livox polling 线程固定 3 s 睡眠；至少在积压 drain 完成
   前不要宣布共享时间 ready。
2. writer 使用明确 `ftruncate(sizeof(...))`、检查所有返回值并关闭 fd。
3. 避免 `getlogin()` 拼路径；使用 launch 参数或固定运行目录。
4. camera reader 可重连，但在 trigger mode 下必须等到 protocol ready
   和新鲜 entry，再开始发布。
5. 显式配置 MVS grab strategy、buffer 数，并在处理低于帧率时告警。
6. 修复 MCU checksum buffer、日期回绕和 ISR 内阻塞打印。

### 13.5 旧 bag 的处理

不要覆盖原 bag。可另生成派生 bag：

- 原样保留每条原始 header、bag record time 和审计 metadata；
- 对 camera index 3…4464 可标注它们等于对应 LiDAR base time，但不能
  在没有硬件触发编号时宣称这是“真相机曝光时间”；
- 启动前三帧无法可靠修复，只能标记 unknown/drop 或在下游排除；
- 若为旧算法兼容而做经验 0.1 s 平移，必须标为派生估计，不得替换原始
  事实数据。

## 14. 预计修改的文件列表

本轮没有修改。后续实现时预计最小涉及：

| 文件 | 预计目的 |
|---|---|
| `src/mvs_ros_driver/src/grab_trigger.cpp` | 使用 SDK 帧/触发/设备时间；可靠协议消费；整数时间；失败诊断；显式 grab strategy |
| `src/livox_ros_driver2/src/lddc.h` | 删除当前无语义 high/low，或定义版本化共享协议 |
| `src/livox_ros_driver2/src/lddc.cpp` | 移除点云 base time 冒充触发时间；修复映射/整数 ROS time/启动状态 |
| `src/livox_ros_driver2/src/livox_ros_driver2.cpp` | 移除 3 s 固定启动积压机制或增加 readiness |
| `src/livox_ros_driver2/src/comm/pub_handler.cpp` | 按设备记录 time_type；明确 NoSync fallback 和单位 |
| `src/stm32_timersync-open/USER/main.c` | 接入真实 GNSS/PPS、明确输出协议 |
| `src/stm32_timersync-open/HARDWARE/TIMER/timer.c` | trigger sequence/time capture；修复 buffer/ISR/日期逻辑 |
| `src/stm32_timersync-open/SYSTEM/usart/usart.c` | 非阻塞或 DMA 传输，避免 ISR 长阻塞 |
| `src/FAST_LIVO2/launch/sensors.launch` | 启动依赖/readiness/时间源参数 |
| `src/FAST_LIVO2/config/mid360.yaml` | 重新标定或删除经验 `img_time_offset: 0.1` |
| 条件新增：一个最小 `sensor_time_bridge` 节点及共享协议头 | 只有在 MCU 触发序号确需进入 Jetson 时新增；当前工作空间没有对应接收者 |

若现有串口/时间桥在工作空间外，应优先复用它，而不是新增包；必须先把其
源码和部署单元纳入同一审计。

## 15. 风险和兼容性影响

1. 改变 camera header 后，FAST-LIVO2 的 `img_time_offset: 0.1` 可能造成
   二次补偿，必须同步重新标定。
2. 修改 `livox_ros_driver2/CustomMsg` 字段会改变 ROS1 MD5，破坏旧节点
   和旧 bag 兼容；优先用单独 diagnostics topic，保留现有消息布局。
3. 将失效时“回退 now”改为 fail-closed 会减少表面消息量，但能避免
   无提示混合时钟域；上层应处理 not-ready。
4. 环形队列必须定义 producer/consumer 内存序、重启 epoch、容量和
   overflow 策略，否则只是把单槽问题推迟。
5. GPS time/UTC/TAI 和 leap seconds 处理错误可造成整秒级偏差；不要
   只把 PPS 当 UTC。
6. 相机设备时间的 tick 频率和 wrap 宽度必须从 SDK/设备实测确认，不能
   假设 `High:Low` 就是 Unix ns。
7. Livox NoSync fallback 若从当前行为改为拒绝发布，会影响无 GNSS/PTP
   场景；应提供显式模式参数和健康状态。
8. MCU 串口协议变化涉及固件、Jetson bridge 和部署顺序，需要版本协商。
9. 旧 bag 无法补回从未记录的相机设备帧号/触发索引，修复只能是估计或
   丢弃不确定帧。

## 16. 编译与测试方案

### 16.1 MCU

1. 用频率计/示波器测 PA1 连续至少 1000 周期，报告均值、标准差和 ppm。
2. 同时测 PB5 与真实 GNSS PPS 的相位和长期漂移。
3. 测 PA9 GPRMC 起始位相对 PPS 的延迟/抖动。
4. 若板子支持 MCO，直接输出 HSE/PLL 分频测真实频率。
5. 对 timer 公式留下一个无框架可运行自检，验证 HSE/APB/PSC/ARR 组合。

### 16.2 共享协议

1. 两进程压力测试：producer 突发、consumer 故意睡眠。
2. 验证序号不重复、不倒退；overflow 可计数；entry 不撕裂。
3. 在写两字段中途杀死 producer，consumer 必须拒绝半条记录。
4. 重启 producer，epoch 改变；旧 mapping/旧值不得继续有效。
5. 验证文件不存在、权限错误、长度错误、版本错误和 stale timeout。

### 16.3 相机硬件

1. 同时记录 MCU trigger sequence、相机 `nTriggerIndex/nFrameNum`、
   device timestamp、host timestamp 和最终 ROS stamp。
2. 人工增加像素处理延迟超过 100 ms，验证 SDK 队列、drop 和匹配策略。
3. 人工丢一个触发/帧，确认序号缺口被报告，而不是继续取最新槽。
4. 对比曝光起始、触发边沿和 device timestamp 的固定延迟。

### 16.4 Livox

1. 分别在 GPS、PTP、NoSync 模式记录 raw `time_type`。
2. 验证 LiDAR 与 IMU 的相同 PPS/UTC 映射和时间单调性。
3. 冷启动时监控 queue depth，确保不再批量 drain 到共享接口。
4. 使用整数 `fromNSec()` 后验证 `timebase==header.toNSec()`。

### 16.5 ROS/集成

1. `catkin_make` 编译受影响包，检查 ROS1 消息 MD5 不变。
2. 录制不少于 30 min 的新 bag，保留 raw diagnostics。
3. 自动统计：
   - header 单调、重复、倒序；
   - device frame/trigger sequence 缺口；
   - `bag_time-header` 稳健斜率；
   - camera trigger↔frame 匹配残差；
   - LiDAR/IMU 同时钟域残差；
   - 启动、writer 重启和 GNSS 失锁前后 change point。
4. 时钟频率估计必须排除启动段并报告置信区间，不能再只用首尾 duration。
5. 保留原 bag；任何修复写入新文件并附映射清单和脚本版本。

## 17. 明确决策

| 问题 | 结论 |
|---|---|
| 单槽共享内存是否要改为环形队列？ | **需要**，如果继续用共享内存；但队列内容必须是真实触发事件，不应继续只是 Livox base time。 |
| 是否增加触发序号？ | **必须**。MCU trigger sequence、相机 `nTriggerIndex/nFrameNum` 和消费序号必须可对账。 |
| 是否使用相机设备时间？ | **需要使用作对应与连续性依据**；先确认 tick 单位/回绕，再映射到统一时间。不能未经标定直接当 Unix ns。 |
| 是否修正 MCU tick 频率？ | **当前源码参数无需凭猜测修改**。先实测 PA1/PB5；只有测得 HSE/PLL 偏差后才校准。需修的是时间协议、伪 UTC、buffer 越界和 ISR 阻塞。 |
| 是否引入 PPS/UTC 映射？ | **需要**，为 GNSS 接入和统一时间轴。使用真实 GNSS PPS+时间报文，明确 GPS/UTC/TAI。 |
| 是否修改 Livox 驱动？ | **需要小范围修改**：去启动积压、记录 `time_type`、整数 ns 转换、明确 NoSync、停止把点云 base time 冒充相机触发。设备 packet 时间主链可保留。 |
| 是否修复旧 bag？ | **只能生成不覆盖原件的派生数据**。index 3 后可确认 header 与 LiDAR base time 相同，但无法恢复未记录的真实相机触发/设备时间；启动前三帧应标记不确定。 |

---

最终判断：旧 bag 的首要问题不是 0.299% 的持续相机晶振快，而是“相机
header 取 Livox 最新单槽”这一错误建模，加上启动期槽位突发前跳。当前
数据在正常段看起来同步，是因为两条消息使用了同一个整数时间值；这并不
等于相机曝光和 LiDAR 采样已经在物理事件层一一对应。下一轮实现应先补齐
trigger/frame sequence 和设备 `time_type` 可观测性，再决定具体 UTC
驯服与映射方案。
