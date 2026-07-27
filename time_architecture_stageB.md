# 阶段 B 设计：连续本地传感器时间、GNSS 绝对时间映射与独立采集模块

审查日期：2026-07-23  
工作空间：`/home/gulu/catkin_ws`  
审查性质：只读源码审查、无破坏诊断、编译与测试检查；本报告不包含任何源码、固件、launch、配置或 bag 修改。

## 结论先行

推荐进入阶段 B 的架构方向是：

```text
STM32 唯一自由运行时钟
        │
        ├── 10 Hz CAMERA_TRIGGER ──> Camera
        ├── 1 Hz LOCAL_PPS_OUTPUT + 合成 RMC ──> MID-360
        └── 带序号和 CRC 的事件流 ──> sensor_time_bridge ──> ROS

GNSS PPS ──> STM32 输入捕获 ──> GNSS_PPS_CAPTURE
GNSS UTC/PVT ──> Jetson GNSS driver ──> UTC 观测

GNSS_PPS_CAPTURE + UTC 观测
        └── t_utc = a * t_local + b
            只维护映射，不回写、不 step LOCAL_SENSOR_TIME
```

最终融合时间轴必须是 MCU 驱动的 `LOCAL_SENSOR_TIME`。MID-360 的 `time_type=2` 虽由协议命名为 GPS，同步源实际应始终是 MCU 产生的“合成本地 PPS + 合成本地 RMC”，而不是真实 UTC。真实 GNSS 只给 Jetson 上的映射器提供 UTC/PPS 对。

当前代码尚不具备直接实现阶段 B 的全部前置条件，主要阻塞项是：没有硬件原理图/实际接线表；Keil 工程使用的中断向量表缺少 TIM4、USART3 等 medium-density 外设入口；GNSS 接收机及 `/ublox_driver/receiver_pvt` 发布节点源码不在工作空间；UWB 协议没有设备轮次号或设备测量时间的证据；还没有证明当前相机型号会稳定填写 `nTriggerIndex`。这些不会推翻架构，但必须在编码前或第一轮硬件 bring-up 中关闭。

---

## 1. 当前代码和硬件资源清单

### 1.1 工作空间和当前工作树

ROS 源包实测清单：

```text
gnss_comm
livox_ros_driver2
mvs_ros_driver
vikit_common
vikit_py
vikit_ros
fast_livo
```

证据：在 `/home/gulu/catkin_ws` source `/opt/ros/noetic/setup.bash` 和 `devel/setup.bash` 后执行 `catkin list --unformatted`。

工作空间根目录不是 Git 仓库。存在三个 Git 根：

```text
/home/gulu/catkin_ws/src/FAST_LIVO2
/home/gulu/catkin_ws/src/gnss_comm
/home/gulu/catkin_ws/src/rpg_vikit
```

本轮开始和编译检查后的未提交状态相同：

```text
FAST_LIVO2:
 M config/gnss_adapter.yaml
 M config/m3dgr/mid360.yaml
 M config/rtk_fixed_lag_backend.yaml
 M launch/m3dgr_outdoor01.launch
?? config/m3dgr/gnss_adapter_outdoor01.yaml
?? config/m3dgr/rtk_fixed_lag_backend_outdoor01.yaml

rpg_vikit:
 M vikit_common/CMakeLists.txt
 M vikit_ros/CMakeLists.txt

gnss_comm:
 clean
```

这些是用户已有修改，本轮没有改动。`livox_ros_driver2`、`mvs_ros_driver`、`stm32_timersync-open` 本身不是 Git 仓库，不能用各自的 `git diff` 证明来源；阶段 A 状态以实际源码、构建和测试为准。

### 1.2 STM32 型号、时钟和实际编译清单

Keil 目标明确声明 `STM32F103C8`：

- `src/stm32_timersync-open/USER/PWM.uvprojx:10,17-21`
- IROM `0x10000`、IRAM `0x5000`，对应 64 KiB Flash、20 KiB SRAM：同文件 21 行。

ST 官方资料说明 STM32F103C8 属于 medium-density F103x8/B，72 MHz Cortex-M3，具有三个 USART、USB、CAN、三个通用 16 位定时器和一个高级 PWM 定时器，并具有 96 位唯一 ID。[STM32F103x8/B 数据手册](https://www.st.com/resource/en/datasheet/stm32f103rb.pdf)

当前时钟源码：

- HSE 默认值 `8,000,000 Hz`：`USER/stm32f10x.h:115-121`
- `SYSCLK_FREQ_72MHz`：`USER/system_stm32f10x.c:115`
- HCLK = SYSCLK：同文件 1021-1022 行
- PCLK2 = HCLK：1024-1025 行
- PCLK1 = HCLK / 2：1027-1028 行
- PLL = HSE × 9：1048-1056 行

所以 APB1 为 36 MHz，但 APB1 定时器在 APB prescaler 不为 1 时得到 2 倍时钟，即 TIM2/TIM3 实际输入为 72 MHz。现有设置的理论周期为：

```text
TIM2: (PSC+1)*(ARR+1)/72 MHz
     = 7200*1000/72,000,000
     = 0.1 s = 10 Hz

TIM3: 7200*10000/72,000,000
     = 1.0 s = 1 Hz
```

调用证据：`USER/main.c:19,22-24`。这也再次证明阶段 A 审计中的约 2990 ppm 不是由把 APB1 的 36 MHz 错当成定时器 72 MHz 造成；如果发生这种错误，比例会是 2 倍，不是 0.299%。

`PWM.uvprojx:383-503` 给出的实际 Keil 编译文件如下：

```text
USER/main.c
USER/stm32f10x_it.c
USER/system_stm32f10x.c
HARDWARE/LED/led.c
HARDWARE/KEY/key.c
HARDWARE/TIMER/timer.c
SYSTEM/delay/delay.c
SYSTEM/sys/sys.c
SYSTEM/usart/usart.c
CORE/core_cm3.c
CORE/startup_stm32f10x_hd.s
STM32F10x_FWLib/src/misc.c
STM32F10x_FWLib/src/stm32f10x_gpio.c
STM32F10x_FWLib/src/stm32f10x_rcc.c
STM32F10x_FWLib/src/stm32f10x_tim.c
STM32F10x_FWLib/src/stm32f10x_usart.c
STM32F10x_FWLib/src/stm32f10x_exti.c
```

工程存在一个必须先修的资源门禁：

- 目标是 medium-density `STM32F103C8`，但文件名是 `startup_stm32f10x_hd.s`：`PWM.uvprojx:453-455`。
- 更严重的是该向量表在 TIM3 后把 TIM4 位置写为 Reserved，在 USART2 后把 USART3 位置写为 Reserved：`CORE/startup_stm32f10x_hd.s:107-121`。
- 它只导出 TIM2、TIM3、USART1、USART2 handler：同文件 209-220 行。
- 因而不能仅因芯片有 TIM4/USART3 就在当前工程中直接启用它们的中断。
- `PWM.uvprojx:341` 的显式宏只有 `USE_STDPERIPH_DRIVER`；设备密度宏可能由 Keil device pack 注入，但工程文件未显式记录。`USER/stm32f10x.h:95-97` 明确要求必须定义密度宏。

现存 `USER/PWM50.map` 的时间是 2025-05-19，且符号仍是阶段 A 前的旧 `timer.c` 布局，不能证明当前固件已用 Keil 编译。当前 Linux 环境中未找到 `UV4`、`armcc`、`arm-none-eabi-gcc` 或 `wine`，本轮无法重编 STM32。

### 1.3 当前占用的 MCU 外设和引脚

“占用”以下面实际初始化调用为准，不以注释猜测：

| 资源 | 当前用途 | 实际引脚 | 代码证据 | 状态 |
|---|---|---:|---|---|
| TIM2 CH2 | 相机 10 Hz PWM | PA1 | `timer.c:169-177,180-193,204-210` | 已运行 |
| TIM3 CH2，partial remap | MID-360 候选 1 Hz PPS | PB5 | `timer.c:228-237,240-264` | 已运行；实际接到 MID 待量测 |
| USART1 TX/RX | 合成 GPRMC 输出 | PA9/PA10 | `usart.c:80-117` | 9600 8N1；TX 已用 |
| GPIOC | LED | PC13 | `led.c:8-20` | 已运行 |
| SWD | 调试 | 通常 PA13/PA14 | `main.c:7-9` 只有接线注释 | 应保留，实际板级连接待确认 |

`key.c:13-22` 引用了 PE4 和 PA0，但 `main.c:13-32` 没有调用 `KEY_Init()`；不能把它们列为运行时占用。尤其 STM32F103C8 常见 48 引脚封装没有 GPIOE，这进一步表明该文件来自通用模板而不是可靠板级资源表。

当前 TIM3 update ISR 做了四件事：清中断、翻 LED、把 `TIM2->CNT` 改到半周期、拉低 PC13、推进 hh:mm:ss 并置 RMC pending：`timer.c:79-112`。阶段 A 已把阻塞 `printf` 移到 main loop：`main.c:28-32`、`timer.c:115-158`。

### 1.4 可用资源的结论

芯片能力层面：

- USART2、USART3、USB FS、TIM1、TIM4 都存在。
- 当前代码层面：TIM1 未初始化；USART2 未初始化且向量存在；TIM4/USART3 未初始化且当前向量表缺入口；USB 没有设备栈、描述符、PA11/PA12 配置或 USB 时钟初始化代码。
- 板级层面：没有原理图、PCB 引脚引出表、连接器定义或 MCU 丝印照片，因此不能确认 PA2/PA3、PB10/PB11、TIM1/TIM4 capture channel 或 PA11/PA12 是否真的引出和空闲。

推荐资源顺序：

1. USART1 保持专用于 MID-360 RMC，避免 RMC 和二进制事件复用同一 TX。
2. 优先验证 USART2 对应引脚是否可用；若可用，用第二个 3.3 V USB-UART 接 Jetson，承载 MCU 二进制事件。
3. GNSS 的数据串口直接进 Jetson；MCU 只捕获 GNSS PPS。这样不需要第三路 MCU UART。
4. GNSS PPS 优先接一个可输入捕获的空闲 TIM1/TIM4 channel；具体 GPIO 必须在板图和示波器确认后决定。
5. USB CDC 只作为板上确有 USB D+/D− 接口时的备选。它需要新增 USB 栈，风险和代码量均高于第二个 USB-UART。
6. 只有确实需要 MCU 解析 GNSS 时间报文时，才启用 USART3；此前必须换成正确的 medium-density 向量表并确认对应管脚。

---

## 2. 推荐接线图

### 2.1 逻辑接线

```text
STM32
 ├─ 现 PA1 / TIM2_CH2 ───────────> Camera LINE0 trigger
 ├─ 现 PB5 / TIM3_CH2 ───────────> MID-360 pin 8 "second pulse"
 ├─ 现 PA9 / USART1_TX ──────────> MID-360 pin 10 "GPS input"
 ├─ 候选 USART2_TX/RX ─ USB-UART ─> Jetson /dev/sensor_mcu
 └─ 候选 timer input capture <──── GNSS PPS

GNSS receiver data ─ USB/serial ──> Jetson /dev/gnss
UWB tag data      ─ USB/serial ───> Jetson /dev/uwb
MID-360 Ethernet ─────────────────> Jetson NIC
Camera USB3/GigE ─────────────────> Jetson
```

Livox 官方说明 MID-360 UART 同步推荐连接 pin 8 的 PPS 和 pin 10 的 GPS input；若信号不是 3.3 V TTL，必须转换电平；UART 应为 9600、8 data bits、no parity，内容包含 GPRMC/GNRMC。[Livox GPS time synchronization](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)

### 2.2 不能在报告里直接指定的物理引脚

以下仅是 MCU 复用功能候选，不是接线定案：

| 功能 | 候选资源 | 定案前必须验证 |
|---|---|---|
| MCU→Jetson 事件流 | USART2；次选 USART3；再次选 USB CDC | GPIO 是否引出、3.3 V 电平、与板上器件冲突、向量表 |
| GNSS PPS 捕获 | TIM1/TIM4 input capture；次选空闲 EXTI | GPIO 是否引出、边沿极性、电平、上拉、捕获 handler |
| 可选 GNSS 时间输入 MCU | 剩余 USART | GNSS 是否有第二串口、是否真的需要 MCU 解析 |

### 2.3 电气和设备名要求

- MID-360、MCU、GNSS PPS 必须共地。
- 不把 RS-232/RS-485 电平直接接 MCU/MID TTL。
- 为 MCU、GNSS、UWB 分别创建固定 udev symlink，例如 `/dev/sensor_mcu`、`/dev/gnss`、`/dev/uwb`。当前 GNSS 和 UWB 默认都为 `/dev/ttyUSB0`，分别见 `FAST_LIVO2/include/gnss_manager.h:189-190`、`include/uwb_manager.h:316-317`，若同时启用会发生明确的串口所有权冲突。

---

## 3. `LOCAL_SENSOR_TIME` 严格定义

### 3.1 选择方案 A

选择：

```text
A. MCU 直接维护 local_tick，并确定性派生 local_stamp_ns 和合成日历
```

不选择纯方案 B（MCU 只发 tick、Jetson 任意映射 local stamp），原因是 MID-360 必须在 MCU 侧实时收到与 1 PPS 一致的完整 RMC 日期时间；如果 MCU 日历和 Jetson 的 tick→stamp 映射分别维护，会产生两个本地时间定义。最终设计仍在事件里同时发送原始 `local_tick`，Jetson必须复算并校验 `local_stamp_ns`，但不能重新定义本地时间。

### 3.2 字段定义

| 字段 | 严格含义 |
|---|---|
| `mcu_boot_id` | 64 位 MCU 启动实例 ID。建议由 96 位 UID、持久化 boot counter、固件 build ID 做稳定 hash；每次 MCU reset 必须变化。 |
| `session_id` | `hash(protocol_version, mcu_boot_id, synthetic_epoch_ns, timing_config_crc)`；一个 MCU boot 对应一个本地时间会话。 |
| `writer_epoch` | Jetson `sensor_time_bridge` 进程实例 ID。bridge 重启时变化，但 MCU 未重启时 `session_id` 不变。 |
| `local_tick` | MCU 自由运行硬件计数器的 64 位软件扩展值，不随 GNSS 或主机时间修改。 |
| `local_tick_hz` | 该会话固定的标称 tick 频率。推荐初版 1,000,000 Hz，得到 1 µs 硬件量化；不得运行中改值。 |
| `synthetic_epoch` | 每次启动使用同一个配置值，推荐 `2000-01-01T00:00:00`，即 `946684800000000000 ns`；处于 MID GPS 允许的 2000-2037 范围内且不是 ROS Time 0。 |
| `local_stamp_ns` | `synthetic_epoch_ns + floor(local_tick*1e9/local_tick_hz)`，只用整数商余数算法。1 MHz 时即 `epoch + tick*1000`。 |
| `clock_source` | 本地时钟来源枚举，不是 GNSS lock 状态。根源固定为 `MCU_HSE_FREE_RUN`；派生数据可标 `MCU_CAMERA_TRIGGER`、`MID360_SYNTHETIC_GPS_INPUT`、`HOST_RECEIVE_LOCAL`、`GNSS_UTC_INVERSE_MAPPED`。 |
| `mapping_version` | LOCAL↔UTC 拟合的原子版本。无映射为 0；每次接受新拟合递增。不能改变 LOCAL。 |
| `time_state` | `LOCAL_ONLY/ACQUIRING/LOCKED/HOLDOVER/RELOCKING`；描述 UTC 映射可用性，不描述本地钟是否运行。 |
| `time_uncertainty_ns` | 给定时间语义的保守 1σ 或明确定义上界；消息中必须同时给出定义枚举。不得把串口接收时间标成零不确定度测量时间。 |

### 3.3 64 位 tick 和溢出

推荐用一个当前空闲且确认可用的 16 位 timer，设 `PSC=71`、`ARR=65535`，在 72 MHz 输入下得到 1 MHz tick。每 65.536 ms update 一次，ISR 只递增高位计数器。

读取算法必须处理 `CNT` 和 UIF 竞争：

1. 短暂关中断或使用 seqlock；
2. 读取 overflow high、CNT、UIF，再复读 high；
3. 若 high 改变则重读；
4. 若 UIF 已置位且 CNT 位于溢出后的低半区，把 high 按 pending overflow 修正；
5. 输入捕获读取 CCR 时使用同样的 UIF/CCR 竞争规则；
6. 64 位共享变量在 Cortex-M3 上不是天然原子，禁止无保护读写。

如果最后选择 TIM4，必须先修复中断向量；若选择 TIM1，也要确认对应 capture 引脚和当前输出没有冲突。具体 timer/pin 不是本报告可凭源码定案的内容。

### 3.4 单调性和会话边界

- 同一 `session_id` 内，各传感器各自的 `header.stamp` 必须严格递增。
- 不同传感器同一物理边沿允许拥有相同 stamp，例如 1 Hz 边沿同时是一个相机触发；全局严格顺序由 `(session_id, local_stamp_ns, event_sequence)` 表示，不得为了“唯一”伪造纳秒偏移。
- MCU reset 后 local 值可以从合成 epoch 重新开始，但 `mcu_boot_id/session_id` 必须变化。所有融合消费者清队列并重置状态，录包器切新 bag。
- MCU reset 而 MID-360 未 reset 时，MID 可能观察到合成时间回跳。阶段 B 实现应把它作为硬会话切换：停止采集、重新同步/必要时重启 MID、等待新 session ready，而不是跨 session 继续融合。
- Jetson bridge 重启不应开启新 session；它从 MCU BOOT/STATUS 恢复当前 `mcu_boot_id`、tick 和 sequence，只改变 `writer_epoch`。

### 3.5 boot ID 持久化

STM32F103C8 没有硬件 RNG。建议：

- 读取 96 位 UID；
- 用专用 Flash 页做带 CRC 的 wear-levelled boot counter，每次启动只追加一次；
- `mcu_boot_id = hash64(uid || boot_counter || firmware_build_id)`；
- BOOT 帧带出 reset cause、counter、UID hash、固件版本和 timing config CRC。

若不愿写 Flash，可用带备份电源的 backup register，但必须先证明板上 VBAT 可保持；单纯使用启动 tick、未初始化 SRAM 或主机当前时间都不能保证每次启动唯一。

---

## 4. 合成本地 RMC/PPS 设计

### 4.1 当前链路的实际证据

```text
TIM3 1 Hz PWM
  └─ timer.c:221-265，PB5

TIM3 update ISR 推进 hh:mm:ss
  └─ timer.c:79-112

main loop 生成：
$GPRMC,hhmmss.00,A,2237.496474,N,11356.089515,E,
0.0,225.5,230520,2.3,W,A*CS
  └─ timer.c:140-157

USART1 9600 8N1，PA9 TX
  └─ main.c:19；usart.c:86-117

MID-360 packet time_type
  └─ /usr/local/include/livox_lidar_def.h:126-139

livox_ros_driver2 直接读取 8-byte timestamp
  └─ src/livox_ros_driver2/src/comm/pub_handler.cpp:96-138,258-269

CustomMsg.timebase/header.stamp
  └─ src/livox_ros_driver2/src/lddc.cpp:827-848

IMU header.stamp
  └─ lddc.cpp:976-1007
```

当前 `MID360_config.json` 只有网络、点类型、扫描模式和外参，没有时间同步设置：`livox_ros_driver2/config/MID360_config.json:1-41`。本地 driver 也没有调用 `SetLivoxLidarRmcSyncTime`。因此若当前 MID 确实为 `time_type=2`，同步应来自外部 PPS/RMC 接线，而不是 ROS launch/JSON；但物理线仍需核对。

Livox 协议明确：

- `time_type=0`：上电时间；
- `time_type=1`：PTP/gPTP；
- `time_type=2`：GPS 同步；
- packet timestamp 是包内第一个点时间，单位 ns；
- IMU 与点云共用同一 packet header 格式。[MID-360 communication protocol](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)

### 4.2 当前固定日期和坐标为何表现为“可工作”

已确认：

- `230520` 是 2020-05-23，处于 MID GPS 同步允许的 2000-01-01 至 2037-12-31 范围。
- 状态位为 `A`，句子有 NMEA XOR 校验。
- 旧 bag 的 LiDAR header 日期正是 2020-05-23，与该固定日期吻合。
- 官方同步原理需要 PPS rising edge 和与该 edge 对应的 GPRMC 时间。

没有证据证明 MID 会使用纬经度、航向或磁偏角来设置时间；也没有本地文档明确说这些字段可任意填写。当前坐标“未妨碍同步”只能作为经验事实，不能推导为“坐标一定被忽略”。阶段 B 初版应保留一组已验证能被当前固件接受的合法静态字段，并在协议文档中明确它们是 compatibility payload，不是 GNSS 位置。

当前固件存在明确的换日问题：`hh` 在 23 后回 0：`timer.c:98-102`，但日期永远是 `230520`：`timer.c:140-147`。连续运行跨午夜时，RMC 日期不进位，可能使 MID 时间回退一天。

### 4.3 阶段 B 的 RMC/PPS 规则

1. `synthetic_epoch = 2000-01-01 00:00:00`。
2. 每条 RMC 的 `hhmmss.ss` 和 `ddmmyy` 都从 `local_stamp_ns` 的整秒部分计算，不能维护另一套独立 hh/mm/ss 变量。
3. Gregorian 日历完整处理 30/31 天、2 月、闰年、换月和换年。
4. 状态位初版保持 `A`；talker 保持 `$GPRMC`；速度和 course 使用已验证值或明确的合法静态值；校验为 `$` 后至 `*` 前的 XOR；结束符改为标准 `\r\n`。
5. 1 PPS rising edge 的事件 stamp 必须正好是对应的 `local_stamp_ns` 整秒。
6. RMC 在该 rising edge 后发送，内容表示同一个整秒。Livox 官方定义“数据端发送与该 rising edge 对应的时间”，并给出 RMC 相对 rising edge 的发送延迟有效范围 0-900 ms、推荐 0-430 ms；9600 bps 下传输约 70 ms。[Livox GPS time synchronization](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)
7. 启动时先建立 tick/session，等待第一个明确的整秒 compare edge；edge 发生后发第一条 RMC。第一条 RMC 前的 MID 数据不得进入融合。
8. RMC 发送只在 main loop/DMA，ISR 只入队。若上条 RMC 尚未发送完，增加 TX backlog/error counter，不覆盖 pending 时间。
9. 本地 PPS、相机触发和 local tick 必须由同一个 HSE 派生；GNSS lock 不改变它们的周期。

### 4.4 当前 PPS/RMC 相位不能由源码完全证明

现有 TIM3 是 PWM2，CCR 设为 ARR/2，同时 update ISR 推进时间并置 pending：`timer.c:240-258,79-112`。RMC 实际从 main loop 发出，物理 rising edge 与 update event 的关系还受 PWM2 极性、启动 CNT/CCR、输出电路反相和 main loop 调度影响。没有示波器记录，不能断言当前 RMC 对应前一个还是后一个 PPS。

阶段 B 不再依赖这种隐含关系：以选定的 output compare rising edge 为唯一事件，在同一调度点生成 `LOCAL_PPS_OUTPUT`，再发送对应 RMC。

### 4.5 防止 MID 自动换时钟源

官方说明 MID 支持 PTP、gPTP 和 GPS，而且 PTP 优先级最高；GPS 和 PTP 同时存在时会优先 PTP。[Livox time synchronization instructions](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html)

因此：

- MID 的 PPS/RMC 物理输入永远只接 MCU synthetic local，不接真实 GNSS。
- GNSS PPS 只进 MCU 的另一个输入捕获脚。
- MID 所在网络禁止未受控 PTP/gPTP master。
- 真实 GNSS 出现/消失只改变 Jetson 的 `time_state`，不改变 MID 输入。
- driver 对融合数据要求 `time_type==2` 且 source switch counter 为 0；`time_type=0/1` 时 fail closed，不用主机时间替代。
- 周期查询 SDK 的 `local_time_now`、`last_sync_time`、`time_offset`、`time_sync_type`；这些 key 在 `/usr/local/include/livox_lidar_def.h:111-114` 存在，但当前 ROS driver 未实现查询。
- 记录 HMS 0x0404～0x0409。官方定义这些分别覆盖 PPS/GPS 异常、同步异常、低精度、GPS/PPS 丢失等。[MID-360 HMS codes](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html)

日期 rollover、RMC 丢失、PPS 丢失后设备是否 hold、多久退回 `time_type=0`、恢复时是否 step，仓库和安装头文件都不能证明，列入硬件测试。

---

## 5. MCU 事件协议

### 5.1 方案比较

| 方案 | 优点 | 缺点 | 结论 |
|---|---|---|---|
| ASCII 行 | 人眼可读、实现简单 | 带宽大、解析歧义、丢字节后边界弱、整数格式和版本演进脆弱 | 只用于调试打印 |
| CBOR | 自描述、可扩展 | F103 需要额外编码库和动态/复杂解析；仍需 framing/CRC | 不采用 |
| 固定二进制 + COBS + CRC32C | 小、确定、无动态分配；0x00 可可靠重同步；易做 golden test | 需要明确版本和字节序 | 采用 |

### 5.2 wire frame v1

串口配置建议 115200 8N1。wire frame：

```text
COBS_ENCODE(decoded_frame) + 0x00
```

`decoded_frame` 全部 little-endian：

| 字段 | 类型 | 说明 |
|---|---:|---|
| `protocol_magic` | u32 | 固定 `0x31544C53`，字节显示为 `SLT1` |
| `protocol_version` | u8 | 初版 1 |
| `message_type` | u8 | 见下表 |
| `header_length` | u16 | v1 固定头长度 |
| `payload_length` | u16 | 防止越界和版本扩展 |
| `flags` | u16 | valid、scheduled/actual、overcapture、queue overflow 等 |
| `mcu_boot_id` | u64 | 当前 boot |
| `event_sequence` | u32 | 所有事件共享的单调模 2³² 序号 |
| `local_tick` | u64 | 事件边沿 tick |
| `local_stamp_ns` | u64 | MCU 按固定公式派生 |
| `local_tick_hz` | u32 | 本会话固定 |
| `payload` | bytes | 由 message type 定义 |
| `crc32c` | u32 | 从 magic 到 payload 的 CRC32C，不含自身 |

基础帧约 46～60 bytes。按 camera 10 Hz、local PPS 1 Hz、GNSS PPS 1 Hz、status 1 Hz 估算小于 1 kB/s，115200 8N1 的有效上限约 11.5 kB/s，余量充足。

### 5.3 消息类型

| 值 | 类型 | payload |
|---:|---|---|
| 1 | `BOOT` | boot counter、reset cause、UID hash、synthetic epoch ns、firmware version、timing config CRC、capabilities |
| 2 | `LOCAL_PPS_OUTPUT` | PPS sequence、edge/polarity、scheduled tick、optional actual capture tick |
| 3 | `CAMERA_TRIGGER` | trigger sequence、edge/polarity、pulse width ticks、scheduled tick、optional loopback capture |
| 4 | `GNSS_PPS_CAPTURE` | input sequence、CCR capture、edge、overcapture flag |
| 5 | `STATUS` | current tick、TX ring depth/high-water、dropped event count、CRC/overrun counters、clock/reset flags |
| 6 | `ERROR` | error code、first affected sequence、context counters |

ISR 只构造固定大小 event record 放入 MCU ring；COBS、CRC、UART 输出全部在 main loop 或 DMA。ring 建议至少 64 条；满时保留“累计 dropped count”和首个丢失序号，在下一可发送 STATUS/ERROR 中报告，不能静默覆盖。

### 5.4 异常处理

- 丢字节/半帧：接收端丢弃至下一个 `0x00`，COBS decode 或长度失败计数。
- CRC 错：丢帧并计数，不部分使用 payload。
- 重复帧：同 `mcu_boot_id + event_sequence` 去重。
- 序号缺口：按模 2³²计算 forward gap，发布诊断；相机触发 gap 会使对应 image fail closed。
- MCU 重启：新 `mcu_boot_id`，立即结束旧 session。
- Jetson 节点重启：新 `writer_epoch`；等到 BOOT/STATUS 和至少连续三帧 sequence 后恢复当前 session。
- 串口重连：清 COBS partial buffer，不清 MCU session；先等待 BOOT/STATUS。
- 版本兼容：未知 major version fail closed；已知 header_length 可跳过新增尾字段；未知 message type 可记录后跳过。
- 消息积压：MCU 和 Jetson 都暴露 queue depth/high-water/drop；bridge 不允许无限队列。
- 字节序：wire 固定 little-endian；测试必须包含人工构造的跨平台 golden bytes。

---

## 6. LOCAL↔UTC 映射状态机

### 6.1 数值表达

外部概念仍是：

```text
t_utc = a * t_local + b
t_local = (t_utc - b) / a
```

消息和计算不直接用巨大 epoch 值求截距，以避免 double 消减精度。发布 reference-pair 形式：

```text
utc_ns = utc_reference_ns
       + a * (local_ns - local_reference_ns)
```

其中 reference 均为 `uint64`，差值用有符号 128 位或安全检查后的 64 位，斜率用 `double`。逆映射围绕相同 reference 计算。`b` 仅用于日志展示。

### 6.2 观测配对

高精度映射对必须是：

```text
MCU GNSS_PPS_CAPTURE 的 local_tick/local_stamp_ns
↔
接收机明确标识同一个 PPS 的 UTC
```

`gnss_comm/msg/GnssTimePulseInfoMsg.msg:4-6` 的本地定义说明其时间是“next time pulse”，正适合做配对，但工作空间内没有发布该消息的节点。实现前要确认实际 GNSS 接收机型号、TIM-TP 语义和发布源码。

不能用“GNSS 串口最后一字节的主机到达时间”与 PPS 做高精度配对。若只有 RMC/ZDA，必须按接收机手册确认它表示前一个还是后一个 PPS，并把串口延迟计入 uncertainty；证据不足时最多进入低精度 ACQUIRING，不宣称硬件锁定。

### 6.3 拟合

推荐默认：

- rolling window 120 对，至少 30 对和 30 s span 才可 LOCK；
- 以窗口中位 local/UTC 为 reference；
- Huber/IRLS 加权线性拟合；
- 先检查 UTC/LOCAL 单调、PPS interval 0.9～1.1 s、session 不变；
- 对接收机 reported accuracy、MCU capture quantization 和 association quality 加权；
- 初次允许 `|a-1| <= 10,000 ppm`，但持续超过硬件预期时报警；
- 输出 residual RMS、max residual、slope ppm、sample count、fit span 和外推时间；
- `time_uncertainty_ns` 至少包含 fit residual、PPS capture quantization、UTC source accuracy 和 holdover drift growth。

阈值必须由目标 GNSS 与示波器数据标定。报告中的样本数和 0.9～1.1 s 是保守初始值，不是硬件规格替代品。

### 6.4 状态机

```text
boot
  └── LOCAL_ONLY
        │ first valid associated UTC/PPS pairs
        v
      ACQUIRING
        │ min samples/span + residual/slope/quality gates pass
        v
       LOCKED
        │ PPS/UTC timeout, quality invalid, residual burst
        v
      HOLDOVER
        │ valid source returns
        v
     RELOCKING
        │ candidate window passes, atomic mapping swap/version++
        └──────────────────────────────> LOCKED
```

行为：

- `LOCAL_ONLY`：LOCAL 全部工作；mapping version 0；GNSS raw 仍录。
- `ACQUIRING`：同时维护 candidate mapping，不改任何已发布 header。GNSS PVT 可发布 raw，不给融合消费者发“可用的 LOCAL epoch”。
- `LOCKED`：发布 atomic mapping；GNSS UTC epoch 可逆映射为 LOCAL header。
- `HOLDOVER`：冻结最近有效 `a/reference pair` 继续外推 UTC，uncertainty 随失锁时间增长；LOCAL 不受影响。
- `RELOCKING`：旧 holdover mapping 继续对外，后台拟合 candidate。candidate 成熟前不切换。
- candidate 与 holdover 的 UTC 差过大、GPS week 跳变或 UTC 倒退时拒绝 candidate，保持 HOLDOVER 并报警。
- 接受 candidate 时 mapping version 加一。LOCAL header 从不重写；尚未发布的 GNSS 测量用新映射且仍需通过每 topic 单调门禁。

建议初始超时为连续丢 3 个 PPS 或 3.5 s，但最终参数应来自接收机输出率。

### 6.5 主机接收时间到 LOCAL

UWB 等无设备时间的串口需要另一条辅助映射：

```text
CLOCK_MONOTONIC_RAW(host) ↔ MCU LOCAL
```

bridge 在收到 MCU 周期事件时记录主机 monotonic raw，到达延迟作为不确定度，拟合 host monotonic→LOCAL。GNSS/UWB driver 在每次 `read()` 返回时立即记录 `CLOCK_MONOTONIC_RAW`，再转换为 `HOST_RECEIVE_LOCAL`。

这只是“主机接收时刻”的本地化，不是设备测量时间。不得把它标为硬件测量 stamp。

---

## 7. GNSS 迁移表

### 7.1 当前分类

当前 `GnssManager` 把 A～I 混在一个 FAST 类中：

| 类别 | 当前实现证据 |
|---|---|
| A 串口所有权 | `include/gnss_manager.h:138-143,188-193,268-271`；`src/gnss_manager.cpp:1297-1449` |
| B 原始行读取 | `readLoop()` 在换行时调用 `handleLine`：`gnss_manager.cpp:1388-1433` |
| C 协议解析 | `parseLine` 支持 KSXT/GGA/RMC/GSA/GST/ZDA/AGRICA/legacy JSON：`gnss_manager.cpp:608-974` |
| D 质量判断 | quality sets：`gnss_manager.h:206-225`；parser classification：`gnss_manager.cpp:674-729` |
| E UTC 测量时间 | 只有 KSXT 真正解析 UTC 到 `device_stamp`：`gnss_manager.cpp:302-338,778-787`；RMC 只读 A/V，不解析时间日期：843-858 行；ZDA 只判字段非空：898-903 行 |
| F PVT 发布 | `GnssManager` 不发布 ROS PVT；工作空间内也没有 `/ublox_driver/receiver_pvt` 的原始发布节点 |
| G LLA→ENU | manager 的 `convertMeasurement`/origin，以及独立 `gnss_adapter.cpp:627-674,710-750` |
| H 融合因子 | `applyPositionUpdateAt`：`gnss_manager.cpp:1947` 起；FAST 调用：`LIVMapper.cpp:898-920` |
| I 日志/回放 | raw/parsed/update 三类文件：`gnss_manager.h:178-183,263-297`、`gnss_manager.cpp:991-1024` |

当前串口每条行的 stamp 是读到换行符后调用 `ros::Time::now()`：`gnss_manager.cpp:1413-1433`；`baseMeasurement` 再加手工 `time_offset_s`：`gnss_manager.cpp:664-671`。这个 stamp 是主机行尾接收时间，不是 GNSS 测量时间。

另外存在一个已经独立成 executable、但仍位于 `fast_livo` 包内的 `gnss_adapter_node`：

- 订阅 `/ublox_driver/receiver_pvt`；
- 发布 `/gnss/enu_odom` 和 `/gnss/status`；
- 见 `gnss_adapter.cpp:102-126`、`launch/gnss_adapter.launch:2-12`。

它把 GPS week/tow 转 UTC 并直接写进 status/odom header：`gnss_adapter.cpp:265-282,528-545,672,710-716`。阶段 B 要改成“UTC 留作 trace，header 使用逆映射后的 LOCAL”。

`gnss_comm/msg/GnssPVTSolnMsg.msg:1-22` 没有 `std_msgs/Header`，只有 GPS week/tow 和 PVT；这使 `/ublox_driver/receiver_pvt` 可以原样保留为 raw compatibility topic，但不能单独表达 LOCAL、UTC、mapping version 和 uncertainty。

### 7.2 逐文件迁移

| 原路径 | 新包/新路径 | 操作 | 兼容接口 | 依赖变化 |
|---|---|---|---|---|
| `FAST_LIVO2/src/gnss_manager.cpp` 中 serial/open/read | `gnss_serial_driver/src/serial_reader.cpp` | 移动；分阶段先复制测试，最终从 FAST 删除 | 参数改为 `gnss_serial_driver/port` 等；同一设备只允许一个 owner | 去掉 PCL/FAST state，依赖 roscpp |
| 同文件 `parseLine` 及 NMEA/KSXT/AGRICA parser | `gnss_serial_driver/src/text_parser.cpp` | 移动并保留 parser tests | `/gnss/raw`、canonical PVT/fix | 独立标准库 |
| `FAST_LIVO2/include/gnss_manager.h` 的 `GnssMeasurement` | `gnss_serial_driver/include/.../gnss_measurement.h` | 拆分采集字段和融合字段 | raw UTC、receive local 均保留 | 不依赖 `common_lib.h` |
| `gnss_manager.cpp` quality classification | `gnss_serial_driver/src/quality.cpp` | 移动 D | `/gnss/status` canonical | 不依赖 FAST |
| `gnss_manager.cpp` origin/ENU 前处理 | `gnss_serial_driver/src/enu_adapter.cpp` | 移动 G | `/gnss/enu_odom` 保持 `nav_msgs/Odometry` | 复用 `gnss_comm` |
| `gnss_manager.cpp:1947+` EKF/state update、frame align、map pause | `FAST_LIVO2/src/gnss_fusion_consumer.cpp` | 保留/重构 H，只订阅 ROS | 不再打开串口 | FAST 增加新消息依赖 |
| `gnss_manager.cpp` raw/parsed logs | 新 driver logging | 移动 I 的采集部分 | bag 为主，文本日志可选 | 与 FAST save_path 解耦 |
| `gnss_manager.cpp` update log | FAST consumer | 保留 I 的融合部分 | 原融合诊断语义 | 无 serial |
| `FAST_LIVO2/include/gnss_adapter.h`、`src/gnss_adapter.cpp`、`src/gnss_adapter_node.cpp` | `gnss_serial_driver/include/src` | 移动 G/D，改收 `GnssPvtStamped` | `/gnss/enu_odom` 保持；节点名可保留 | 从 fast_livo 解耦 |
| `FAST_LIVO2/src/gnss_adapter_self_test.cpp` | 新包 test | 移动并扩充 LOCAL/UTC tests | 行为回归 | 无 GTSAM/PCL |
| `FAST_LIVO2/msg/GnssStatus.msg` | 新 canonical status msg | 分阶段迁移 | 见下方 datatype 风险 | 最终不让采集包依赖 FAST |
| `gnss_comm/msg/GnssPVTSolnMsg.msg` | 原地保留 | 保留，不改 MD5 | `/ublox_driver/receiver_pvt` | 新 driver 依赖 gnss_comm |
| `gnss_comm/msg/GnssTimePulseInfoMsg.msg` | 原地保留 | 保留 | 作为 UTC/PPS association 输入 | 发布节点仍缺失 |
| `FAST_LIVO2/config/gnss_adapter*.yaml`、`launch/gnss_adapter.launch` | 新包 config/launch | 移动；旧 launch 暂作 include shim | 参数名给 deprecation warning | 最终 FAST 不承载采集配置 |

### 7.3 兼容性处理

- `/ublox_driver/receiver_pvt` 保持 `gnss_comm/GnssPVTSolnMsg` 和原 MD5 `d18171357d7a159f76d4d7c0b12fb631`。
- `/gnss/enu_odom` 保持 `nav_msgs/Odometry`，但 header 从 UTC 改成 LOCAL；这是时间语义变化，必须增加诊断和 release note。
- `/gnss/status` 当前是 `fast_livo/GnssStatus`。要让 A～G 最终不依赖 FAST，就不能永久保持这个 ROS datatype。迁移期应新发 `/gnss/status_v2`，旧 adapter 继续发旧 `/gnss/status`；消费者全部升级后，将 v2 remap 回 `/gnss/status` 并移除旧接口。topic 名可以最终保持，datatype MD5 无法在“完全解耦”和“原包类型不变”之间同时满足。
- 当前 `/ublox_driver/receiver_pvt` 发布者源码不在工作空间。实现前必须找到其包/仓库，或者明确目标接收机协议后新写最小 driver；不能根据 topic 名假定它是 ublox 官方 ROS driver。

---

## 8. UWB 迁移表

### 8.1 当前分类

`UwbManager` 同样混合 A～H：

| 类别 | 当前实现证据 |
|---|---|
| A 串口/文件读取 | `include/uwb_manager.h:254-260,313-335`；`src/uwb_manager.cpp:520-650,1424-1760` |
| B 原始解析 | `parseLine`：`uwb_manager.cpp:1904-2020`，支持 `distance[id],x`、`[UWBDBG]`、pairs、values |
| C 轮次聚合 | 当前没有显式 round object；同一输入行解析出的多 anchor 共享同一 `stamp`：1908-1921、2004-2011 行 |
| D 时间戳 | serial 在行尾用 `ros::Time::now()`：1504-1532 行；replay 用文件首列：1591-1607 行 |
| E 锚点配置 | 大量 anchor、baseline、external frame 参数：`uwb_manager.cpp:949-1042` 及后续 |
| F 质量诊断 | range correction/filter/repeat filter：`uwb_manager.cpp:494-517,2022-2078` |
| G EKF/状态更新 | `applyRangeUpdateAt`：3370 行起；FAST 调用：`LIVMapper.cpp:792-896` |
| H 日志回放 | raw/update log：`uwb_manager.cpp:520-566`；file replay：1548-1884 行 |

明确问题：

- `measurement.stamp` 是串口行尾主机 ROS wall time，不是测量时刻。
- serial 路径的融合参考 `now` 仍用 `ros::Time::now()` 而不是传入的 LiDAR LOCAL：`uwb_manager.cpp:3370-3379`。
- 一行 pairs/values 的多个 anchor 已共享 stamp；多个独立 `distance[...]` 行之间没有 round ID，不能证明属于同一轮。
- 当前协议字段中没有设备时间。

### 8.2 逐文件迁移

| 原路径 | 新包/新路径 | 操作 | 兼容接口 | 依赖变化 |
|---|---|---|---|---|
| `FAST_LIVO2/src/uwb_manager.cpp` serial open/read | `uwb_serial_driver/src/serial_reader.cpp` | 移动 A | 新 `/uwb/raw` | 去掉 FAST/PCL |
| 同文件 `parseLine` | `uwb_serial_driver/src/parser.cpp` | 移动 B | 新 `/uwb/ranges` | 标准库/regex |
| `filterRepeatedRanges`、range scale/limit/bias | `uwb_serial_driver/src/range_filter.cpp` | 移动 F | status 统计保持 | driver-owned 物理质量 |
| file replay loader/scheduler | `uwb_serial_driver/src/replay_source.cpp` | 移动 A/H raw | serial/file 参数保持能力 | 不依赖 laserMapping |
| anchor ID、enable、range bias、合法量程 | 新 driver config | 移动 E 的采集部分 | `/uwb/status` | 与 world frame 解耦 |
| anchor world position、frame alignment、baseline initialization | FAST consumer config | 保留 E 的融合几何部分 | 原算法参数可迁移命名空间 | 属于 G，不应放 driver |
| `applyRangeUpdateAt` 及全部 state/covariance/relocalization | `FAST_LIVO2/src/uwb_fusion_consumer.cpp` | 保留/重构 G，只订阅 `/uwb/ranges` | 融合结果不变 | FAST 增加 uwb msg 依赖 |
| raw log | driver/bag | 移动 H raw | `/uwb/raw` 可完整重放 | 与 FAST save_path 解耦 |
| update/debug log | FAST consumer | 保留 H fusion | attempt/result 语义保持 | 无 serial |
| `include/uwb_manager.h` | 拆为 driver measurement 和 FAST fusion state | 拆分 | 不再共享 `common_lib.h` | 边界清晰 |
| `src/uwb_manager_self_test.cpp` | parser/filter tests 移新包；EKF tests 留 FAST | 拆分 | 保留旧回归 | 各测各自依赖 |

### 8.3 UWB 时间语义

- 有设备 measurement time：映射到 LOCAL，标 `DEVICE_TIME_MAPPED_LOCAL`。
- 当前无设备 measurement time：在 `read()` 返回时立即采 `CLOCK_MONOTONIC_RAW`，转换到 LOCAL，标 `HOST_RECEIVE_LOCAL`，并带 serialization/scheduling/mapping uncertainty。
- 不能把行尾到达时间字段命名为 `measurement_time`；应叫 `host_receive_local_stamp`。
- 一条 batch/pairs line 是一个 round，所有 anchor 共享同一 `header.stamp` 和 `round_id`。
- 单独多行只有在协议提供相同 round counter 时才能合并。当前格式没有该证据，因此初版每行独立；不使用时间窗口猜轮次。
- replay `preserve` 模式保留原 session/stamp，只用于离线；`rebase` 模式可按相对时间放进当前 session，但必须标 `REPLAY_REBASED`，不能冒充原测量时间。

---

## 9. ROS 包与消息设计

### 9.1 最终包结构

```text
sensor_time_msgs
  只放跨传感器的时间/事件消息和枚举

sensor_time_bridge
  MCU 串口唯一 owner、COBS/CRC、tick 校验、事件 ring、
  host-monotonic↔LOCAL、LOCAL↔UTC mapping、状态机和诊断

gnss_serial_driver
  GNSS 串口/协议/原始发布/PVT/UTC/PPS association/质量/ENU
  不含 FAST state update

uwb_serial_driver
  serial + file replay、parser、round、range quality、raw/status
  不含 FAST state update

sensor_recording_bringup
  无 FAST 的采集 launch、ready gate、rosbag topic profile、udev 文档

fast_livo
  只保留 GNSS/UWB fusion consumer 和后端
```

### 9.2 common 消息

`sensor_time_msgs/McuEvent.msg`：

```text
std_msgs/Header header             # 事件 LOCAL_SENSOR_TIME
uint64 session_id
uint64 mcu_boot_id
uint64 writer_epoch
uint32 event_sequence
uint8 message_type
uint64 local_tick
uint32 local_tick_hz
uint64 local_stamp_ns
uint32 flags
uint32 source_sequence             # trigger/PPS 自身序号
uint64 host_receive_monotonic_ns
uint64 time_uncertainty_ns
```

`sensor_time_msgs/TimeMapping.msg`：

```text
std_msgs/Header header             # 发布时刻 LOCAL
uint64 session_id
uint64 mcu_boot_id
uint64 writer_epoch
uint32 mapping_version
uint8 time_state
uint64 local_reference_ns
uint64 utc_reference_ns
float64 utc_ns_per_local_ns
float64 slope_ppm
uint64 valid_from_local_ns
uint64 fit_span_ns
uint32 sample_count
float64 residual_rms_ns
float64 residual_max_ns
uint64 time_uncertainty_ns
uint32 flags
```

`sensor_time_msgs/TimeStatus.msg`：

```text
std_msgs/Header header             # 当前 LOCAL
uint64 session_id
uint64 mcu_boot_id
uint64 writer_epoch
uint32 mapping_version
uint8 time_state
uint8 clock_source
bool local_ready
bool utc_mapping_valid
uint32 last_event_sequence
uint64 event_gap_count
uint64 crc_error_count
uint64 frame_error_count
uint64 mcu_dropped_event_count
uint64 bridge_queue_depth
uint64 time_uncertainty_ns
string detail
```

`sensor_time_msgs/RawSerialFrame.msg`：

```text
std_msgs/Header header             # HOST_RECEIVE_LOCAL
uint64 session_id
uint64 writer_epoch
uint64 source_sequence
uint64 host_receive_monotonic_ns
uint64 time_uncertainty_ns
uint8 stamp_source
string device
string protocol
uint8[] data
```

GNSS 专用：

- `gnss_serial_driver/GnssTimeObservation.msg`：associated local PPS ns、UTC ns、association type、receiver time system、validity、uncertainty、mapping version。
- `gnss_serial_driver/GnssPvtStamped.msg`：`Header` 为测量 epoch 逆映射后的 LOCAL；内含原 `gnss_comm/GnssPVTSolnMsg`、UTC ns、LOCAL ns、mapping version、stamp source、uncertainty、`valid_for_fusion`。
- `gnss_serial_driver/GnssStatus.msg`：保留现有质量字段，增加 session/mapping/UTC trace。

UWB 专用：

- `uwb_serial_driver/UwbRange.msg`：anchor ID、raw/corrected range、bias、diag、valid/reject reason。
- `UwbRangeArray.msg`：`Header` 为整轮共同 stamp；session、round ID、stamp source、uncertainty、ranges[]。
- `UwbStatus.msg`：串口、parser、round、drop、repeat、queue counters。

### 9.3 话题语义

| 话题 | 消息 | `header.stamp` | 可直接融合 | 录 bag |
|---|---|---|---:|---:|
| `/sensor_time/mcu_trigger` | `McuEvent` CAMERA_TRIGGER | MCU 边沿 LOCAL | 是，供相机匹配 | 必录 |
| `/sensor_time/mcu_pps` | `McuEvent` LOCAL_PPS_OUTPUT | MCU PPS LOCAL | 映射/诊断 | 必录 |
| `/sensor_time/gnss_pps_capture` | `McuEvent` GNSS_PPS_CAPTURE | 捕获边沿 LOCAL | 映射输入 | 必录 |
| `/sensor_time/mapping` | `TimeMapping`，latched | 发布时 LOCAL | 元数据，不作传感器量测 | 必录 |
| `/sensor_time/status` | `TimeStatus`，latched+1 Hz | 当前 LOCAL | 门禁 | 必录 |
| `/gnss/raw` | `RawSerialFrame` | HOST_RECEIVE_LOCAL | 否 | 必录 |
| `/gnss/time_observation` | `GnssTimeObservation` | associated PPS LOCAL | 仅映射器 | 必录 |
| `/ublox_driver/receiver_pvt` | `gnss_comm/GnssPVTSolnMsg` | 无 Header；内部 GNSS week/tow | 否，raw compatibility | 必录 |
| `/gnss/pvt_local` | `GnssPvtStamped` | GNSS epoch→LOCAL | quality/lock 通过后可用 | 必录解析 profile |
| `/gnss/enu_odom` | `nav_msgs/Odometry` | GNSS epoch→LOCAL | 是 | 必录解析 profile |
| `/gnss/status` | 新 status | callback/measurement LOCAL，字段明确 | 门禁 | 必录 |
| `/uwb/raw` | `RawSerialFrame` | HOST_RECEIVE_LOCAL | 否 | 必录 |
| `/uwb/ranges` | `UwbRangeArray` | DEVICE_MAPPED 或 HOST_RECEIVE_LOCAL | 后者只能带 uncertainty 使用 | 必录解析 profile |
| `/uwb/status` | `UwbStatus` | 当前 LOCAL | 门禁 | 必录 |

---

## 10. 无 FAST-LIVO 采集 launch 设计

新增：

```text
sensor_recording_bringup/launch/record_sensors.launch
sensor_recording_bringup/config/topics_raw.yaml
sensor_recording_bringup/config/topics_parsed.yaml
sensor_recording_bringup/scripts/session_recorder.py
```

launch 顶层强制：

```xml
<param name="/use_sim_time" value="false"/>
```

参数：

```text
enable_time_bridge:=true
enable_livox:=true
enable_camera:=true
enable_gnss:=true
enable_gnss_adapter:=true
enable_uwb:=true
enable_diagnostics:=true
enable_recording:=true

mcu_port:=/dev/sensor_mcu
mcu_baud:=115200
gnss_port:=/dev/gnss
gnss_baud:=<hardware-confirmed>
uwb_source:=serial|file
uwb_port:=/dev/uwb
uwb_baud:=115200
uwb_replay_file:=

record_profile:=raw|parsed|both
output_dir:=...
bag_name:=sensors
```

启动顺序：

1. `sensor_time_bridge` 为 required；收到 BOOT 和连续有效事件后 `local_ready=true`。
2. Livox、camera、GNSS、UWB 可独立开关；驱动启动不等待 GNSS lock。
3. GNSS raw 从第一字节开始保留；LOCAL_ONLY/ACQUIRING 数据不丢。
4. `session_recorder` 只等待 `local_ready`，不等待 UTC mapping；随后按显式 topic list 启动 rosbag。
5. MCU session 改变时 recorder 关闭当前 bag、写完索引并开新 bag；融合消费者不在此 launch。
6. 不启动 `/laserMapping`、`fastlivo_mapping`、GNSS/UWB fusion consumer 或 RTK backend。

现有 `FAST_LIVO2/launch/sensors.launch:1-57` 已证明“不启动 laserMapping 直接启 Livox+camera”的基本模式可复用，但它：

- 放在算法包内；
- 使用全局参数和 `rosbag record -a`；
- 没有 MCU bridge、GNSS、UWB、ready gate 或 session rotation；
- 默认启动 RViz；
- 不适合作为最终 bringup。

新 launch 应 include 各传感器自己包内的 launch，不复制 driver 参数。bag 不默认 `-a`，避免把大体积 debug/图像派生话题意外全录。

---

## 11. 相机、LiDAR、IMU 和后端的 LOCAL 接入点

### 11.1 Camera

阶段 A 当前仍读取 `LIDAR_BASE_TIME_LEGACY` 单槽：

- 取帧：`mvs_ros_driver/src/grab_trigger.cpp:553-563`
- SDK metadata：576-581 行
- 读 shared state 并赋 header：585-615、689-703 行
- trigger 模式无效时间 fail closed，不回退 now：624-639 行
- 默认 `LatestImagesOnly`、SDK buffer 3：756-792、900-926 行

SDK 头文件确认：

- `nFrameNum`、`nDevTimeStampHigh/Low`、`nHostTimeStamp`：`/opt/MVS/include/CameraParams.h:416-426`
- `nFrameCounter`、`nTriggerIndex`：444-445 行。

阶段 B：

1. bridge 保留按 `CAMERA_TRIGGER.source_sequence` 索引的 ring，不再提供“最新槽”。
2. camera 收帧后用 `nTriggerIndex` 对 MCU trigger sequence 建立模 2³² offset，并至少以 3 个连续 frame/trigger 样本确认。
3. 匹配成功后，image header = 对应 MCU trigger `local_stamp_ns`。
4. frame/trigger gap、ring miss、session change、offset 未锁定时丢帧，不用 latest stamp 或 `ros::Time::now()`。
5. camera restart 会重置 trigger index，必须重新 acquisition；MCU session 不一定变化。
6. `nDevTimeStampHigh/Low` 用于验证帧间隔、丢帧和 camera clock affine consistency，不作为初版主时间源。
7. 若目标相机实际不提供有效 `nTriggerIndex`，FIFO 在 `LatestImagesOnly` 丢帧时不可靠。此时需要 camera exposure output 回捕 MCU，或对 camera device time 做经过硬件验证的映射；不能退回单槽。

结论：阶段 B 必须把单槽改成有 sequence 的事件 ring；增加 trigger sequence；使用相机设备时间做诊断和备用映射，但主 header 优先使用 MCU trigger edge。

### 11.2 MID-360 LiDAR/IMU

driver 当前在 `time_type=1/2` 时直接使用设备 8-byte ns；`time_type=0` 时改用 `system_clock::now()`：`pub_handler.cpp:258-269`。阶段 B 对融合 topic 必须：

- 只接受 `time_type=2` synthetic LOCAL；
- `CustomMsg.timebase == header.stamp.toNSec()`；
- IMU 同样要求 `time_type=2`；
- `time_type` 切换、0、1 均 fail closed 并诊断，不能 fallback host；
- Stage A 的 integer `fromNSec` 和 startup ready gate 保留。

### 11.3 `/backend/livo_odom_raw`

当前 raw backend odom 已从 `LidarMeasures.last_lio_update_time` 赋 stamp：`FAST_LIVO2/src/LIVMapper.cpp:669-689`。只要 LiDAR 是 LOCAL，它自然属于 LOCAL。必须保留其严格单调 drop 逻辑，并在 session change 时重置/重启 backend。

注意 FAST 当前仍对相机接收时间加 `img_time_offset`：`LIVMapper.cpp:2075`，配置中常见 `0.1`。阶段 B 完成真实 trigger mapping 后，这个偏移会成为重复补偿；实现阶段必须通过光电/运动标定决定是否归零，但本轮不改配置。

---

## 12. 预计修改文件

### 新包/新文件

```text
src/sensor_time_msgs/{CMakeLists.txt,package.xml,msg/*.msg}
src/sensor_time_bridge/{CMakeLists.txt,package.xml,include/**,src/**,test/**,launch/**}
src/gnss_serial_driver/{CMakeLists.txt,package.xml,include/**,src/**,msg/**,test/**,launch/**,config/**}
src/uwb_serial_driver/{CMakeLists.txt,package.xml,include/**,src/**,msg/**,test/**,launch/**,config/**}
src/sensor_recording_bringup/{CMakeLists.txt,package.xml,launch/**,config/**,scripts/**}
```

### STM32

```text
src/stm32_timersync-open/USER/PWM.uvprojx
src/stm32_timersync-open/USER/main.c
src/stm32_timersync-open/USER/stm32f10x_it.c/.h
src/stm32_timersync-open/HARDWARE/TIMER/timer.c/.h
src/stm32_timersync-open/SYSTEM/usart/usart.c/.h
src/stm32_timersync-open/CORE/<correct medium-density startup>
src/stm32_timersync-open/HARDWARE/TIMEBASE/*
src/stm32_timersync-open/HARDWARE/PROTOCOL/*
```

### 现有 ROS 包

```text
src/livox_ros_driver2/src/comm/pub_handler.cpp
src/livox_ros_driver2/src/lddc.cpp/.h
src/livox_ros_driver2/CMakeLists.txt
src/livox_ros_driver2/package*.xml

src/mvs_ros_driver/src/grab_trigger.cpp
src/mvs_ros_driver/include/mvs_ros_driver/timestamp_monitor.h
src/mvs_ros_driver/CMakeLists.txt
src/mvs_ros_driver/package.xml

src/FAST_LIVO2/include/LIVMapper.h
src/FAST_LIVO2/src/LIVMapper.cpp
src/FAST_LIVO2/include/gnss_manager.h
src/FAST_LIVO2/src/gnss_manager.cpp
src/FAST_LIVO2/include/uwb_manager.h
src/FAST_LIVO2/src/uwb_manager.cpp
src/FAST_LIVO2/CMakeLists.txt
src/FAST_LIVO2/package.xml
src/FAST_LIVO2/config/*.yaml（仅迁移阶段）
src/FAST_LIVO2/launch/*.launch（仅兼容 include/shim）
```

`gnss_comm/GnssPVTSolnMsg.msg` 和 `livox_ros_driver2/CustomMsg.msg` 不建议修改，以保留现有 MD5 和 bag 兼容。

---

## 13. 风险和兼容性

| 风险 | 影响 | 控制 |
|---|---|---|
| MCU startup/vector 与 C8 不匹配 | TIM4/USART3 IRQ 不工作或进默认 handler | 实现前改正确 startup，Keil clean build，核对 map/vector |
| MCU HSE 实际频率/温漂未知 | LOCAL 速率偏差、holdover UTC 误差 | 频率计/示波器标定；GNSS affine 吸收 UTC 映射，不 step local |
| MID 自动选 PTP | LiDAR 离开 LOCAL | 隔离 PTP master，监控 time_type/query/HMS，非 2 fail closed |
| RMC/PPS 相位错误 | MID 整秒错一秒 | compare edge 单源生成；四通道示波器测试 |
| MCU reset | local value 回到 epoch；MID 可能回跳 | 新 session、bag rotation、消费者 reset、MID resync |
| 相机 trigger index 无效 | 无法可靠一一对应 | 实机验证；必要时 exposure output loopback 或 device time mapping |
| ROS topic datatype 迁移 | 旧 `/gnss/status` consumer 连接失败 | v2 并行期、逐 consumer 切换；PVT/odom 保持原类型 |
| 无 GNSS publisher 源码 | 无法保证 TIM-TP/PVT 语义 | 找回包或先确认接收机协议再实现 |
| UWB 无 round ID/device time | 多 anchor 轮次和测量时刻不精确 | 不猜 round；HOST_RECEIVE_LOCAL + uncertainty；推动固件增加 round/time |
| 旧 bag 无 session/mapping topic | 无法恢复完整 Stage B 元数据 | 标为 LEGACY；只做离线分析，不伪造 session |
| `img_time_offset=0.1` | Stage B 后重复时间补偿 | 硬件标定后单独迁移，保留回滚参数 |
| USB 设备枚举变化 | 串口串用 | udev symlink、进程锁、启动前 owner 检查 |

旧 bag 不需要“修复”才能保留原始证据，也不应覆盖。若为了算法重放生成修正版，必须输出新 bag、记录转换脚本、mapping version 和 provenance；原 bag 保持只读。

---

## 14. 自动测试方案

### 14.1 本轮实际回归结果

在原有 `catkin_make` 工作空间中执行：

```text
catkin_make -j2
```

结果：全工作空间 100% 构建成功，包括 `livox_ros_driver2_node`、`grabImgWithTrigger`、`fastlivo_mapping`、GNSS/UWB self-test binaries。

阶段 A tests：

```text
catkin_make run_tests_livox_ros_driver2 run_tests_mvs_ros_driver
catkin_test_results build/test_results --all
```

结果：

```text
livox_ros_driver2/gtest-shared_timestamp_state_test.xml: 12 tests
mvs_ros_driver/gtest-timestamp_monitor_test.xml: 6 tests
Summary: 18 tests, 0 errors, 0 failures, 0 skipped
```

GTest 控制台实际 testcase 为 Livox 6、MVS 3；`catkin_test_results` 对含 aggregate suite 的 XML 汇总显示 18。阶段 A 的验收口径是现有 18 项汇总，必须保持 0 failure。

另执行：

```text
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_analyze_sensor_timestamps.py
```

5 项通过。

消息 MD5：

```text
livox_ros_driver2/CustomMsg     e4d6829bdfe657cb6c21a746c86b21a6
gnss_comm/GnssPVTSolnMsg       d18171357d7a159f76d4d7c0b12fb631
```

### 14.2 阶段 B 新增最小测试矩阵

MCU/协议 host tests：

- 16 位 overflow、pending UIF、capture/overflow race；
- 64 位 tick 单调和 1 MHz integer ns；
- 2000-2037 Gregorian rollover、闰年、午夜；
- RMC golden sentences/checksum/CRLF；
- COBS golden bytes、半帧、随机丢字节、CRC 错、未知版本/type；
- sequence wrap/gap/duplicate、BOOT/session change、ring overflow。

bridge/mapping：

- synthetic 0、±100、±3000 ppm；
- outlier、UTC week jump、PPS drop、holdover、relock；
- `LOCAL_ONLY→ACQUIRING→LOCKED→HOLDOVER→RELOCKING→LOCKED`；
- 任意状态切换前后 local headers 不变、不倒序；
- affine forward/inverse round-trip 和大 epoch 精度；
- bridge restart 保持 session、writer epoch 变化；
- MCU restart 产生新 session 并让旧 queue fail closed。

Camera：

- trigger ring 命中、gap、wrap、camera restart offset reacquire；
- LatestImagesOnly 丢帧时仍按 trigger index 对应；
- shared legacy 回归保留到切换完成；
- Stage A 现有 18 项继续通过。

GNSS/UWB：

- 当前 KSXT/GGA/RMC/GSA/GST/ZDA/AGRICA parser corpus；
- UTC 与 LOCAL 同时追踪；
- `/ublox_driver/receiver_pvt` serialization MD5 不变；
- UWB 一行多 anchor 同 round/stamp；
- 无 round ID 的多行不错误聚合；
- HOST_RECEIVE_LOCAL 明确 stamp source/uncertainty；
- serial owner lock 防止两个进程打开同一设备；
- replay preserve/rebase 语义。

launch/integration：

- `rostest` 启动 bringup 后断言没有 `/laserMapping`、FAST backend；
- `/use_sim_time=false`；
- GNSS 未锁时 recorder 仍启动并录 raw；
- 禁用任一传感器不影响其他节点；
- session change 自动切 bag；
- 10 分钟 synthetic stream：所有融合 topic 同 session、各 stream 严格单调、无 host-now fallback。

---

## 15. 硬件测试方案

### 15.1 MCU 和相位

四通道示波器/逻辑分析仪同时看：

```text
CH1 Camera trigger
CH2 MID local PPS
CH3 USART1 RMC TX
CH4 GNSS PPS input
```

测：

- camera 100 ms period、PPS 1 s period、pulse width、jitter；
- RMC start 相对 local PPS rising edge，确认同一整秒且 0-430 ms；
- GNSS PPS input capture 与物理 edge 的偏差；
- 连续 30 min/2 h 的 MCU HSE ppm 和温漂；
- TIM overflow race 压力。

### 15.2 MID-360

- 冷启动、先 PPS 后 RMC、先 RMC 后 PPS、正常顺序；
- 记录 packet `time_type`，必须稳定为 2；
- 比较 packet timestamp 与 MCU local PPS：整秒相位、slope、无 step；
- RMC 午夜、月底、2 月、年末换日；
- 拔 RMC、拔 PPS、恢复，记录退锁时间和恢复行为；
- 网络出现/撤走 PTP master，验证优先级和 fail-closed；
- 查询 `local_time_now/last_sync_time/time_offset/time_sync_type`；
- 记录 HMS 0x0404～0x0409；
- MCU reset 时验证 MID 行为并落实 session/MID resync 流程。

### 15.3 Camera

- 确认目标型号每帧 `nTriggerIndex`、`nFrameNum`、device timestamp 的有效性和 wrap；
- 人工造成 USB 拥塞/CPU 高负载/resize 重负载，验证 image N 仍匹配 trigger N；
- 主动丢帧，验证 ring 不发生 N→N+1 错配；
- camera 单独重启，验证 offset reacquire；
- 光电二极管或 camera exposure output 测 trigger edge 到曝光起点/中点，决定 header 应表示 trigger、exposure start 还是 exposure midpoint，并据此处理 `img_time_offset`。

### 15.4 GNSS/UWB/室内外循环

- GNSS 接收机型号、PPS 极性/电平、TIM-TP “next pulse”语义；
- 室内启动至少 10 min：LOCAL_ONLY 全传感器连续；
- 出室外：ACQUIRING→LOCKED，LOCAL 无 step；
- 回室内：HOLDOVER，测 UTC uncertainty 增长；
- 再出室外：RELOCKING→LOCKED，LOCAL 无重复/倒序；
- 注入错误 week、UTC jump、PPS 缺失，mapping 不污染 local；
- UWB 确认实际协议是否含 round ID/device time；若无，测串口 serialization 和主机调度延迟上界；
- 同时拔插 MCU/GNSS/UWB USB-UART，验证 udev 名、owner lock、重连、bag 保留。

---

## 16. 尚未确认的信息与进入实现阶段条件

### 16.1 尚未确认

1. 实物 MCU 的完整料号/封装、板级原理图、HSE 晶体型号和负载电容。
2. PA2/PA3、PB10/PB11、TIM1/TIM4 capture GPIO、PA11/PA12 是否引出/空闲。
3. 当前 PA1、PB5、PA9 的实际电缆终点、电平和是否反相。
4. 当前 TIM3 PWM2 的真实 rising edge 与 RMC 发送相位。
5. MID-360 固件版本；RMC 经纬度字段是否完全忽略；丢 PPS/RMC、换日、重同步的具体状态机。
6. GNSS 接收机型号、数据协议、PPS/TIM-TP 输出和 `/ublox_driver/receiver_pvt` 发布节点源码。
7. UWB 型号、协议、是否有 round ID、device measurement time、每轮 anchor 输出形式。
8. 相机型号实测是否可靠提供 `nTriggerIndex`，以及该 index 对应 trigger、曝光开始还是 frame。
9. Keil 当前工程能否 clean rebuild；现存 map 是旧产物。

### 16.2 明确决策

| 项目 | 决策 |
|---|---|
| 单槽共享内存改为环形队列 | **需要**。最终不用 legacy latest slot；按 trigger sequence 的 event ring。 |
| 增加触发序号 | **需要**。MCU trigger sequence + global event sequence。 |
| 使用相机设备时间 | **需要用于验证/备用映射**；主 header 首选 MCU trigger event。 |
| 修正 MCU tick 频率 | **需要建立独立 64 位 tick**；现有 10 Hz/1 Hz 理论配置本身正确，不能把旧 bag 2990 ppm 当作晶振校正依据。 |
| 引入 PPS/UTC 映射 | **需要**。只建立 affine mapping，不 step local。 |
| 修改 LiDAR driver | **需要**。保留整数 ns，增加 LOCAL source 门禁、查询/诊断和非 2 fail closed。 |
| 修改 LiDAR 消息定义 | **不需要**。保留 CustomMsg MD5。 |
| 修复旧 bag | **不需要且禁止覆盖**。若生成派生 bag，必须另存并记录 provenance。 |
| 第二 USB-UART | **高概率需要**。若 USART2 引出，最小风险方案是独立 MCU event USB-UART；USB CDC 仅板级确认后考虑。 |

### 16.3 是否具备进入实现阶段的条件

**结论：架构已经具备进入“接口/软件骨架实现”的条件，但还不具备直接上硬件闭环或一次性替换 legacy 链路的条件。**

可以立即开始、且不依赖未知硬件的工作：

1. 定稿 ROS 消息和 wire protocol v1；
2. 实现 host 侧 COBS/CRC/parser、mapping state machine 和纯单元测试；
3. 拆分 GNSS/UWB parser 与 fusion consumer；
4. 新建独立 bringup 和 recorder；
5. 保持阶段 A legacy 并行，不切换 camera header。

开始 MCU 固件和硬件接线前必须关闭：

1. 拿到原理图/实物 pin audit；
2. 修正 Keil medium-density startup/vector 和显式 device define；
3. 确认 USART2/捕获 timer 引脚；
4. 确认 GNSS 接收机/TIM-TP、UWB round/time、相机 trigger index；
5. 准备示波器验收。

最终切换条件：

- 新链路所有自动测试通过，阶段 A 汇总 18 项保持 0 failure；
- MID `time_type` 全程为 2 且与 MCU LOCAL affine slope/phase 合格；
- Camera trigger index 与 MCU sequence 经丢帧/积压测试仍一一对应；
- indoor→outdoor→indoor→outdoor 全状态机测试中，所有融合 header 无 step、重复或倒序；
- legacy `LIDAR_BASE_TIME_LEGACY` 仅保留兼容诊断，不再作为相机最终时间源。
