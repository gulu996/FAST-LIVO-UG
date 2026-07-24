# UWB 串口控制线与录包等待状态修复报告

日期：2026-07-24  
工作空间：`/home/gulu/catkin_ws`

## 1. 根因分析

### 1.1 UWB 停止输出

实际调用链为：

```text
uwb_serial_node.py::_run_serial()
  -> exclusive_serial.py::open_locked()
  -> os.open(O_RDWR | O_NOCTTY | O_NONBLOCK)
  -> fcntl.flock(LOCK_EX | LOCK_NB)
  -> tty.setraw / termios.tcsetattr
  -> os.read()
```

原 `open_locked()` 只配置了串口格式、波特率、`VMIN` 和 `VTIME`，没有在打开后恢复设备要求的 `DTR=True、RTS=False`。USB 串口驱动在 `open()` 时可能改变 modem control lines，因此能够解释“独立 pyserial 程序正常、ROS 节点获得端口后模块停止输出”的现场现象。

UWB 和 GNSS 均使用 advisory `flock`。两者指向同一设备节点时，后打开的一方会收到独占冲突；本次没有修改 GNSS。

### 1.2 WAIT_LOCAL 误导

`SessionRecorder` 原先只在初始化时发布 `WAIT_LOCAL`。当 LOCAL 已就绪但 session 无效或 required topics 缺失时，没有重新发布更具体的等待原因，所以 `/uwb/raw` 缺失仍显示 `WAIT_LOCAL`。

原 `required_topics` 只保存在 Python 内存，没有写入 ROS 参数服务器。

## 2. 修改文件

UWB：

- `src/uwb_serial_driver/src/uwb_serial_driver/exclusive_serial.py`
- `src/uwb_serial_driver/scripts/uwb_serial_node.py`
- `src/uwb_serial_driver/launch/uwb_serial.launch`
- `src/uwb_serial_driver/test/test_exclusive_serial.py`（新增）

Recorder：

- `src/sensor_recording_bringup/CMakeLists.txt`
- `src/sensor_recording_bringup/launch/record_all.launch`
- `src/sensor_recording_bringup/launch/sensors_only.launch`
- `src/sensor_recording_bringup/scripts/session_recorder.py`
- `src/sensor_recording_bringup/src/sensor_recording_bringup/recorder_gate.py`
- `src/sensor_recording_bringup/test/test_recorder_gate.py`
- `src/sensor_recording_bringup/test/test_session_recorder.py`（新增）
- `src/sensor_recording_bringup/test/software_simulation_test.py`

没有修改 GNSS 协议或驱动、LOCAL_SENSOR_TIME、FAST-LIVO2 融合、Livox、相机、STM32、消息定义或 bag。

## 3. DTR/RTS 参数传递链

```text
record_all.launch
  uwb_dtr=true
  uwb_rts=false
    -> sensors_only.launch
       uwb_dtr / uwb_rts
         -> uwb_serial.launch
            dtr / rts
              -> uwb_serial_driver 节点私有参数 ~dtr / ~rts
                 -> UwbSerialNode.dtr / UwbSerialNode.rts
                    -> open_locked(port, baud, dtr, rts)
```

默认值：

```text
~dtr = true
~rts = false
```

端口和波特率仍由 `uwb_port`、`uwb_baud` 控制，没有硬编码 `/dev/ttyUSB1`。

成功打开后的日志格式：

```text
UWB owns /dev/ttyUSB1 at 115200 baud, DTR=True RTS=False
```

每次重连重新调用 `open_locked()`，所以每次重连都会重新配置 DTR/RTS。读取循环中没有 ioctl，也不会反复切换控制线。

## 4. ioctl 方案及选择理由

采用 Linux `TIOCMGET/TIOCMSET`：

1. 保留已有 `os.open + termios + flock + os.read` 实现；
2. 不新增 pyserial 包依赖；
3. 不改变当前非阻塞读取、超时、重连和独占语义；
4. 修改集中在串口完成 termios 配置后的一个步骤。

`TIOCM_DTR` 和 `TIOCM_RTS` 通过读—改—写设置。系统缺少相关 ioctl，或 USB 串口驱动返回 `ENOTTY`/其他错误时：

- 异常信息包含端口、目标 DTR/RTS 和底层原因；
- `open_locked()` 在重新抛出前关闭 fd；
- 节点通过现有串口错误日志输出错误并按原语义重连；
- 不会静默忽略控制线失败。

没有改用 pyserial，因为这会重写已工作的 fd 生命周期和读取链，并要求增加包级依赖；对于仅缺少两根控制线配置的问题没有必要。

## 5. Recorder 状态机

等待状态现在由 `RecorderGate.status()` 计算：

```text
local_ready=false
  -> WAIT_LOCAL

local_ready=true, session_id=0
  -> WAIT_SESSION

LOCAL/session 有效，但 required topics 不完整
  -> WAIT_TOPICS missing=/topic1,/topic2

全部门槛满足
  -> READY
  -> 启动 rosbag
  -> RECORDING session=<session_id>
```

缺失话题按字典序排序，因此输出稳定。

以下情况发布 `ERROR`：

- 输出目录无法创建或不可写；
- `subprocess.Popen()` 启动 rosbag 失败；
- 已启动的 rosbag 子进程异常退出。

已有 session 轮转行为和额外的 `ROTATING` 状态保留。GNSS fix、RTK fix、UTC mapping 仍不是录制门槛。

## 6. effective_required_topics

启动时执行：

```text
rosparam set /session_recorder/effective_required_topics <最终列表>
```

并打印：

```text
Recorder required topics: /sensor_time/events, /uwb/raw
```

自动生成规则：

- 基础门槛始终包含 `/sensor_time/events`；
- `enable_livox=true` 增加 `/livox/lidar`；
- `enable_camera=true` 增加 `/left_camera/image`；
- `enable_gnss=true` 增加 `/gnss/raw`；
- `enable_uwb=true` 增加 `/uwb/raw`；
- 已关闭的传感器不进入 required topics；
- `record_profile` 只选择 rosbag 录制集合，不扩展 gate；
- parsed topics 不会因为 `record_profile=both` 自动成为 gate。

UWB-only 的实际自动门槛：

```yaml
- /sensor_time/events
- /uwb/raw
```

GNSS-only 的实际自动门槛：

```yaml
- /sensor_time/events
- /gnss/raw
```

显式提供 `~required_topics` 时继续保留原有覆盖语义，并进行稳定去重。

## 7. 新增和扩展测试

### UWB 控制线

`test_exclusive_serial.py` 覆盖：

- 默认 DTR=true、RTS=false；
- DTR/RTS 覆盖值；
- 每次 reopen/reconnect 重新配置控制线；
- ioctl 失败关闭 fd，错误包含端口；
- 读取循环每个连接只调用一次 `open_locked()`；
- 读取循环没有 ioctl，`finally` 中关闭 fd；
- 三层 launch 默认值和覆盖参数贯通。

硬件 ioctl 全部使用 mock，没有访问真实 `/dev/ttyUSB*`。

### Recorder

`test_recorder_gate.py` 覆盖：

- `WAIT_LOCAL`；
- `WAIT_SESSION`；
- 单个和多个缺失话题；
- 缺失列表稳定排序；
- 全部话题到达后的 `READY`；
- UWB-only 和 GNSS-only required topics；
- parsed topics 不影响 raw gate；
- 显式 required topics 稳定去重。

`test_session_recorder.py` 覆盖：

- `READY` 在 `RECORDING` 前发布；
- rosbag 启动失败进入 `ERROR`；
- rosbag 异常退出进入 `ERROR`。

simulation rostest 验证：

- LOCAL_ONLY 模式继续录制；
- GNSS/UWB file 模式接口未破坏；
- `/session_recorder/effective_required_topics` 实际写入参数服务器；
- `record_profile=both` 不要求 parsed topics 先出现。

## 8. 编译结果

执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/gulu/catkin_ws
catkin_make \
  -DROS_EDITION=ROS1 \
  -DCMAKE_BUILD_TYPE=Release \
  -j2
```

结果：成功，全部 12 个 catkin 包完成构建。

构建环境仍输出原有 VTK/PCL 可选组件缺失警告，但没有造成配置、编译或链接失败，与本次修复无关。

消息兼容性复核：

```text
gnss_comm/GnssPVTSolnMsg    d18171357d7a159f76d4d7c0b12fb631
livox_ros_driver2/CustomMsg e4d6829bdfe657cb6c21a746c86b21a6
livox_ros_driver2/CustomPoint 109a3cc548bb1f96626be89a5008bd6d
```

## 9. 测试结果

执行：

```bash
catkin_make run_tests
catkin_test_results build/test_results --all
```

结果：

```text
Summary: 77 tests, 0 errors, 0 failures, 0 skipped
```

XML 检查均通过：

```bash
xmllint --noout src/sensor_recording_bringup/launch/record_all.launch
xmllint --noout src/sensor_recording_bringup/launch/sensors_only.launch
xmllint --noout src/uwb_serial_driver/launch/uwb_serial.launch
```

UWB-only launch 静态展开只包含：

```text
/sensor_time_bridge
/mcu_event_simulator
/uwb_serial_driver
/session_recorder
```

## 10. Jetson 硬件验证步骤

本机没有连接目标 UWB 模块，以下步骤尚未执行，不能视为硬件验收通过。

启动：

```bash
roslaunch sensor_recording_bringup record_all.launch \
  enable_livox:=false \
  enable_camera:=false \
  enable_gnss:=false \
  enable_gnss_adapter:=false \
  enable_uwb:=true \
  uwb_port:=/dev/ttyUSB1 \
  uwb_dtr:=true \
  uwb_rts:=false \
  mcu_input_mode:=simulation \
  sim_start_without_gnss_s:=86400 \
  output_dir:=/home/jetson/bags/uwb_test \
  bag_prefix:=uwb_only \
  record_profile:=both
```

检查：

```bash
rostopic hz /uwb/raw
rostopic echo -n 1 /uwb/raw
rostopic hz /uwb/ranges
rostopic echo -n 1 /sensor_time/status
rostopic echo /sensor_recording/status
rosparam get /session_recorder/effective_required_topics
ps -ef | grep '[r]osbag record'
```

预期：

```text
UWB owns /dev/ttyUSB1 at 115200 baud, DTR=True RTS=False
local_ready: True
/sensor_time/events 有消息
/uwb/raw 有消息
/sensor_recording/status: RECORDING session=...
```

录制期间应存在：

```text
/home/jetson/bags/uwb_test/uwb_only_*.bag.active
```

正常停止后应成为：

```text
/home/jetson/bags/uwb_test/uwb_only_*.bag
```

若 LOCAL/session 已就绪但 UWB 无数据，状态应为：

```text
WAIT_TOPICS missing=/uwb/raw
```

## 11. 尚未验证的硬件风险

- Jetson 上实际 USB-UART 驱动是否完整支持 `TIOCMGET/TIOCMSET`；
- `open()` 到显式 ioctl 之间的短暂控制线状态是否会令特定模块复位；
- 现场 UWB 固件是否始终要求相同 DTR/RTS 极性；
- `/dev/ttyUSB1` 的 udev 映射和用户权限；
- 其他不使用 `flock` 的进程仍可能绕过 advisory lock 打开同一串口；
- 实际数据率下的重连、USB 拔插和长时间录包稳定性；
- `.bag.active` 到 `.bag` 的正常收尾仍需在 Jetson 上验证。

## 12. 回滚

持久化备份：

```text
/home/gulu/catkin_ws/.codex_backups/uwb_dtr_rts_20260724_170113/
```

回滚脚本：

```text
/home/gulu/catkin_ws/.codex_backups/uwb_dtr_rts_20260724_170113/restore.sh
```

执行：

```bash
bash /home/gulu/catkin_ws/.codex_backups/uwb_dtr_rts_20260724_170113/restore.sh
```

脚本恢复本次修改前的全部相关文件，并移除本次新增测试和报告。

本次没有执行 Git commit 或 Git push，也没有读取、修改或覆盖任何 bag。
