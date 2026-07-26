# timeshare 路径跨用户适配报告

生成日期：2026-07-26  
工作空间：`/home/gulu/catkin_ws`  
范围：`livox_ros_driver2`、`mvs_ros_driver`、`sensor_recording_bringup`

## 1. 结论

本次修改已将运行时代码中的 `/home/gulu/timeshare` 硬编码移除，并统一为同一套路径解析实现：

1. 私有 ROS 参数 `~timeshare_path` 非空时优先；
2. 否则使用进程环境变量 `HOME`；
3. `HOME` 缺失或为空时使用 `getpwuid(geteuid())->pw_dir`；
4. 全部失败时输出 FATAL 并退出。

解析结果必须是绝对路径；`~/timeshare` 和相对路径会被明确拒绝，绝不会传给 `open(2)`。标准 launch 默认值均为 `$(env HOME)/timeshare`，外层覆盖参数统一名为 `timeshare_path`。

Livox 和 MVS 共用
`src/livox_ros_driver2/include/livox_ros_driver2/timeshare_path.h`
中的唯一解析实现，没有两份转换逻辑。全工作空间 Release 编译通过，回归结果为 97 tests、0 errors、0 failures、0 skipped。

共享文件结构、64 字节长度、字段偏移、seqlock、Livox 时间门控和相机 Stage A 时间戳行为均未改变。没有修改消息定义、bag、雷达网络配置或 MVS 曝光/触发逻辑。

## 2. 修改前审查

### 2.1 原始硬编码

备份中的原始位置如下：

- `src/livox_ros_driver2/src/lddc.h:180`
  - `shared_timestamp_path_ = "/home/gulu/timeshare"`
- `src/livox_ros_driver2/src/lddc.cpp:194-196`
  - 私有参数名为 `shared_timestamp_path`，默认值为
    `/home/gulu/timeshare`
- `src/mvs_ros_driver/src/grab_trigger.cpp:756`
  - 局部变量默认值为 `/home/gulu/timeshare`
- `src/mvs_ros_driver/src/grab_trigger.cpp:761-763`
  - 私有参数名为 `shared_timestamp_path`，默认值为
    `/home/gulu/timeshare`

修改后生产代码中不再存在该绝对路径或旧参数名。当前
`shared_timestamp_state_test.cpp:139` 中仍出现
`/home/gulu/timeshare`，它只是任务要求的 HOME 输入/期望值测试夹具，
不参与运行时路径选择。

`src/mvs_ros_driver/beifen/grab_trigger.cpp:378` 是未被
`mvs_ros_driver/CMakeLists.txt` 编译的历史备份源码，使用
`"/home/" + login_name + "/timeshare"`。它不包含 gulu/jetson 硬编码，
也不是当前相机节点实现，因此本次未修改；当前唯一构建入口仍是
`src/mvs_ros_driver/src/grab_trigger.cpp`。

在两个目标包的 Python、YAML/JSON、shell、README 和原有 launch 中未发现
其他活动的 `/home/gulu/timeshare` 写入/读取实现。

### 2.2 Writer

- 进程/可执行文件：`livox_ros_driver2_node`
- launch 节点名：`livox_lidar_publisher2`
- 实现：`src/livox_ros_driver2/src/lddc.cpp`
- 当前路径解析和单次启动日志：`lddc.cpp:195-217`
- 创建/调整长度/mmap：`lddc.cpp:263-321`
- 更新时间戳：`lddc.cpp:347-376`

打开行为保持不变：

- `open(path, O_CREAT | O_RDWR, 0666)`
- `ftruncate(fd, sizeof(SharedTimestampState))`
- `mmap(..., PROT_READ | PROT_WRITE, MAP_SHARED, ...)`
- 父目录不自动创建；失败日志包含最终路径和 `errno`
- `0666` 是修改前既有请求模式，实际权限仍受进程 `umask` 限制；本次没有扩大权限

### 2.3 Reader

- 进程/可执行文件：`grabImgWithTrigger`
- launch 节点名：`mvs_camera_trigger`
- 节点入口：`src/mvs_ros_driver/src/grab_trigger.cpp`
- 路径解析和单次启动日志：`grab_trigger.cpp:656-675`
- 读端实现：
  `src/mvs_ros_driver/include/mvs_ros_driver/shared_timestamp_reader.h`
- 帧取得后的共享时间读取：`grab_trigger.cpp:454-538`

打开行为保持不变：

- `open(path, O_RDONLY)`
- `fstat` 后要求文件长度严格等于 64 字节
- `mmap(..., PROT_READ, MAP_SHARED, ...)`
- 打开、长度或 mmap 失败时不保存无效指针

文件暂时不存在时，Reader 返回错误并按最短 0.1 秒的配置间隔重试；
相机工作线程以 5 秒限频错误日志丢弃无有效共享时间的触发帧，
不回退 `ros::Time::now()`。后续文件出现后可以重新打开并恢复。
因此 Camera 可以先启动，不存在严格的“Livox 必须先启动”依赖；
不过在 Writer 就绪前的触发帧会按 Stage A 规则被丢弃。

## 3. 统一路径解析

实现文件：
`src/livox_ros_driver2/include/livox_ros_driver2/timeshare_path.h`

- `timeshare_path.h:13-21`：拒绝空、`~` 和相对路径；
- `timeshare_path.h:23-44`：可注入输入的确定性解析核心；
- `timeshare_path.h:46-63`：运行时读取 `HOME`，必要时调用
  `getpwuid(geteuid())`。

两个节点都只在启动时读取一次私有参数并解析一次。底层
`open`/`mmap` 组件只接收最终绝对路径，不自行猜测路径。

若节点由标准 launch 启动，`$(env HOME)` 在 roslaunch 解析期展开；
若直接运行可执行文件且参数为空，C++ 的 HOME/getpwuid 回退生效。
若以 root 用户运行并解析到 `/root/...`，Writer 和 Reader 都会输出明确警告。

## 4. launch 参数链

实际链路为：

```text
record_all.launch
  timeshare_path
    └─ sensors_only.launch
         timeshare_path
           ├─ include livox_ros_driver2/msg_MID360.launch
           │    └─ /livox_lidar_publisher2/timeshare_path
           └─ include mvs_ros_driver/mvs_camera_trigger.launch
                └─ /mvs_camera_trigger/timeshare_path
```

对应位置：

- `record_all.launch:26,28-51`
- `sensors_only.launch:29,58-71`
- `msg_MID360.launch:17,32-36`
- `mvs_camera_trigger.launch:2-15`

`sensors_only.launch` 已改为 include
`mvs_camera_trigger.launch`，不再重复定义相机节点。MVS 节点自己的
`LD_LIBRARY_PATH` 环境配置保留在 node 内，bringup include 时将
`rviz_enable` 设为 false，避免传感器组合启动时额外拉起 RViz。

## 5. 共享文件格式兼容性

结构定义仍位于：
`src/livox_ros_driver2/include/livox_ros_driver2/shared_timestamp_state.h:21-31`

该文件未被本次修改。实测/单测确认布局如下：

| 字段 | 偏移（字节） | 大小（字节） |
|---|---:|---:|
| `magic` | 0 | 4 |
| `version` | 4 | 4 |
| `writer_epoch` | 8 | 8 |
| `write_sequence` | 16 | 8 |
| `stamp_ns` | 24 | 8 |
| `clock_source` | 32 | 4 |
| `ready` | 36 | 4 |
| `last_update_monotonic_ns` | 40 | 8 |
| `reserved` | 48 | 16 |

- `sizeof(SharedTimestampState) == 64`
- `alignof(SharedTimestampState) == 64`
- 写入仍使用奇/偶 `write_sequence` 和原子 acquire/release seqlock
- `magic`、`version`、epoch、ready、clock_source 的含义不变
- Writer 仍写入 `LIDAR_BASE_TIME_LEGACY`
- MID-360 `time_type` 门控、时间戳生成和网络/IP 配置未改
- Camera 仍丢弃无效、重复、倒序、epoch 切换和 jump 异常帧
- 没有新增 `ros::Time::now()` 回退

## 6. 修改文件清单

新增：

- `src/livox_ros_driver2/include/livox_ros_driver2/timeshare_path.h`
- `src/mvs_ros_driver/include/mvs_ros_driver/shared_timestamp_reader.h`

修改：

- `src/livox_ros_driver2/src/lddc.h`
- `src/livox_ros_driver2/src/lddc.cpp`
- `src/livox_ros_driver2/launch_ROS1/msg_MID360.launch`
- `src/livox_ros_driver2/test/shared_timestamp_state_test.cpp`
- `src/mvs_ros_driver/src/grab_trigger.cpp`
- `src/mvs_ros_driver/launch/mvs_camera_trigger.launch`
- `src/mvs_ros_driver/test/timestamp_monitor_test.cpp`
- `src/sensor_recording_bringup/launch/sensors_only.launch`
- `src/sensor_recording_bringup/launch/record_all.launch`

无需修改 CMake/package 依赖：MVS 原本已经依赖
`livox_ros_driver2`，Livox 已导出并安装自己的 `include/`，MVS 现有构建
也已包含本包 `include/`。没有引入新依赖或循环依赖。

用户在任务开始前已有的
`src/livox_ros_driver2/config/MID360_config.json`
IP 修改保持原样，本次没有触碰或回滚。

## 7. 测试和验证结果

### 7.1 XML

以下四项均返回 0：

```bash
xmllint --noout src/livox_ros_driver2/launch_ROS1/msg_MID360.launch
xmllint --noout src/mvs_ros_driver/launch/mvs_camera_trigger.launch
xmllint --noout src/sensor_recording_bringup/launch/sensors_only.launch
xmllint --noout src/sensor_recording_bringup/launch/record_all.launch
```

### 7.2 构建

执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/gulu/catkin_ws
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2
```

结果：成功。`livox_ros_driver2_node` 和 `grabImgWithTrigger` 均重新编译、
链接通过；当前 PC 的 x86_64 构建通过，没有新增架构专用编译参数。

### 7.3 回归测试

执行：

```bash
catkin_make run_tests
catkin_make run_tests_livox_ros_driver2 -j2
catkin_test_results /home/gulu/catkin_ws/build/test_results --all
```

最终结果：

```text
Summary: 97 tests, 0 errors, 0 failures, 0 skipped
```

覆盖：

- 显式绝对路径优先且保持不变；
- HOME 为 `/home/gulu` 和 `/home/jetson`；
- HOME 空时实际调用有效用户 passwd home；
- `~` 和相对路径拒绝；
- Writer/Reader 同输入结果一致；
- HOME/passwd 都不可用时失败；
- Reader 面对缺失文件不崩溃；
- Writer 后续创建文件后 Reader 恢复；
- 64 字节结构大小、对齐和全部字段偏移；
- 原有跨进程 seqlock、防 torn read、ready gate、整数纳秒转换；
- Stage A MVS 监控测试；
- Stage B1 `sensor_time_bridge`、GNSS、UWB 和 recorder 既有回归。

### 7.4 launch 静态解析

`roslaunch --dump-params` 实测：

```text
开发机默认：
/livox_lidar_publisher2/timeshare_path: /home/gulu/timeshare
/mvs_camera_trigger/timeshare_path: /home/gulu/timeshare

HOME=/home/jetson：
/livox_lidar_publisher2/timeshare_path: /home/jetson/timeshare
/mvs_camera_trigger/timeshare_path: /home/jetson/timeshare

timeshare_path:=/tmp/test_timeshare：
/livox_lidar_publisher2/timeshare_path: /tmp/test_timeshare
/mvs_camera_trigger/timeshare_path: /tmp/test_timeshare
```

上述默认和覆盖均分别验证了两个独立 launch、`sensors_only.launch`
和 `record_all.launch`。`roslaunch --nodes sensors_only.launch` 只列出一个
`/mvs_camera_trigger`。

### 7.5 消息兼容

没有任何 `msg/*.msg` 差异。当前 MD5：

```text
livox_ros_driver2/CustomMsg   e4d6829bdfe657cb6c21a746c86b21a6
livox_ros_driver2/CustomPoint 109a3cc548bb1f96626be89a5008bd6d
sensor_msgs/Image             060021388200f6f0f447d0fcd9c64743
```

`git diff --check` 通过。工作空间内没有 bag 文件，本次没有创建、覆盖或
修改 bag。

## 8. 开发 PC 验证命令

Livox：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

预期：

```text
[TIMESHARE] role=writer path=/home/gulu/timeshare
```

相机（PC 使用 64 位 MVS SDK 路径时显式覆盖该已有架构参数）：

```bash
roslaunch mvs_ros_driver mvs_camera_trigger.launch \
  mvs_library_dir:=/opt/MVS/lib/64
```

预期：

```text
[TIMESHARE] role=reader path=/home/gulu/timeshare
```

显式路径覆盖：

```bash
roslaunch sensor_recording_bringup record_all.launch \
  timeshare_path:=/tmp/test_timeshare
```

## 9. Jetson 验证命令

```bash
source /opt/ros/noetic/setup.bash
source /home/jetson/catkin_ws/devel/setup.bash

roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch mvs_ros_driver mvs_camera_trigger.launch
```

预期启动日志：

```text
[TIMESHARE] role=writer path=/home/jetson/timeshare
[TIMESHARE] role=reader path=/home/jetson/timeshare
```

文件和进程环境：

```bash
ls -l "$HOME/timeshare"
stat "$HOME/timeshare"

PID=$(pgrep -n grabImgWithTrigger)
tr '\0' '\n' < /proc/${PID}/environ | grep '^HOME='
```

完整启动也可显式覆盖：

```bash
roslaunch sensor_recording_bringup record_all.launch \
  timeshare_path:=/tmp/test_timeshare
```

不需要设置全局 `TIMESHARE_PATH` 环境变量。

## 10. 尚未完成的硬件验证与风险

本次在 x86_64 开发机完成软件构建、单测和 launch 静态解析，没有声称以下
硬件验证已经通过：

- Jetson aarch64 上的实际编译和启动；
- MID-360 实机创建/更新 `$HOME/timeshare`；
- 海康相机实机读取、丢帧/恢复和图像发布；
- Livox 与 Camera 同时启动后的实际单次 `[TIMESHARE]` 日志；
- 真机权限、umask 和长时间运行；
- `record_all.launch` 的全传感器采集。

部署注意事项：

- Writer 和 Reader 应以同一普通用户运行；若分属不同用户，应通过同一个
  绝对 `timeshare_path` 显式覆盖并配置权限；
- 父目录必须存在，代码不会创建目录；
- 外部旧 launch 若仍设置 `~shared_timestamp_path`，需改为
  `~timeshare_path`；
- 以 sudo/root 启动会解析到 `/root/timeshare`，代码会告警；
- Writer 创建失败会限频重试，Camera 在 Writer ready 前会丢弃触发帧，
  这是保留的 Stage A 行为。

## 11. 回滚

修改前备份：

```text
/home/gulu/catkin_ws/.codex_backups/timeshare_home_path_20260726_150328/
```

回滚脚本：

```text
/home/gulu/catkin_ws/.codex_backups/timeshare_home_path_20260726_150328/restore.sh
```

执行：

```bash
bash /home/gulu/catkin_ws/.codex_backups/timeshare_home_path_20260726_150328/restore.sh
```

脚本已通过 `bash -n` 检查。它恢复本次修改的 9 个原文件，并删除本次新增的
2 个头文件和本报告；不会修改用户已有的 `MID360_config.json`。

## 12. Git 状态

未执行 `git commit`，未执行 `git push`。测试产生的已跟踪 `.pyc`
变化已单独清理，未混入本次修改。
