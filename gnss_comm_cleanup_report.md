# gnss_comm 纯消息兼容包清理报告

日期：2026-07-23  
工作空间：`/home/gulu/catkin_ws`

## 1. 执行摘要

本轮已将 `gnss_comm` 缩减为纯 ROS 消息兼容包。最终源码树只包含：

```text
gnss_comm/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── package.xml
└── msg/
    ├── GnssBestXYZMsg.msg
    ├── GnssEphemMsg.msg
    ├── GnssGloEphemMsg.msg
    ├── GnssMeasMsg.msg
    ├── GnssObsMsg.msg
    ├── GnssPVTSolnMsg.msg
    ├── GnssSvsMsg.msg
    ├── GnssTimeMsg.msg
    ├── GnssTimePulseInfoMsg.msg
    └── StampedFloat64Array.msg
```

`LICENSE` 因许可证义务保留；`README.md` 因需要明确兼容边界和迁移后的实现
归属而保留。没有保留任何上游实现兼容头文件。

消息包名、10 个消息文件、字段、顺序和 ROS datatype 均未修改。
`gnss_comm/GnssPVTSolnMsg` 最终 MD5 为：

```text
d18171357d7a159f76d4d7c0b12fb631
```

仍被工作空间使用的 GPS 时间、WGS-84 ECEF/LLA、ECEF 增量到 ENU 功能已集中
到 `gnss_serial_driver/gnss_serial_math`。FAST 的两个调用方已经切换到该库；
`gnss_comm` 不再生成或导出实现库。原 Python ENU 适配节点中的第二套坐标公式
也已改成调用同一库的 C++ 节点，避免双份维护。

最终结果：

- `catkin_make -j2`：通过；
- `catkin_make run_tests -j2`：通过；
- `catkin_test_results build/test_results --all`：62 tests，0 error，
  0 failure，0 skipped；
- Stage A：Livox 12、MVS 6 的 XML 汇总均通过，离线分析器 5 项通过；
- Stage B1：原测试全部保持通过，新增迁移数学实际 testcase 2 项通过；
- FAST `external_topic` 边界、ENU adapter、fault injector、UWB、RTK backend
  self-test：全部退出码 0；
- 兼容发布实测：`/ublox_driver/receiver_pvt` 的 runtime datatype 和 MD5
  分别为 `gnss_comm/GnssPVTSolnMsg` 和目标 MD5；
- 无循环依赖，无旧实现引用，无 `libgnss_comm.so` 生成物；
- 未修改或覆盖任何原始 bag，未执行 Git commit 或 push。

## 2. 原始文件审查与分类

以下分类依据原文件内容和全工作空间调用搜索，不依据文件名推断。原始文件完整
快照位于：

```text
/home/gulu/catkin_ws/.codex_backups/gnss_comm_cleanup_20260723_200109/gnss_comm
```

| 分类 | 原文件 | 内容/结论 |
|---|---|---|
| 消息定义 | `msg/*.msg` 共 10 个 | 当前 ROS 兼容面，全部原样保留 |
| 消息生成配置 | `CMakeLists.txt`、`package.xml` | 原配置同时构建实现库；已缩成 message generation/runtime 最小配置 |
| 通用数学工具 | `include/gnss_comm/gnss_constant.hpp`、`include/gnss_comm/gnss_utility.hpp`、`src/gnss_utility.cpp` | 含卫星编号、历元、时间、星历、坐标、对流层等混合工具 |
| 时间转换 | `gnss_utility.hpp/.cpp` | `gpst2time`、`gpst2utc`、`time2sec` 等；FAST 有两个实际调用者 |
| ECEF/ENU 转换 | `gnss_utility.hpp/.cpp` | `geo2ecef`、`ecef2geo`、`ecef2enu`；FAST adapter 实际调用 |
| 消息/内部结构转换 | `include/gnss_comm/gnss_ros.hpp`、`src/gnss_ros.cpp` | ephemeris、measurement、PVT、SV 的 ROS 转换；工作空间无调用者 |
| SPP 求解 | `include/gnss_comm/gnss_spp.hpp`、`src/gnss_spp.cpp` | 伪距位置、Doppler 速度、卫星状态；工作空间无调用者 |
| 解析器 | `include/gnss_comm/rinex_helper.hpp`、`src/rinex_helper.cpp` | RINEX 导航/观测解析和观测写出；工作空间无调用者 |
| CMake 查找模块 | `cmake/FindEigen.cmake`、`cmake/FindGlog.cmake` | 只服务于已删除实现库 |
| Docker/example build | `docker/Dockerfile`、`docker/Makefile` | 只构建原混合实现包 |
| 可执行节点 | 无 | 原包没有 `add_executable` 或节点源文件 |
| launch | 无 | 原包没有 launch |
| config | 无 | 原包没有 config |
| examples | 无 | 原包没有 examples |
| tests | 无 | 原包没有 tests |

全工作空间调用图确认，`gnss_comm` 实现库的外部调用者只有：

```text
FAST_LIVO2/src/gnss_adapter.cpp
FAST_LIVO2/src/gnss_fault_injector.cpp
```

消息调用者继续使用 `gnss_comm::GnssPVTSolnMsg`，没有改成
`gnss_serial_driver/GnssPVTSolnMsg`。

## 3. 保留文件

### 3.1 原样保留

```text
gnss_comm/LICENSE
gnss_comm/msg/GnssBestXYZMsg.msg
gnss_comm/msg/GnssEphemMsg.msg
gnss_comm/msg/GnssGloEphemMsg.msg
gnss_comm/msg/GnssMeasMsg.msg
gnss_comm/msg/GnssObsMsg.msg
gnss_comm/msg/GnssPVTSolnMsg.msg
gnss_comm/msg/GnssSvsMsg.msg
gnss_comm/msg/GnssTimeMsg.msg
gnss_comm/msg/GnssTimePulseInfoMsg.msg
gnss_comm/msg/StampedFloat64Array.msg
```

清理前后对 10 个 `.msg` 逐一执行 SHA-256，结果完全相同。由于注释也参与 ROS
MD5 的规范化语义边界，消息文件未做任何“顺手清理”。

### 3.2 保留但缩减/更新

```text
gnss_comm/CMakeLists.txt
gnss_comm/package.xml
gnss_comm/README.md
```

- `CMakeLists.txt` 只保留 `message_generation`、`std_msgs`、10 个显式消息清单、
  `generate_messages` 和 `message_runtime` 导出；
- `package.xml` 移除 `roscpp`、`rospy` 和无必要的 message-generation
  runtime 声明；
- `README.md` 改为明确该包只承担 datatype/MD5 兼容。

## 4. 迁移文件与功能映射

新增或更新于 `gnss_serial_driver`：

```text
include/gnss_serial_driver/gnss_math.hpp
src/gnss_math.cpp
src/gnss_adapter_node.cpp
test/gnss_math_self_test.cpp
CMakeLists.txt
package.xml
launch/gnss_adapter.launch
```

功能映射：

| 原 `gnss_comm` API | 新唯一实现 |
|---|---|
| `gpst2time` + `gpst2utc` + `time2sec` | `gnss_serial_driver::gpsWeekTowToUnixUtc` |
| `geo2ecef` | `gnss_serial_driver::geodeticToEcef` |
| `ecef2geo` | `gnss_serial_driver::ecefToGeodetic` |
| `ecef2enu` | `gnss_serial_driver::ecefDeltaToEnu` |

实现证据：

- `gnss_serial_driver/src/gnss_math.cpp:8-37`：GPS epoch、leap-second history、
  WGS-84 常量；
- `gnss_serial_driver/src/gnss_math.cpp:55-70`：GPS week/tow 到 Unix UTC；
- `gnss_serial_driver/src/gnss_math.cpp:72-133`：LLA/ECEF 双向转换；
- `gnss_serial_driver/src/gnss_math.cpp:135-164`：ECEF 增量到 ENU；
- `FAST_LIVO2/src/gnss_adapter.cpp:13,251,273-274,504,631,667-668`：
  FAST adapter 只调用新库；
- `FAST_LIVO2/src/gnss_fault_injector.cpp:11,41-42`：fault injector 只调用新库。

原 `gnss_serial_driver/scripts/gnss_adapter_node.py` 中还存在独立 Python ECEF/ENU
公式。为满足“不重复维护两份转换实现”，该脚本已由
`src/gnss_adapter_node.cpp` 取代。launch 的节点名、输入
`/gnss/pvt_local`、输出 `/gnss/enu_odom`、`origin_mode`、`origin_lla`、
`frame_id` 和 `child_frame_id` 接口保持。节点 target 使用唯一名字
`gnss_serial_adapter_node`，输出文件仍为 `gnss_adapter_node`，避免和
FAST 包内同名 CMake target 冲突。

未迁移 SPP、RINEX 和内部结构转换代码。理由不是文件名判断，而是全工作空间
零调用；复制到串口驱动只会把未使用的 Eigen/Glog/RTKLIB 风格业务面继续
维护一份。它们已删除，原始版本可由本轮备份恢复。

## 5. 删除文件

从 `gnss_comm` 删除：

```text
.gitignore
cmake/FindEigen.cmake
cmake/FindGlog.cmake
docker/Dockerfile
docker/Makefile
include/gnss_comm/gnss_constant.hpp
include/gnss_comm/gnss_ros.hpp
include/gnss_comm/gnss_spp.hpp
include/gnss_comm/gnss_utility.hpp
include/gnss_comm/rinex_helper.hpp
src/gnss_ros.cpp
src/gnss_spp.cpp
src/gnss_utility.cpp
src/rinex_helper.cpp
```

从 `gnss_serial_driver` 删除：

```text
scripts/gnss_adapter_node.py
```

同时清理了增量构建留下、当前 CMake 不再生成的精确生成物：

```text
devel/lib/libgnss_comm.so
build/gnss_comm/CMakeFiles/gnss_comm.dir/
devel/lib/gnss_serial_driver/gnss_adapter_node.py
devel/lib/gnss_serial_driver/gnss_math_self_test
```

再次执行 `catkin_make -j2` 后这些文件均未重生。

## 6. 依赖变化与循环依赖检查

### 6.1 `gnss_comm`

清理前：

```text
roscpp, rospy, std_msgs, message_generation/message_runtime
Eigen3, Glog
导出 libgnss_comm 和 include/gnss_comm 实现头
```

清理后：

```text
build: message_generation, std_msgs
exec:  message_runtime, std_msgs
不导出实现 include，不生成库，不生成节点
```

`gnss_comm/package.xml` 和 `CMakeLists.txt` 均不含
`gnss_serial_driver`。

### 6.2 `gnss_serial_driver`

保留既有对 `gnss_comm` 消息的单向依赖；新增：

- Eigen：公开数学头使用 `Eigen::Vector3d`；
- roscpp：共享实现的 C++ ENU adapter 节点；
- 导出 `gnss_serial_math`。

最终拓扑中的相关顺序：

```text
gnss_comm
sensor_time_msgs
gnss_serial_driver
...
fast_livo
```

`catkin_topological_order src` 成功，没有循环。链接检查：

```text
libgnss_adapter.so          -> libgnss_serial_math.so
libgnss_fault_injector.so   -> libgnss_serial_math.so
```

两者均不链接 `libgnss_comm.so`。全 `src/` 搜索没有旧实现头、旧实现函数或
`libgnss_comm` 引用。

## 7. 消息 MD5 与 datatype 兼容

最终命令：

```bash
rosmsg md5 gnss_comm/GnssPVTSolnMsg
rosmsg show gnss_comm/GnssPVTSolnMsg
```

结果：

```text
datatype: gnss_comm/GnssPVTSolnMsg
md5:      d18171357d7a159f76d4d7c0b12fb631
```

生成的 Python 消息类也报告相同 `_type` 和 `_md5sum`，slots 顺序为：

```text
time, fix_type, valid_fix, diff_soln, carr_soln, num_sv,
latitude, longitude, altitude, height_msl, h_acc, v_acc, p_dop,
vel_n, vel_e, vel_d, vel_acc
```

这保持了旧 bag connection 所需的 package/type、message definition 和 MD5。

## 8. 编译和测试结果

最终执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/gulu/catkin_ws
catkin_make -j2
catkin_make run_tests -j2
catkin_test_results build/test_results --all
```

结果：

```text
Summary: 62 tests, 0 errors, 0 failures, 0 skipped
```

明细：

| XML 组 | catkin 汇总 |
|---|---:|
| `gnss_serial_driver/gtest-gnss_math_test.xml` | 4 |
| `gnss_serial_driver/nosetests-test.xml` | 7 |
| `livox_ros_driver2/gtest-shared_timestamp_state_test.xml` | 12 |
| `mvs_ros_driver/gtest-timestamp_monitor_test.xml` | 6 |
| `sensor_time_bridge/gtest-sensor_time_core_test.xml` | 24 |
| `uwb_serial_driver/nosetests-test.xml` | 5 |
| recorder nose/rostest/rosunit | 4 |

GTest XML 的 aggregate suite 会重复计数；新增数学测试的实际 testcase 为 2：

- modern GPST→UTC、GPS epoch、NaN/越界 TOW；
- WGS-84 LLA↔ECEF 往返和 ECEF→ENU 三轴。

Stage A：

- Livox XML：12，0 failure；
- MVS XML：6，0 failure；
- `scripts/test_analyze_sensor_timestamps.py`：5 passed。

Stage B1/FAST 边界：

- GNSS parser/time policy：7 passed；
- sensor time、UWB、recorder 全部保持通过；
- `gnss_manager_self_test`：退出码 0，覆盖 `external_topic` 有效消息入队及
  无效 fusion 消息拒绝；
- `uwb_manager_self_test`：PASS；
- `gnss_adapter_self_test`：PASS；
- `gnss_fault_injector_self_test`：PASS；
- `rtk_fixed_lag_backend_self_test`：PASS。

构建中仍有工作空间既有的 VTK imported-target 缺文件提示和 GTSAM deprecated
警告；两者与本轮迁移无关，未造成目标失败。

## 9. 兼容 PVT 发布与 ENU adapter 实测

使用 `/tmp/gnss_cleanup_replay.txt` 的一条校验和正确 KSXT 记录，启动临时
ROS master 和 `gnss_serial_node.py`，实测：

```text
/ublox_driver/receiver_pvt
type = gnss_comm/GnssPVTSolnMsg
md5  = d18171357d7a159f76d4d7c0b12fb631
```

实际收到的 PVT 包含 week 2174、tow 125372.0、RTK fixed、24 satellites、
latitude 30.27413、longitude 120.15516、altitude 16.0。

对新的 `gnss_serial_driver/gnss_adapter_node` 发布一条 fusion-valid
`GnssPvtStamped`，实测 `/gnss/enu_odom`：

- 输入 stamp `946684801.250000000` 原样保留；
- frame/child 为 `gnss_enu`/`gnss`；
- 首个有效点成为原点，ENU 约 `[0,0,0]`；
- NED `[north=1,east=2,down=-0.5]` 映射为 ENU `[2,1,0.5]`；
- 位置和速度 covariance 按既有接口生成。

## 10. 旧 bag 兼容验证

没有覆盖、修改或 reindex 任何原 bag。

对 `/home/gulu` 下排除 build/devel/cache 的全部 27 个 bag 执行
`rosbag info --yaml`：

```text
discovered=27
readable=27
unreadable=0
matched /ublox_driver/receiver_pvt or gnss_comm/GnssPVTSolnMsg=0
```

特别地，已有审计中的 `/home/gulu/Downloads/0709.bag` 只含 camera、Livox
LiDAR 和 Livox IMU，不含 GNSS。因此当前机器没有可直接执行“旧
`/ublox_driver/receiver_pvt` bag 读回”的样本，不能伪造该项实测通过。

作为序列化补充验证，在此前不存在的
`/tmp/gnss_comm_cleanup_compat.bag` 新建一条消息（没有覆盖文件），随后用
`rosbag` 读回：

```text
topic: /ublox_driver/receiver_pvt
type:  gnss_comm/GnssPVTSolnMsg
md5:   d18171357d7a159f76d4d7c0b12fb631
count: 1
```

结论：旧连接的二进制兼容条件已经通过原样消息定义、datatype 和 MD5 固定；
新 bag 的 record/read 也已实测。若要把“某个历史 GNSS bag 实读”作为独立
验收项，仍需提供一个实际包含该 topic 的旧 bag 路径。

## 11. 风险与兼容性影响

1. 消息消费者和旧 bag：无接口变化。
2. 直接链接旧 `libgnss_comm` 或包含旧实现头的工作空间外程序：不再受支持；
   本工作空间内已确认没有此类调用者。这是纯消息包化的预期 breaking change。
3. 删除的 SPP/RINEX/内部转换函数没有迁入 driver，因为零调用。若未来确需，
   应从备份按实际需求拆成独立算法包，不应重新塞回消息包。
4. leap-second 表截至 2017-01-01 的 18 秒，与当前 GNSS driver 配置和旧实现
   一致；未来 IERS 新增 leap second 时需在唯一实现处更新并增加测试。
5. 新 ENU adapter 对未知 `origin_mode` 明确失败；当前受支持且原 launch 使用
   的模式为 `first_valid` 和 `manual`。
6. `/tmp/gnss_comm_cleanup_compat.bag` 是本轮临时验证产物，不是用户原始 bag。

## 12. 回滚

完整备份：

```text
/home/gulu/catkin_ws/.codex_backups/gnss_comm_cleanup_20260723_200109
```

回滚脚本：

```text
/home/gulu/catkin_ws/.codex_backups/gnss_comm_cleanup_20260723_200109/rollback_gnss_comm_cleanup.sh
```

脚本通过固定路径恢复：

- `gnss_comm` 的全部非 `.git` 源文件；
- `gnss_serial_driver` 的本轮前完整快照；
- FAST 的 `CMakeLists.txt`、`package.xml`、`gnss_adapter.cpp` 和
  `gnss_fault_injector.cpp`。

脚本已执行 `bash -n` 语法检查，`rsync` 可用。回滚后应重新运行
`catkin_make -j2` 以重建旧生成物。

## 13. Git 与数据保护

- 未运行 `git commit`；
- 未运行 `git push`；
- `gnss_comm` 与 FAST 的修改均保持为工作区变更；
- FAST 中本轮前已有的 Stage A/B1 和用户配置变更均保留；
- 未删除、覆盖或修改任何用户 bag。
