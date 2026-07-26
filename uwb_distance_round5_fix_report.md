# UWB `distance_round5` 串口协议修复报告

日期：2026-07-26  
工作空间：`/home/gulu/catkin_ws`  
主要包：`src/uwb_serial_driver`

## 1. 结论

已将当前硬件默认解析模式从宽松的逐行 `auto` 解析改为严格的
`distance_round5` 轮次状态机：

```text
[UWBDBG]（轮次边界）
  -> [TWR]（只忽略）
  -> 5 条 distance（聚合）
  -> 0 或 1 条 UwbRangeArray
```

真实 49 行样本得到 7 个完整轮次，其中 6 个轮次发布，1 个全零空轮不发布；
6 个结果依次为 `1.405, 1.362, 1.391, 1.374, 1.308, 1.342 m`，
全部为 `anchor_id=1`。`UWBDBG` 的 `dist` 和 `TWR` 的整数均不会进入测距结果。

`/uwb/raw` 仍逐行发布全部非空输入；`/uwb/ranges` 的时间元数据来自该轮
第一条非零 `distance` 原始帧。消息定义和 MD5 未改变。

本轮完成了纯 Python、file replay ROS 集成和全工作空间回归；未连接真实 UWB
串口，因此 Jetson 实机串口行为仍需按第 13 节验证。

## 2. 原始误解析原因与代码证据

修改前文件保存在：

`/home/gulu/catkin_ws/.codex_backups/uwb_distance_round5_20260726_170925/files/`

原始问题对应位置如下：

1. 当前硬件默认值为 `auto`：
   - `src/uwb_serial_driver/config/default.yaml:4`
   - `src/uwb_serial_driver/scripts/uwb_serial_node.py:34`
2. `distance_pattern` 没有锚定整行，并由
   `parser.py:58` 的 `search()` 在任意位置搜索。
3. `parser.py:66-80` 会读取 `[UWBDBG]`/`dist=` 中的
   `target`、`dist`、`diag` 并生成 `uwbdbg` 或 `debug_distance` 测距。
4. `parser.py:84` 对其他任意文本调用 `number_pattern.findall()`；
   因而 `[TWR]` 中的 `Ra/Rb/Da/Db/num/den` 可能被解释为距离或锚点—距离对。
5. `uwb_serial_node.py:214-218` 把所有空解析结果归入
   `parse_error_count`，无法区分调试行、零槽位、待完成轮次和真正错误。
6. `uwb_serial_node.py:219` 明确采用“一输入行就是一轮”的模型；
   `:224-248` 每行单独创建并发布 `UwbRangeArray`。
7. 原 `parser.py:47-52` 会将零值构造成 invalid range；
   原节点 `uwb_serial_node.py:245-246` 随后增加 `range_reject_count`。

根因不是串口收包层，而是解析层把“固定七行协议”降格成了“每行独立且可从任意
文本提取数字”的协议。修复点因此放在共享解析器及其唯一发布调用链，而没有修改
DTR/RTS、串口独占锁或消息定义。

## 3. 新状态机设计

纯 Python `DistanceRoundAssembler` 位于：

`src/uwb_serial_driver/src/uwb_serial_driver/parser.py:134`

状态：

- `WAIT_DEBUG`：尚未见到可靠轮次边界；中途收到 `distance` 只识别为
  `OUT_OF_SYNC_DISTANCE`，不发布、不计 parse error。
- `WAIT_DISTANCE_LINES`：已经见到 `[UWBDBG]`，等待严格匹配的 5 条
  `distance`。

重同步和超时：

- 每个行首 `[UWBDBG]` 都是强制新边界。
- 新 `[UWBDBG]` 到来时若旧轮不足 5 条，旧轮被丢弃并增加
  `incomplete_round_count`。
- 从边界开始超过 `round_timeout_s` 时丢弃 pending round，回到
  `WAIT_DEBUG`。
- 串口空闲轮询会检查超时；非循环 file replay 在文件尾也会处理未完成轮次。

状态机只接收字符串、时间上下文 token 和单调时刻，不导入 `rospy` 或 ROS
消息类型，因此相同协议逻辑可由 serial 与 file replay 共用并独立测试。

## 4. 三类协议行的规则

### `[UWBDBG]`

由 `^\s*\[UWBDBG\]` 严格匹配行首，只作为新轮次边界。
`target/dist/diag/status` 不生成 `ParsedRange`。该行计入
`ignored_debug_line_count`，不会增加 `parse_error_count`。

### `[TWR]`

由 `^\s*\[TWR\]` 严格匹配行首，只作为已知调试行忽略，不改变待收集轮次。
不会对 `Ra/Rb/Da/Db/num/den` 做数字提取，也不会增加
`parse_error_count`。

### `distance`

使用锚定整行的语法：

```text
^\s*distance\s*\[\s*(\d+)\s*\]\s*,\s*<number>\s*(?:,.*)?$
```

只提取 `anchor_id` 和 `range_m`；第二个逗号后的诊断后缀整体忽略。
合法行使本轮槽位计数加一。形似 `distance` 但不符合语法的行增加
`malformed_distance_count` 和 `parse_error_count`。其他未知行只增加
`parse_error_count`，不会生成测距。

保留的旧 `auto/pairs/values/distance/uwb` 模式仅用于兼容；其中数字兜底现只
接受“整行都是数字记录”的输入，调试文本或任意混合文本不能再被解释为测距。
严格硬件模式完全不进入该兜底。

## 5. 五行聚合和多基站规则

每个 `[UWBDBG]` 后恰好收集 5 条合法 `distance`：

- 有至少一个非零记录：产生一个且仅一个 `UwbRangeArray`。
- 同轮多个不同非零 anchor：全部放入同一个 array，不拆轮。
- 同轮相同 anchor 多次为正：保留第一条，后续记录增加
  `duplicate_anchor_count` 并输出限频警告。
- 重复的零 anchor 不属于重复错误。
- 收满 5 条前出现新边界或超时：整轮丢弃，不发布半轮数据。

尺度、bias 和非零值有效性继续使用原 `make_range()` 语义：

```text
raw_range_m       = distance_value * range_scale
corrected_range_m = raw_range_m - range_bias_m[anchor_id]
```

非零但非有限、负数、修正后非有限或越界的记录仍生成带原有
`valid/reject_reason` 语义的 `UwbRange`，并由节点增加
`range_reject_count`。

## 6. 零值和空轮规则

原始 `distance_value == 0.0` 时：

- 只增加 `zero_slot_count`；
- 不调用尺度/bias/范围拒绝逻辑；
- 不创建 `UwbRange`；
- 不增加 `parse_error_count` 或 `range_reject_count`。

5 个槽位全部为零时：

- `complete_round_count` 和 `empty_round_count` 各增加一次；
- 不发布 `/uwb/ranges`；
- `round_sequence` 与 `parsed_round_count` 不增加；
- 输出限频空轮诊断。

真实样本统计为：`complete=7`、`published=6`、`empty=1`、
`zero_slots=29`、`ignored_debug=14`。

## 7. 时间戳和原始记录语义

`uwb_serial_node.py:213-249` 仍在每条完整非空行到达时先构造并发布
`RawSerialFrame`，保留：

- `header.stamp`
- `session_id`
- `writer_epoch`
- `source_sequence`
- `host_receive_stamp`
- `host_receive_monotonic_ns`
- `time_uncertainty_ns`
- `timestamp_source`
- `device`
- `protocol`
- `data`

随后节点把这一 `RawSerialFrame` 作为纯 Python assembler 的上下文 token。
assembler 只保存本轮第一条非零 `distance` 的 token；轮次完成后，
`uwb_serial_node.py:300-323` 从该帧复制 array 的 `header`、`session_id`、
`timestamp_source`、uncertainty 和 host receive 元数据。

因此：

- 不使用 `[UWBDBG]`、`[TWR]` 或最后一个零槽位的时间；
- serial 与 file replay 走同一个 `_handle_line()` 和 assembler；
- preserve 模式下，原先在 `uwb_serial_node.py:218-222` 写入 raw frame 的
  preserved 时间会随所选非零行传入 array；
- rebase 模式同理保留所选非零行的 rebased 时间；
- LOCAL 服务失败仍为零 `ros::Time`/`INVALID`，没有新增
  `rospy.Time.now()` header 回退。

`host_receive_stamp` 仍按原 Stage B1 语义记录主机墙钟接收时刻，不被用来伪造
LOCAL `header.stamp`。

## 8. 统计和诊断

`UwbStatus.msg` 未修改。现有字段按以下逻辑更新：

- `raw_line_count`：全部非空输入行。
- `parsed_round_count`：至少含一条非零记录且经过 repeat filter 后成功发布的轮次。
- `parse_error_count`：未知协议行或非法 `distance` 语法；不含调试行、零槽位、
  正常空轮、启动于流中间的槽位。
- `range_reject_count`：仅非零记录经过尺度/bias/有效范围检查后的 reject。
- `repeat_drop_count` 和 `reconnect_count`：保留原有语义。

内部统计写入简要 `UwbStatus.detail`，并由 20 秒
`[UWB_PROTO]` 限频汇总日志输出。正常 `UWBDBG`、`TWR` 和 pending
distance 不逐行发布冗长状态；错误、空轮、完整轮及不完整轮会更新状态。

## 9. 修改文件清单

业务实现与配置：

- `src/uwb_serial_driver/src/uwb_serial_driver/parser.py`
- `src/uwb_serial_driver/scripts/uwb_serial_node.py`
- `src/uwb_serial_driver/config/default.yaml`
- `src/uwb_serial_driver/launch/uwb_serial.launch`
- `src/sensor_recording_bringup/launch/sensors_only.launch`
- `src/sensor_recording_bringup/launch/record_all.launch`
- `src/sensor_recording_bringup/launch/software_simulation.launch`

测试注册：

- `src/uwb_serial_driver/CMakeLists.txt`
- `src/uwb_serial_driver/package.xml`

测试：

- `src/uwb_serial_driver/test/test_parser.py`
- `src/uwb_serial_driver/test/data/distance_round5_sample.txt`（新增，49 行）
- `src/uwb_serial_driver/test/distance_round5_integration.test`（新增）
- `src/uwb_serial_driver/test/distance_round5_integration_test.py`（新增）

为使高速 file replay 的测试订阅者在回放前可靠连上，节点新增
`~replay_start_delay_s`；生产默认值为 `0.0`，只在集成测试中设为 `1.0`，
不改变正常启动时序。

未修改：

- `src/uwb_serial_driver/msg/UwbRange.msg`
- `src/uwb_serial_driver/msg/UwbRangeArray.msg`
- `src/uwb_serial_driver/msg/UwbStatus.msg`
- DTR/RTS 和串口独占锁实现
- LOCAL 时间服务
- GNSS、Livox、Camera、FAST-LIVO2
- 任何 bag

## 10. 默认配置和 launch 透传

当前硬件默认值：

```yaml
parser_mode: distance_round5
distance_lines_per_round: 5
round_timeout_s: 2.0
range_scale: 1.0
range_bias_m: {}
min_range_m: 0.05
max_range_m: 250.0
```

`uwb_serial.launch`、`sensors_only.launch` 和 `record_all.launch` 的实际
`roslaunch --dump-params` 均得到：

```text
/uwb_serial_driver/parser_mode: distance_round5
/uwb_serial_driver/distance_lines_per_round: 5
/uwb_serial_driver/round_timeout_s: 2.0
```

上层可通过 `uwb_parser_mode`、`uwb_distance_lines_per_round` 和
`uwb_round_timeout_s` 覆盖。Stage B1 的旧软件模拟数据不是七行硬件协议，
因此 `software_simulation.launch` 显式选择兼容模式 `pairs`；其现有回归通过。

全部相关 launch/XML 已通过 `xmllint --noout`。

## 11. 测试结果

### 纯 Python

执行了请求中的 `unittest discover` 命令，结果：

```text
Ran 25 tests
OK
```

其中新解析器测试 19 项，已有串口独占/DTR/RTS 测试 6 项。覆盖：
调试行隔离、五行轮次、后缀、真实 49 行样本、29 个零槽位、不完整轮次、
中途启动、多 anchor、重复正 anchor、时间上下文、超时、非法语法、尺度/bias、
serial/file 一致性、旧数字模式和 repeat filter。

### ROS file replay 集成

`catkin_make run_tests_uwb_serial_driver` 最终通过：

```text
UWB test result XML: 27 tests, 0 errors, 0 failures, 0 skipped
```

其中 rostest 功能测试为 1 项（catkin 同时列出 rostest 包装结果和 rosunit
用例结果）。断言实际验证：

- `/uwb/raw`：49 条（7 UWBDBG + 7 TWR + 35 distance）；
- `/uwb/ranges`：6 条 array；
- 距离顺序：`1.405, 1.362, 1.391, 1.374, 1.308, 1.342`；
- 每条 array 仅一个 `anchor_id=1`、`valid=true`；
- `source_format=distance_round5`；
- `parsed_round_count=6`；
- `parse_error_count=0`；
- `range_reject_count=0`；
- array 时间和 host receive 元数据等于对应轮次第一条非零 raw 帧；
- 第 5 个全零轮次不发布。

### 构建与全工作空间回归

执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/gulu/catkin_ws
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2
catkin_make run_tests_uwb_serial_driver
catkin_make run_tests
catkin_test_results build/test_results --all
```

结果：

```text
Release 构建：通过
全工作空间：113 tests, 0 errors, 0 failures, 0 skipped
```

Stage A、Stage B1 软件模拟及其他既有包测试均包含在这 113 项中。
`git diff --check` 通过。

## 12. 消息 MD5

修改前后相同：

```text
uwb_serial_driver/UwbRange
43c4ece2df281f479060c8c1c4ea5dcb

uwb_serial_driver/UwbRangeArray
f70481b55f677d52d6e6c5da524486ec

uwb_serial_driver/UwbStatus
d9a50c3bfe71d71344174953cab378a7
```

`git diff -- src/uwb_serial_driver/msg` 为空。

## 13. Jetson 实机验证命令

本节尚未在真实 Jetson/UWB 串口上执行。

构建：

```bash
source /opt/ros/noetic/setup.bash
cd /home/jetson/catkin_ws

catkin_make \
  -DROS_EDITION=ROS1 \
  -DCMAKE_BUILD_TYPE=Release \
  -j1

source /home/jetson/catkin_ws/devel/setup.bash
```

单独启动 UWB：

```bash
roslaunch uwb_serial_driver uwb_serial.launch \
  source:=serial \
  port:=/dev/ttyUSB1 \
  baud:=115200 \
  dtr:=true \
  rts:=false
```

检查：

```bash
rostopic echo /uwb/raw
rostopic echo /uwb/ranges
rostopic echo -n 1 /uwb/status
```

当前只部署 1 号基站时，期望 `/uwb/ranges` 只出现：

```yaml
ranges:
  - anchor_id: 1
    raw_range_m:  # 大于 0
    corrected_range_m:  # 大于 0
    valid: true
    source_format: distance_round5
```

不应出现由 `TWR` 生成的 anchor 2/3/4/5，也不应反复出现
`anchor_id=0, raw_range_m=0.0`。稳定运行时期望：

```text
serial_open: true
parse_error_count: 0
range_reject_count: 0
```

真实串口偶发截断可以使 `parse_error_count` 少量增加；正常 `[UWBDBG]`、
`[TWR]` 和零槽位不能增加该计数。还应观察 `detail` 中
`incomplete/malformed/duplicate` 是否持续增长，以判断链路是否丢行或截断。

## 14. 备份与回滚

备份目录：

`/home/gulu/catkin_ws/.codex_backups/uwb_distance_round5_20260726_170925/`

回滚脚本：

`/home/gulu/catkin_ws/.codex_backups/uwb_distance_round5_20260726_170925/restore.sh`

脚本已通过 `bash -n`。它会恢复本轮修改前的 10 个既有文件，并删除本轮新增的
3 个测试/样本文件和本报告。回滚会放弃本轮 UWB 修复，应只在明确需要时执行：

```bash
bash /home/gulu/catkin_ws/.codex_backups/uwb_distance_round5_20260726_170925/restore.sh
```

本轮未执行 `git commit`，未执行 `git push`，未覆盖任何 bag。
