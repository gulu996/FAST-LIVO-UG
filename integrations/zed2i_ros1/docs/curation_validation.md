# 整理副本的软件验证（2026-10-15）

本次仅在独立 Git worktree 创建 integrations/zed2i_ros1，构建/数据输出放在外部临时目录。
没有启动、停止、重启相机或雷达节点，没有重新做硬件曝光/同步验收。
原始 L1/L2-B 工作空间及原仓库 dirty 文件保持不动。

## 实际变更

- L1 启动、预览、录包脚本使用 ZED_WS/ZED_EVIDENCE_DIR/ROS_MASTER_URI；启动文件按脚本位置定位。
- 相机真实序列号从副本删除，改为 launch serial_number 参数，默认0；方向及标定设置保留。
- 预览使用标准 ctypes.util.find_library 查系统 OpenCV，仍仅限制本预览进程线程数。
- L1 两个审计工具要求显式新输出目录，拒绝覆盖；bag自检包含防覆盖断言。
- L2-B 协议模块通过环境变量、显式record参数或已有Python导入路径加载；CMake测试路径显式检查。
  测试不再访问原开发者绝对路径。time_mapping.py和correlate_imus.py内容保持原样。
- 独立CLI检查发现预解析器把 --anchor 当成 --anchor-module-root 缩写；副本设置allow_abbrev=False。
  新增check_integration.py覆盖该根因、三种协议导入方式、带空格路径、master透传及bag防覆盖。
- 原一次性回放脚本不保留：已有ros_audit、bag_audit及README命令覆盖所需功能。
- 完整本机报告改为公开技术摘要；不复制官方仓库、采集数据或设备标识。

## 本轮复测结果

| 检查 | 结果与范围 |
|---|---|
| 单包catkin/CMake configure + build | PASS；独立devel空间，仅overlay /opt/ros/noetic；Python包，无新增C++二进制 |
| CTest/unittest | 36/36 PASS，9.093s；CTest9.36s |
| 精度/超时/会话/双目/原戳 | 原36项覆盖整数纳秒、int64边界、漂移、锚点陈旧/停更、重启/跳时/回退、缓存拒绝、双目冻结和独立IMU |
| 新参数化自检 | PASS；Python AST、bash -n、XML、配置门限、模拟roslaunch/rosbag及三种CLI协议路径+重放 |
| L1 ros_audit --self-test | PASS；精度、频率、配对、异常时间、/proc解析 |
| L1 bag_audit --self-test | PASS；临时小ROS bag往返、双目同戳、篡改检测、已有输出拒绝 |
| YAML/OpenCV | PASS；OFF保留字符串、关键配置匹配；系统OpenCV查找/线程设置成功，未开GUI |
| roslaunch --dump-params | PASS；解析真实固定官方launch，检查serial覆盖、HD1080/30、flip OFF、深度/跟踪关闭；未启动节点 |
| rospack / rosrun | PASS；隔离devel找到包，CLI --help可用；双IMU无输入返回HARDWARE_PENDING |
| 已有有效合成sidecar重放 | 78记录、54事件全部有效，完整结果0差异 |
| 已有L1元数据sidecar重放 | 18788事件全部保持CLOCK_MISSING invalid，完整结果0差异；不编造缺失历史锚点 |
| 新主机时钟记录及重放 | 3秒，610记录（60夹测+550锚点轮询），0传感器事件，0差异；显式指向临时目录缺失anchor，未访问真实雷达 |

构建命令、软件检查入口见上级README。首次路径检查暴露的参数缩写冲突已修复，
最终结果采用修复后的测试；未修改原始证据掩盖初次失败。
软件测试没有将系统时钟跳变施加到真实主机；相关故障全部内存合成。
测试源码和外部输出分离，没有生成文件进入提交。

## 仍未验证

本次没有重新编译整个官方Wrapper或原FAST-LIVO2，没有重新连接相机、预览或完整硬件bag回放；
这些仅保留L1历史结果。整理后的脚本经过配置解析、软件bag和命令模拟验证，硬件复测仍需独立窗口。
没有MID-360实机同步、双IMU真实运动、双设备性能A/B、曝光/卷帘光学校准或Jetson/Open Capture验证。
有效软件映射不等于物理曝光同步。
