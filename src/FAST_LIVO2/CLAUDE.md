# FAST-LIVO2 + GNSS/UWB 多源融合项目说明

## 1. 项目范围

本仓库是修改版 **FAST-LIVO2**（LiDAR-IMU-视觉前端），叠加：

- 独立 GTSAM `IncrementalFixedLagSmoother` 后端：GNSS 位置因子 + UWB 测距因子；
- 后端只输出优化位姿，不回写前端 ESIKF 的 state/P；
- 海康（mvs_ros_driver）、ZED、Livox/MID-360、串口 GNSS/UWB 采集链路均属于用户现有环境，修改时不得破坏。
- 2026-10-28 公开仓库整理后，公开 Git 树不再跟踪 `src/mvs_ros_driver`、
  `src/stm32_timersync-open` 与 `src/FAST-Calib`（后者原为无 `.gitmodules` 的 gitlink）；
  维护者本机文件仍保留，真实相机/固件/标定工具需从
  [LIV_handhold](https://github.com/xuankuzcr/LIV_handhold) 或
  [FAST-Calib](https://github.com/hku-mars/FAST-Calib) 单独安装。

Git 仓库根目录是工作区根（用 `git rev-parse --show-toplevel` 确认），本包位于 `src/FAST_LIVO2`。
本文件用 `<workspace>`、`<tunnel-bag>`、`<competition-bag>` 等占位符，避免把维护者本机路径写入公开仓库。
`src/FAST_LIVO2/.git` 是空目录，`git` 实际使用工作区根仓库；所有路径以工作区根为准。

当前基线（2026-10-28 核对）：

- 公开 `origin/main`（2026-10-28 整理前）：`e991e383 docs: document public tunnel reproduction and release status`；
  其算法提交为 `9fe7e510 fix(lio): use floor voxel keys and metre neighbourhood units`，树与旧
  SHA `5be5538` 相同。
- 2026-10-28 历史清理只移除了旧提交中的 Claude 自动 co-author trailer；旧 SHA `5be5538`、
  `9190aa4` 仅用于历史引用。本轮许可证/公开树整理在隔离分支
  `release/oss-cleanup-20261028`，尚未 push。
- `config/mid360.yaml` 与海康驱动存在用户未提交修改；不要 reset/checkout/stash 它们。
- 公开树新增 `src/rpg_vikit/LICENSE`（GPLv3）和五个自研包的 BSD-3-Clause `LICENSE`；
  `src/FAST_LIVO2/package.xml` 的 `BSD` 已更正为 `GPLv2`。GPLv2-only 与 GPLv3 的
  组合授权问题**未解决**，见 `THIRD_PARTY_LICENSES.md`。
- 公开隧道数据（维护者自采）：百度网盘 `https://pan.baidu.com/s/1vArBc4Gn5unoczOqP880RQ?pwd=mbpr`，
  提取码 `mbpr`，文件 `suidao2_resized.bag`，16,334,415,367 bytes；本地副本 SHA-256
  `9289318857b0ca009dab4570ac6f51915de1681fe60b2abc0b481c20d6711169`。链接/提取码已于
  2026-10-28 通过百度接口核验，但未重新下载；详见 `docs/experiments/tunnel.md`。

## 2. 架构与关键文件

```
LiDAR/IMU/Camera
  -> src/LIVMapper.cpp          事件调度、LIO/VIO 事务、地图提交
  -> src/IMU_Processing.cpp     IMU 传播、F/Q、deskew 历史
  -> src/voxel_map.cpp          LiDAR 体素地图、平面拟合、点面关联、ESIKF
  -> src/vio.cpp                视觉 patch、参考观测、VIO 更新
  -> src/gnss_adapter.cpp       GNSS 质量/原点/杆臂
  -> src/rtk_fixed_lag_backend.cpp  GNSS + UWB 固定滞后因子图
  -> src/preprocess.cpp         点云预处理
```

关键头文件：

- `include/common_lib.h`：`StatesGroup`、19 维误差状态（0-2 旋转，3-5 位置，6 曝光，7-9 速度，10-12 陀螺零偏，13-15 加计零偏，16-18 重力）。
- `include/utils/so3_math.h`：`Exp/Log`；状态更新 `operator+`/`+=` 调用标量 `Exp(v1,v2,v3)`。
- `include/voxel_map.h`：`VOXEL_LOCATION`、LiDAR 体素索引、平面查询辅助。
- `include/lio_update_transaction.h` / `include/livo_scan_lifecycle.h`：LIO 扫描事务。
- `include/vio.h`：视觉观测、参考 patch、VIO 事务。

约定：LiDAR 观测在 LiDAR 坐标系；外参 `extR_/extT_` 为 LiDAR→IMU；`state_.rot_end/pos_end` 为 IMU→world。误差状态旋转采用**右扰动**。

## 3. 构建与测试

主工作区（用户正式版本）：

```bash
cd <workspace>
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='sensor_time_msgs;sensor_time_bridge;gnss_comm;gnss_serial_driver;uwb_serial_driver;vikit_common;vikit_ros;fast_livo;sensor_recording_bringup'
source devel/setup.bash
rosrun fast_livo lio_plane_map_self_test
```

说明：

- 新修复应在独立 git worktree / 独立 build+devel 中编译，**不要覆盖** `devel/lib/fast_livo` 与正式二进制，除非候选已通过验证。
- `NDEBUG` 会屏蔽 `assert`；断言式自测目标已使用 `-UNDEBUG`。
- `lio_plane_map_self_test` 当前检查平面有效性、切向距离、体素索引和 Patch 正深度；修改对应几何行为时必须同步扩展/更新它。

常用自测：`lio_plane_map_self_test`、`lio_update_transaction_self_test`、`lio_motion_consistency_self_test`、`vio_update_transaction_self_test`、`preprocess_xyzirt_self_test`、`livo_scan_transaction_self_test`。

## 4. 回归数据与命令

### suidao2（隧道，约 355 s，Livox CustomMsg）

历史回归使用 **resized bag** 和 `config/mid360.yaml`（不是 `HH.launch`/`HH.yaml`）：

```bash
roslaunch fast_livo mapping_mid360.launch rviz:=false use_sim_time:=true \
  livo_scan_contract:=true cmd_name:=/tmp/suidao2_run
rosbag play --clock -r 1 -d 2 "$TUNNEL_BAG" \
  --topics /livox/lidar /livox/imu /left_camera/image
```

等价的独立 launch 关键参数：加载 `config/mid360.yaml`、`camera_pinhole_mid360.yaml`，
`/diagnostics/livo_scan_contract_en=true`，`/pcd_save/pcd_save_en=false`。
`config/mid360.yaml` 中用户未提交的 `common/img_en: 1` 必须保留；不要用 worktree 里
committed 的 `img_en: 0` 配置跑 LIVO。
未 resize 的原始包（图像 2600x2160）不是历史对照所用输入；
两者 LiDAR/IMU 相同，但不要混用结果。`HH.yaml` 的预处理（`point_filter_num=1`,
`filter_size_surf=0.1`）与历史 suidao2 配置不同，不能用于对比 0.5 m 基线。

### competition 804 s

```bash
roslaunch fast_livo mapping_competition.launch rviz:=false use_sim_time:=true output_root:=/tmp/competition_run
rosbag play --clock -r 1 -d 2 \
  "$COMPETITION_BAG" \
  --topics /tiaozhanbei/lidar /tiaozhanbei/imu /tiaozhanbei/camera/left/image/compressed
```

`mapping_competition.launch` 通过 `image_transport republish` 解码压缩图像；外部 GNSS/UWB 默认关闭。
Bag 内包含 `/tiaozhanbei/gnss/pose`，但前端无外部因子模式不订阅它。

评估口径：首末距离、最大扫描差分速度、完整 state/P、输入计数、事务一致性。首末距离是重访诊断，不是 ATE；两条轨迹互不为真值。

## 5. 当前状态与本轮待办

当前公开算法树：`9fe7e510 fix(lio): use floor voxel keys and metre neighbourhood units`
（`e991e383` 为公开 main 的文档基线；本轮许可证/公开树清理在其上单独提交，未 push）。
本轮不改算法。

已合入并通过回归的修复：

- `027fc06`：平面有效性（非退化特征值间隔/有限性）与 LIO/VIO 统一平方切向距离；
- `d9497af`：VIO 按 LiDAR 地图真实尺寸/索引查询，Patch 正深度检查；
- `02343a3`：视觉观测维护、参考选择、历史裁剪生命周期解耦；
- `9fe7e510`（旧 SHA `5be5538`，树相同）：LiDAR 体素 key 统一为 `floor(p/size)`（含精确负整数边界），
  邻域回退改为米制 `point_w` 与 `voxel_center_ ± quater_length_` 比较。验证：原生测试 PASS；
  suidao2 首末 `0.542 m`（基线 0.549 m）、233–260 s 反向 0.473 m；competition 804 s 首末
  `1.014 m`（历史基线 1.043/1.070 m）、4029 次 LIO、最大相邻扫描 1.27 m/s、无连续 >5 m/s。
  该隧道运行使用 `img_en=1`，是 LIVO 而非纯 LIO；首末距离不是 ATE。

仍未合入（详见 `reports/lidar_geometry_repair_20261028/README.md`）：

| ID | 位置 | 问题 | 2026-10-28 结果 |
|---|---|---|---|
| A2 | `voxel_map.cpp` 查询（及存图/插图） | 点协方差混用 LiDAR/IMU/world 系；旋转项未左乘 `R_WI`；缺姿态-位置交叉项 | 查询修正原生 PASS；suidao2 首末 2.159 m、300–340 s 反向 9.69 m → REJECTED；查询+存图完整版 suidao2 首末 15.316 m → REJECTED |
| A6 | `utils/so3_math.h` 标量 `Exp` | `norm <= 1e-5` 直接返回单位阵，状态更新死区 | 原生 PASS；suidao2 首末 0.5475 m、反向 0.224 m；**competition 首末 72.76 m → REJECTED** |
| 组合 A5/A1+A6 | — | 两项单独 suidao2 通过 | 组合 suidao2 首末 0.535 m，但 233–260 s 反向 **3.008 m** → REJECTED，不能叠加合并 |

A1/A5 与 A6 单独都通过 suidao2，但组合出现 3 m 反向段；A6 单独 competition 又严重漂移。因此只合入 A5/A1，A6 与 A2 保留为独立失败/待研究补丁。

历史数学反例（仍有效）：帧 2420 的 1.428 m 残差，旧查询方差 `0.54956 m²`（1.93σ，放行）；
用实际 `H` 与完整 19 维 `P` 计算为 `0.02318 m²`（9.38σ，应拒绝）。移除该关联后单次线性化
纵向位置增量由 9.26 cm 降到 1.00 cm。**该修正单独未通过长期轨迹回归**。

成功经验（`02343a3`）：当正确的关联/参考生命周期修复改变了轨迹时，隧道回退可消失。四项缺陷
应在当前 HEAD 上重新实现、独立验证，不能假设旧失败结论必然重演，也不能把旧补丁原样贴上；
单独 suidao2 通过也不能替代 competition 验证。

## 6. 工作约束（必须遵守）

1. 不调噪声、不关闭传感器、不限速、不放宽视觉或几何门限来“通过”回归。
2. 不修改 IMU/VIO 事务机制时不需要 DROP2；若修改了才需要。
3. 保留用户 `config/mid360.yaml`、海康驱动、ZED 工作树、GNSS/UWB 后端与锚点配置。
   公开树外置 `mvs_ros_driver`/`stm32_timersync-open`/`FAST-Calib` 不表示可以删除本机文件。
4. 修复候选未通过长期测试时：保留失败证据、单独存放补丁、不合并 main。
5. 不自动 push；不覆盖未经验证的正式二进制；提交/合并需在验证通过后谨慎执行。
6. 每个修复保持独立差异，便于定位回归；共享 helper 变更必须列出全部消费者。
7. 结论分层：已验证事实 / 推断 / 未观测（NOT_OBSERVABLE）/ 未运行（NOT_RUN），不要把局部数学正确或首末距离变小当作长期通过。

## 7. 已有证据位置

- 本仓库：`reports/landmark_*`、`reports/historical_schedule_map_20261013_resume/`、`docs/experiments/*`。
- 系统审计（外部盘，当前未挂载）：`<maintainer-diagnostics>/` 下的 `SYSTEMATIC_ESTIMATOR_AUDIT_20261026_152031/`、`FAST_LIVO2_SYSTEMATIC_BUG_FIX_20261026_155718/`、`SUIDAO_REGRESSION_20261027_123258/`、`SUIDAO_MEASUREMENT_REPAIR_20261027_163201/`、`SUIDAO_REFERENCE_MAINTENANCE_20261027_193954/`。
- Codex 会话记录：`<maintainer-session-logs>/`（包含 A1/A2/A5/A6 的反例、旧候选源码与失败结果）。
- 若外部盘未挂载，不要假装读取过其中文件；可从 Codex 会话中提取已记录的数字/补丁内容，但需标明来源是历史记录。

## 8. 已知易错点

- `git rev-parse --show-toplevel` 从本包运行会得到工作区根；提交路径带 `src/FAST_LIVO2/` 前缀。
- 体素尺寸单位是米；`point_w/voxel_size` 是索引，不要与米制中心/四分之一边长比较。
- `plane.plane_var_` 由地图点 `pointWithVar::var` 拟合而来；改点协方差必须同时审查查询、首次建图、后续插图、诊断分支。
- `StateEstimationInternal` 主循环、`rebuild_correspondences`、`p5EvaluateObjective`、P4 固定速度分支各有一套点协方差计算，容易漏改。
- 回归必须 `-r 1`；2 倍速不等价。EOF 后检查最后回调/事务完成，不能只看播放器退出。
验证必须使用**独立 ROS master 端口**（先启动 `roscore -p <随机端口>` 并设置
`ROS_MASTER_URI`/`ROS_IP`/`ROS_HOME`/`ROS_LOG_DIR`）。默认 11311 可能被用户
终端同时运行的 `mapping_mid360.launch` 占用，两个 mapper 会抢同一批 bag 话题，
导致轨迹与计数无效。发现端口冲突时不要杀用户的进程。

## 9. 许可证与公开发布

- `src/FAST_LIVO2/LICENSE` 是 GPLv2；上游没有 “or later” 授权，按 **GPLv2-only** 处理。
  `package.xml` 的 `BSD` 元数据已更正为 `GPLv2`，不是重新授权。
- `rpg_vikit` 的 `vikit_common`/`vikit_ros` 为 GPLv3，且 `homography.cpp`、
  `robust_cost.cpp` 实际编入 `libvikit_common.so` 并被 `fast_livo` 链接；
  `gnss_comm` 为 GPL-3.0-or-later。GPLv2-only 与 GPLv3 的组合授权问题**未解决**：
  submodule、独立 clone 或独立共享库都不能自行消除该问题。
- 五个自研 ROS 包为 BSD-3-Clause，已补充完整 LICENSE 文本；根目录 GPL 文本不覆盖它们。
- `mvs_ros_driver` 的 `TODO` 许可证、STM32 厂商/ST/ARM 声明不因为 LIV_handhold 顶层
  GPLv3 而自动解决；它们已移出公开跟踪，但本机保留。
- 需要上游书面澄清/例外：FAST-LIVO2 作者（Chunran Zheng、Fu Zhang）、
  rpg_vikit 维护者（Christian Forster）、robust_cost 作者（Christian Kerl）、
  gnss_comm 维护者（CAO Shaozu）。具体文件与联系方式见根目录
  `THIRD_PARTY_LICENSES.md`。不要自行宣称兼容性已解决。
