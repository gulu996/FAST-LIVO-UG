# P6 最终 production 冻结报告（2026-09-21）

## 最终状态

`CITY_FAILURE_FIXED = NO`

`CITY_FAILURE_RESEARCH_CLOSED = YES`

`PRODUCTION_BASELINE_FROZEN = YES`

`PRODUCTION_RECOMMENDATION = KEEP_BASELINE`

`P5_MULTISTART_ENABLED = NO`

本报告永久关闭 city1 约 900 s 快速漂移问题链。没有 P7；没有 commit 或 push。

冻结 production baseline 的含义是：保留已验证的 atomic transaction 与 VIO warp safety fix，所有 P2--P5 研究机制默认关闭。它不表示 city1 已修复。

## 1. Production 默认配置审计

实际加载链为：

```text
config/rtk_slam_dataset/ours.yaml
  -> launch/mapping_rtk_slam_dataset_ours.launch
  -> optional config_override（正式 production run 未提供）
  -> 源码参数默认值
```

| 阶段/机制 | 实际参数 | production 值 | 行为 |
|---|---|---:|---|
| P2 sequential hard gate | `lio_direction_guard/mode` | `off` | 不拒绝、不修改状态 |
| P2 map lifecycle | `lio_map_guard/mode` | `off` | 不冻结插图 |
| P2-D attenuation/damping | `lio_degeneracy/enable_state_intervention` | `false` | 无真实状态干预 |
| P2-D directional shadow | `lio_degeneracy/directional_shadow_enable` | YAML 未设，源码默认 `false` | 不运行 |
| P3 shadow/feedback | `fullstate_shadow/enable` | `false` | 节点仅供 child audit；无订阅、无反馈 |
| P4-A diagnostic | `p4_frontend/enable` | `false` | 不运行 |
| P4-B harness | `p4b_snapshot/enable`, `run_all_branches` | `false`, `false` | 不捕获、不分支 |
| P5 diagnostic | `p5_seed_basin/enable` | `false` | 不运行 |
| P5 multistart | `p5_seed_basin/prototype_enable` | `false` | registration seed 不变 |

seq2 与 construction 正式 run 均未加载 override；stdout 再次确认上述值。seq1 的细粒度诊断 run 只开启 CSV/内存 telemetry，并再次显式钉死全部研究机制；它不用于 production 性能通过判定。

## 2. Build、self-test 与静态门禁

指定构建命令：

```bash
catkin_make --pkg fast_livo -DCMAKE_BUILD_TYPE=Release -j2
```

结果：PASS。

- 17/17 个已有 C++ `*self_test` PASS。
- `vio_update_transaction_self_test` PASS，覆盖 finite warp、NaN/Inf、nonfinite coordinate、所有边界 OOB、最近合法 bilinear footprint、forward/inverse transaction。
- 17 个带 `--self-test` 的 estimator/diagnostic Python 工具 PASS。
- `tools/test_lio_sequential_shadow.py` PASS。
- scoped `git diff --check` PASS。
- 全量 `git diff --check` 仅报告既有 `tools/make_competition_submission/petrochemical_stage6b_schedule.json` 尾随空格：`PRE_EXISTING_UNRELATED_DIFF_WARNING`。P6 未修改它。

## 3. stadtgarten_seq1 最终故障结论

### 3.1 无 telemetry 的冻结 production baseline

权威 production 性能 run：

```text
/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/
raw_outputs/ours/stadtgarten_seq1/2026-09-21-143340-prefix
```

- 0.8x，prefix 912 s，runner/7 child exits/trajectory/SHA-256 PASS。
- 18,234 raw LIVO rows。
- LIO `18200/18200/0`，VIO `18199/12270/5929`，overflow `0`（末次周期快照）。
- 900--906 s speed median/p95/max：`4.1038 / 6.0354 / 6.3543 m/s`。
- correspondence min/median/max：`314 / 527 / 660`，零 correspondence 帧 `0`。
- frame 18000 Schur-conditioned translation eigenvalues：`(9955.47, 68444.55, 79222.32)`。
- wall `1171.298 s`，CPU `246%`，max RSS `2,307,308 KiB`；末次 LIO event rate `15.538 attempt/s`（固定 0.8x replay）。

### 3.2 P6 当前二进制细粒度诊断复核

```text
/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/
raw_outputs/ours/stadtgarten_seq1/2026-09-21-174106-prefix
```

该 run 使用 telemetry-only override 收集逐帧 geometry；所有 estimator/research 参数保持 production 值。runner、7 child exits、两份 3620 行 backend trajectory、SHA-256 PASS。

900--906 s：

| 指标 | P6 结果 |
|---|---:|
| raw LIVO rows / gaps | 120 / 119 |
| speed median / p95 / max | `4.6953 / 5.9888 / 6.3676 m/s` |
| timestamp gap p95 / max | `0.05242 / 0.05554 s` |
| correspondence min / median / max | `356 / 510 / 645` |
| Schur min eig min / median / max | `5133.96 / 11036.10 / 18361.07` |
| frame 18000 Schur eigenvalues | `(14029.80, 62888.31, 82164.86)` |
| LIO commit / reject | `120 / 0` |
| map insertion | `120 / 120` |
| VIO EKF attempt / accept / reject in window | `0 / 0 / 0` |
| safely rejected OOB warp candidates | `1235` |

整段 LIO `18200/18200/0`，VIO `18199/12260/5939`，其中 12,260 次 accepted 证明 VIO 没有被关闭或削弱。整段 OOB warp candidate `60,763` 个均安全拒绝；无 nonfinite coordinate reject、SIGSEGV 或 abnormal exit。

telemetry 写入 `visual_patch_quality.csv` 和 `motion_consistency_shadow.csv` 带来额外 I/O，约 670 s 时产生 8 次有限队列 overflow；因此该 run 只用于逐帧诊断，不用于 production 性能或 overflow gate。无 telemetry 的正式 baseline overflow 为 0。

P6 与 P5 的速度、数百 correspondence 和非零 Schur geometry 属于同一 failure class。没有提前进入 inertial-only flight，也没有修复约 5--6 m/s build-up：

`CITY_FAILURE_FIXED = NO`

## 4. Normal production regression

两套正式 run 均为 0.8x、无 override、ground truth 不进入估计器。

### stadtgarten_seq2 0--600 s

```text
.../raw_outputs/ours/stadtgarten_seq2/2026-09-21-180218-prefix
```

- runner、7 child exits、1972 行 online/offline backend trajectory、SHA-256 PASS。
- raw LIVO 11,995 rows，覆盖 0.20--599.90 s。
- gap p95/max `0.05208/0.05921 s`。
- translation increment p95/max `0.07823/0.11784 m`。
- rotation increment p95/max `3.410/8.400 deg`。
- speed p95/max `1.5579/2.1451 m/s`。
- LIO `11800/11800/0`；VIO `11799/10029/1770`；overflow `0`；abnormal exit `0`。
- wall `778.211 s`，CPU `294%`，max RSS `1,704,564 KiB`，末次 LIO event rate `15.163 attempt/s`。

这些指标与 P5 冻结 S0-only normal baseline 一致；没有 prototype 的 11.8% frame loss、长 gap 或 pose-step tail。

### construction_seq2 有效 bootstrap

```text
.../raw_outputs/ours/construction_seq2/2026-09-21-181542-smoke
```

- 输入明确为 `play_start=+1.047924 s`、310 s；不是无效 prefix0。
- runner、7 child exits、855 行 online/offline backend trajectory、SHA-256 PASS。
- raw LIVO 6,194 rows，覆盖 0.25--309.91 s。
- gap p95/max `0.05208/0.06395 s`。
- translation increment p95/max `0.07898/0.10257 m`。
- rotation increment p95/max `3.109/7.997 deg`。
- speed p95/max `1.5720/1.9839 m/s`。
- LIO `6000/6000/0`；VIO `5999/4661/1338`；overflow `0`；abnormal exit `0`。
- wall `414.452 s`，CPU `289%`，max RSS `1,544,292 KiB`，末次 LIO event rate `14.477 attempt/s`。

这些指标与 P5 冻结 S0-only construction baseline 一致；没有 prototype 的 20.9% frame loss。

当前日志没有独立的“LiDAR callback received 但未形成 split-LIO segment”最终 drop counter；因此不伪造 drop=0。可审计的处理量是 raw LIVO rows、LIO attempted/committed/rejected、trajectory gap 和 buffer overflow。两套 normal run 的 LIO reject 与 overflow 均为 0。

## 5. VIO warp safety regression

- Release `vio_update_transaction_self_test` PASS。
- seq2 0--600 s：VIO 11,799 attempts、10,029 accepted，运行至目标末端，无 SIGSEGV。
- construction：VIO 5,999 attempts、4,661 accepted，无 SIGSEGV。
- seq1 telemetry run：60,763 个 OOB candidate 安全拒绝并继续运行到 912 s。
- invalid warp 被逐 feature 跳过；视觉总体仍启用且大量 accepted，没有通过关闭/削弱 VIO 过门。

因此既有 warp safety fix 无回归。

## 6. Tunnel 与 real-data regression

### suidao2

```text
/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/p6_release_20260921/
suidao2_2026-09-21-182914
```

- 完整 355.09 s bag；status 与 SHA-256 PASS；abnormal exit `0`。
- 运行时 `local_map/half_map_size=100`，`lio/voxel_size=0.8 m`；该参数是 root-voxel half extent，约每轴 `+/-80 m`，不是字面 100 m。
- 3,537 raw LIVO rows、354.00 s；max step `0.2638 m`，`>0.3 m` step `0`，没有快速后退型跳变。
- LIO `3400/3400/0`；VIO `3399/72/3327`；overflow `0`。
- closure 3D `0.5326 m`，XY `0.5193 m`，delta-z `-0.1182 m`。这属于已知隧道覆盖限制，不转化为新研究阶段。

`TUNNEL_REGRESSION = PASS`

### MID360 804 real bag

```text
/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/p6_release_20260921/
804_2026-09-21-183613
```

- 完整 1194.96 s real bag；LiDAR/IMU/Image 与 `/gnss/pvt_local` schema 实测匹配。
- `average_fixed` 10/10 成功建立真实 GNSS origin。
- status 与 SHA-256 PASS；abnormal exit `0`；无残留 owned 进程。
- 11,918 raw LIVO rows、1194.30 s；gap p95/max `0.10032/0.20017 s`；max step `0.3091 m`。
- LIO `11800/11800/0`；VIO `11799/103/11696`；overflow `0`。

`REAL_DATA_REGRESSION = PASS`

## 7. 研究结论冻结

- **P1**：candidate rollback/map contamination 防护正确，但没有解决故障。
- **P2/P2-D**：weak translation direction 可经 cross covariance 主导 hidden-state velocity correction；attenuation/damping 对正常/故障没有足够选择性。
- **P3**：显式 X/V/B + IMU preintegration 仍复制故障，故障不是 ESIKF hidden-state 架构独有。
- **P4-A**：未发现 failure-specific correspondence churn、持续 signed residual bias 或 deskew 异常。
- **P4-B**：snapshot 工程验证成功，但 diagnostic B0 未重现原始 failure，不能建立 map causal 结论。
- **P5**：failure-specific seed sensitivity 存在，但不构成稳定 local basin；唯一 multi-start prototype 未修复 seq1，且破坏 normal 实时处理。

现有证据不支持继续叠加 heuristic。city1 根因没有被唯一因果隔离，这是未解析限制，不是后续阶段。

## 8. Git 审计与修改分类

P6 没有修改任何算法、production YAML 或 launch；本轮仓库内只新增本报告。当前 dirty worktree 保留了 P1--P5 与用户既有修改：

1. **production safety fix**：atomic LIO/VIO transaction、VIO warp validity guard 及其必要调用链。
2. **diagnostic/shadow**：P2-D motion/source shadow、P3 full-state shadow、P4-A/P4-B、P5 seed-basin；production 全部默认关闭。
3. **self-test**：LIO transaction/motion、full-state、P4 snapshot/frontend、VIO transaction 等。
4. **evaluator/report**：P2--P6 analyzer、metrics 和报告。
5. **pre-existing user changes**：petrochemical/GNSS/RTK 配置等既有 dirty 内容，P6 未覆盖。
6. **unrelated dirty files**：competition schedule 尾随空格与其它既有 RViz/展示文件；P6 未修复、未计入 scoped gate。

未执行 reset、clean、checkout、stash、commit 或 push。

## 9. Known limitations

- city1 weak-geometry failure 仍未修复。
- 根因未被唯一因果隔离；当前 evidence 不支持继续加入 heuristic。
- suidao2 当前闭环 3D 为 0.533 m，室内/超长隧道覆盖仍有限。
- production parallel execution 仍存在历史已知非确定性边界。
- 周期事件快照没有单独给出最终 LiDAR callback-to-segment drop counter；报告只使用可直接审计的处理量和连续性指标。

## Final

`CITY_FAILURE_FIXED = NO`

`CITY_FAILURE_RESEARCH_CLOSED = YES`

`PRODUCTION_BASELINE_FROZEN = YES`

`PRODUCTION_RECOMMENDATION = KEEP_BASELINE`

`P5_MULTISTART_ENABLED = NO`
