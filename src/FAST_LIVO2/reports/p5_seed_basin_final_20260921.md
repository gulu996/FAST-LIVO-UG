# P5 seed-basin / weak-axis multi-start 最终报告（2026-09-21）

## 最终状态

`P5_BASELINE_REPRODUCED = YES`

`SEED_BASIN_MECHANISM_SUPPORTED = NO`

`PROTOTYPE_READY = NO`

P5 已到达负结论终态，可进入 P6 做最终工程验收与结论冻结。P4-B 保持永久关闭；本阶段没有重开 snapshot、B1/B2、frozen map、recent-N 或 P4-C。

这里需要区分两个结论：

- `FAILURE_SPECIFIC_SEED_SENSITIVITY_OBSERVED = YES`：seq1 的五 seed 终点分散、profile 不规则性和 LiDAR/MAP 排名分歧都明显强于两个 control。
- 但这些差异不足以证明稳定的离散 local basin 是约 5 m/s build-up 的核心机制：绝大多数终点仍被压缩到同一小邻域，profile 只在少数帧呈现两个内部极小值；最重要的是，唯一获准的三 seed MAP prototype 没有消除 build-up，并损伤了正常集的实时处理。因此正式机制状态为 `NO`。

全阶段未使用 GT 做 seed、candidate 选择、阈值、调参或结论方向判断。所有 runner manifest 均记录 `ground_truth_used=false`。

## 1. Baseline gate

正式 production 配置以 0.8 倍速重跑：

```text
/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/
raw_outputs/ours/stadtgarten_seq1/2026-09-21-143340-prefix
```

runner、7 个 roslaunch child exit、轨迹及 SHA-256 manifest 均 PASS。900--906 s 的 raw-LIVO pose-derived speed 为：

| 指标 | median | p95 | max |
|---|---:|---:|---:|
| speed (m/s) | 4.1038 | 6.0354 | 6.3543 |

同窗 production 最终 correspondence 为 314 / 527 / 660（min / median / max），零 correspondence 帧为 0；frame 18000 的 Schur-conditioned translation eigenvalues 为 `(9955.47, 68444.55, 79222.32)`。因此 baseline 属于历史 G0 同类：约 5 m/s 级 build-up、仍有数百 correspondence、geometry 非零、没有提前 inertial flight。

baseline runtime：1171.298 s wall、246% CPU、2,307,308 KiB max RSS。

先前 1.0 倍速的 `2026-09-21-141021-prefix` 没有复现 G0，已被正确的 0.8 倍速 gate 取代，不参与结论。

## 2. 固定 diagnostic protocol

- failure：`stadtgarten_seq1 899--906 s`
- controls：`stadtgarten_seq2 300--307 s`、`construction_seq2 300--307 s`
- 固定 stride：每 10 个 LIO frame
- `L = preprocess/filter_size_surf = 0.25 m`
- weak direction：production point-to-plane 6x6 information 对 rotation 做 Schur complement 后的 `Lambda_t|r` 最小特征向量
- seeds：`S0, S-1, S+1, S-2, S+2 = 0, -0.5L, +0.5L, -L, +L`
- profile：仅固定的 `alpha = {-1,-.75,-.5,-.25,0,.25,.5,.75,1}`
- joint objective：`J_lidar + delta_pose^T P_pose^-1 delta_pose`，没有额外 prior 权重

三套 read-only shadow run：

```text
seq1 failure:
.../stadtgarten_seq1/2026-09-21-153034-prefix/native/frontend/
    2026-09-21-153059/p5_seed_basin

seq2 control:
.../stadtgarten_seq2/2026-09-21-151135-prefix/native/frontend/
    2026-09-21-151147/p5_seed_basin

construction control (valid smoke start):
.../construction_seq2/2026-09-21-152319-smoke/native/frontend/
    2026-09-21-152326/p5_seed_basin
```

共 38 个 sample frame 的 state、covariance、production correspondence 和 point-count restore parity 全部 exact PASS。

## 3. Diagnostic 结果

| 数据 | frames | final pose span median / max | weak span median / max | LiDAR/MAP 排名分歧 | weighted-RMS 双内部极小值 |
|---|---:|---:|---:|---:|---:|
| seq1 failure | 14 | 5.058 mm / 45.102 mm | 4.995 mm / 39.376 mm | 7/14 | 4/14 |
| seq2 control | 14 | 0.153 mm / 0.303 mm | 0.089 mm / 0.251 mm | 0/14 | 0/14 |
| construction control | 10 | 0.439 mm / 1.317 mm | 0.287 mm / 0.858 mm | 1/10 | 0/10 |

seq1 有 12/14 帧的 final pose span 超过两个 control 各自的最大值；最大帧在 905.298 s 达 4.51 cm，并伴随最低 association Jaccard 约 0.67。这个 failure-specific 信号足以触发协议允许的唯一 prototype。

但它不是稳定的离散多盆地证据：seq1 median final separation 仅为 `0.020 L`，最大也只有 `0.180 L`；五个初始 seed 大多连续、单调地收缩到同一方向的小邻域。14 帧中只有 4 帧出现两个 weighted-RMS 内部极小值，另有全局 argmin 落在 profile 边界，不能把边界下降直接解释为局部 basin。association sensitivity 也非 failure 独有（seq2 的最低 overlap 同样可低于 0.8）。

seq1 shadow 的 S0 correspondence 为 163 / 295.5 / 704，最弱 conditional eigenvalue 为 6442.8 / 12095.1 / 20316.9（min / median / max），所以 diagnostic 信号不是零 geometry 伪影。

## 4. 唯一 prototype 与 covariance 语义

实现的是协议唯一允许的 `Weak-Axis Multi-Start MAP Registration`：

1. S0 用原始 propagated state 运行真实 production scan-to-map solver，并从其 production geometry 得到 `u_weak`。
2. 从同一 propagated state/covariance 运行 `S-1` 和 `S+1`，translation 分别为 `-0.5L u_weak`、`+0.5L u_weak`，rotation 不变。
3. 三者运行完全相同的 transform、voxel query、plane association、H/R、IEKF relinearization、step limit、transaction validation。
4. 只在 committed 且 objective finite 的 candidate 中选择 `argmin(J_lidar + J_prior)`；没有 speed/residual/GT/timestamp/bag selector。
5. 恢复被选 candidate 自身的 state、最后 linearization correspondence、diagnostics、hysteresis 和 posterior covariance。随后只有该 candidate 进入现有 `_pv_list` / map-insertion 路径，其他 candidate 不插图。

production 默认配置中 diagnostic 和 prototype 均为 `false`；prototype 只由 `p5_weak_axis_multistart.yaml` 显式启用。

三套正式 prototype run 中：

- 31,354 / 31,354 frame 都恰好有 S0/S-1/S+1、唯一 selected、且 selected 与无调权 MAP argmin exact 一致；
- selected commit 31,354 / 31,354，reject 0；
- selected covariance 全 finite；
- covariance maximum asymmetry `2.13e-17`；
- covariance minimum eigenvalue `2.72e-9`；
- 非 S0 被选 22,450 帧。

因此 prototype 的 candidate selection、atomic commit 和 chosen-basin covariance 语义通过；失败原因不是 covariance 复用错误。

## 5. Prototype 正式验证

### seq1 full/prefix 912 s

```text
.../stadtgarten_seq1/2026-09-21-163005-prefix
```

runner、7 child exits、3380 行 online/offline trajectory 均 PASS。900--906 s：

| 指标 | S0-only baseline | 3-seed prototype |
|---|---:|---:|
| speed median (m/s) | 4.1038 | 4.5524 |
| speed p95 (m/s) | 6.0354 | 5.6510 |
| speed max (m/s) | 6.3543 | 6.2371 |
| max timestamp gap (s) | 0.05554 | 0.05554 |

prototype 没有消除 build-up，median 反而更高。目标窗 selected correspondence 为 176 / 304.5 / 523；仍有非零 geometry，轨迹时间连续。120 帧中选择 S-1/S+1/S0 = 48/45/27，说明失败不是“prototype 从未选择 alternate seed”。

### seq2 prefix 620 s

```text
.../stadtgarten_seq2/2026-09-21-161439-prefix
```

runner、7 child exits、1974 行 online/offline trajectory 均 PASS。与可比的 0.8x S0-only 内部轨迹在共同 60--600 s 比较：

| 指标 | S0-only | prototype |
|---|---:|---:|
| trajectory rows | 10799 | 9522 |
| timestamp gap p95 / max (s) | 0.0521 / 0.0592 | 0.1000 / 0.2522 |
| translation increment p95 / max (m) | 0.0783 / 0.1178 | 0.1204 / 0.3551 |
| rotation increment p95 / max (deg) | 3.388 / 8.400 | 4.060 / 20.042 |
| speed p95 / max (m/s) | 1.558 / 2.145 | 1.546 / 2.089 |

速度包络没有爆炸，但可处理轨迹帧减少 11.8%，并出现明显更长的时间缺口和更大的 pose step。selected candidate 无 reject，日志采样点 `map_insert_skipped=0`；损伤来自三完整 registration 的处理负载/采样稀疏，而不是 transaction reject。

### construction valid smoke 310 s

```text
.../construction_seq2/2026-09-21-165033-smoke
```

runner、7 child exits、823 行 online/offline trajectory 均 PASS；实际 start offset 为 runner 固定的有效 `+1.047924 s`。共同 20--307 s：

| 指标 | S0-only | prototype |
|---|---:|---:|
| trajectory rows | 5702 | 4513 |
| timestamp gap p95 / p99 (s) | 0.0521 / 0.0556 | 0.1514 / 0.2482 |
| translation increment p95 (m) | 0.0795 | 0.1889 |
| rotation increment p95 (deg) | 3.161 | 4.873 |
| speed p95 / max (m/s) | 1.575 / 1.984 | 1.561 / 1.999 |

可处理轨迹帧减少 20.9%，正常集同样受损。

启动阶段的 `stadtgarten_seq2/2026-09-21-161158-smoke` 没有有效 correspondence，且 60 s 时 RTK trajectory converter 无有效行，已按门禁排除；construction 的无效 prefix0 attempt 也未用于结论。

## 6. Runtime

| 数据 | wall / CPU / max RSS | 3-registration frame time median / p95 / p99 / max |
|---|---|---|
| seq1 prototype | 1177.212 s / 384% / 2,336,740 KiB | 26.15 / 97.27 / 158.13 / 344.02 ms |
| seq2 prototype | 797.938 s / 398% / 1,865,660 KiB | 24.84 / 90.02 / 176.25 / 251.75 ms |
| construction prototype | 413.604 s / 393% / 1,741,200 KiB | 28.81 / 135.26 / 228.89 / 313.57 ms |

S0-only seq1 是 1171.298 s / 246% / 2,307,308 KiB；construction S0-only 是 410.200 s / 307% / 1,634,388 KiB。wall time 看似接近是因为 rosbag 按固定时钟继续发布；正常集实际出现 11.8%--20.9% 的 frame loss/稀疏化。p95/p99 也明显超过约 20 Hz LIO 的 50 ms frame budget，因此 runtime 不可接受。协议禁止通过异常 detector、改 queue、改 rate 或改 OMP 来回避此成本。

## 7. 十五项回答

1. **正式 production 是否重现约 5 m/s 故障？** 是。0.8x baseline 的 900--906 s speed median/p95/max 为 4.10/6.04/6.35 m/s，correspondence 数百且 geometry 非零。
2. **weak translation direction 如何定义？** production point-to-plane 6x6 information 对 rotation 做 Schur complement 后，`Lambda_t|r` 最小特征值对应的单位 translation eigenvector。
3. **L 来自哪里？** 实际 scan-to-map preprocess leaf：`preprocess/filter_size_surf = 0.25 m`。
4. **五 seed 在 failure 中是否进入不同 basin？** 出现 failure-specific 终点分散，但不足以认定频繁进入稳定离散 basin；median 仅 5.06 mm，映射大多连续并收缩到同一方向邻域。
5. **两个 normal 是否相同行为？** 否。control 的 final pose span 最大仅 0.303 mm 和 1.317 mm，显著更稳定。
6. **profile 是否多峰/多盆地？** 仅 seq1 的 4/14 帧有两个 weighted-RMS 内部极小值，多数帧不是稳定多盆地；control 为 0。
7. **LiDAR-only 与 joint MAP 排名是否不同？** 是。seq1 7/14 帧不同；seq2 0/14，construction 1/10。
8. **是否支持 seed/local-basin 核心机制？** 否。存在 seed sensitivity，但稳定离散 basin 证据不足，且获准 prototype 不改变故障 build-up。
9. **是否实现唯一三 seed prototype？** 是，仅 S0 和 weak-axis `+/-0.5L`，默认关闭。
10. **prototype 如何选择？** 在 committed/finite candidates 中精确选择无调权 `argmin(J_lidar + J_prior)`；31,354 帧审计 exact PASS。
11. **covariance 如何与 chosen basin 一致？** commit 被选 candidate 自己的最后 IEKF linearization posterior covariance，并连同对应 state/correspondence/hidden diagnostics 一起恢复；finite/symmetric/PSD 全通过。
12. **seq1 build-up 是否改变？** 没有实质改善；median 4.10 -> 4.55 m/s，max 6.35 -> 6.24 m/s。
13. **normal 数据是否受损？** 是。seq2/ construction 可处理轨迹帧分别减少 11.8%/20.9%，gap 和 pose-step tail 增大。
14. **runtime 是否可接受？** 否。三注册 frame time p95 90--135 ms、p99 158--229 ms，超过 50 ms frame budget，并导致正常数据稀疏化。
15. **是否达到 P6 最终验收条件？** 达到负结论的工程关闭条件，可进入 P6 冻结；没有达到正向 prototype acceptance。

## 8. 验证与产物

- Release build：`catkin_make --pkg fast_livo -DCMAKE_BUILD_TYPE=Release -j2` PASS
- `p4_frontend_diagnostics_self_test` PASS（含 P5 prior/association/MAP selector）
- `lio_update_transaction_self_test` PASS
- `p4_fork_snapshot_self_test` PASS
- `vio_update_transaction_self_test` PASS
- `evaluate_p5_seed_basin.py --self-test` PASS
- `evaluate_p5_multistart.py --self-test` PASS
- `evaluate_p5_trajectory.py --self-test` PASS

机器可读汇总：

- `reports/p5_seed_basin_metrics_20260921.json`
- `reports/p5_multistart_seq1_20260921.json`
- `reports/p5_multistart_seq2_20260921.json`
- `reports/p5_multistart_construction_20260921.json`
- `reports/p5_seq1_failure_window_trajectory_20260921.json`
- `reports/p5_seq2_trajectory_internal_20260921.json`
- `reports/p5_construction_trajectory_internal_20260921.json`
- `reports/p5_construction_target_window_20260921.json`

没有 commit，没有 push。

`PROTOTYPE_READY = NO`
