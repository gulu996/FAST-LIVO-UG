# Re-observation Counterfactual Contribution Audit — 2026-10-02

本轮完成 20m、40m 的完整真实回放和隔离的 A/B/C batch counterfactual。
**40m 原来的 2.883315 m 大变化主要来源于 MOTION_ONLY 路径：默认 B
已有 2.859385 m，新增 return Landmark 的 B→C 边际作用为 0.212211 m。**
这些是 Shadow Graph 内部的反事实作用量，不是实际轨迹精度或 production correction。
审计结束后停止软件功能扩展，等待正式 unique-ID 实体 Board。

## 1. 数据、事件与生产隔离

| 数据 | 完整路径 | `rosbag info` duration | ID1 return |
| --- | --- | ---: | --- |
| 20m | `/home/gulu/data/qr_detect/20m.bag` | 119.227516 s | observation 24，K14，episode 1→3，relative t=83.899724 s |
| 40m | `/home/gulu/data/qr_detect/40m.bag` | 119.351045 s | observation 20，K12，episode 2→3，relative t=117.500253 s |

均为 `LEGACY_SAME_ID_REAL_REPLAY`，不是正式 unique-ID Board 数据。实际 topics：
`/left_camera/image` (`sensor_msgs/Image`，596 条)，`/livox/lidar`
(`livox_ros_driver2/CustomMsg`，596 条)，`/livox/imu` (`sensor_msgs/Imu`)。
先 20m，再 40m，完整 rate=1.0 回放；没有换用其他 bag，也没有以 ID3 为核心分析。

与上一轮保存的有效参数比较，`aruco_landmarks`（除新增只读捕获开关）、
`extrin_calib`、`vio`、`lio`、`common`、`diagnostics` 块均相同。
Candidate A、legacy 实物几何、PnP、8 px gate、motion/visual covariance、
factor weights、SparseKeyPose policy 均未修改。没有改 ESIKF、LiDAR 执行逻辑、
GNSS/UWB、fixed-lag 或 publisher。既有 fixed-lag backend 仍是 production
correction owner；`latestCorrection()` 仍返回无有效全局地标修正。

## 2. 为什么需要重新回放

上一轮 CSV 有当前事件测量、K/L 局部 pre/post 值，但没有完整旧因子集合和
所有旧变量的 pre-estimate，不能严格重建相同 pre-state。因此本轮补充默认
关闭的 `/aruco_landmarks/global_backend/counterfactual_snapshot_enable`。
独立回放 wrapper 显式启用该只读开关，不改任何生产 YAML。

在正式 return 更新之前保存：原图全部因子、`estimate_`、原样 `new_values`
raw seed、pending normal motion factor、真实 candidate visual factor；
正式原有一次 `isam_.update(factors, values)` 后保存实际 runtime post Values。
图只追加不可变的 Pose3 Prior/Between 因子，快照保留独立 Values 和冻结的
factor list，不复制 ISAM2 Bayes tree，不额外 update、不替换估计、不 rollback。
捕获错误独立计数，不改变原图 admission/update/failure 路径。

快照在正常 worker shutdown drain 后落盘。文本以 17 位精度保存全部 R/t、
完整 6×6 covariance、key 和事件标识；支持当前真实图的 Pose3 Gaussian
Prior/Between 类型，未知类型/噪声、非有限值、非 SPD covariance 拒绝读取。
原 Candidate A 的 rounded R 在真实快照中的 SO(3) 偏差为约 `2.819e-6`。
最初过严的 `1e-8` 读取检查拒绝了它；离线检查改为 `1e-4` near-rotation
容差，**保留矩阵原始数值，不投影、不归一化、不修改标定**。自测覆盖此情况。
因此直接复用已捕获的 20m 快照，不为读取器修正重复回放。

记录的 20m return pre 图为 26 factors / 15 variables，40m 为 22 / 13。
每个真实事件新增一个 K、一个正常 motion factor、一个 return visual factor；
L1 已存在，继续作为无 prior 的 Pose3 random variable。

## 3. A/B/C 的精确定义

- **A / PRE / RAW**：旧图、旧 Values，不优化、不加 return motion/visual。
  新 K 的 raw seed 只用于候选残差/位移评估，不是旧 ISAM2 中已有的变量。
- **B / MOTION_ONLY**：同一旧图 + 同一新 K raw seed + 正常 return motion，
  不加入 candidate return visual。旧图中原有的视觉因子仍保留。
- **C / MOTION_PLUS_VISUAL**：从完全相同的 initial Values 独立开始，
  B 的因子集合 + 同一真实 candidate visual。不是从 B 优化结果 warm start。

B/C 使用安装的 GTSAM 临时 Levenberg–Marquardt batch optimizer：最多 100 次，
relative/absolute error tolerance 均 `1e-9`，`MULTIFRONTAL_QR`；所有旧变量可动，
gauge prior 原样保留。默认解先算，后续仅在 offline 副本扫描 covariance。
每次返回值均检查有限且其自身相同 factor set 的 cost 不增加。

**这个 B 不是“关闭全部视觉的纯 LIO”。**它把 raw-motion leaf 接到保留全部
历史约束的旧图，同时重优化旧图。现有 ISAM2 pre-estimate 未充分非线性收敛，
所以旧变量在 B 中也会变化，不能把所有 raw→B 位移归因于新增 motion 因子
单独在无历史图上的作用。B→C 才隔离本次新增 return visual 的 batch 边际作用。

## 4. 默认参数：KeyPose 位移

平移为 world-frame position 差的范数，旋转为相对旋转 Logmap 范数。

| 数据 | RAW_TO_MOTION_ONLY_DELTA | RAW_TO_MOTION_VISUAL_DELTA | LANDMARK_MARGINAL_CORRECTION（B→C） |
| --- | --- | --- | --- |
| 20m | 0.408995 m / 11.897774° | 0.946222 m / 13.651402° | **1.273400 m / 2.564945°** |
| 40m | **2.859385 m / 9.219488°** | 2.914544 m / 11.128893° | **0.212211 m / 2.209202°** |

即 `LANDMARK_MARGINAL_CORRECTION_TRANSLATION_M` 为 20m `1.273400299`、
40m `0.212211157`；对应 rotation 为 `2.564944974` / `2.209201812` deg。

范数不能相加或相减当成贡献分解；world translation vectors 则满足
`d(raw,C) = d(raw,B) + d(B,C)`，已对全部离线 case 检查：

| 数据 | raw→B vector (m) | B→C vector (m) | raw→C vector (m) |
| --- | --- | --- | --- |
| 20m | (-0.148036, 0.264419, -0.274672) | (0.163836, -1.204814, 0.378324) | (0.015800, -0.940395, 0.103652) |
| 40m | (-0.245886, -0.639878, 2.776001) | (-0.206143, 0.032516, 0.038489) | (-0.452029, -0.607361, 2.814489) |

### 与实际 runtime C 的差异

| 数据 | 实际 raw→runtime C | batch C→runtime C |
| --- | --- | --- |
| 20m | 0.953098 m / 13.651034° | 0.008309 m / 0.060752° |
| 40m | 2.883315 m / 11.312617° | 0.037142 m / 0.215554° |

实际 runtime 的上述 delta、pre-existing cost、pre-augmented cost、post-full
cost 均与上一轮回放数值一致。离线 C 使用相同因子和 pre-state，但充分 batch
优化不等价于一次增量 ISAM2 更新；不能称其为完全精确复现实际单次更新。
完整 graph cost 和旧变量变化的差异见下文，不能仅凭新 K 接近就宣称全图相同。

40m B 已产生 2.859385 m，大于新增 visual 的 0.212211 m 边际量约 13.47 倍；
原 runtime 2.883315 m 与 batch C 的差距为 0.037142 m，远小于二者作用量的
区分。因此默认 **`LARGE_CORRECTION_40M_PRIMARY_SOURCE=MOTION`**：明确指
含历史图重优化的 motion-only 路径，不是声称 raw LiDAR 测量凭空产生了位移。
没有把 `2.883315 - 2.859385` 的标量差当成地标作用，也没有给出虚假的可加百分比。

## 5. 同一 visual Z 的残差

沿用实际 graph chart：`Z=T_body_camera*T_camera_landmark`，预测 `K.inverse()*L`，
`r=Pose3::Logmap(Z.inverse()*prediction)`，右局部 `[omega(rad), rho(m)]`。
translation residual 为 `||rho||`，不是非零旋转下 `E.translation()` 的欧式长度。
每个 case 使用该同一 Z、同一对应 noise model，核验 Logmap/unwhitenedError
一致、Gaussian factor error=`0.5*||whitenedError||²`。
B 的 candidate visual residual 仅评估，不把该因子加入 B optimizer。

| 数据 / metric | VISUAL_FACTOR_PRE_RESIDUAL / RAW | MOTION_ONLY_VISUAL_RESIDUAL / B | MOTION_VISUAL_POST_RESIDUAL / batch C |
| --- | ---: | ---: | ---: |
| 20m translation (m) | 1.337028308 | 1.334379230 | 2.199621e-6 |
| 20m rotation (deg) | 13.652087516 | 2.565883041 | 0.000992687 |
| 20m whitened norm | 2058.813704 | 2068.901140 | 0.001523898 |
| 20m factor error | 2119356.933892 | 2140175.963786 | 1.161133e-6 |
| 40m translation (m) | 3.206182004 | 0.206258511 | 3.500098e-6 |
| 40m rotation (deg) | 11.312314921 | 2.209766288 | 0.000497309 |
| 40m whitened norm | 5808.020493 | 126.828548 | 0.001071127 |
| 40m factor error | 16866551.024658 | 8042.740296 | 5.736564e-7 |

20m 的 B 虽降低角度残差，却未降低 candidate whitened norm；不把不在 B
中的测量误差解释成 B 的优化目标。40m B 已把候选视觉残差从 3.206182 m
降到 0.206259 m，即大部分该初始不一致无需本次 return visual 就已消失。
实际 runtime C 的 visual residual 仍为上一轮的 20m `0.005501730 m /
0.054733809°`、40m `0.024776113 m / 0.131011683°`；不能替换成 batch C
的近零残差去夸大在线效果。近零图残差也不证明真实 PnP/covariance 精度。

## 6. 同一 return motion factor 的残差

| 数据 / metric | RAW | B | batch C |
| --- | ---: | ---: | ---: |
| 20m translation (m) | 0.387019094 | 7.94e-15 | 0.224685112 |
| 20m rotation (deg) | 11.866100788 | 7.78e-15 | 6.677943874 |
| 20m whitened norm | 0.347691295 | 3.91e-15 | 0.197544990 |
| 20m factor error | 0.060444618 | 7.65e-30 | 0.019512011 |
| 40m translation (m) | 2.834239427 | 6.22e-9 | 0.212212558 |
| 40m rotation (deg) | 9.395774597 | 1.93e-7 | 2.208352012 |
| 40m whitened norm | 1.512746563 | 5.77e-9 | 0.124423723 |
| 40m factor error | 1.144201082 | 1.67e-17 | 0.007740631 |

B 中新 K 是只带一个 motion edge 的自由 leaf，旧图优化后可满足该边，因此
其接近零 motion residual 符合拓扑预期。C 加入 visual 后形成约束折中，motion
residual 非零。motion uncertainty 是原保守模型，尚未真实校准，未根据结果调小。

## 7. Landmark random variable 变化

| 数据 | L pre→B | L pre→C | L B→C |
| --- | --- | --- | --- |
| 20m | 0.004353423 m / 0.037325018° | 0.004353498 m / 0.037341774° | 1.180791e-7 m / 4.884266e-5° |
| 40m | 0.014879029 m / 0.236094783° | 0.014878871 m / 0.236156758° | 4.282508e-7 m / 6.855691e-5° |

B 中没有新增 visual，但旧图并未固定，原有视觉因子和旧 Landmark 被 batch
重优化；故 L pre→B 不为零。实际 runtime 单次更新中的 L 变化依然仅为
20m `1.378889e-7 m / 5.142687e-5°`、40m `3.394320e-7 m / 1.848751e-5°`。
batch C→runtime C 的 L gap 为 20m `0.004353434 m / 0.037320845°`、
40m `0.014879049 m / 0.236162752°`。不把这部分旧图收敛差异说成新视觉边的作用。
实现没有 Landmark prior/freeze；既有 tilted/drift synthetic 测试仍验证 L 可动
（runtime synthetic L 改变约 0.003900 m），不是 fixed constant。

## 8. 成本：只作相同 factor set 的前后比较

| 数据 / case | Initial cost | Optimized cost |
| --- | ---: | ---: |
| 20m B（old + motion） | 3796.861572 | 7.322516132 |
| 20m C（old + motion + visual） | 2123153.795464 | 7.361621956 |
| 40m B（old + motion） | 760.021359 | 3.320914031 |
| 40m C（old + motion + visual） | 16867311.046017 | 3.328655719 |

以上每行的 initial/optimized 才是同集合比较；**不直接比较 B/C 总 cost
并声称哪个解更优**。分项的 candidate/motion cost 已在上两节列出。

| Common existing factor set cost | RAW | B | batch C |
| --- | ---: | ---: | ---: |
| 20m | 3796.801127 | 7.322516132 | 7.342108784 |
| 40m | 758.877158 | 3.320914031 | 3.320914514 |

实际一次 runtime C 的 full graph cost 为 20m `9680.458888`、40m `1778.141673`，
与 batch C 不同；本轮没为其增加在线优化迭代或修改 ISAM2 设置。
快照重建的 RAW/common-old、C initial、runtime C 分项之和与原事件 CSV
在 `rel_tol=1e-10, abs_tol=1e-9` 下核验一致。

## 9. OFFLINE ONLY weight sensitivity

Scope：一次只把**全部旧/新 motion-family Between(x,x)** 或**全部旧/新
visual-family Between(x,l)** 的 covariance 乘以 scale（不是 sigma 乘 scale）。
测量、所有 pre Values、原 gauge prior 均固定；不重算 frontend covariance，
不做 Cartesian product，不改生产 YAML。每数据集 9 个唯一 case，默认 `(1,1)`
先算，再分别做 `.5,2,5,10`；共享默认行即两族各自的 scale=1。
这不是只缩放最后一个 return 因子的实验：旧图平衡可随 family 权重而变化。

40m（完整 residual 向量、whitened error、pose、cost 在 CSV）：

| motion scale | visual scale | raw→B m | raw→C m | B→C m | B→C deg | visual C m / deg | motion C m / deg |
| ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 1 | 1 | 2.859385 | 2.914544 | 0.212211 | 2.209202 | 3.500e-6 / 0.000497 | 0.212213 / 2.208352 |
| 1 | 0.5 | 2.411208 | 2.468061 | 0.212399 | 2.221907 | 1.762e-6 / 0.000250 | 0.212406 / 2.221454 |
| 1 | 2 | 3.340220 | 3.393750 | 0.211940 | 2.191065 | 6.928e-6 / 0.000987 | 0.211932 / 2.189578 |
| 1 | 5 | 3.955637 | 4.006929 | 0.211484 | 2.160093 | 1.699e-5 / 0.002432 | 0.211457 / 2.157445 |
| 1 | 10 | 4.378285 | 4.428004 | 0.211165 | 2.136349 | 3.346e-5 / 0.004811 | 0.211116 / 2.132809 |
| 0.5 | 1 | 3.340220 | 3.393750 | 0.211940 | 2.191065 | 6.928e-6 / 0.000987 | 0.211932 / 2.189578 |
| 2 | 1 | 2.411208 | 2.468061 | 0.212399 | 2.221907 | 1.762e-6 / 0.000250 | 0.212406 / 2.221454 |
| 5 | 1 | 1.905418 | 1.966813 | 0.212544 | 2.232021 | 7.082e-7 / 0.000100 | 0.212555 / 2.231835 |
| 10 | 1 | 1.596478 | 1.665529 | 0.212603 | 2.236151 | 3.548e-7 / 0.000050 | 0.212615 / 2.236058 |

20m：

| motion scale | visual scale | raw→B m | raw→C m | B→C m | B→C deg | visual C m / deg | motion C m / deg |
| ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 1 | 1 | 0.408995 | 0.946222 | 1.273400 | 2.564945 | 2.200e-6 / 0.000993 | 0.224685 / 6.677944 |
| 1 | 0.5 | 0.317614 | 0.970011 | 1.234011 | 2.403620 | 1.114e-6 / 0.000494 | 0.223744 / 6.698095 |
| 1 | 2 | 0.518966 | 0.930505 | 1.304354 | 2.873176 | 4.330e-6 / 0.002001 | 0.222925 / 6.612059 |
| 1 | 5 | 0.671045 | 0.918930 | 1.335094 | 3.400438 | 1.053e-5 / 0.005060 | 0.217771 / 6.465600 |
| 1 | 10 | 0.782279 | 0.915526 | 1.359359 | 3.784342 | 2.064e-5 / 0.010182 | 0.214050 / 6.338988 |
| 0.5 | 1 | 0.518966 | 0.930505 | 1.304354 | 2.873176 | 4.330e-6 / 0.002001 | 0.222925 / 6.612059 |
| 2 | 1 | 0.317614 | 0.970011 | 1.234011 | 2.403620 | 1.114e-6 / 0.000494 | 0.223744 / 6.698095 |
| 5 | 1 | 0.331761 | 1.016802 | 1.196421 | 2.454876 | 4.514e-7 / 0.000197 | 0.221778 / 6.681606 |
| 10 | 1 | 0.528898 | 1.063365 | 1.236801 | 2.726934 | 2.255e-7 / 0.000098 | 0.227229 / 6.679916 |

定义：对指定数据的 `raw→C translation/rotation`、`B→C translation/rotation`
四个非零基准量，计算各单族 scale 相对默认值的最大偏离比例，取最大值 S。
全部 case 数值健康且未达迭代上限才分类：S≤10% 为 LOW，10%<S≤50% 为
MODERATE，S>50% 为 HIGH；数值失败/无有效基准才为 UNRESOLVED。
这是本次明确的诊断分类，不是 estimator gate 或调参目标。

- **40m 综合/large-correction sensitivity = HIGH**，S=`51.927840%`。
  visual family 改变时 raw→C translation 为 `2.468061–4.428004 m`；
  motion family 改变时为 `1.665529–3.393750 m`（最大默认偏离 `42.854571%`）。
- **40m 新视觉边际量单独 = LOW**：B→C translation 全范围
  `0.211165–0.212603 m`，最大偏离 `0.493162%`；rotation
  `2.136349–2.236151°`，最大偏离 `3.297709%`。
  总位移 HIGH 主要反映旧图 family 平衡改变，不是本次边际量 HIGH。
- 20m 同一定义为 MODERATE，S=`47.540854%`，来自 B→C rotation。

40m 各 scale 中 raw→B 始终显著大于 B→C，默认 MOTION 主来源判断未反转，
但作用量绝对大小依赖未校准权重。没有据此选择“最佳 scale”、调参或推断绝对精度。
所有默认/扫描 B/C 在 4–6 次迭代结束，未达 100 上限。

## 10. Runtime regression / VIS-B

| Count / lifecycle | 20m | 40m |
| --- | ---: | ---: |
| Completed raw scans | 592 | 592 |
| IMU init scans（不要求 LIO transaction） | 2 | 2 |
| LIO attempted / committed / rejected | 590 / 590 / 0 | 590 / 590 / 0 |
| Lifecycle map insertions / ordinary CSV insertions | 296 / 295 | 296 / 295 |
| Image received / synced / processed | 596 / 590 / 589 | 596 / 590 / 589 |
| Backend submitted / accepted / processed | 24 / 24 / 24 | 21 / 21 / 21 |
| Duplicate submissions / overflow / out-of-order / invalid | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| Queue peak / shutdown queue / final queue | 1 / 0 / 0 | 1 / 0 / 0 |
| Worker starts / stops / drain / join | 1 / 1 / COMPLETE / COMPLETE | 1 / 1 / COMPLETE / COMPLETE |
| Actual ISAM2 update count（与上一轮相同） | 14 | 13 |
| Final motion / visual factors / gauge priors | 13 / 14 / 1 | 12 / 13 / 1 |
| Graph duplicate / suppressed visuals（原稀疏去重） | 10 / 10 | 8 / 8 |
| Accepted cross-episode snapshots | 1 | 1 |
| Capture / reobs diagnostic / ISAM failures | 0 / 0 / 0 | 0 / 0 / 0 |

每个 measurement-required completed scan 对应一笔 transaction，timestamp 与
scan end 差 <1e-6 s；每 scan map insertion≤1，state timestamp monotonic。
生命周期计数比 ordinary CSV 多一次初始化 BuildVoxelMap，不是图像触发 insertion。
589 条实际输出 trajectory 均为 8 列有限且时间严格递增；`scans_pos.json` 的真实
内容是 TUM 文本而非 JSON。两个 replay、roslaunch、roscore 正常退出，未发现
FATAL/segfault/进程异常退出/快照写错；worker join/drain 完成，无 deadlock。
inactive legacy CSV 的 NaN placeholder 不作估计 NaN；新快照/反事实所有数值健康。

```text
SCAN_TRANSACTION_CONTRACT = PASS
PARTIAL_CLOUD_LIO_REINTRODUCED = NO
IMAGE_TRIGGERED_MAP_INSERTION = NO
STATE_TIMESTAMP_MONOTONIC = PASS
VIS_B_EXECUTION_CONTRACT_PRESERVED = YES
```

## 11. 实现、检查与复现

本轮起始 HEAD=`2cd38f2`，worktree 已包含上一轮未提交的 re-observation 诊断、
legacy replay overlay 和报告；原有改动均保留。`visionpro_web_viewer` 与当前
IDE 活动文件不在任务修改范围内。没有 reset/clean/checkout/stash/commit/push。

新增 `landmark_counterfactual.h/.cpp`（快照/独立 batch helper）和
`landmark_counterfactual_probe.cpp`（离线默认解 + 单族扫描 CSV）。原 graph
仅添加 opt-in 只读捕获、shutdown writer；backend/VIO 只接入诊断开关、计数和
文件输出，CMake 接入工具，现有 self-test 增加隔离/round-trip/非法输入检查，
原 replay runner 复用 load chain 并提供临时 wrapper。没有新依赖。

`catkin_make --pkg fast_livo -j4` 和读取容差修正后的完整增量构建通过。
最终 8 项相关自测全部 PASS：frontend、legacy adapter、measurement、architecture、
persistent backend、shadow graph、LIO scan transaction、VIO transaction；
其中 `LANDMARK_COUNTERFACTUAL_ISOLATION_SELF_TEST=PASS`。
测试验证 synthetic B 保持 raw leaf、C 有新增视觉边际作用、快照还原 cost、
真实 rounded calibration 原值保留、gauge 不缩放、离线结果不改 live graph、
非法输入拒绝。`git diff --check` 与 runner `py_compile` 通过。

保留一次过早的 VIO 自测失败日志：首次完整构建尚在重编译 libvio 时运行了
该测试，出现 `corrupted size vs. prev_size`；完整构建结束后、及最终修正后
分别复跑 8 项均通过。该失败不被隐藏，也不计为成功回放的运行故障。
另外最初 offline reader 因真实 rounded R 拒绝输入的问题已修正并覆盖测试。

证据根目录：`Log/landmark_counterfactual_20261002/`：

- `analysis.json`：两次回放计数、所有 case、分类和 `VERIFICATION=PASS`。
- `20m/2026-10-02-200258/landmark_counterfactual_obs24.snapshot`。
- `40m/2026-10-02-200718/landmark_counterfactual_obs20.snapshot`。
- `20m/counterfactual.csv`、`40m/counterfactual.csv`：每个 9 行，含 RAW/B/C/runtime
  K/L pose、world delta、unwhitened/whitened 6-vector、分项 cost、iteration 数。
- 每个 root 的完整 `bag_info.yaml`、有效参数、原配置/launch snapshot、完整
  ROS logs、playback metadata；每个 run 的 scan/LIO/trajectory/frontend/reobs CSV
  和 backend shutdown summary。
- `build.log`、`reader_build.log`、最终 `counterfactual_verified_*_self_test.log`、
  `premature_vio_test_before_build_finished.log`、`source.diff`、`source_snapshot`、
  `source_snapshot_final`。后者还保存可重复验证证据的 `verify.py`。
- `capture20m_runtime_sha256.txt`：20m 捕获时 executable/library/source hashes。
  `capture40m_and_offline_runtime_sha256.txt`：读取器修正后 40m 和离线工具 hashes。
  二者仅离线 reader/测试改变，不混称同一 build。

仅使用 process-local system libusb preload，未改主机库配置。ROS masters 分别
隔离在 11636/11637，只关闭 runner 自己的 process groups。

复现（fresh output root，不覆盖旧证据）：

```bash
source /home/gulu/catkin_ws/devel/setup.bash
python3 reports/run_landmark_reobservation_replay.py 20m NEW_20M_ROOT --port 11636 --counterfactual-snapshots
# 检查 20m 后，以另一个全新 root 完整运行 40m
python3 reports/run_landmark_reobservation_replay.py 40m NEW_40M_ROOT --port 11637 --counterfactual-snapshots
# 每个 run 的对应事件快照，output CSV 必须尚不存在
rosrun fast_livo landmark_counterfactual_probe EVENT.snapshot NEW_OUTPUT.csv
python3 Log/landmark_counterfactual_20261002/source_snapshot_final/verify.py
```

## 12. 最终状态及停止点

```text
LANDMARK_COUNTERFACTUAL_AUDIT_COMPLETE = YES
REAL_20M_MOTION_ONLY_COUNTERFACTUAL = PASS
REAL_40M_MOTION_ONLY_COUNTERFACTUAL = PASS
REAL_20M_LANDMARK_MARGINAL_EFFECT_READY = YES
REAL_40M_LANDMARK_MARGINAL_EFFECT_READY = YES
LARGE_CORRECTION_40M_PRIMARY_SOURCE = MOTION
REOBSERVATION_WEIGHT_SENSITIVITY = HIGH
PRODUCTION_PARAMETERS_CHANGED = NO
LANDMARK_BACKEND_SHADOW_ONLY = YES
ESIKF_MODIFIED_BY_GLOBAL_BACKEND = NO
NEW_MAP_TO_ODOM_PUBLISHER_CREATED = NO
PRODUCTION_CORRECTION_OWNER = EXISTING_FIXED_LAG_BACKEND
FORMAL_UNIQUE_ID_BOARD_VALIDATION = NOT_RUN
```

HIGH 指 §9 定义的 total/marginal 综合量；40m **新增地标边际量单独 LOW**。
没有 ground truth、production feedback 或正式实体 unique-ID Board，不能声称
地标修正了真实轨迹 X 米，也没有验证真实 PnP 精度、真实 covariance 校准、
partial visibility 或生产 loop closure correction。本轮到此停止，不启动 L4、
production correction、GNSS/UWB 迁移；正式下一阶段等待 unique-ID 实物 Board。
