# L0–L2-B 技术总结（历史证据，2026-10-15）

来源是原 ZED 工作空间的 L1、L2-A、L2-B 完整报告与实际源码。下列实机结果属于历史实验，
本次整理仅重新验证副本软件，不把历史成绩冒充当前硬件复测。

## L1 真实相机与工程兼容性

Ubuntu20.04/Noetic、RTX2060 Max-Q 6GB、CUDA12.1.1、SDK4.1.4，官方 Wrapper SHA见上级README。
左右1920×1080，基线各1038帧、29.780044Hz；IMU raw12558样本、360.300968Hz。
左右Header纳秒精确一致，基线无重复/回退，但图像间隔32.922–100.546ms，不能称严格无漏帧30Hz。
400Hz是IMU轮询配置上限。CameraInfo共用彩色/灰度发布路径，同stamp常重复两份。

官方rqt双目预览历史通过，约29.727Hz；短NVMe/LZ4 bag左右各142帧，约29.684Hz，
两路IMU各1705条。1倍速完整回放六个必需话题数量与有序完整序列化消息SHA256一致；
/tf_static与/diagnostics已录入但不属于六话题hash验收。未证明预览+长期录包的全负载表现。

矫正图像及D=0的CameraInfo经过SDK参数对照。未矫正raw CameraInfo有上游畸变系数顺序/模型表达缺陷，
不应直接用于PnP或去畸变；没有修改工厂标定或Wrapper。SVO149/150帧差异仍未解决；
SVO路径会改用当前ROS时间，不用于验证原录制时间。

FAST-LIVO2历史增量编译、动态库解析及30秒历史bag回归通过；没有安装前精度基线。
原MVS库路径可能抢占系统libusb，曾使用局部进程库优先级解决；未改系统BLAS或海康代码。
这些结果只说明当时构建、加载及短运行兼容，不等于轨迹精度等价或ZED融合验收。

## 源码时间链与不可观测误差

固定上游 `zed_nodelets/src/zed_nodelet/src/zed_wrapper_nodelet.cpp`：单次grab后取左右Mat，
3031–3052行读取；3114行使用Mat.timestamp，3171/3220行共用stamp发布；2428行CameraInfo同stamp。
`zed_nodelets/src/tools/src/sl_tools.cpp:116–121`整数秒/纳秒转换。
SDK4.1.4本机Camera.hpp:9153–9166定义IMAGE为完整图像在PC内存可用的参考，**不是曝光中心**。
IMU sync=false走自己的acquisition timestamp（Wrapper3462–3489、3665行）；相邻相等过滤不是全局回退保护。
SDK内部时钟估计闭源，CURRENT与system_clock吻合不证明IMAGE/IMU的物理误差为零。
历史SDK180组IMAGE=left_mat=right_mat；IMU baseline有1682/12558个arrival wall−Header负值，
最小−2.056225ms，原因尚未确定，不截断、不补常数。

原仓库 `src/livox_ros_driver2/src/comm/pub_handler.cpp:104–122,262–273`：
同步type1/PTP或2/GPS使用packet时间，其他类型回退主机时间；当前链路预期type2。
IMU callback同时采CLOCK_MONOTONIC；lddc.cpp:398–425出队写锚点，额外记录update MONOTONIC。
`imu_time_anchor.h`定义64字节IMA1 v1；Python读取器是现有sensor_time_bridge/imu_anchor.py。
writer_epoch表示进程实例，不是设备boot；同进程设备回退仍需独立分段。

STM32源码虚拟GPRMC日期固定230520，时分秒重启/跨日回绕，不能当真实UTC或GPS week/TOW，
不能额外加18秒或时区偏移。实际PPS有效边沿/RMC秒对应仍需测量。
旧timeshare为最近点云base，不能给自由运行ZED直接赋值；旧host service的MONOTONIC_RAW→MCU LOCAL
也不是这里的CLOCK_MONOTONIC→MID域。MONOTONIC与RAW不可直接互换，系统校频/挂起需分段处理。

```text
M_mid = M_before + floor((M_after-M_before)/2)
R(M) = R0 + b_R + alpha_R*(M-M0)
M_Z = round(inverse_R(Z))
D0 = A_D + round(alpha_D*(M_Z-A_M)) - C_ns
```

Z为原SDK/ROS纳秒，R为REALTIME，M为MONOTONIC，A_D/A_M是同packet设备时间/callback时间。
整数/中心化有理数计算，C默认0（未校准），正C使时间更早，不继承海康img_time_offset=0.1。
恒定单向接收延迟与设备时钟截距无法仅由锚点分离。若callback晚ℓ、IMAGE晚于曝光d（均M域持续时间），
则曝光估计需在C=0的D0基础上加alpha_D*ℓ、减alpha_D*d；两项都未知。
卷帘逐行时刻无法由单Header表达。拟合残差/配置uncertainty不构成物理误差上界。

## L2-B 软件能力与历史验证

- SHADOW_ONLY；原戳不改、无Image发布，无FAST-LIVO2输入接入，无真实共享文件写入。
- 当前Writer年龄/sequence推进和历史目标距anchor分别校验；缺失、未ready、错域、超过100ms等拒绝。
- host boot/跳时/挂起、epoch/inode、设备或ZED回退建立新段；旧valid缓存不能绕过当前故障。
  隐藏SDK重启若所有可见计数/时间连续仍不可观测。
- 左右/CameraInfo以session+raw_stamp共用不可变结果；IMU独立顺序检查，CameraInfo重复不造新组。
- 十进制整数原戳与clock/anchor按处理顺序保存，离线只用当次sidecar并比较完整结果。
  无有效映射时输出null+reason，不复制最近雷达戳、不补帧、不用last+1伪造单调。
- 历史36项测试通过；近int64上界、正负漂移及补偿、重启/超时等测试中参考误差≤1ns，
  特定漂移样例最大0ns（数值结果，不是物理精度）。合成78记录/54有效事件，重放0差异。
  历史L1元数据18788事件因缺clockpairs/anchors全部CLOCK_MISSING，重放0差异。

双IMU工具：不同采样率带限、模长多窗互相关、非周期激励、偏移/漂移、bootstrap置信区间及留出验证。
定义corr(Z(t),MID(t+δ))，δ应加到ZED暂映射时间；若未来应用对应C=-δ，工具本身不应用。
360Hz/200Hz合成+25ms估25.000093ms，−35ms估−34.999942ms；+500ppm估500.811ppm，
−350ppm估−350.760ppm，留出最大残差分别0.630189ms、0.446636ms。
200Hz网格CI半宽下限2.5ms，未自动达到初拟±2ms目标。相关测的是有效惯性延迟，不是曝光偏差。

历史性能：15秒clock-only约单核3.04%、VmRSS16.76MiB，无传感器事件/有效雷达映射；
纯核心合成负载约单核5.14%、RSS14.8MiB（不含ROS/CSV）。真实双设备负载A/B未执行，不能称<5%目标达标。
两路1080P30 BGRA8载荷497.664MB/s；新增旁路无全图搬运，GPU增量未受控实测。

## 后续验收条件

真实MID-360/IMU、有效type2锚点、ZED USB3同时可用，确认STM32实际固件和PPS/RMC关联；
两传感器刚性固定，记录起止静止段和2–5分钟非周期多轴运动+原数据/sidecar。
在多个连续会话内分别估偏移/漂移，并用未参与拟合的窗口验证；不能跨会话拟合。
受控A/B比较CPU/RSS/GPU/频率/长间隔；全图转发或Wrapper改动需另外评估和批准。
曝光校准使用独立LED/光电二极管与高阻示波器观测既有PPS/RMC，不改STM32固件，
测参考行、逐行扫描及曝光/负载依赖。双IMU相关不能替代该光学实验。
截至整理：MID-360物理同步、B5双设备性能、B6曝光校准均HARDWARE_PENDING。
