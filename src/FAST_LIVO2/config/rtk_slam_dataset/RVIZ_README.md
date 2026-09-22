# RTK-SLAM 真实数据回放与 RViz

## 推荐：一个终端启动算法，一个终端播放数据

这套便捷入口显示来自真实相机图像的 RGB 点云，已经集成 GNSS 转换、图像解码、定位后端及 RViz。先正常结束上一轮运行，再打开两个终端。无需重新编译。

终端一，以城市花园序列 2 为例：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

roslaunch fast_livo mapping_rtk_slam_rgb.launch sequence:=stadtgarten_seq2
```

终端二：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

rosbag play --clock --wait-for-subscribers -r 0.5 --duration 875.932452917 \
  '/media/gulu/一只沙糖桔/Dataset/ros1/stadtgarten_seq2.bag' \
  --topics /livox/lidar /livox/imu /camera/image_raw/compressed /gnss/fix
```

`-r 0.5` 是半速，方便观察；需要原速可改为 `-r 1`。两个终端的序列名称必须一致。相机视野内的点才有真实 RGB 颜色，因此彩色点云不会覆盖激光的全部视野。

切换序列时，同时修改 roslaunch 的 sequence、rosbag 文件名与播放时长：

| sequence | --start | --duration |
| --- | --- | --- |
| construction_seq1 | 0.046315095 | 740.969538212 |
| construction_seq2 | 0.047924 | 599.054272413 |
| stadtgarten_seq1 | 0.050672 | 1602.254393101 |
| stadtgarten_seq2 | 0 | 875.932452917 |

表中的时间窗沿用已有传感器共同覆盖范围。便捷入口使用所选 bag 的第一条有效 Fixed 解初始化 ENU 原点，适合从头回放；若跳到中间或做严格评测，使用下方 benchmark runner。

RGB 配置是 `/publish/colorize_cloud_en=true`、`/publish/pub_scan_num=1`、RViz `Color Transformer=RGB8`。新的输出目录自动创建在 `Log/rtk_slam_rgb/`；运行时执行 `rosparam get /rtk_slam_rgb/run_dir` 可查看本次确切路径。PCD 保存默认关闭。

播放完毕后可保留终端一与 RViz，调整视角截图；结束时在终端一按 Ctrl+C 并等待轨迹保存完成。后端蓝色轨迹显示最近 20 秒活跃窗口，灰色为前端完整轨迹，点云来自前端建图。

2026-09-08 实际验证：独立 ROS 端口回放城市花园序列 2 前 45 秒，收到 895 帧非空 RGB 点云、899 帧解码图像、450 条 GNSS 里程计及 130 条融合输出。点云字段为 x/y/z/rgb，颜色样本包含大量非灰色点。测试未启动图形窗口；RViz RGB8 配置及 launch 参数展开另行检查通过。验证记录位于 `artifacts/rtk_slam_rgb_check_20260908/validation.json`。

## 原 benchmark runner 入口

以下命令运行本仓库的 FAST-LIVO2 前端与 GNSS 固定滞后后端，使用现有已编译程序和 `drift_repair_buffered.yaml`。不需要另开 roscore、rosbag play 或 GNSS 转换节点，现有 runner 已负责这些步骤。

终端一运行算法，以城市花园序列 2 为例：

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash

export RTK_SLAM_DATASET_ROOT='/media/gulu/一只沙糖桔/Dataset'
export RTK_SLAM_ROS_PORT=11331
export RTK_SLAM_OURS_OVERRIDE='/home/gulu/catkin_ws/src/FAST_LIVO2/config/rtk_slam_dataset/drift_repair_buffered.yaml'

bash /home/gulu/TiaoZhanBei/eval_20260830/results/drift_repair_20260906/scripts/run_one.sh \
  ours stadtgarten_seq2 --full
```

终端二连接同一个 ROS master 并打开 RViz。`--wait` 会等待终端一启动 master，不会抢先创建另一个 master。

```bash
source /opt/ros/noetic/setup.bash
source /home/gulu/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://127.0.0.1:11331
export ROS_HOSTNAME=127.0.0.1
export LD_LIBRARY_PATH=/home/gulu/catkin_ws/devel/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/noetic/lib:/home/gulu/Sophus/build:/usr/local/lib
export LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0

roslaunch --wait fast_livo view_rtk_slam_dataset.launch
```

可将 `stadtgarten_seq2` 替换为 `construction_seq1`、`construction_seq2` 或 `stadtgarten_seq1`。对应共同传感器时间窗约为 14.6、12.3、10.0、26.7 分钟。runner 按共同时间范围播放，不包含某种传感器已经结束后的单独雷达尾段。

RViz 白底显示真实激光点云、灰色完整前端轨迹和蓝色最近 20 秒 GNSS 融合窗口。点云颜色来自真实激光强度，不是相机 RGB。GNSS 轨迹在初始对齐成功后出现，需要积累至少 20 对观测和 8 米运动基线。蓝色话题是后端当前活跃窗口，不代表完整历史融合轨迹。

`view_rtk_slam_dataset.launch` 添加 `odom → camera_init` 单位变换。它只连接本配置中同一前端坐标的两个名称，没有拟合或修改轨迹。后台已有 `map → odom` 变换；不要同时使用将后端 odom 名称改成 camera_init 的另一套覆盖配置。

结果位于 `/home/gulu/TiaoZhanBei/eval_20260830/results/drift_repair_20260906/raw_outputs/ours/序列名/时间戳-full/`。`native/rtk_optimized_online.tum` 是在线轨迹，`native/rtk_optimized_final.tum` 是最终结果。完成后 canonical 结果指向最新成功运行，原来的 canonical 文件会保存在新运行的 `previous_canonical` 目录。

默认实验配置不保存 PCD；RViz 显示的是累计前端点云，后端没有把整个历史点云逐点重新优化。可调整视角截取真实建图图像用于 PPT。正常结束或需要提前停止时，在终端一按 Ctrl+C 并等待退出保存轨迹，最后关闭终端二。

2026-09-08 核查：已确认 bag 实际话题、动态库解析、runner shell 语法、roslaunch 节点展开和最终参数，未在本轮执行全程回放或宣称已验证 GUI 运行。现有修复实验曾完成对应全程回放。
