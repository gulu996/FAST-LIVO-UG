# Dataset format audit

## DATASET FORMAT AUDIT

`1.zip` 与“融合定位”的目录角色、扩展名和文件命名语义兼容，但 ZIP 内部 48,262 个文件均为 AES 加密。本轮可验证中央目录元数据，不能验证 ZIP 内 JPEG 解码、CSV 表头、时间单位、RINEX 头或标定内容。

- FORMAT_STATUS = `PARTIAL_AUDIT_WAITING_FOR_ZIP_PASSWORD`
- ZIP_CONTENT_LIST_ENCRYPTED = `NO`
- IMAGE_CONTENT_NOT_VALIDATED_DUE_TO_PASSWORD
- 参考数据：left = 10,196，right = 10,196，matched = 10,196
- 1.zip：left = 24,115，right = 24,115，matched = 24,115
- 1.zip：left_only = 0，right_only = 0，duplicate_left = 0，duplicate_right = 0
- STEREO_PAIR_STATUS = `PASSWORD_BLOCKED`（文件名级配对为 PASS，内容级检查等待密码）

## 结构与时间证据

| Item | 融合定位 | 1.zip 中央目录 |
|---|---:|---:|
| 左目 JPEG | 10,196 | 24,115 |
| 右目 JPEG | 10,196 | 24,115 |
| 左目时间范围 | 1785900504390125535–1785900844277363535 ns | 1785914542592395476–1785915346508102476 ns |
| 左目 median dt | 0.033371 s | 0.033372 s |
| 左目 P95 dt | 0.033388 s | 0.033386 s |
| 左目 max dt | 0.066751 s | 0.099999 s |
| LiDAR CSV 未压缩大小 | 9,140,173,429 B | 22,021,548,339 B |
| IMU CSV 大小 | 14,846,109 B | 35,053,165 B |

两者都有 `camera/{left,right}`、`camera/stereo.yaml`、`imu/imu.csv`、`imu/imu.yaml`、`lidar/points.csv`、`sensor_extrinsics.yaml` 和 Rover/Base/Navigation RINEX。参考目录额外含 GNSS 解算结果和辅助工具等派生文件；`1.zip/camera/left` 另有 24 个 `.baiduyun.uploading.cfg` 临时文件，转换器不会把它们当图像。

参考数据内容实扫确认：左右目共 20,392 张文件的 JPEG 首尾标记完整，首/中/末共 6 张 JPEG 由 OpenCV 正常解码；Camera 文件名、IMU `timestamp_ns`、LiDAR `frame_timestamp_ns` 已在同一 Unix 纳秒绝对时间域，未施加 offset；LiDAR `point_time_s` 是秒制帧内相对时间。LiDAR 1,704 帧，median dt 0.199398041 s，P95 0.202344895 s；IMU 67,990 条，严格 200 Hz。首/中/末 LiDAR 帧点时间最大值均为 0.1999754725 s，不存在全 0 点时间。

## REFERENCE_BAG_SCHEMA

参考 bag：`/home/gulu/TiaoZhanBei/TIAOZHANBEI_FASTLIVO2_GNSS_CODE_20260826/rosbags/competition.bag`，SHA-256 `fab46f92c4fe8f405d79f547bda876cad012cab238c1f077473d0fb524392dba`。

| Topic | Type | Count | frame_id |
|---|---|---:|---|
| `/tiaozhanbei/camera/left/image/compressed` | `sensor_msgs/CompressedImage` | 10,196 | `cam0` |
| `/tiaozhanbei/gnss/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 2,426 | `gnss_enu` |
| `/tiaozhanbei/imu` | `sensor_msgs/Imu` | 67,990 | `imu0` |
| `/tiaozhanbei/lidar` | `sensor_msgs/PointCloud2` | 1,704 | `lidar0` |

参考 bag 只含左目压缩 JPEG。PointCloud2 字段严格为 `x,y,z,intensity,time,ring`，其中 `time` 是 offset 16 的 FLOAT64 秒制帧内相对时间；GNSS 坐标为 x=East、y=North、z=Up，协方差来自正式离线解算。
