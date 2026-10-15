#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -ne 1 ]; then echo 'Usage: record_bag.sh /absolute/path/new_capture.bag' >&2; exit 2; fi
bag_path=$1
if [[ "$bag_path" != /* || "$bag_path" != *.bag || -e "$bag_path" || -e "$bag_path.active" ]]; then echo 'Use a new absolute .bag path; existing output will not be overwritten.' >&2; exit 2; fi
unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source "${ZED_WS:?Set ZED_WS to the isolated wrapper workspace}/devel/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11331}"
# ponytail: five-second bounded recording; long captures require a disk-throughput check.
exec rosbag record --lz4 --duration=5 --buffsize=2048 -O "$bag_path" \
 /zed2i/zed_node/left/image_rect_color /zed2i/zed_node/right/image_rect_color \
 /zed2i/zed_node/left/camera_info /zed2i/zed_node/right/camera_info \
 /zed2i/zed_node/imu/data /zed2i/zed_node/imu/data_raw /tf_static /diagnostics
