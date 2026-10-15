#!/usr/bin/env bash
set -e
# Local process environment avoids the unrelated MVS libusb path.
unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source "${ZED_WS:?Set ZED_WS to the isolated wrapper workspace}/devel/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11331}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${ZED_EVIDENCE_DIR:-$ZED_WS/evidence}/ros_logs}"
mkdir -p "$ROS_LOG_DIR"
exec roslaunch "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/zed_l1.launch" "$@"
