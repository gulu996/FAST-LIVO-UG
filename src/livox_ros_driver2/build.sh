#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic was not found at /opt/ros/noetic"
  exit 1
fi

source /opt/ros/noetic/setup.bash

cd "${WS_DIR}"

catkin_make \
  -DROS_EDITION=ROS1 \
  -DCMAKE_BUILD_TYPE=Release \
  "$@"
