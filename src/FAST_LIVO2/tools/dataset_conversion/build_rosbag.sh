#!/usr/bin/env bash
set -euo pipefail

script_args=("$@")
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Sourcing a catkin setup can consume positional parameters in some packaged workspaces.
set --
source /opt/ros/noetic/setup.bash
if [[ -f /home/gulu/catkin_ws/devel/setup.bash ]]; then
  source /home/gulu/catkin_ws/devel/setup.bash
fi
set -- "${script_args[@]}"

exec python3 "$script_dir/build_rosbag.py" "$@"
