#!/usr/bin/env bash
set -e
unset LD_LIBRARY_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
source "${ZED_WS:?Set ZED_WS to the isolated wrapper workspace}/devel/setup.bash"
script_dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
evidence_dir=${ZED_EVIDENCE_DIR:-$ZED_WS/evidence}
mkdir -p "$evidence_dir"
cd "$evidence_dir"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11331}"
export XDG_CONFIG_HOME="$evidence_dir/rqt_config"
left_pid=; right_pid=
trap 'kill -INT "$left_pid" "$right_pid" 2>/dev/null || true' EXIT
/usr/bin/python3 "$script_dir/preview.py" -s rqt_image_view/ImageView --args /zed2i/zed_node/left/image_rect_color &
left_pid=$!
XDG_CONFIG_HOME="$evidence_dir/rqt_config_right" /usr/bin/python3 "$script_dir/preview.py" -s rqt_image_view/ImageView --args /zed2i/zed_node/right/image_rect_color &
right_pid=$!
wait
