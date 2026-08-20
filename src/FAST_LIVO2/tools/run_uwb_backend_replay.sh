#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 6 || $# -gt 10 ]]; then
  echo "usage: $0 BAG UWB_TXT HALF_MAP UWB_BACKEND(true|false) ALLOWLIST RATE [PORT] [LABEL] [OVERLAY_YAML] [SAVE_MAP]" >&2
  exit 2
fi

bag=$1
uwb_txt=$2
half_map=$3
uwb_backend=$4
allowlist=$5
rate=$6
port=${7:-11321}
label=${8:-uwb_replay}
overlay=${9:-}
save_map=${10:-false}
workspace=/home/gulu/catkin_ws
repo=/home/gulu/catkin_ws/src/FAST_LIVO2
run_tmp=/tmp/fast_livo_uwb_${port}_${label}
mkdir -p "$run_tmp/ros"

source "$workspace/devel/setup.bash"
# The MVS SDK ships an old libusb without libusb_set_option; PCL needs the
# system ABI. Keep the SDK path for the camera driver, but resolve libusb first.
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_HOME="$run_tmp/ros"
export ROS_LOG_DIR="$run_tmp/ros/log"

master_pid=
mapping_pid=
backend_pid=
cleanup() {
  set +e
  [[ -n "$mapping_pid" ]] && kill -INT "$mapping_pid" 2>/dev/null
  [[ -n "$backend_pid" ]] && kill -INT "$backend_pid" 2>/dev/null
  sleep 1
  [[ -n "$master_pid" ]] && kill -INT "$master_pid" 2>/dev/null
  wait "$mapping_pid" "$backend_pid" "$master_pid" 2>/dev/null
}
trap cleanup EXIT INT TERM

roscore -p "$port" >"$run_tmp/roscore.log" 2>&1 &
master_pid=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done

rosparam set /use_sim_time true
rosparam load "$repo/config/mid360.yaml"
rosparam load "$repo/config/rtk_fixed_lag_backend_mid360.yaml"
rosparam load "$repo/config/camera_pinhole_mid360.yaml" /laserMapping
if [[ -n "$overlay" ]]; then
  rosparam load "$overlay"
fi
rosparam set /gnss_fusion/enable false
rosparam set /local_map/half_map_size "$half_map"
rosparam set /uwb/replay_file "$uwb_txt"
rosparam set /uwb/update_en false
rosparam set /uwb/anchor_frame_align_en false
rosparam set /rtk_backend/enable true
rosparam set /rtk_backend/uwb_factor_backend_en "$uwb_backend"
rosparam set /rtk_backend/uwb_anchor_allowlist "$allowlist"
rosparam set /pcd_save/pcd_save_en "$save_map"
if [[ "$uwb_backend" == true ]]; then
  rosparam set /uwb/enable true
else
  rosparam set /uwb/enable false
fi

rosrun fast_livo fastlivo_mapping __name:=laserMapping \
  _save_path:="" _use_gnss_fusion_enable:=true \
  >"$run_tmp/mapping.log" 2>&1 &
mapping_pid=$!
rosrun fast_livo rtk_fixed_lag_backend_node \
  __name:=rtk_fixed_lag_backend _use_gnss_fusion_enable:=true \
  >"$run_tmp/backend.log" 2>&1 &
backend_pid=$!

for _ in $(seq 1 300); do
  run_directory=$(rosparam get /fast_livo/run_log_directory 2>/dev/null || true)
  [[ -n "$run_directory" && "$run_directory" != "__PENDING__" ]] && break
  if ! kill -0 "$mapping_pid" 2>/dev/null || ! kill -0 "$backend_pid" 2>/dev/null; then
    echo "startup failed; inspect $run_tmp" >&2
    exit 1
  fi
  sleep 0.1
done
if [[ -z "${run_directory:-}" || "$run_directory" == "__PENDING__" ]]; then
  echo "run directory was not published; inspect $run_tmp" >&2
  exit 1
fi

rosbag play "$bag" --clock -r "$rate" -d 2 \
  >"$run_tmp/rosbag.log" 2>&1
sleep 3
cleanup
trap - EXIT INT TERM

printf '%s\n' "$run_directory"
