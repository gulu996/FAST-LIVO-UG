#!/usr/bin/env bash
set -euo pipefail

[ "$#" -eq 3 ] || {
  echo "usage: $0 SEQUENCE PREFIX_SECONDS OVERRIDE_YAML" >&2
  exit 64
}
sequence=$1
prefix_seconds=$2
override=$3
runner=/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/scripts/run_one.sh
output_root=/media/gulu/一只沙糖桔/FAST_LIVO2_experiments/city1_recovery_20260910/raw_outputs/ours/$sequence
[ -x "$runner" ] || { echo "missing runner: $runner" >&2; exit 66; }
[ -f "$override" ] || { echo "missing override: $override" >&2; exit 66; }

created_after=$(mktemp)
touch "$created_after"
runner_pid=
bag_pgid=
bag_stopped=0
resume_bag() {
  if [ -n "$bag_pgid" ] && [ "$bag_stopped" -eq 1 ]; then
    kill -CONT -- "-$bag_pgid" 2>/dev/null || true
    bag_stopped=0
  fi
}
cleanup() {
  resume_bag
  rm -f "$created_after"
}
trap cleanup EXIT INT TERM HUP

RTK_SLAM_OURS_OVERRIDE="$override" \
  bash "$runner" ours "$sequence" --prefix "$prefix_seconds" &
runner_pid=$!
run_dir=
while kill -0 "$runner_pid" 2>/dev/null; do
  if [ -z "$run_dir" ]; then
    run_dir=$(find "$output_root" -maxdepth 1 -mindepth 1 -type d \
      -newer "$created_after" -printf '%T@ %p\n' 2>/dev/null |
      sort -nr | head -1 | cut -d' ' -f2-)
  fi
  if [ -n "$run_dir" ]; then
    pause_dir="$run_dir/native/p4b_same_snapshot"
    if [ -f "$pause_dir/capture_pause.request" ] && [ "$bag_stopped" -eq 0 ]; then
      if [ -z "$bag_pgid" ]; then
        bag_pgid=$(ps -eo ppid=,pgid=,args= | awk -v parent="$runner_pid" '
          $1 == parent && /rosbag play/ { print $2; exit }')
      fi
      if [ -n "$bag_pgid" ] && kill -0 -- "-$bag_pgid" 2>/dev/null; then
        kill -STOP -- "-$bag_pgid"
        bag_stopped=1
        : > "$pause_dir/capture_pause.ack"
      fi
    fi
    if [ "$bag_stopped" -eq 1 ] && [ -f "$pause_dir/capture_pause.done" ]; then
      resume_bag
      : > "$pause_dir/capture_pause.resumed"
    fi
  fi
  sleep 0.02
done
set +e
wait "$runner_pid"
rc=$?
set -e
resume_bag
exit "$rc"
