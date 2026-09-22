#!/usr/bin/env bash
# Reuse the audited dataset conversion; prepare its origin before subscribing.
set -euo pipefail
if [ "$#" -lt 3 ]; then
  echo "usage: $0 BAG RUN_DIR GNSS_HELPERS_DIR [ROS remappings]" >&2
  exit 64
fi
bag=$1
run_dir=$2
helpers=$3
shift 3
[ -f "$bag" ] || { echo "Missing RTK-SLAM bag: $bag" >&2; exit 66; }
mkdir -p "$run_dir"
# ponytail: the convenience replay uses the bag's first valid Fixed solution;
# the benchmark runner remains the entry point for custom evaluation windows.
/usr/bin/python3 "$helpers/extract_enu_origin.py" "$bag" "$run_dir/enu_origin.json"
exec /usr/bin/python3 "$helpers/gnss_adapter.py" \
  --input /gnss/fix --origin "$run_dir/enu_origin.json" \
  --pose-output /rtk_slam/gnss/enu_pose \
  --stats "$run_dir/gnss_adapter_stats.json" "$@"
