#!/usr/bin/env bash
set -euo pipefail

rate=${1:-2.0}
port=${2:-11400}
repo=/home/gulu/catkin_ws/src/FAST_LIVO2
runner="$repo/tools/run_uwb_backend_replay.sh"
manifest=/tmp/uwb_backend_matrix_${port}.tsv
: >"$manifest"

run_case() {
  local dataset=$1 half_map=$2 backend=$3 allowlist=$4 repeat=$5 save_map=$6
  local bag=/home/gulu/Downloads/${dataset}.bag
  local txt=/home/gulu/Downloads/uwb/${dataset}.txt
  local overlay=$repo/config/experiments/uwb_${dataset}_calibrated.yaml
  local mode=base
  [[ "$backend" == true ]] && mode=uwb
  local label=${dataset}_h${half_map}_${mode}_${allowlist//[^0-9]/}_r${repeat}
  local run_directory
  run_directory=$("$runner" "$bag" "$txt" "$half_map" "$backend" \
    "$allowlist" "$rate" "$port" "$label" "$overlay" "$save_map")
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$dataset" "$half_map" "$backend" "$allowlist" "$repeat" \
    "$label" "$run_directory" | tee -a "$manifest"
  port=$((port + 1))
}

for dataset in 0709 0709b; do
  for half_map in 10 20; do
    run_case "$dataset" "$half_map" false '[]' 1 true
    run_case "$dataset" "$half_map" true '[0,1]' 1 true
  done
  # Two configured anchors means the two-anchor and all-anchor cases are the
  # same experiment. Anchor 0 is used for the cleaner single-anchor ablation.
  run_case "$dataset" 10 true '[0]' 1 false
  for repeat in 2 3; do
    run_case "$dataset" 10 false '[]' "$repeat" false
    run_case "$dataset" 10 true '[0,1]' "$repeat" false
  done
done

printf 'manifest\t%s\n' "$manifest"
