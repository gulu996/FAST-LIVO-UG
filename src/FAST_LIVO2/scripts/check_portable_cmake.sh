#!/usr/bin/env bash
set -euo pipefail

src_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
vikit_files=(
  "${src_dir}/rpg_vikit/vikit_common/CMakeLists.txt"
  "${src_dir}/rpg_vikit/vikit_ros/CMakeLists.txt"
)

for cmake_file in "${vikit_files[@]}"; do
  if grep -En -- '(-march=|-mcpu=|-mtune=|-m(sse|mmx))' "${cmake_file}"; then
    echo "Architecture-specific compiler option found in ${cmake_file}" >&2
    exit 1
  fi
done

mvs_cmake="${src_dir}/mvs_ros_driver/CMakeLists.txt"
if grep -n 'CMAKE_HOST_SYSTEM_PROCESSOR' "${mvs_cmake}"; then
  echo "MVS SDK selection must use the target architecture" >&2
  exit 1
fi
grep -q 'CMAKE_SYSTEM_PROCESSOR' "${mvs_cmake}"

echo "Portable CMake checks passed."
