#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: $0 COMMAND [ARG ...]" >&2
  exit 2
fi

ready_topic="${FAST_LIVO_READY_TOPIC:-/fast_livo/subscribers_ready}"
echo "[fast_livo startup] waiting for ${ready_topic}=true" >&2
until timeout 5 rostopic echo -n 1 "$ready_topic" 2>/dev/null | grep 'data: True' >/dev/null; do
  if ! rosnode list >/dev/null 2>&1; then sleep 0.2; fi
done
echo "[fast_livo startup] subscribers ready; starting: $*" >&2
exec "$@"
