#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 -m http.server "${PORT:-8080}" --bind 0.0.0.0
