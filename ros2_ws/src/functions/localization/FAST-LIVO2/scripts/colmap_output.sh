#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help ]]; then echo 'Usage: FAST_LIVO_OUTPUT_ROOT=/path/in/runtime colmap_output.sh'; exit 0; fi
root="${FAST_LIVO_OUTPUT_ROOT:-${TANK_RUNTIME_ROOT:?source deploy/env.sh first}/fast_livo}"
for suffix in images sparse/0; do
  target="$root/Log/Colmap/$suffix"
  rm -rf -- "$target"
  mkdir -p -- "$target"
done
