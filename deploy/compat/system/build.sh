#!/usr/bin/env bash
if [[ "${1:-}" == --wireless-only ]]; then shift; set -- --packages-select agv_bridge "$@"; fi
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/build.sh" "$@"
