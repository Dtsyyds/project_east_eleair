#!/usr/bin/env bash
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/packages/inspection_runtime/scripts/replay_vehicle_collection.sh" "$@"
