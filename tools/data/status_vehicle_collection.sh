#!/usr/bin/env bash
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/packages/inspection_runtime/scripts/status_vehicle_collection.sh" "$@"
