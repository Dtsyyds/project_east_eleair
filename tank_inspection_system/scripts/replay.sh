#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo "usage: replay.sh RUN_ID MODE [RATE] [OFFSET] [DURATION]"
  echo "canonical entry: physic_bev/scripts/replay_vehicle_collection.sh; arguments/exit status/signals are forwarded"
  exit 0
fi
if [[ -z "${FIELD_PHYSIC_ROOT:-}" ]]; then source "$SCRIPT_DIR/env.sh"; fi
exec bash "$FIELD_PHYSIC_ROOT/scripts/replay_vehicle_collection.sh" "$@"
