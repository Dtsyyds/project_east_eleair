#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo "usage: validate.sh RUN_DIR --report-dir ANALYSIS_DIR/metrics"
  echo "canonical entry: physic_bev/scripts/validate_vehicle_collection.py; arguments/exit status/signals are forwarded"
  exit 0
fi
if [[ -z "${FIELD_PHYSIC_ROOT:-}" ]]; then source "$SCRIPT_DIR/env.sh"; fi
exec python3 "$FIELD_PHYSIC_ROOT/scripts/validate_vehicle_collection.py" "$@"
