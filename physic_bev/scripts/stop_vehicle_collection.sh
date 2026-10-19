#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${1:-${FIELD_RUN_ID:-}}"
if [[ -z "$RUN_ID" ]]; then
  RUN_ID="$(python3 "$SCRIPT_DIR/vehicle_collection_manager.py" active-run \
    --data-root "$DATA_ROOT")" || exit $?
fi
[[ "$RUN_ID" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$ ]] || {
  echo "invalid run id: $RUN_ID" >&2
  exit 2
}
RUN_DIR="${DATA_ROOT}/runs/${RUN_ID}"
[[ -f "$RUN_DIR/run_manifest.json" ]] || {
  echo "missing run manifest: $RUN_DIR/run_manifest.json" >&2
  exit 1
}
if [[ -f "$RUN_DIR/complete.json" ]] && python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); sys.exit(0 if d.get("finalized") is True else 1)' "$RUN_DIR/complete.json"; then
  echo "run is already sealed: $RUN_ID"
  exit 0
fi
LOCK="$RUN_DIR/logs/stop.lock"
if ! mkdir "$LOCK" 2>/dev/null; then
  echo "stop/finalize is already running or completed for $RUN_ID" >&2
  exit 1
fi
trap 'rmdir "$LOCK" 2>/dev/null || true' EXIT

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/tros/humble/setup.bash ]]; then source /opt/tros/humble/setup.bash; fi
PHYSIC_INSTALL="${FIELD_PHYSIC_INSTALL:-$ROOT_DIR/install}"
if [[ -f "$PHYSIC_INSTALL/setup.bash" ]]; then source "$PHYSIC_INSTALL/setup.bash"; fi
TANK_INSTALL="${FIELD_TANK_INSTALL:-${ROOT_DIR}/../tank_vision_inspection/ros2_ws/install_s100p_humble}"
if [[ -f "$TANK_INSTALL/setup.bash" ]]; then source "$TANK_INSTALL/setup.bash"; fi
set -u

OPTIONS=()
if [[ "${FIELD_SKIP_FINALIZE:-}" == "1" ]]; then OPTIONS+=(--skip-finalize); fi
python3 "$SCRIPT_DIR/stop_vehicle_collection.py" "$RUN_DIR" "${OPTIONS[@]}"
