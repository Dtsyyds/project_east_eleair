#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${1:-}"
MODE="${2:-mapping-only}"
RATE="${3:-1.0}"
OFFSET="${4:-0}"
DURATION="${5:-0}"
[[ -n "$RUN_ID" ]] || { echo "usage: replay_mapping_eddy_collection.sh RUN_ID {mapping-only|mapping-eddy|full} [0.5|1.0] [offset] [duration]" >&2; exit 2; }
[[ "$MODE" == mapping-only || "$MODE" == mapping-eddy || "$MODE" == full ]] || { echo "invalid replay mode: $MODE" >&2; exit 2; }
RUN_DIR="$DATA_ROOT/runs/$RUN_ID"
[[ -f "$RUN_DIR/run_manifest.json" ]] || { echo "missing run: $RUN_DIR" >&2; exit 1; }

python3 "$SCRIPT_DIR/vehicle_collection_manager.py" assert-no-active --data-root "$DATA_ROOT" >/dev/null || {
  echo "an active formal collection exists; replay refused" >&2; exit 2;
}
COLLECTION_DOMAIN="$(jq -r '.collection_ros_domain_id // 0' "$RUN_DIR/run_manifest.json")"
export ROS_DOMAIN_ID="${FIELD_REPLAY_ROS_DOMAIN_ID:-91}"
[[ "$ROS_DOMAIN_ID" != "$COLLECTION_DOMAIN" ]] || {
  echo "replay ROS_DOMAIN_ID must differ from collection domain $COLLECTION_DOMAIN" >&2; exit 2;
}

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/tros/humble/setup.bash ]]; then source /opt/tros/humble/setup.bash; fi
source "$ROOT_DIR/install/setup.bash"
TANK_ROOT="${VISION_ROOT:-${ROOT_DIR}/../tank_vision_inspection}"
source "${FIELD_TANK_INSTALL:-${TANK_ROOT}/ros2_ws/install_s100p_humble}/setup.bash"
set -u
export PYTHONPATH="$TANK_ROOT/ros2_ws/src/tank_vision_ros:$TANK_ROOT${PYTHONPATH:+:$PYTHONPATH}"

OPTIONS=()
if [[ "${FIELD_REPLAY_PUBLISH_CLOCK:-0}" == 1 ]]; then
  CLOCK_INFO="$(ros2 topic info /clock 2>/dev/null || true)"
  if grep -Eq 'Publisher count: [1-9]' <<<"$CLOCK_INFO"; then
    echo "an existing /clock publisher is present in replay domain $ROS_DOMAIN_ID" >&2; exit 2
  fi
  OPTIONS+=(--publish-clock)
fi
exec python3 "$SCRIPT_DIR/replay_mapping_eddy_collection.py" "$RUN_DIR" \
  --mode "$MODE" --rate "$RATE" --start-offset "$OFFSET" --duration "$DURATION" "${OPTIONS[@]}"
