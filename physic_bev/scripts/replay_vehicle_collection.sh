#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${1:-}"
MODE="${2:-core-only}"
RATE="${3:-1.0}"
OFFSET="${4:-0}"
DURATION="${5:-0}"
if [[ -z "$RUN_ID" ]]; then
  echo "usage: replay_vehicle_collection.sh RUN_ID {core-only|sensor-only|perception|full-sensor|algorithm-replay} [0.5|1.0] [offset_sec] [duration_sec]" >&2
  exit 2
fi
RUN_DIR="$DATA_ROOT/runs/$RUN_ID"
[[ -f "$RUN_DIR/run_manifest.json" ]] || { echo "missing run: $RUN_DIR" >&2; exit 1; }

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/tros/humble/setup.bash && -f /opt/tros/humble/local_setup.bash ]]; then source /opt/tros/humble/setup.bash; fi
source "${FIELD_PHYSIC_INSTALL:-$ROOT_DIR/install}/local_setup.bash"
TANK_ROOT="${VISION_ROOT:-${ROOT_DIR}/../tank_vision_inspection}"
TANK_INSTALL="${FIELD_TANK_INSTALL:-${TANK_ROOT}/ros2_ws/install_s100p_humble}"
source "$TANK_INSTALL/local_setup.bash"
set -u
ANALYSIS_DIR="${FIELD_ANALYSIS_DIR:-${DATA_ROOT}/analysis/${RUN_ID}/replay_$(date +%Y%m%dT%H%M%S)_$$}"
mkdir -p "$ANALYSIS_DIR/logs" "$ANALYSIS_DIR/metrics"

TRACKER_PID=""
cleanup() {
  if [[ -n "$TRACKER_PID" ]] && kill -0 "$TRACKER_PID" 2>/dev/null; then
    kill -INT -- "-$TRACKER_PID" 2>/dev/null || kill -INT "$TRACKER_PID" 2>/dev/null || true
    wait "$TRACKER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [[ "$MODE" == algorithm-replay ]]; then
  CONFIG="$(find "$RUN_DIR/frozen_configs" -type f -name '*.yaml' | head -1)"
  REFERENCE="$(find "$RUN_DIR/frozen_configs" -type f -name '*reference*.json' | head -1)"
  [[ -n "$CONFIG" && -n "$REFERENCE" ]] || {
    echo "frozen tracker config/reference missing" >&2
    exit 3
  }
  setsid ros2 run tank_vision_ros stable_seam_node --ros-args \
    -r __node:=field_w5_algorithm_replay \
    -p input_mode:=ros_topic \
    -p image_topic:=/replay/camera/front/image_raw \
    -p camera_info_topic:=/replay/camera/front/camera_info \
    -p geometry_topic:=/replay/algorithm/seam_geometry \
    -p seam_state_topic:=/replay/algorithm/seam_state \
    -p debug_image_topic:=/replay/algorithm/debug_image \
    -p config_file:="$CONFIG" -p reference_file:="$REFERENCE" \
    -p publish_debug_image:=false -p auto_save_failures:=false \
    -p geometry_provenance_sampling_enabled:=false \
    >"$ANALYSIS_DIR/logs/algorithm_replay.log" 2>&1 &
  TRACKER_PID=$!
  sleep 2
fi

OPTIONS=()
if [[ -n "${FIELD_REPLAY_DISCOVERY_WAIT_SEC:-}" ]]; then OPTIONS+=(--discovery-wait-sec "$FIELD_REPLAY_DISCOVERY_WAIT_SEC"); fi
if [[ "${FIELD_REPLAY_PUBLISH_CLOCK:-0}" == "1" ]]; then OPTIONS+=(--publish-clock); fi
python3 "$SCRIPT_DIR/replay_vehicle_collection.py" "$RUN_DIR" \
  --mode "$MODE" --rate "$RATE" --start-offset "$OFFSET" --duration "$DURATION" \
  --report-dir "$ANALYSIS_DIR/metrics" "${OPTIONS[@]}"
