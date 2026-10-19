#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
VISION_ROOT="${VISION_ROOT:-${ROOT_DIR}/../tank_vision_inspection}"
PHYSIC_INSTALL="${FIELD_PHYSIC_INSTALL:-${ROOT_DIR}/install}"
TANK_INSTALL="${FIELD_TANK_INSTALL:-${VISION_ROOT}/ros2_ws/install}"
RUN_ID="${1:-${FIELD_RUN_ID:-}}"
if [[ -z "${RUN_ID}" ]]; then
  echo "usage: stop_field_visual.sh RUN_ID" >&2
  exit 2
fi
RUN_DIR="${DATA_ROOT}/runs/${RUN_ID}"
PID_FILE="${RUN_DIR}/logs/launch.pid"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${PHYSIC_INSTALL}/setup.bash" ]]; then
  source "${PHYSIC_INSTALL}/setup.bash"
fi
if [[ -f "${TANK_INSTALL}/setup.bash" ]]; then
  source "${TANK_INSTALL}/setup.bash"
fi
set -u
if [[ ! -f "${PID_FILE}" ]]; then
  echo "missing launch pid: ${PID_FILE}" >&2
  exit 1
fi
PID="$(<"${PID_FILE}")"
if kill -0 "${PID}" 2>/dev/null; then
  kill -INT -- "-${PID}" 2>/dev/null || kill -INT "${PID}"
  for _ in {1..120}; do
    kill -0 "${PID}" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 "${PID}" 2>/dev/null; then
    kill -TERM -- "-${PID}" 2>/dev/null || kill -TERM "${PID}"
    for _ in {1..40}; do
      kill -0 "${PID}" 2>/dev/null || break
      sleep 0.25
    done
  fi
fi

FINALIZE_OPTIONS=()
if [[ -n "${FIELD_RECORD_RAW_IMAGE:-}" ]]; then
  FINALIZE_OPTIONS+=(--visual-with-raw)
fi
/usr/bin/python3 "${SCRIPT_DIR}/finalize_field_run.py" "${RUN_DIR}" \
  --task-id "${RUN_ID}" --asset-id "${FIELD_ASSET_ID:-UNASSIGNED}" \
  --course-id "${FIELD_COURSE_ID:-UNASSIGNED}" --plate-id "${FIELD_PLATE_ID:-UNASSIGNED}" \
  "${FINALIZE_OPTIONS[@]}"
