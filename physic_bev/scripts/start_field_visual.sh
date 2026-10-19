#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${FIELD_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
VISION_ROOT="${VISION_ROOT:-${ROOT_DIR}/../tank_vision_inspection}"
PHYSIC_INSTALL="${FIELD_PHYSIC_INSTALL:-${ROOT_DIR}/install}"
TANK_INSTALL="${FIELD_TANK_INSTALL:-${VISION_ROOT}/ros2_ws/install}"
TRACKER_CONFIG="${FIELD_TRACKER_CONFIG:-${VISION_ROOT}/configs/s100p_guided_anchor_seam_4096x3072.yaml}"
TRACKER_REFERENCE="${FIELD_TRACKER_REFERENCE:-${VISION_ROOT}/configs/seam_geometry_reference_s100p_live_v1.json}"
REPORT_DIR="$(mktemp -d "/tmp/field_visual_preflight_${RUN_ID}.XXXXXX")"
LAUNCH_LOG="$(mktemp "/tmp/field_visual_launch_${RUN_ID}.XXXXXX.log")"
RUN_DIR="${DATA_ROOT}/runs/${RUN_ID}"
CAMERA_SERIAL="${FIELD_CAMERA_SERIAL:-}"
if [[ -n "${CAMERA_SERIAL}" && "${CAMERA_SERIAL}" != sn_* ]]; then
  CAMERA_SERIAL="sn_${CAMERA_SERIAL}"
fi

if [[ -e "${RUN_DIR}" ]] && [[ -n "$(find "${RUN_DIR}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "refusing to overwrite non-empty run: ${RUN_DIR}" >&2
  exit 1
fi

# ROS setup files reference optional variables that may be unset on a clean
# shell.  Keep strict mode for the launcher itself, but do not let that
# prevent a normal environment setup.
set +u
source /opt/ros/humble/setup.bash
source "${PHYSIC_INSTALL}/setup.bash"
if [[ -f "${TANK_INSTALL}/setup.bash" ]]; then
  source "${TANK_INSTALL}/setup.bash"
fi
set -u

PREFLIGHT_OPTIONS=()
if [[ -n "${FIELD_ALLOW_HARDWARE_PENDING:-}" ]]; then
  PREFLIGHT_OPTIONS+=(--allow-hardware-pending)
fi
/usr/bin/python3 "${SCRIPT_DIR}/field_visual_preflight.py" \
  --data-root "${DATA_ROOT}" --run-id "${RUN_ID}" \
  --tracker-config "${TRACKER_CONFIG}" --tracker-reference "${TRACKER_REFERENCE}" \
  --camera-serial "${CAMERA_SERIAL}" --exposure-mode "${FIELD_EXPOSURE_MODE:-auto}" \
  --exposure-us "${FIELD_EXPOSURE_US:-0}" --report-dir "${REPORT_DIR}" \
  "${PREFLIGHT_OPTIONS[@]}"

# Put launch and all of its ROS children in a dedicated process group so the
# stop script can close the complete run without leaving orphaned camera or
# rosbag processes.
setsid ros2 launch inspection_bringup visual_field_validation.launch.py \
  data_root:="${DATA_ROOT}" run_id:="${RUN_ID}" \
  tracker_config:="${TRACKER_CONFIG}" tracker_reference:="${TRACKER_REFERENCE}" \
  camera_serial:="${CAMERA_SERIAL}" "$@" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
for _ in {1..50}; do
  [[ -f "${RUN_DIR}/run_manifest.json" ]] && break
  sleep 0.1
done
if [[ ! -f "${RUN_DIR}/run_manifest.json" ]]; then
  echo "launch did not create run_manifest: ${RUN_DIR}" >&2
  tail -80 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi
mkdir -p "${RUN_DIR}/logs"
echo "${LAUNCH_PID}" > "${RUN_DIR}/logs/launch.pid"
cp "${LAUNCH_LOG}" "${RUN_DIR}/logs/launch.log"
cp "${REPORT_DIR}/preflight_report.json" "${RUN_DIR}/preflight_report.json"
cp "${REPORT_DIR}/preflight_report.txt" "${RUN_DIR}/preflight_report.txt"
echo "run_id=${RUN_ID}"
echo "run_dir=${RUN_DIR}"
echo "launch_pid=${LAUNCH_PID}"
