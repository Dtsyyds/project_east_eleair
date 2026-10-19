#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PROJECT_ROOT="$(cd -- "${PACKAGE_ROOT}/../../.." && pwd)"
PHYSIC_ROOT="$(cd -- "${PROJECT_ROOT}/../physic_bev" && pwd)"

if [[ -f /opt/tros/humble/setup.bash ]]; then
  source /opt/tros/humble/setup.bash
else
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${PHYSIC_ROOT}/install/setup.bash" ]]; then
  source "${PHYSIC_ROOT}/install/setup.bash"
fi
if [[ -f "${PROJECT_ROOT}/.venv_rerun/bin/activate" ]]; then
  source "${PROJECT_ROOT}/.venv_rerun/bin/activate"
else
  echo "ERROR: frozen Rerun environment is missing: ${PROJECT_ROOT}/.venv_rerun" >&2
  exit 2
fi
if [[ -f "${PROJECT_ROOT}/ros2_ws/install/setup.bash" ]]; then
  if ! source "${PROJECT_ROOT}/ros2_ws/install/setup.bash" >/dev/null 2>&1; then
    echo "WARNING: existing project install overlay is incomplete; using source package" >&2
  fi
fi

set -u

# Do not inherit the interactive shell's domain (often 23 on this board).
# The offline builder does not create DDS readers, but keep its recorded domain
# isolated and deterministic.  Override only through the builder-specific name.
export ROS_DOMAIN_ID="${RRD_ROS_DOMAIN_ID:-184}"
export RERUN_ANALYTICS_ENABLED=false
export PYTHONPATH="${PACKAGE_ROOT}:${PYTHONPATH:-}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"

if [[ $# -ne 1 || "${1}" == -* ]]; then
  echo "Usage: $0 /media/mcap_data/runs/RUN_ID" >&2
  echo "The argument must be one finalized run directory, not an individual MCAP file." >&2
  exit 2
fi

RUN_DIR="$(realpath -e -- "${1}")"
if [[ ! -d "${RUN_DIR}" ]]; then
  echo "ERROR: run directory does not exist: ${RUN_DIR}" >&2
  exit 2
fi
RUN_ID="$(basename -- "${RUN_DIR}")"
CONFIG_PATH="${PACKAGE_ROOT}/config/rerun.yaml"
OUTPUT_PATH="${RUN_DIR}/rerun/${RUN_ID}.rrd"
BUILD_MODE="${RRD_BUILD_MODE:-strict}"

if [[ "${BUILD_MODE}" != "strict" && "${BUILD_MODE}" != "allow-incomplete" ]]; then
  echo "ERROR: RRD_BUILD_MODE must be strict or allow-incomplete: ${BUILD_MODE}" >&2
  exit 2
fi

exec nice -n 10 ionice -c2 -n7 \
  "${PROJECT_ROOT}/.venv_rerun/bin/python" "${SCRIPT_DIR}/build_run_rrd.py" \
  --run-dir "${RUN_DIR}" \
  --config "${CONFIG_PATH}" \
  --output "${OUTPUT_PATH}" \
  --mode "${BUILD_MODE}" \
  --ros-domain-id "${ROS_DOMAIN_ID}" \
  --replay-rate 1.0 \
  --timeout-sec 7200 \
  --include-camera \
  --camera-mode preview \
  --keep-partial-on-failure \
  --log-level INFO
