#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export RRD_BUILD_MODE=allow-incomplete

if [[ $# -eq 1 && "${1}" != -* ]]; then
  RUN_DIR="$(realpath -e -- "${1}" 2>/dev/null || true)"
  if [[ -n "${RUN_DIR}" && -d "${RUN_DIR}" ]]; then
    RUN_ID="$(basename -- "${RUN_DIR}")"
    OUTPUT_DIR="${RUN_DIR}/rerun"
    PARTIAL="${OUTPUT_DIR}/${RUN_ID}_INCOMPLETE.rrd.partial"
    LOCK="${OUTPUT_DIR}/.build_run_rrd.lock"
    if [[ -f "${PARTIAL}" && ! -e "${LOCK}" ]]; then
      FAILED_PARTIAL="${PARTIAL}.failed-$(date -u +%Y%m%dT%H%M%SZ)-$$"
      mv -- "${PARTIAL}" "${FAILED_PARTIAL}"
      echo "Archived previous failed partial: ${FAILED_PARTIAL}" >&2
    fi
  fi
fi

exec "${SCRIPT_DIR}/build_run_rrd_nice.sh" "$@"
