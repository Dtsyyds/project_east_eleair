#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="/home/sunrise/east_eleair"
TANK="$ROOT/tank_vision_inspection"
DATA_ROOT="${DATA_ROOT:-/media/mcap_data}"
CASE="${1:?usage: run_field_w3_perf_isolation.sh p0|p1|p2}"
case "$CASE" in p0|p1|p2) ;; *) echo "case must be p0, p1 or p2" >&2; exit 2 ;; esac
RUN_ID="${RUN_ID:-FIELD-W3-PERF-${CASE^^}-$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="$TANK/output/field_w3_perf_isolation"
DURATION_SEC="${PERF_DURATION_SEC:-60}"

case "$CASE" in
  p0)
    export AUTO_SAVE_FAILURES=false GEOMETRY_PROVENANCE_ENABLED=false
    export GEOMETRY_PROVENANCE_SAMPLE_PERIOD_SEC=30.0 GEOMETRY_PROVENANCE_MAX_EVENTS=3
    ;;
  p1)
    export AUTO_SAVE_FAILURES=true AUTO_SAVE_FULL_FRAME=false GEOMETRY_PROVENANCE_ENABLED=false
    export GEOMETRY_PROVENANCE_SAMPLE_PERIOD_SEC=30.0 GEOMETRY_PROVENANCE_MAX_EVENTS=3
    ;;
  p2)
    export AUTO_SAVE_FAILURES=false GEOMETRY_PROVENANCE_ENABLED=true
    export GEOMETRY_PROVENANCE_SAMPLE_PERIOD_SEC=30.0 GEOMETRY_PROVENANCE_MAX_EVENTS=3
    ;;
esac
export RUN_ID DATA_ROOT PUBLISH_DEBUG_IMAGE=false ROI_VALIDATION_ENABLED=false PER_FRAME_METRICS_LOG=true

START_OUTPUT="$(bash "$TANK/scripts/run_field_visual_e2e.sh")"
printf '%s\n' "$START_OUTPUT"
RUN_DIR="$(printf '%s\n' "$START_OUTPUT" | sed -n 's/^RUN_DIR=//p' | tail -1)"
[[ -n "$RUN_DIR" && -d "$RUN_DIR" ]] || { echo "run directory was not reported" >&2; exit 3; }

MONITOR_PID=""
cleanup() {
  if [[ -n "$MONITOR_PID" ]] && kill -0 "$MONITOR_PID" 2>/dev/null; then
    kill "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
/usr/bin/python3 "$TANK/scripts/field_w3_perf_monitor.py" \
  --run-dir "$RUN_DIR" --output "$RUN_DIR/logs/perf_monitor.jsonl" \
  --duration-sec "$DURATION_SEC" --interval-sec 1.0 &
MONITOR_PID=$!
sleep "$DURATION_SEC"
bash "$TANK/scripts/stop_field_visual_e2e.sh" "$RUN_ID"
wait "$MONITOR_PID" 2>/dev/null || true
/usr/bin/python3 "$TANK/scripts/field_w3_perf_isolation.py" \
  --case "${CASE,,}" --run-dir "$RUN_DIR" --output-dir "$OUTPUT_DIR"
echo "CASE=${CASE,,}"
echo "RUN_DIR=$RUN_DIR"
echo "OUTPUT_DIR=$OUTPUT_DIR"
