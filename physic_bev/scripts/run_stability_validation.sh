#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
usage: run_stability_validation.sh

Configuration is supplied with environment variables:
  DURATION_SEC       monitoring duration (default: 120)
  PHASE              output phase name (default: smoke_2min)
  RUN_ID             run directory name (default: current timestamp)
  STABILITY_CONFIG   YAML configuration path
  STABILITY_OUTPUT   explicit output directory
  REQUIRE_TRUSTED_TIME  true blocks formal output until NTP is synchronized (default: true)

This script never starts hardware or MCAP recording.
EOF
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PHYSIC_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$PHYSIC_ROOT/.." && pwd)"
source "$PROJECT_ROOT/tank_inspection_system/scripts/env.sh"

DURATION_SEC="${DURATION_SEC:-120}"
PHASE="${PHASE:-smoke_2min}"
CONFIG="${STABILITY_CONFIG:-$PHYSIC_ROOT/src/inspection_tools/config/stability_config.yaml}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUTPUT="${STABILITY_OUTPUT:-$PROJECT_ROOT/output/stability_validation/$RUN_ID/$PHASE}"
mkdir -p "$OUTPUT"

TIME_ARGS=(--output "$OUTPUT/time_readiness.json")
if [[ "${REQUIRE_TRUSTED_TIME:-true}" == true ]]; then
  TIME_ARGS+=(--require-ready)
fi
python3 -m inspection_tools.time_readiness "${TIME_ARGS[@]}"

python3 -m inspection_tools.timestamp_audit \
  --config "$CONFIG" --output "$OUTPUT/timestamp_audit.json"
python3 -m inspection_tools.livox_continuity observe \
  --output "$OUTPUT" --duration-sec "$DURATION_SEC" \
  >"$OUTPUT/livox_probe.log" 2>&1 &
LIVOX_PROBE_PID=$!
cleanup_probe() {
  if kill -0 "$LIVOX_PROBE_PID" 2>/dev/null; then
    kill -TERM "$LIVOX_PROBE_PID" 2>/dev/null || true
    wait "$LIVOX_PROBE_PID" 2>/dev/null || true
  fi
}
trap cleanup_probe EXIT INT TERM
set +e
python3 -m inspection_tools.stability_monitor \
  --config "$CONFIG" --output "$OUTPUT" --duration-sec "$DURATION_SEC"
MONITOR_RC=$?
wait "$LIVOX_PROBE_PID"
PROBE_RC=$?
set -e
trap - EXIT INT TERM
python3 -m inspection_tools.time_sync_analysis \
  --config "$CONFIG" --timing-csv "$OUTPUT/topic_timing.csv" \
  --output "$OUTPUT/time_sync_report.json"
set +e
python3 -m inspection_tools.livox_continuity classify \
  --monitor-report "$OUTPUT/stability_report.json" \
  --probe-report "$OUTPUT/livox_probe_report.json" \
  --output "$OUTPUT/livox_continuity_report.json"
LIVOX_RC=$?
set -e

python3 - "$OUTPUT" "$MONITOR_RC" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
report = json.loads((root / "stability_report.json").read_text())
sync = json.loads((root / "time_sync_report.json").read_text())
livox_path = root / "livox_continuity_report.json"
livox = json.loads(livox_path.read_text()) if livox_path.exists() else {
    "classification": "UNKNOWN", "reason": "independent probe report unavailable"}
clock = json.loads((root / "time_readiness.json").read_text())
lines = ["# Stability validation summary", "", f"- Actual duration: {report['actual_duration_sec']:.3f} s",
         f"- Monitor result: {'PASS' if report['pass'] else 'FAIL'}",
         f"- System time: {clock['status']} (RTC valid: {clock['rtc_valid']})",
         f"- Livox three-way classification: {livox['classification']} — {livox['reason']}",
         "", "## Topics", "",
         "| Topic | Count | Hz | Regressions | Result |", "|---|---:|---:|---:|---|"]
for topic, item in report["topics"].items():
    if item["received_count"] == 0 and item["expected_min_hz"] == 0:
        result = "SKIP"
    else:
        result = "PASS" if item["received_count"] > 1 and item["timestamp_regression_count"] == 0 and item["rate_pass"] is not False else "FAIL"
    lines.append(f"| {topic} | {item['received_count']} | {item['actual_rate_hz']:.3f} | {item['timestamp_regression_count']} | {result} |")
lines += ["", "## Physical synchronization", "", "- BLOCKED: " + sync["physical_sync_accuracy"]["reason"], ""]
(root / "validation_summary.md").write_text("\n".join(lines), encoding="utf-8")
PY

echo "validation output: $OUTPUT"
if (( MONITOR_RC != 0 )); then exit "$MONITOR_RC"; fi
if (( PROBE_RC != 0 )); then exit "$PROBE_RC"; fi
exit "$LIVOX_RC"
