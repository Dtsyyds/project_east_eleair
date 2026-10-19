#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Run bounded D405 lifecycle cycles without USB reset/rebind.

Environment:
  MODE=d405|d405_mindvision|four  (default: d405)
  CYCLES=5                        (default: 1)
  DURATION_SEC=120                (default: 120)
  READY_TIMEOUT_SEC=60            (default: 60)
  RUN_ID=<unique name>            (default: timestamp)
  ROS_DOMAIN_ID=77                (default: 77)

The script owns only the launch process group it starts. It sends SIGINT, then
SIGTERM only if the owned group does not exit within 45 seconds. It never uses
USB reset, driver rebind, controller reset, or SIGKILL. A missing USB/SDK device
stops all remaining cycles and preserves the evidence directory.
EOF
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PHYSIC_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$PHYSIC_ROOT/.." && pwd)"
source "$PROJECT_ROOT/tank_inspection_system/scripts/env.sh"
OVERLAY_SETUP="$PHYSIC_ROOT/install_lifecycle/setup.bash"
if [[ ! -f "$OVERLAY_SETUP" ]]; then
  OVERLAY_SETUP="$PHYSIC_ROOT/install/setup.bash"
fi
if [[ -f "$OVERLAY_SETUP" ]]; then
  set +u
  source "$OVERLAY_SETUP"
  set -u
fi
export PYTHONPATH="$PHYSIC_ROOT/src/inspection_tools:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

MODE="${MODE:-d405}"
CYCLES="${CYCLES:-1}"
DURATION_SEC="${DURATION_SEC:-120}"
READY_TIMEOUT_SEC="${READY_TIMEOUT_SEC:-60}"
RUN_ID="${RUN_ID:-d405_lifecycle_$(date +%Y%m%d_%H%M%S)}"
OUTPUT="${LIFECYCLE_OUTPUT:-$PROJECT_ROOT/output/stability_validation/$RUN_ID}"
mkdir -p "$OUTPUT"

case "$MODE" in
  d405)
    DEVICE_ARGS=(start_d405:=true start_mindvision:=false start_fisheye:=false start_mid360:=false) ;;
  d405_mindvision)
    DEVICE_ARGS=(start_d405:=true start_mindvision:=true start_fisheye:=false start_mid360:=false) ;;
  four)
    DEVICE_ARGS=(start_d405:=true start_mindvision:=true start_fisheye:=true start_mid360:=true) ;;
  *) echo "unsupported MODE=$MODE" >&2; exit 2 ;;
esac

event() {
  python3 - "$1" "$2" <<'PY'
import json, sys, time
print(json.dumps({"system_timestamp_ns": time.time_ns(),
                  "steady_timestamp_ns": time.monotonic_ns(),
                  "event": sys.argv[1], "detail": sys.argv[2]}), flush=True)
PY
}

wait_owned_group() {
  local pid="$1" timeout="$2" began now
  began="$(date +%s)"
  while kill -0 "$pid" 2>/dev/null; do
    if [[ -r "/proc/$pid/stat" ]] && [[ "$(awk '{print $3}' "/proc/$pid/stat")" == Z ]]; then
      return 0
    fi
    now="$(date +%s)"
    if (( now - began >= timeout )); then return 1; fi
    sleep 0.25
  done
}

overall=0
for cycle in $(seq 1 "$CYCLES"); do
  cycle_dir="$OUTPUT/cycle_$(printf '%02d' "$cycle")"
  mkdir -p "$cycle_dir"
  events="$cycle_dir/lifecycle_events.jsonl"
  event PREFLIGHT_BEGIN "mode=$MODE cycle=$cycle" | tee -a "$events"
  python3 -m inspection_tools.usb_lifecycle snapshot --sdk \
    --sdk-timeout-sec 15 --output "$cycle_dir/preflight.json"
  if ! python3 - "$cycle_dir/preflight.json" <<'PY'
import json, sys
d=json.load(open(sys.argv[1]))
raise SystemExit(0 if d["usb_present"] and d["sdk_enumeration"].get("d405_found") else 1)
PY
  then
    event PREFLIGHT_BLOCKED "D405 absent from USB or SDK enumeration" | tee -a "$events"
    overall=3
    break
  fi

  event LAUNCH_BEGIN "hardware_pipeline mode=$MODE" | tee -a "$events"
  setsid ros2 launch inspection_bringup hardware_pipeline.launch.py \
    record:=false start_sync:=false start_eddy:=false \
    d405_serial_number:=260322276866 \
    d405_rgb_profile:=1280x720x30 d405_depth_profile:=1280x720x30 \
    "${DEVICE_ARGS[@]}" >"$cycle_dir/launch_console.log" 2>&1 &
  launch_pid=$!
  event LAUNCH_PROCESS_STARTED "pid=$launch_pid pgid=$launch_pid" | tee -a "$events"

  set +e
  python3 -m inspection_tools.usb_lifecycle observe \
    --duration-sec "$DURATION_SEC" --output "$cycle_dir/observation.json"
  observer_rc=$?
  set -e
  event OBSERVATION_ENDED "returncode=$observer_rc" | tee -a "$events"

  if kill -0 "$launch_pid" 2>/dev/null; then
    event SIGINT_SENT "owned_pgid=$launch_pid" | tee -a "$events"
    kill -INT -- "-$launch_pid"
  fi
  if ! wait_owned_group "$launch_pid" 45; then
    event SIGTERM_SENT "owned group did not exit within 45 seconds" | tee -a "$events"
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait_owned_group "$launch_pid" 15 || true
  fi
  set +e
  wait "$launch_pid"
  launch_rc=$?
  set -e
  event LAUNCH_PROCESS_EXITED "returncode=$launch_rc" | tee -a "$events"

  ready_begin="$(date +%s)"
  usb_ready=false
  while (( $(date +%s) - ready_begin < READY_TIMEOUT_SEC )); do
    if [[ -e /sys/bus/usb/devices/4-1/idVendor ]] && \
       [[ "$(< /sys/bus/usb/devices/4-1/idVendor)" == 8086 ]] && \
       [[ "$(< /sys/bus/usb/devices/4-1/idProduct)" == 0b5b ]]; then
      usb_ready=true
      break
    fi
    if compgen -G '/sys/bus/usb/devices/*/idVendor' >/dev/null; then
      while IFS= read -r vendor_file; do
        device_dir="${vendor_file%/idVendor}"
        if [[ "$(< "$vendor_file")" == 8086 ]] && \
           [[ -r "$device_dir/idProduct" ]] && [[ "$(< "$device_dir/idProduct")" == 0b5b ]]; then
          usb_ready=true
          break 2
        fi
      done < <(find /sys/bus/usb/devices -mindepth 2 -maxdepth 2 -name idVendor -type f 2>/dev/null)
    fi
    sleep 0.5
  done
  ready_elapsed="$(( $(date +%s) - ready_begin ))"
  event USB_READINESS "present=$usb_ready elapsed_sec=$ready_elapsed" | tee -a "$events"
  python3 -m inspection_tools.usb_lifecycle snapshot --sdk \
    --sdk-timeout-sec 15 --output "$cycle_dir/post_stop.json"
  python3 -m inspection_tools.usb_lifecycle analyze-logs "$cycle_dir/launch_console.log" \
    --output "$cycle_dir/driver_events.json"

  set +e
  python3 - "$cycle_dir" "$MODE" "$cycle" "$observer_rc" "$launch_rc" "$usb_ready" <<'PY'
import json, sys
from pathlib import Path
root=Path(sys.argv[1])
pre=json.loads((root/'preflight.json').read_text())
post=json.loads((root/'post_stop.json').read_text())
obs=json.loads((root/'observation.json').read_text()) if (root/'observation.json').exists() else None
report={"schema_version":"tank_d405_lifecycle_cycle_v1", "mode":sys.argv[2],
        "cycle":int(sys.argv[3]), "observer_returncode":int(sys.argv[4]),
        "launch_returncode":int(sys.argv[5]), "usb_ready_after_stop":sys.argv[6]=='true',
        "preflight":pre, "post_stop":post, "observation":obs,
        "pass": bool(obs and all(v['received_count'] > 0 for v in obs['topics'].values())
                     and post['usb_present'] and post['sdk_enumeration'].get('d405_found'))}
(root/'cycle_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
raise SystemExit(0 if report['pass'] else 1)
PY
  cycle_rc=$?
  set -e
  if (( cycle_rc != 0 )); then
    event CYCLE_FAILED "stop remaining cycles; preserve evidence" | tee -a "$events"
    overall=1
    break
  fi
  event CYCLE_PASSED "USB and SDK available after stop" | tee -a "$events"
done

echo "lifecycle output: $OUTPUT"
exit "$overall"
