#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="/home/sunrise/east_eleair"; TANK="$ROOT/tank_vision_inspection"; DATA_ROOT="${DATA_ROOT:-/media/mcap_data}"
RUN_ID="${1:?usage: stop_field_visual_e2e.sh RUN_ID}"; RUN_DIR="$DATA_ROOT/runs/$RUN_ID"; CONTROL="$DATA_ROOT/field_visual_e2e_control/$RUN_ID.pid"
set +u
source /opt/ros/humble/setup.bash
source "$TANK/ros2_ws/install_s100p_humble/setup.bash"
source "$ROOT/physic_bev/install/setup.bash"
set -u
[[ -d "$RUN_DIR" ]] || { echo "run not found: $RUN_DIR" >&2; exit 2; }
PID="$(cat "$CONTROL" 2>/dev/null || true)"
if [[ "$PID" =~ ^[0-9]+$ ]] && kill -0 "$PID" 2>/dev/null; then
  kill -INT -- "-$PID" 2>/dev/null || kill -INT "$PID" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
  if kill -0 "$PID" 2>/dev/null; then kill -TERM -- "-$PID" 2>/dev/null || kill -TERM "$PID" 2>/dev/null || true; fi
  for _ in $(seq 1 30); do kill -0 "$PID" 2>/dev/null || break; sleep 1; done
fi
ps -eo pid,ppid,pgid,stat,cmd > "$RUN_DIR/logs/processes_after_stop.txt"
ros2 bag info "$RUN_DIR/mcap" > "$RUN_DIR/logs/ros2_bag_info.txt" 2>&1 || true
VALIDATE=(ros2 run inspection_tools validate_bag "$RUN_DIR/mcap" --visual-profile --output "$RUN_DIR/mcap/validation_report.json")
if "${VALIDATE[@]}" > "$RUN_DIR/logs/validate_bag.log" 2>&1; then
  ros2 run inspection_tools finalize_bag "$RUN_DIR/mcap" --task-id "$RUN_ID" --asset-id "tank_visual" --course-id "field_w3" --plate-id "visual" --validation-report "$RUN_DIR/mcap/validation_report.json" > "$RUN_DIR/logs/finalize_bag.log" 2>&1 || true
else
  echo "validation failed; finalize forbidden" >&2
fi
/usr/bin/python3 "$TANK/scripts/field_visual_e2e_report.py" --run-dir "$RUN_DIR" --output "$TANK/output/field_w3_system_e2e_closeout" || true
echo "RUN_DIR=$RUN_DIR"; echo "report=$TANK/output/field_w3_system_e2e_closeout"
