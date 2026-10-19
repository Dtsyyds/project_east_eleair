#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="/home/sunrise/east_eleair"
TANK="$ROOT/tank_vision_inspection"
BEV="$ROOT/physic_bev"
DATA_ROOT="${DATA_ROOT:-/media/mcap_data}"
RUN_ID="${RUN_ID:-FIELD-W3-E2E-$(date +%Y%m%d_%H%M%S)}"
SERIAL="${CAMERA_SERIAL:-046081820650}"
SERIAL_ARG="${SERIAL}"
[[ "$SERIAL_ARG" == sn_* ]] || SERIAL_ARG="sn_${SERIAL_ARG}"
SDK_ROOT="${MINDVISION_SDK_ROOT:-$ROOT/MindVisionlinuxSDK}"
TRACKER_CONFIG="$TANK/configs/s100p_guided_anchor_seam_4096x3072.yaml"
TRACKER_REFERENCE="$TANK/configs/seam_geometry_reference_s100p_live_v1.json"
PROVENANCE_ENABLED="${GEOMETRY_PROVENANCE_ENABLED:-false}"
PROVENANCE_SAMPLE_PERIOD_SEC="${GEOMETRY_PROVENANCE_SAMPLE_PERIOD_SEC:-10.0}"
PROVENANCE_MAX_EVENTS="${GEOMETRY_PROVENANCE_MAX_EVENTS:-30}"
AUTO_SAVE_FAILURES="${AUTO_SAVE_FAILURES:-true}"
AUTO_SAVE_FULL_FRAME="${AUTO_SAVE_FULL_FRAME:-false}"
PUBLISH_DEBUG_IMAGE="${PUBLISH_DEBUG_IMAGE:-false}"
ROI_VALIDATION_ENABLED="${ROI_VALIDATION_ENABLED:-false}"
PER_FRAME_METRICS_LOG="${PER_FRAME_METRICS_LOG:-true}"
CONTROL="$DATA_ROOT/field_visual_e2e_control"
set +u
source /opt/ros/humble/setup.bash
source "$TANK/ros2_ws/install_s100p_humble/setup.bash"
source "$BEV/install/setup.bash"
set -u
export MINDVISION_SDK_ROOT="$SDK_ROOT"
export PYTHONPATH="$TANK/ros2_ws/src/tank_vision_ros:$TANK${PYTHONPATH:+:$PYTHONPATH}"
LOCAL_EVIDENCE="$TANK/output/field_w3_system_e2e_closeout"
mkdir -p "$LOCAL_EVIDENCE"
PREFLIGHT="$LOCAL_EVIDENCE/preflight_$RUN_ID.json"
/usr/bin/python3 "$TANK/scripts/field_visual_preflight.py" --data-root "$DATA_ROOT" --run-id "$RUN_ID" --output "$PREFLIGHT" --manifest-output "$LOCAL_EVIDENCE/frozen_software_manifest_$RUN_ID.json"
mkdir -p "$CONTROL" "$DATA_ROOT/preflight_reports"
LAUNCH_LOG="$LOCAL_EVIDENCE/$RUN_ID.launch.log"
CMD=(ros2 launch inspection_bringup visual_field_validation.launch.py
  data_root:="$DATA_ROOT" run_id:="$RUN_ID" sdk_root:="$SDK_ROOT" camera_serial:="$SERIAL_ARG"
  camera_topic:=/camera/front/image_raw camera_info_topic:=/camera/front/camera_info resolution_index:=-1
  exposure_mode:=auto exposure_us:=0.0 ae_min_exposure_us:=100.0 ae_max_exposure_us:=50000.0
  ae_min_analog_gain_raw:=2 ae_max_analog_gain_raw:=16 warmup_frames:=60 camera_publish_rate_hz:=10.0 timeout_ms:=1000
  tracker_config:="$TRACKER_CONFIG" tracker_reference:="$TRACKER_REFERENCE" publish_debug_image:="$PUBLISH_DEBUG_IMAGE"
  auto_save_failures:="$AUTO_SAVE_FAILURES" auto_save_full_frame:="$AUTO_SAVE_FULL_FRAME"
  auto_save_context_jpeg:=true save_lossless_roi:=true auto_save_policy:=event
  per_frame_metrics_log:="$PER_FRAME_METRICS_LOG" failure_snapshot_cooldown_sec:=2.0 max_failure_snapshots:=100
  roi_validation_recording_enabled:="$ROI_VALIDATION_ENABLED"
  record_mcap:=true record_raw_image:=false start_mid360:=false start_sync:=false
  geometry_provenance_sampling_enabled:="$PROVENANCE_ENABLED" geometry_provenance_sample_period_sec:="$PROVENANCE_SAMPLE_PERIOD_SEC"
  geometry_provenance_queue_size:=8 geometry_provenance_max_events:="$PROVENANCE_MAX_EVENTS")
printf '%q ' "${CMD[@]}" > "$LOCAL_EVIDENCE/$RUN_ID.command"; printf '\n' >> "$LOCAL_EVIDENCE/$RUN_ID.command"
setsid "${CMD[@]}" >"$LAUNCH_LOG" 2>&1 &
PID=$!
echo "$PID" > "$CONTROL/$RUN_ID.pid"
for _ in $(seq 1 30); do
  if [[ -f "$DATA_ROOT/runs/$RUN_ID/run_manifest.json" ]]; then
    RUN_DIR="$DATA_ROOT/runs/$RUN_ID"; cp "$PREFLIGHT" "$RUN_DIR/logs/preflight.json"; cp "${PREFLIGHT%.json}.txt" "$RUN_DIR/logs/preflight.txt"; cp "$LAUNCH_LOG" "$RUN_DIR/logs/ros2_launch.log" || true; echo "$PID" > "$RUN_DIR/launch.pid"; break
  fi
  if ! kill -0 "$PID" 2>/dev/null; then echo "launch exited before run manifest; see $LAUNCH_LOG" >&2; exit 4; fi
  sleep 1
done
[[ -n "${RUN_DIR:-}" ]] || { echo "run manifest not created; see $LAUNCH_LOG" >&2; exit 5; }
echo "RUN_ID=$RUN_ID"; echo "RUN_DIR=$RUN_DIR"; echo "PID=$PID"; echo "stop: $TANK/scripts/stop_field_visual_e2e.sh $RUN_ID"
