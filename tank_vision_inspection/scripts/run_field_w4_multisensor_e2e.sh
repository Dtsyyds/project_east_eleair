#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="/home/sunrise/east_eleair"
TANK="$ROOT/tank_vision_inspection"
BEV="${PHYSIC_BEV_ROOT:-$ROOT/physic_bev}"
DATA_ROOT="${DATA_ROOT:-/media/mcap_data}"
OUTPUT="$TANK/output/field_w4_multisensor_e2e"
PHASE="${1:-all}"
W4_DURATION_SEC="${W4_DURATION_SEC:-60}"
W4B_LONG_DURATION_SEC="${W4B_LONG_DURATION_SEC:-600}"
MID360_CONFIG="${MID360_CONFIG:-$BEV/src/livox_ros_driver2/config/MID360_config.json}"
SERIAL="${CAMERA_SERIAL:-046081820650}"
SERIAL_ARG="$SERIAL"
[[ "$SERIAL_ARG" == sn_* ]] || SERIAL_ARG="sn_${SERIAL_ARG}"
SDK_ROOT="${MINDVISION_SDK_ROOT:-$ROOT/MindVisionlinuxSDK}"
TRACKER_CONFIG="$TANK/configs/s100p_guided_anchor_seam_4096x3072.yaml"
TRACKER_REFERENCE="$TANK/configs/seam_geometry_reference_s100p_live_v1.json"
CONTROL="$DATA_ROOT/field_w4_multisensor_control"

set +u
source /opt/ros/humble/setup.bash
source "$TANK/ros2_ws/install_s100p_humble/setup.bash"
source "$BEV/install/setup.bash"
set -u
export MINDVISION_SDK_ROOT="$SDK_ROOT"
export PYTHONPATH="$TANK/ros2_ws/src/tank_vision_ros:$TANK${PYTHONPATH:+:$PYTHONPATH}"

W4PY=(/usr/bin/python3 "$TANK/scripts/field_w4_multisensor_e2e.py")
mkdir -p "$OUTPUT" "$CONTROL"

"${W4PY[@]}" checkpoint --output "$OUTPUT/w3_checkpoint.json"

stop_group() {
  local pid="${1:-}"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 60); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  fi
}

wait_run_manifest() {
  local run_dir="$1"
  for _ in $(seq 1 45); do
    [[ -f "$run_dir/run_manifest.json" ]] && return 0
    sleep 1
  done
  return 1
}

run_phase() {
  local phase="$1"
  local duration="$2"
  local observe_phase="$phase"
  [[ "$observe_phase" == w4b-* ]] && observe_phase="w4b"
  local run_id="FIELD-W4-${phase^^}-$(date +%Y%m%d_%H%M%S)"
  local run_dir="$DATA_ROOT/runs/$run_id"
  local launch_log="$OUTPUT/$run_id.launch.log"
  local inventory="$run_dir/logs/topic_inventory.json"
  local topic_args="$run_dir/logs/topic_args.json"

  if [[ "$phase" == "w4a" ]]; then
    "${W4PY[@]}" init-run --data-root "$DATA_ROOT" --run-id "$run_id" --phase "$phase" >/dev/null
  fi

  local launch_pid=""
  if [[ "$phase" == "w4a" ]]; then
    setsid ros2 launch inspection_bringup hardware_pipeline.launch.py \
      start_mindvision:=false start_mid360:=true start_sync:=false start_eddy:=false record:=false \
      mid360_config_path:="$MID360_CONFIG" mid360_publish_freq:=10.0 \
      publish_rough_extrinsics:=false >"$launch_log" 2>&1 &
  else
    setsid ros2 launch inspection_bringup visual_field_validation.launch.py \
      data_root:="$DATA_ROOT" run_id:="$run_id" sdk_root:="$SDK_ROOT" camera_serial:="$SERIAL_ARG" \
      camera_topic:=/camera/front/image_raw camera_info_topic:=/camera/front/camera_info \
      resolution_index:=-1 exposure_mode:=auto exposure_us:=0.0 \
      ae_min_exposure_us:=100.0 ae_max_exposure_us:=50000.0 \
      ae_min_analog_gain_raw:=2 ae_max_analog_gain_raw:=16 warmup_frames:=60 \
      camera_publish_rate_hz:=10.0 timeout_ms:=1000 \
      tracker_config:="$TRACKER_CONFIG" tracker_reference:="$TRACKER_REFERENCE" \
      publish_debug_image:=false auto_save_failures:=true auto_save_full_frame:=false \
      auto_save_context_jpeg:=true save_lossless_roi:=true auto_save_policy:=event \
      per_frame_metrics_log:=true failure_snapshot_cooldown_sec:=2.0 max_failure_snapshots:=100 \
      roi_validation_recording_enabled:=false record_mcap:=false record_raw_image:=false \
      start_mid360:=true mid360_config_path:="$MID360_CONFIG" start_sync:="$([[ "$phase" == "w4c" ]] && echo true || echo false)" \
      geometry_provenance_sampling_enabled:=false geometry_provenance_sample_period_sec:=5.0 \
      geometry_provenance_queue_size:=8 geometry_provenance_max_events:=30 >"$launch_log" 2>&1 &
  fi
  launch_pid=$!
  echo "$launch_pid" > "$CONTROL/$run_id.launch.pid"

  if [[ "$phase" != "w4a" ]]; then
    wait_run_manifest "$run_dir" || { echo "run manifest not created: $run_dir" >&2; stop_group "$launch_pid"; return 5; }
  fi
  mkdir -p "$run_dir/logs"
  cp "$launch_log" "$run_dir/logs/ros2_launch.log" 2>/dev/null || true

  local inventory_rc=1
  for _ in $(seq 1 40); do
    if "${W4PY[@]}" inventory --phase "$observe_phase" --output "$inventory" >/dev/null 2>&1; then
      if [[ "$phase" == "w4a" ]] && jq -e '.actual_mid360_lidar_topics | length > 0' "$inventory" >/dev/null 2>&1; then inventory_rc=0; break; fi
      if [[ "$phase" != "w4a" ]] && jq -e '(.actual_mid360_lidar_topics | length > 0) and (.required_visual_topics_present | length >= 3)' "$inventory" >/dev/null 2>&1; then inventory_rc=0; break; fi
    fi
    sleep 1
  done
  if [[ "$inventory_rc" != 0 ]]; then
    echo "required topics did not appear; phase=$phase" >&2
    stop_group "$launch_pid"
    "${W4PY[@]}" process-audit --output "$run_dir/logs/process_shutdown_audit.json" || true
    return 6
  fi

  if [[ "$phase" == "w4a" ]]; then
    "${W4PY[@]}" preflight --mid360-config "$MID360_CONFIG" --output "$OUTPUT/mid360_preflight.json" || true
  fi

  local required=""
  if [[ "$observe_phase" == "w4b" || "$observe_phase" == "w4c" ]]; then
    required="/camera/front/camera_info,/vision/front/seam_geometry,/vision/front/seam_state"
  fi
  if ! "${W4PY[@]}" topic-args --phase "$observe_phase" --inventory "$inventory" --required "$required" --output "$topic_args" >"$run_dir/logs/record_topics.txt"; then
    echo "required recording topics missing; phase=$phase" >&2
    stop_group "$launch_pid"
    return 7
  fi
  mapfile -t RECORD_TOPICS < "$run_dir/logs/record_topics.txt"
  ((${#RECORD_TOPICS[@]} > 0)) || { echo "no recording topics" >&2; stop_group "$launch_pid"; return 8; }

  if [[ -d "$run_dir/mcap" ]]; then
    rmdir "$run_dir/mcap"
  fi
  ros2 bag record --storage mcap --output "$run_dir/mcap" \
    --max-bag-size 10737418240 --max-bag-duration 900 --max-cache-size 1073741824 \
    --storage-config-file "$BEV/src/inspection_bringup/config/mcap_writer_options.yaml" \
    --qos-profile-overrides-path "$BEV/src/inspection_bringup/config/qos_overrides.yaml" \
    "${RECORD_TOPICS[@]}" >"$run_dir/logs/ros2_bag_record.log" 2>&1 &
  local bag_pid=$!

  "${W4PY[@]}" probe --inventory "$topic_args" --duration-sec "$duration" --output "$run_dir/logs/topic_probe.json" &
  local probe_pid=$!
  "${W4PY[@]}" monitor --duration-sec "$duration" --output "$run_dir/logs/resource_usage.json" &
  local monitor_pid=$!
  sleep "$duration"
  wait "$probe_pid" || true
  wait "$monitor_pid" || true
  stop_group "$bag_pid"
  stop_group "$launch_pid"
  wait "$bag_pid" || true
  wait "$launch_pid" || true

  ps -eo pid,ppid,pgid,stat,args > "$run_dir/logs/processes_after_stop.txt"
  "${W4PY[@]}" resource-summary --input "$run_dir/logs/resource_usage.json" --output "$run_dir/logs/resource_usage_summary.json" || true
  "${W4PY[@]}" process-audit --output "$run_dir/logs/process_shutdown_audit.json" || true
  "${W4PY[@]}" bag-report --bag "$run_dir/mcap" --output "$run_dir/logs/mcap_validation.json" --phase "$phase" --run-id "$run_id" || true
  cp "$launch_log" "$run_dir/logs/ros2_launch.log" 2>/dev/null || true
  echo "$phase=$run_dir" >> "$OUTPUT/completed_phases.txt"
  echo "completed phase=$phase run_dir=$run_dir"
}

case "$PHASE" in
  w4a)
    run_phase w4a "$W4_DURATION_SEC"
    ;;
  w4b)
    run_phase w4b "$W4_DURATION_SEC"
    ;;
  w4c)
    run_phase w4c "$W4_DURATION_SEC"
    ;;
  all)
    if grep -q '^w4a=' "$OUTPUT/completed_phases.txt" 2>/dev/null; then
      W4A_RUN="$(grep '^w4a=' "$OUTPUT/completed_phases.txt" | tail -1 | cut -d= -f2-)"
    else
      run_phase w4a "$W4_DURATION_SEC"
      W4A_RUN="$(tail -1 "$OUTPUT/completed_phases.txt" | cut -d= -f2-)"
    fi
    if ! jq -e '.validated and .finalized' "$W4A_RUN/logs/mcap_validation.json" >/dev/null 2>&1; then
      echo "W4-A did not pass; W4-B and W4-C are intentionally not started" >&2
      "${W4PY[@]}" aggregate --output-dir "$OUTPUT" --phase-run "w4a=$W4A_RUN" || true
      exit 20
    fi
    run_phase w4b-smoke "$W4_DURATION_SEC"
    SMOKE_RUN="$(tail -1 "$OUTPUT/completed_phases.txt" | cut -d= -f2-)"
    if ! jq -e '.validated and .finalized' "$SMOKE_RUN/logs/mcap_validation.json" >/dev/null 2>&1; then
      echo "W4-B smoke MCAP failed; W4-B long and W4-C are intentionally not started" >&2
      "${W4PY[@]}" aggregate --output-dir "$OUTPUT" --phase-run "w4a=$W4A_RUN" --phase-run "w4b-smoke=$SMOKE_RUN" || true
      exit 21
    fi
    run_phase w4b-long "$W4B_LONG_DURATION_SEC"
    LONG_RUN="$(tail -1 "$OUTPUT/completed_phases.txt" | cut -d= -f2-)"
    "${W4PY[@]}" aggregate --output-dir "$OUTPUT" --phase-run "w4a=$W4A_RUN" \
      --phase-run "w4b-smoke=$SMOKE_RUN" --phase-run "w4b-long=$LONG_RUN" || true
    if ! jq -e '.phases["w4b-long"].pass' "$OUTPUT/system_acceptance.json" >/dev/null 2>&1; then
      echo "W4-B long did not pass; W4-C is intentionally not started" >&2
      exit 22
    fi
    run_phase w4c "$W4_DURATION_SEC"
    ;;
  *)
    echo "usage: $0 {w4a|w4b|w4c|all}" >&2
    exit 2
    ;;
esac

mapfile -t PHASE_RUNS < <(sed 's/^/--phase-run /' "$OUTPUT/completed_phases.txt")
AGG_ARGS=("${W4PY[@]}" aggregate --output-dir "$OUTPUT")
while IFS='=' read -r phase run_dir; do
  [[ -n "$phase" && -n "$run_dir" ]] || continue
  AGG_ARGS+=(--phase-run "$phase=$run_dir")
done < "$OUTPUT/completed_phases.txt"
"${AGG_ARGS[@]}" || true
