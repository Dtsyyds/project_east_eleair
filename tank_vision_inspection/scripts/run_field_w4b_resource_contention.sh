#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="/home/sunrise/east_eleair"
TANK="$ROOT/tank_vision_inspection"
BEV="${PHYSIC_BEV_ROOT:-$ROOT/physic_bev}"
DATA_ROOT="${DATA_ROOT:-/media/mcap_data}"
OUTPUT="$TANK/output/field_w4b_resource_contention"
DURATION="${W4B_RESOURCE_DURATION_SEC:-60}"
SETTLE_SEC="${W4B_RESOURCE_SETTLE_SEC:-5}"
MID360_CONFIG="${MID360_CONFIG:-$BEV/src/livox_ros_driver2/config/MID360_config.json}"
SERIAL="${CAMERA_SERIAL:-046081820650}"
SERIAL_ARG="$SERIAL"
[[ "$SERIAL_ARG" == sn_* ]] || SERIAL_ARG="sn_${SERIAL_ARG}"
SDK_ROOT="${MINDVISION_SDK_ROOT:-$ROOT/MindVisionlinuxSDK}"
TRACKER_CONFIG="$TANK/configs/s100p_guided_anchor_seam_4096x3072.yaml"
TRACKER_REFERENCE="$TANK/configs/seam_geometry_reference_s100p_live_v1.json"
CONTROL="$TANK/output/field_w4b_resource_contention/control"

set +u
source /opt/ros/humble/setup.bash
source "$TANK/ros2_ws/install_s100p_humble/setup.bash"
source "$BEV/install/setup.bash"
set -u
export MINDVISION_SDK_ROOT="$SDK_ROOT"
export PYTHONPATH="$TANK/ros2_ws/src/tank_vision_ros:$TANK/scripts:$TANK${PYTHONPATH:+:$PYTHONPATH}"

BASEPY=(/usr/bin/python3 "$TANK/scripts/field_w4_multisensor_e2e.py")
RESO=(/usr/bin/python3 "$TANK/scripts/field_w4b_resource_contention.py")
mkdir -p "$OUTPUT" "$CONTROL"
export ROS_LOG_DIR="$OUTPUT/ros_logs"
mkdir -p "$ROS_LOG_DIR"
if ! /usr/bin/python3 -c 'import os,sys; p=os.path.realpath(sys.argv[1]); raise SystemExit(0 if os.path.ismount(p) and os.access(p, os.W_OK) else 1)' "$DATA_ROOT"; then
  echo "DATA_ROOT must be a writable mount point: $DATA_ROOT" >&2
  exit 30
fi

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

stop_launch_children() {
  local launch_log="${1:-}"
  [[ -f "$launch_log" ]] || return 0
  local -a pids=()
  mapfile -t pids < <(sed -nE 's/.*process started with pid \[([0-9]+)\].*/\1/p' "$launch_log" | sort -u)
  local pid args
  for pid in "${pids[@]}"; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    case "$args" in
      *mindvision_camera_node*|*stable_seam_node*|*livox_ros_driver2_node*|*livox_imu_adapter*|*static_transform_publisher*)
        kill -INT "$pid" 2>/dev/null || true
        ;;
    esac
  done
  for _ in $(seq 1 40); do
    local alive=0
    for pid in "${pids[@]}"; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      if kill -0 "$pid" 2>/dev/null; then
        args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
        case "$args" in
          *mindvision_camera_node*|*stable_seam_node*|*livox_ros_driver2_node*|*livox_imu_adapter*|*static_transform_publisher*) alive=1 ;;
        esac
      fi
    done
    (( alive == 0 )) && return 0
    sleep 0.25
  done
  for pid in "${pids[@]}"; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    case "$args" in
      *mindvision_camera_node*|*stable_seam_node*|*livox_ros_driver2_node*|*livox_imu_adapter*|*static_transform_publisher*)
        kill -TERM "$pid" 2>/dev/null || true
        ;;
    esac
  done
}

wait_manifest() {
  local run_dir="$1"
  for _ in $(seq 1 45); do
    [[ -f "$run_dir/run_manifest.json" ]] && return 0
    sleep 1
  done
  return 1
}

inventory_ready() {
  local inventory="$1" with_lidar="$2"
  if [[ "$with_lidar" == true ]]; then
    jq -e '(.required_visual_topics_present | index("/camera/front/camera_info")) and (.required_visual_topics_present | index("/vision/front/seam_geometry")) and (.required_visual_topics_present | index("/vision/front/seam_state")) and (.actual_mid360_lidar_topics | length > 0)' "$inventory" >/dev/null
  else
    jq -e '(.required_visual_topics_present | index("/camera/front/camera_info")) and (.required_visual_topics_present | index("/vision/front/seam_geometry")) and (.required_visual_topics_present | index("/vision/front/seam_state"))' "$inventory" >/dev/null
  fi
}

assert_no_stack_processes() {
  local matches
  if command -v rg >/dev/null 2>&1; then
    matches="$(ps -eo pid=,ppid=,pgid=,stat=,args= | rg 'mindvision_camera_node|stable_seam_node|livox_ros_driver2_node|livox_imu_adapter|inspection_sync' || true)"
  else
    matches="$(ps -eo pid=,ppid=,pgid=,stat=,args= | grep -E 'mindvision_camera_node|stable_seam_node|livox_ros_driver2_node|livox_imu_adapter|inspection_sync' | grep -v '[g]rep -E' || true)"
  fi
  if [[ -n "$matches" ]]; then
    echo "existing field stack processes detected; refusing to start a new phase:" >&2
    echo "$matches" >&2
    return 31
  fi
}

record_args() {
  local inventory="$1" mode="$2"
  local -n out_ref="$3"
  out_ref=()
  if [[ "$mode" == visual || "$mode" == unified ]]; then
    mapfile -t out_ref < <(jq -r '.all_topics | keys[] | select(. == "/camera/front/camera_info" or . == "/vision/front/seam_geometry" or . == "/vision/front/seam_state" or . == "/diagnostics" or . == "/tf_static")' "$inventory")
  fi
  if [[ "$mode" == livox || "$mode" == unified ]]; then
    mapfile -t LIDAR_TOPICS < <(jq -r '(.actual_mid360_lidar_topics | keys[]), (.actual_mid360_imu_topics | keys[])' "$inventory")
    out_ref+=("${LIDAR_TOPICS[@]}")
  fi
}

run_experiment() {
  local label="$1" mid="$2" mode="$3"
  local run_id="FIELD-W4B-RESOURCE-${label}-$(date +%Y%m%d_%H%M%S)"
  local run_dir="$DATA_ROOT/runs/$run_id"
  local launch_log="$OUTPUT/$run_id.launch.log"
  local inventory="$run_dir/logs/topic_inventory.json"
  local launch_pid="" visual_bag_pid="" livox_bag_pid=""
  local visual_bag="$run_dir/mcap"
  local livox_bag="$run_dir/mcap_livox"

  sleep "$SETTLE_SEC"
  assert_no_stack_processes

  if [[ "$mid" == true ]]; then
    setsid ros2 launch inspection_bringup visual_field_validation.launch.py \
      data_root:="$DATA_ROOT" run_id:="$run_id" sdk_root:="$SDK_ROOT" camera_serial:="$SERIAL_ARG" \
      camera_topic:=/camera/front/image_raw camera_info_topic:=/camera/front/camera_info \
      resolution_index:=-1 exposure_mode:=auto exposure_us:=0.0 \
      ae_min_exposure_us:=100.0 ae_max_exposure_us:=50000.0 \
      ae_min_analog_gain_raw:=2 ae_max_analog_gain_raw:=16 warmup_frames:=60 \
      camera_publish_rate_hz:=10.0 timeout_ms:=1000 tracker_config:="$TRACKER_CONFIG" \
      tracker_reference:="$TRACKER_REFERENCE" publish_debug_image:=false \
      auto_save_failures:=true auto_save_full_frame:=false auto_save_context_jpeg:=true \
      save_lossless_roi:=true auto_save_policy:=event per_frame_metrics_log:=true \
      failure_snapshot_cooldown_sec:=2.0 max_failure_snapshots:=100 \
      roi_validation_recording_enabled:=false record_mcap:=false record_raw_image:=false \
      start_mid360:=true mid360_config_path:="$MID360_CONFIG" start_sync:=false \
      geometry_provenance_sampling_enabled:=false geometry_provenance_sample_period_sec:=5.0 \
      geometry_provenance_queue_size:=8 geometry_provenance_max_events:=30 >"$launch_log" 2>&1 &
  else
    setsid ros2 launch inspection_bringup visual_field_validation.launch.py \
      data_root:="$DATA_ROOT" run_id:="$run_id" sdk_root:="$SDK_ROOT" camera_serial:="$SERIAL_ARG" \
      camera_topic:=/camera/front/image_raw camera_info_topic:=/camera/front/camera_info \
      resolution_index:=-1 exposure_mode:=auto exposure_us:=0.0 \
      ae_min_exposure_us:=100.0 ae_max_exposure_us:=50000.0 \
      ae_min_analog_gain_raw:=2 ae_max_analog_gain_raw:=16 warmup_frames:=60 \
      camera_publish_rate_hz:=10.0 timeout_ms:=1000 tracker_config:="$TRACKER_CONFIG" \
      tracker_reference:="$TRACKER_REFERENCE" publish_debug_image:=false \
      auto_save_failures:=true auto_save_full_frame:=false auto_save_context_jpeg:=true \
      save_lossless_roi:=true auto_save_policy:=event per_frame_metrics_log:=true \
      failure_snapshot_cooldown_sec:=2.0 max_failure_snapshots:=100 \
      roi_validation_recording_enabled:=false record_mcap:=false record_raw_image:=false \
      start_mid360:=false start_sync:=false \
      geometry_provenance_sampling_enabled:=false geometry_provenance_sample_period_sec:=5.0 \
      geometry_provenance_queue_size:=8 geometry_provenance_max_events:=30 >"$launch_log" 2>&1 &
  fi
  launch_pid=$!
  echo "$launch_pid" > "$CONTROL/$run_id.launch.pid"
  wait_manifest "$run_dir" || {
    stop_group "$launch_pid"
    stop_launch_children "$launch_log"
    return 5
  }
  mkdir -p "$run_dir/logs"

  local ready=1
  for _ in $(seq 1 45); do
    "${BASEPY[@]}" inventory --phase w4b --output "$inventory" >/dev/null 2>&1 || true
    if inventory_ready "$inventory" "$mid"; then ready=0; break; fi
    sleep 1
  done
  if [[ "$ready" != 0 ]]; then
    stop_group "$launch_pid"
    stop_launch_children "$launch_log"
    "${BASEPY[@]}" process-audit --output "$run_dir/logs/process_shutdown_audit.json" \
      >"$run_dir/logs/process_audit_stdout.log" 2>&1 || true
    return 6
  fi
  cp "$launch_log" "$run_dir/logs/ros2_launch.log" 2>/dev/null || true

  local -a VISUAL_TOPICS_ARRAY=() LIDAR_TOPICS=() RECORD_TOPICS=()
  record_args "$inventory" visual VISUAL_TOPICS_ARRAY
  record_args "$inventory" livox LIDAR_TOPICS
  record_args "$inventory" "$mode" RECORD_TOPICS

  # The launch creates empty storage directories as part of the run contract;
  # rosbag2 requires a non-existing output directory, so remove only these
  # newly-created empty directories before starting each external recorder.
  if [[ "$mode" == visual || "$mode" == unified || "$mode" == split ]]; then
    [[ -d "$visual_bag" ]] && rmdir "$visual_bag"
  fi
  if [[ "$mode" == split ]]; then
    [[ -d "$livox_bag" ]] && rmdir "$livox_bag"
  fi

  case "$mode" in
    visual|unified)
      setsid ros2 bag record --storage mcap --output "$visual_bag" \
        --max-bag-size 10737418240 --max-bag-duration 900 --max-cache-size 1073741824 \
        --storage-config-file "$BEV/src/inspection_bringup/config/mcap_writer_options.yaml" \
        --qos-profile-overrides-path "$BEV/src/inspection_bringup/config/qos_overrides.yaml" \
        "${RECORD_TOPICS[@]}" >"$run_dir/logs/ros2_bag_visual.log" 2>&1 &
      visual_bag_pid=$!
      ;;
    none) : ;;
  esac
  if [[ "$mode" == livox ]]; then
    setsid ros2 bag record --storage mcap --output "$livox_bag" \
      --max-bag-size 10737418240 --max-bag-duration 900 --max-cache-size 1073741824 \
      --storage-config-file "$BEV/src/inspection_bringup/config/mcap_writer_options.yaml" \
      --qos-profile-overrides-path "$BEV/src/inspection_bringup/config/qos_overrides.yaml" \
      "${RECORD_TOPICS[@]}" >"$run_dir/logs/ros2_bag_livox.log" 2>&1 &
    livox_bag_pid=$!
  elif [[ "$mode" == split ]]; then
    setsid ros2 bag record --storage mcap --output "$visual_bag" \
      --max-bag-size 10737418240 --max-bag-duration 900 --max-cache-size 1073741824 \
      --storage-config-file "$BEV/src/inspection_bringup/config/mcap_writer_options.yaml" \
      --qos-profile-overrides-path "$BEV/src/inspection_bringup/config/qos_overrides.yaml" \
      "${VISUAL_TOPICS_ARRAY[@]}" >"$run_dir/logs/ros2_bag_visual.log" 2>&1 &
    visual_bag_pid=$!
    setsid ros2 bag record --storage mcap --output "$livox_bag" \
      --max-bag-size 10737418240 --max-bag-duration 900 --max-cache-size 1073741824 \
      --storage-config-file "$BEV/src/inspection_bringup/config/mcap_writer_options.yaml" \
      --qos-profile-overrides-path "$BEV/src/inspection_bringup/config/qos_overrides.yaml" \
      "${LIDAR_TOPICS[@]}" >"$run_dir/logs/ros2_bag_livox.log" 2>&1 &
    livox_bag_pid=$!
  fi

  local probe_inventory="$inventory"
  "${BASEPY[@]}" probe --inventory "$probe_inventory" --duration-sec "$DURATION" \
    --output "$run_dir/logs/topic_probe.json" >"$run_dir/logs/topic_probe_stdout.log" 2>&1 &
  local probe_pid=$!
  "${RESO[@]}" resource-monitor --duration-sec "$DURATION" --output "$run_dir/logs/resource_usage.json" \
    --visual-bag "$([[ "$mode" == visual || "$mode" == unified || "$mode" == split ]] && echo "$visual_bag" || true)" \
    --livox-bag "$([[ "$mode" == split ]] && echo "$livox_bag" || true)" &
  local monitor_pid=$!
  sleep "$DURATION"
  wait "$probe_pid" || true
  wait "$monitor_pid" || true
  stop_group "$visual_bag_pid"
  stop_group "$livox_bag_pid"
  stop_group "$launch_pid"
  stop_launch_children "$launch_log"
  [[ -n "$visual_bag_pid" ]] && wait "$visual_bag_pid" || true
  [[ -n "$livox_bag_pid" ]] && wait "$livox_bag_pid" || true
  [[ -n "$launch_pid" ]] && wait "$launch_pid" || true

  ps -eo pid,ppid,pgid,stat,args > "$run_dir/logs/processes_after_stop.txt"
  "${RESO[@]}" resource-summary --input "$run_dir/logs/resource_usage.json" \
    --output "$run_dir/logs/resource_summary.json" --csv "$run_dir/logs/per_process_resources.csv" || true
  "${BASEPY[@]}" process-audit --output "$run_dir/logs/process_shutdown_audit.json" \
    >"$run_dir/logs/process_audit_stdout.log" 2>&1 || true
  local -a BAG_SPECS=()
  [[ "$mode" == visual || "$mode" == unified || "$mode" == split ]] && BAG_SPECS+=(--bag "visual=$visual_bag")
  [[ "$mode" == split ]] && BAG_SPECS+=(--bag "livox=$livox_bag")
  "${RESO[@]}" validate --run-dir "$run_dir" --phase "$label" --output "$run_dir/logs/mcap_validation.json" "${BAG_SPECS[@]}" || true
  cp "$launch_log" "$run_dir/logs/ros2_launch.log" 2>/dev/null || true
  printf '%s=%s\n' "$label" "$run_dir" >> "$OUTPUT/completed_phases.txt"
  echo "completed $label $run_dir"
}

run_all() {
  : > "$OUTPUT/completed_phases.txt"
  run_experiment B0 false visual
  run_experiment B1 true none
  run_experiment B2 true visual
  run_experiment B3 true split
  run_experiment B4 true unified
  local -a args=("${RESO[@]}" aggregate --output-dir "$OUTPUT")
  while IFS='=' read -r label run_dir; do
    [[ -n "$label" && -n "$run_dir" ]] || continue
    args+=(--phase-run "$label=$run_dir")
  done < "$OUTPUT/completed_phases.txt"
  "${args[@]}" || true
}

case "${1:-all}" in
  all) run_all ;;
  B0|B1|B2|B3|B4)
    run_experiment "$1" "$([[ "$1" == B0 ]] && echo false || echo true)" \
      "$([[ "$1" == B1 ]] && echo none || [[ "$1" == B3 ]] && echo split || [[ "$1" == B4 ]] && echo unified || echo visual)"
    ;;
  *) echo "usage: $0 {all|B0|B1|B2|B3|B4}" >&2; exit 2 ;;
esac
