#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VISION_ROOT="${VISION_ROOT:-${ROOT_DIR}/../tank_vision_inspection}"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${FIELD_RUN_ID:-FIELD-VEHICLE-$(date +%Y%m%d_%H%M%S)}"
CAMERA_SERIAL="${FIELD_CAMERA_SERIAL:-046081820650}"
CAMERA_RESOLUTION_INDEX="${FIELD_CAMERA_RESOLUTION_INDEX:--1}"
RECORD_CORE=true
RECORD_MAPPING=false
RECORD_MID360=true
RECORD_D405=true
RECORD_EDDY=true
RECORD_FRONT_VIDEO=true
AUTO_SAVE_FAILURES=true
ENABLE_TRACKER=false
PROVENANCE_ENABLED=false
RECORD_FRONT_RAW_BURST=false
RAW_BURST_MODE="${FIELD_RAW_BURST_MODE:-full}"
RAW_BURST_DURATION_SEC="${FIELD_RAW_BURST_DURATION_SEC:-10}"
D405_ENABLE_INFRA1="${FIELD_D405_ENABLE_INFRA1:-true}"
D405_ENABLE_INFRA2="${FIELD_D405_ENABLE_INFRA2:-true}"
D405_DEPTH_PROFILE="${FIELD_D405_DEPTH_PROFILE:-848,480,30}"
D405_INFRA_PROFILE="${FIELD_D405_INFRA_PROFILE:-848,480,30}"
D405_COLOR_PROFILE="${FIELD_D405_COLOR_PROFILE:-848,480,30}"
D405_GLOBAL_TIME_ENABLED="${FIELD_D405_GLOBAL_TIME_ENABLED:-false}"
D405_ERROR_POLLING_ENABLED="${FIELD_D405_ERROR_POLLING_ENABLED:-false}"
D405_OWNER_LOCK="${FIELD_D405_OWNER_LOCK:-/tmp/physic_bev_d405.lock}"
D405_EXPECTED_FPS="${D405_DEPTH_PROFILE##*,}"
EDDY_DEVICE="${FIELD_EDDY_DEVICE:-/dev/eddy}"
EDDY_BAUD="${FIELD_EDDY_BAUD:-4000000}"
EDDY_CHANNEL_MEAN_TOPIC="${FIELD_EDDY_CHANNEL_MEAN_TOPIC:-/eddy/channel_mean}"
COLLECTION_PROFILE="vehicle_multisensor_split_recorders"
ATTACH_TO_EXISTING_MAPPING=true
START_MAPPING=false

for argument in "$@"; do
  key="${argument%%:=*}"
  value="${argument#*:=}"
  if [[ "$argument" != *":="* ]]; then
    echo "invalid argument (expected name:=value): $argument" >&2
    exit 2
  fi
  case "$key" in
    run_id) RUN_ID="$value" ;;
    collection_profile) COLLECTION_PROFILE="$value" ;;
    record_core) RECORD_CORE="$value" ;;
    record_mapping) RECORD_MAPPING="$value" ;;
    record_mid360) RECORD_MID360="$value" ;;
    record_mid360_raw) RECORD_MID360="$value" ;;
    record_d405) RECORD_D405="$value" ;;
    record_eddy) RECORD_EDDY="$value" ;;
    record_front_video) RECORD_FRONT_VIDEO="$value" ;;
    auto_save_failures) AUTO_SAVE_FAILURES="$value" ;;
    enable_tracker) ENABLE_TRACKER="$value" ;;
    provenance_enabled) PROVENANCE_ENABLED="$value" ;;
    record_front_raw_burst) RECORD_FRONT_RAW_BURST="$value" ;;
    raw_burst_mode) RAW_BURST_MODE="$value" ;;
    raw_burst_duration_sec) RAW_BURST_DURATION_SEC="$value" ;;
    attach_to_existing_mapping) ATTACH_TO_EXISTING_MAPPING="$value" ;;
    start_mapping) START_MAPPING="$value" ;;
    *) echo "unsupported launch argument: $key" >&2; exit 2 ;;
  esac
done

for value in "$RECORD_CORE" "$RECORD_MAPPING" "$RECORD_MID360" "$RECORD_D405" "$RECORD_EDDY" "$RECORD_FRONT_VIDEO" \
  "$AUTO_SAVE_FAILURES" "$PROVENANCE_ENABLED" "$RECORD_FRONT_RAW_BURST" \
  "$D405_ENABLE_INFRA1" "$D405_ENABLE_INFRA2" "$D405_GLOBAL_TIME_ENABLED" \
  "$D405_ERROR_POLLING_ENABLED" "$ATTACH_TO_EXISTING_MAPPING" "$START_MAPPING"; do
  [[ "$value" == "true" || "$value" == "false" ]] || {
    echo "boolean launch values must be true or false, got: $value" >&2
    exit 2
  }
done
[[ "$COLLECTION_PROFILE" == "vehicle_multisensor_split_recorders" || "$COLLECTION_PROFILE" == "mapping_eddy" ]] || {
  echo "unsupported collection_profile: $COLLECTION_PROFILE" >&2; exit 2;
}
if [[ "$COLLECTION_PROFILE" == mapping_eddy ]]; then
  [[ "$RECORD_MID360" == false && "$RECORD_D405" == false && "$ENABLE_TRACKER" == false \
     && "$PROVENANCE_ENABLED" == false && "$RECORD_FRONT_RAW_BURST" == false ]] || {
    echo "mapping_eddy profile forbids D405, MID-360 raw, tracker, provenance and raw front MCAP" >&2
    exit 2
  }
  [[ "$ATTACH_TO_EXISTING_MAPPING" == true && "$START_MAPPING" == false ]] || {
    echo "FIELD-W6 v1 formal entry supports attach-only: attach_to_existing_mapping:=true start_mapping:=false" >&2
    exit 2
  }
fi
[[ "$ENABLE_TRACKER" == "true" || "$ENABLE_TRACKER" == "false" ]] || {
  echo "enable_tracker must be true or false" >&2
  exit 2
}
if [[ "$ENABLE_TRACKER" == false && "$AUTO_SAVE_FAILURES" == true ]]; then
  echo "tracker is disabled; disabling tracker-dependent automatic failure snapshots" >&2
  AUTO_SAVE_FAILURES=false
fi
[[ "$RAW_BURST_MODE" == "full" || "$RAW_BURST_MODE" == "roi" ]] || {
  echo "raw_burst_mode must be full or roi" >&2
  exit 2
}
if [[ "$CAMERA_SERIAL" != sn_* ]]; then CAMERA_SERIAL="sn_${CAMERA_SERIAL}"; fi

PHYSIC_INSTALL="${FIELD_PHYSIC_INSTALL:-${ROOT_DIR}/install}"
TANK_INSTALL="${FIELD_TANK_INSTALL:-${VISION_ROOT}/ros2_ws/install_s100p_humble}"
TRACKER_CONFIG="${FIELD_TRACKER_CONFIG:-${VISION_ROOT}/configs/s100p_guided_anchor_seam_4096x3072.yaml}"
TRACKER_REFERENCE="${FIELD_TRACKER_REFERENCE:-${VISION_ROOT}/configs/seam_geometry_reference_s100p_live_v1.json}"
MID360_CONFIG="${FIELD_MID360_CONFIG:-${ROOT_DIR}/src/livox_ros_driver2/config/MID360_config.json}"
RUN_DIR="${DATA_ROOT}/runs/${RUN_ID}"
PREFLIGHT_DIR="$(mktemp -d "/tmp/field_w5_preflight_${RUN_ID}.XXXXXX")"
PREFLIGHT_REPORT="${PREFLIGHT_DIR}/preflight_report.json"
TIME_READINESS_REPORT="${PREFLIGHT_DIR}/time_readiness.json"
RUN_INITIALIZED=false

cleanup_start_failure() {
  local rc=$?
  trap - ERR
  if [[ "$RUN_INITIALIZED" == true && -f "$RUN_DIR/run_manifest.json" ]]; then
    echo "startup failed (rc=$rc); stopping only processes registered for $RUN_ID" >&2
    FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
  fi
  exit "$rc"
}
trap cleanup_start_failure ERR

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/tros/humble/setup.bash ]]; then source /opt/tros/humble/setup.bash; fi
source "${TANK_INSTALL}/setup.bash"
source "${PHYSIC_INSTALL}/setup.bash"
set -u
export MINDVISION_SDK_ROOT="${MINDVISION_SDK_ROOT:-${ROOT_DIR}/../MindVisionSDK_v2.1.0.50/sdk}"
if [[ "${FIELD_USE_INSTALLED_CODE:-0}" != 1 ]]; then
  export PYTHONPATH="${VISION_ROOT}/ros2_ws/src/tank_vision_ros:${VISION_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
fi

# Formal collection records require a trustworthy Unix epoch.  This check is
# read-only: it never changes the system clock, and basic device diagnostics
# remain available when the gate rejects a formal recording.
TIME_READINESS_ARGS=(--output "$TIME_READINESS_REPORT")
if [[ "${FIELD_REQUIRE_TRUSTED_TIME:-true}" == true ]]; then
  TIME_READINESS_ARGS+=(--require-ready)
fi
PYTHONPATH="${ROOT_DIR}/src/inspection_tools${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m inspection_tools.time_readiness "${TIME_READINESS_ARGS[@]}"

python3 "${SCRIPT_DIR}/vehicle_collection_manager.py" preflight \
  --data-root "$DATA_ROOT" --run-id "$RUN_ID" --output "$PREFLIGHT_REPORT" \
  --tracker-config "$TRACKER_CONFIG" --tracker-reference "$TRACKER_REFERENCE" \
  --mid360-config "$MID360_CONFIG" \
  --record-core "$RECORD_CORE" --record-mid360 "$RECORD_MID360" \
  --record-mapping "$RECORD_MAPPING" \
  --record-d405 "$RECORD_D405" --record-eddy "$RECORD_EDDY" \
  --record-front-video "$RECORD_FRONT_VIDEO" --eddy-device "$EDDY_DEVICE" \
  --auto-save-failures "$AUTO_SAVE_FAILURES" --provenance-enabled "$PROVENANCE_ENABLED" \
  --record-front-raw-burst "$RECORD_FRONT_RAW_BURST" \
  --raw-burst-mode "$RAW_BURST_MODE" --raw-burst-duration-sec "$RAW_BURST_DURATION_SEC" \
  --video-bitrate-mbps "${FIELD_VIDEO_BITRATE_MBPS:-16}" \
  --storage-safety-factor "${FIELD_STORAGE_SAFETY_FACTOR:-1.5}" \
  --benchmark-size-mib "${FIELD_STORAGE_BENCHMARK_MIB:-512}"

MAPPING_AUDIT=""
MAPPING_QOS=""
MAPPING_PATH_GROWTH=""
MAPPING_BASELINE=""
if [[ "$RECORD_MAPPING" == true ]]; then
  MAPPING_AUDIT="$PREFLIGHT_DIR/mapping_topic_inventory.json"
  MAPPING_QOS="$PREFLIGHT_DIR/qos_overrides.yaml"
  MAPPING_PATH_GROWTH="$PREFLIGHT_DIR/path_growth_analysis.json"
  MAPPING_BASELINE="$PREFLIGHT_DIR/baseline_performance.json"
  python3 "${SCRIPT_DIR}/mapping_eddy_live_probe.py" \
    --output "$MAPPING_AUDIT" --qos-output "$MAPPING_QOS" \
    --path-output "$MAPPING_PATH_GROWTH" \
    --performance-output "$MAPPING_BASELINE" \
    --seconds "${FIELD_MAPPING_BASELINE_SEC:-60}"
fi

VISUAL_LOG="${PREFLIGHT_DIR}/visual_launch.log"
ROI_ENABLED=false
if [[ "$RECORD_FRONT_RAW_BURST" == true && "$RAW_BURST_MODE" == roi ]]; then ROI_ENABLED=true; fi
CAMERA_QOS=best_effort
if [[ "$RECORD_FRONT_VIDEO" == true ]]; then CAMERA_QOS=reliable; fi
setsid ros2 launch inspection_bringup visual_field_validation.launch.py \
  data_root:="$DATA_ROOT" run_id:="$RUN_ID" sdk_root:="$MINDVISION_SDK_ROOT" \
  camera_serial:="$CAMERA_SERIAL" camera_publish_rate_hz:="${FIELD_CAMERA_HZ:-10.0}" \
  resolution_index:="$CAMERA_RESOLUTION_INDEX" \
  camera_qos_reliability:="$CAMERA_QOS" \
  tracker_config:="$TRACKER_CONFIG" tracker_reference:="$TRACKER_REFERENCE" \
  publish_debug_image:=false auto_save_failures:="$AUTO_SAVE_FAILURES" \
  enable_tracker:="$ENABLE_TRACKER" \
  failure_snapshot_cooldown_sec:="${FIELD_FAILURE_COOLDOWN_SEC:-2.0}" \
  max_failure_snapshots:="${FIELD_MAX_FAILURE_SNAPSHOTS:-100}" \
  roi_validation_recording_enabled:="$ROI_ENABLED" \
  roi_validation_max_duration_sec:="$RAW_BURST_DURATION_SEC" \
  geometry_provenance_sampling_enabled:=false record_mcap:=false record_raw_image:=false \
  start_mid360:=false start_sync:=false >"$VISUAL_LOG" 2>&1 &
VISUAL_PID=$!

for _ in $(seq 1 60); do
  [[ -f "$RUN_DIR/run_manifest.json" ]] && break
  if ! kill -0 "$VISUAL_PID" 2>/dev/null; then
    tail -120 "$VISUAL_LOG" >&2 || true
    exit 10
  fi
  sleep 0.5
done
if [[ ! -f "$RUN_DIR/run_manifest.json" ]]; then
  kill -INT -- "-$VISUAL_PID" 2>/dev/null || true
  echo "visual launch did not initialize $RUN_DIR" >&2
  tail -120 "$VISUAL_LOG" >&2 || true
  exit 11
fi
mkdir -p "$RUN_DIR/reports"
cp -- "$TIME_READINESS_REPORT" "$RUN_DIR/reports/time_readiness.json"

python3 "${SCRIPT_DIR}/vehicle_collection_manager.py" initialize \
  --data-root "$DATA_ROOT" --run-id "$RUN_ID" --camera-serial "$CAMERA_SERIAL" \
  --record-core "$RECORD_CORE" --record-mid360 "$RECORD_MID360" \
  --record-mapping "$RECORD_MAPPING" \
  --record-d405 "$RECORD_D405" --record-eddy "$RECORD_EDDY" \
  --record-front-video "$RECORD_FRONT_VIDEO" \
  --auto-save-failures "$AUTO_SAVE_FAILURES" --record-front-raw-burst "$RECORD_FRONT_RAW_BURST" \
  --tracker-enabled "$ENABLE_TRACKER" \
  --d405-enable-infra1 "$D405_ENABLE_INFRA1" \
  --d405-enable-infra2 "$D405_ENABLE_INFRA2" \
  --d405-depth-profile "$D405_DEPTH_PROFILE" \
  --d405-infra-profile "$D405_INFRA_PROFILE" \
  --d405-expected-fps "$D405_EXPECTED_FPS" \
  --d405-global-time-enabled "$D405_GLOBAL_TIME_ENABLED" \
  --d405-error-polling-enabled "$D405_ERROR_POLLING_ENABLED" \
  --eddy-device "$EDDY_DEVICE" --eddy-channel-mean-topic "$EDDY_CHANNEL_MEAN_TOPIC" \
  --collection-profile "$COLLECTION_PROFILE" \
  --ros-domain-id "${ROS_DOMAIN_ID:-0}" \
  --attach-to-existing-mapping "$ATTACH_TO_EXISTING_MAPPING" \
  --start-mapping "$START_MAPPING" \
  --raw-burst-mode "$RAW_BURST_MODE" \
  --freeze-file "$TRACKER_CONFIG" --freeze-file "$TRACKER_REFERENCE" \
  --freeze-file "$MID360_CONFIG" \
  --freeze-file "${ROOT_DIR}/src/inspection_bringup/launch/visual_field_validation.launch.py" \
  --freeze-file "${SCRIPT_DIR}/start_vehicle_collection.sh" \
  --freeze-file "${SCRIPT_DIR}/vehicle_collection_manager.py" \
  --freeze-file "${SCRIPT_DIR}/front_h264_recorder.py" \
  --freeze-file "${SCRIPT_DIR}/mapping_eddy_live_probe.py" \
  --freeze-file "${SCRIPT_DIR}/start_mapping_eddy_collection.sh" \
  --freeze-file "${SCRIPT_DIR}/stop_mapping_eddy_collection.sh" \
  --freeze-file "${SCRIPT_DIR}/validate_mapping_eddy_collection.py" \
  --freeze-file "${SCRIPT_DIR}/replay_mapping_eddy_collection.py" \
  --freeze-file "${SCRIPT_DIR}/rebuild_compact_trajectory.py" \
  --freeze-file "${ROOT_DIR}/src/eddy_driver/msg/EddyChannelMean.msg" \
  --freeze-file "${ROOT_DIR}/src/eddy_driver/src/eddy_node.cpp"
RUN_INITIALIZED=true
cp "$PREFLIGHT_REPORT" "$RUN_DIR/reports/preflight_report.json"
if [[ "$RECORD_MAPPING" == true ]]; then
  cp "$MAPPING_AUDIT" "$RUN_DIR/mapping_topic_inventory.json"
  cp "$MAPPING_AUDIT" "$RUN_DIR/reports/mapping_topic_inventory.json"
  cp "$MAPPING_QOS" "$RUN_DIR/frozen_configs/qos_overrides.yaml"
  cp "$MAPPING_PATH_GROWTH" "$RUN_DIR/reports/path_growth_analysis.json"
  cp "$MAPPING_BASELINE" "$RUN_DIR/reports/baseline_performance.json"
fi
# Moving an open log file keeps the launch process' file descriptor attached,
# so the run-local log continues to receive all later diagnostics and stats.
mv "$VISUAL_LOG" "$RUN_DIR/logs/visual_launch.log"
python3 "${SCRIPT_DIR}/vehicle_collection_manager.py" register --run-dir "$RUN_DIR" --name visual --pid "$VISUAL_PID" >/dev/null

register_pid() {
  python3 "${SCRIPT_DIR}/vehicle_collection_manager.py" register \
    --run-dir "$RUN_DIR" --name "$1" --pid "$2" >/dev/null
}

if [[ "$RECORD_MID360" == true ]]; then
  setsid ros2 launch inspection_bringup hardware_pipeline.launch.py \
    start_mindvision:=false start_mid360:=true start_eddy:=false start_sync:=false \
    record:=false publish_rough_extrinsics:=false mid360_config_path:="$MID360_CONFIG" \
    mid360_publish_freq:=10.0 >"$RUN_DIR/logs/mid360_launch.log" 2>&1 &
  MID360_PID=$!
  register_pid mid360_driver "$MID360_PID"
fi

if [[ "$RECORD_EDDY" == true ]]; then
  setsid ros2 run eddy_driver eddy_driver_node --ros-args \
    -p "device:=$EDDY_DEVICE" -p "baud:=$EDDY_BAUD" \
    -p "frame_id:=eddy_probe_link" -p "data_topic:=/eddy/data" \
    -p "channel_mean_topic:=$EDDY_CHANNEL_MEAN_TOPIC" \
    -p "status_topic:=/eddy/status" -p "command_topic:=/eddy/cmd" \
    >"$RUN_DIR/logs/eddy_launch.log" 2>&1 &
  EDDY_PID=$!
  register_pid eddy_driver "$EDDY_PID"
fi
if [[ "$RECORD_D405" == true ]]; then
  # These librealsense options must be present as parameter overrides before
  # the sensor starts. Setting them only after node startup leaves the global
  # timestamp/error polling threads alive and can trigger a long-run UVC stall.
  D405_STARTUP_CONFIG="$RUN_DIR/frozen_configs/d405_startup_parameters.yaml"
  {
    printf 'depth_module.global_time_enabled: %s\n' "$D405_GLOBAL_TIME_ENABLED"
    printf 'depth_module.error_polling_enabled: %s\n' "$D405_ERROR_POLLING_ENABLED"
  } >"$D405_STARTUP_CONFIG"
  # Keep an advisory lock for the complete launch lifetime.  --no-fork makes
  # the registered process remain the session leader, so the normal targeted
  # process-group shutdown still reaches the launch and all of its children.
  setsid flock --nonblock --no-fork "$D405_OWNER_LOCK" ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=d405 camera_name:=camera enable_color:=true enable_depth:=true \
    enable_infra1:="$D405_ENABLE_INFRA1" enable_infra2:="$D405_ENABLE_INFRA2" \
    depth_module.color_profile:="$D405_COLOR_PROFILE" \
    depth_module.depth_profile:="$D405_DEPTH_PROFILE" \
    depth_module.infra_profile:="$D405_INFRA_PROFILE" \
    config_file:="$D405_STARTUP_CONFIG" \
    diagnostics_period:=0.0 publish_tf:=true \
    >"$RUN_DIR/logs/d405_launch.log" 2>&1 &
  D405_PID=$!
  register_pid d405_driver "$D405_PID"
  D405_PARAMS_READY=false
  D405_PARAM_LOG="$RUN_DIR/logs/d405_parameter_config.log"
  for _ in $(seq 1 60); do
    if ros2 param get /d405/camera depth_module.global_time_enabled >/dev/null 2>&1 && \
      ros2 param get /d405/camera depth_module.error_polling_enabled >/dev/null 2>&1; then
      {
        ros2 param set /d405/camera depth_module.global_time_enabled "$D405_GLOBAL_TIME_ENABLED"
        ros2 param set /d405/camera depth_module.error_polling_enabled "$D405_ERROR_POLLING_ENABLED"
        ros2 param get /d405/camera depth_module.global_time_enabled
        ros2 param get /d405/camera depth_module.error_polling_enabled
      } >"$D405_PARAM_LOG" 2>&1 && D405_PARAMS_READY=true
      break
    fi
    if ! kill -0 "$D405_PID" 2>/dev/null; then break; fi
    sleep 0.25
  done
  if [[ "$D405_PARAMS_READY" != true ]]; then
    echo "D405 runtime parameters could not be configured" >&2
    cat "$D405_PARAM_LOG" >&2 2>/dev/null || true
    FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
    exit 18
  fi
fi
# Allow sensor drivers and the tracker to advertise their actual topics. The
# inventory, not guessed D405 topic names, controls every recorder.
sleep "${FIELD_DISCOVERY_DELAY_SEC:-12}"
python3 "${SCRIPT_DIR}/vehicle_collection_manager.py" inventory --run-dir "$RUN_DIR" \
  >"$RUN_DIR/logs/inventory_command.json"

if [[ "$RECORD_EDDY" == true ]]; then
  jq -e --arg topic "$EDDY_CHANNEL_MEAN_TOPIC" \
    '.record_groups.eddy[$topic] == "eddy_driver/msg/EddyChannelMean"' \
    "$RUN_DIR/topic_inventory.json" >/dev/null || {
      echo "Eddy channel-mean topic did not appear with the expected type" >&2
      FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
      exit 19
  }
fi
if [[ "$COLLECTION_PROFILE" == mapping_eddy && "$RECORD_EDDY" == true ]]; then
  jq -e '.record_groups.eddy["/eddy/data"] == "eddy_driver/msg/EddyFrame"
    and .record_groups.eddy["/eddy/status"] == "eddy_driver/msg/EddyStatus"
    and (.topic_info_verbose["/eddy/data"].stdout | contains("Reliability: BEST_EFFORT"))
    and (.topic_info_verbose["/eddy/channel_mean"].stdout | contains("Reliability: BEST_EFFORT"))
    and (.topic_info_verbose["/eddy/status"].stdout | contains("Reliability: RELIABLE"))' \
    "$RUN_DIR/topic_inventory.json" >/dev/null || {
      echo "Eddy data/status topic type or offered QoS is incompatible" >&2
      FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
      exit 21
    }
fi

if [[ "$RECORD_MAPPING" == true ]]; then
  jq -e '.mapping_sources["/cloud_registered"] == "sensor_msgs/msg/PointCloud2"
    and .mapping_sources["/path"] == "nav_msgs/msg/Path"
    and .mapping_sources["/aft_mapped_to_init"] == "nav_msgs/msg/Odometry"
    and .record_groups.mapping == {"/aft_mapped_to_init":"nav_msgs/msg/Odometry"}
    and .mapping_storage_policy == "compact_odometry_only"' \
    "$RUN_DIR/topic_inventory.json" >/dev/null || {
      echo "required mapping sources changed or compact odometry-only policy was not applied" >&2
      FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
      exit 20
    }
fi

if [[ "$RECORD_CORE" == true ]]; then
  CORE_TOPICS_READY=false
  if [[ "$ENABLE_TRACKER" == true ]]; then
    jq -e '.record_groups.core["/vision/front/seam_geometry"] and .record_groups.core["/vision/front/seam_state"] and .record_groups.core["/camera/front/camera_info"]' \
      "$RUN_DIR/topic_inventory.json" >/dev/null && CORE_TOPICS_READY=true
  else
    jq -e '.record_groups.core["/camera/front/camera_info"] and .record_groups.core["/diagnostics"]' \
      "$RUN_DIR/topic_inventory.json" >/dev/null && CORE_TOPICS_READY=true
  fi
  if [[ "$CORE_TOPICS_READY" != true ]]; then
      echo "required core topics did not appear" >&2
      FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
      exit 12
  fi
fi
if [[ "$RECORD_MID360" == true ]]; then
  jq -e '([.record_groups.mid360[] | select(test("PointCloud2"))] | length > 0) and ([.record_groups.mid360[] | select(test("Imu"))] | length > 0)' \
    "$RUN_DIR/topic_inventory.json" >/dev/null || {
      echo "MID-360 point cloud and IMU topics did not both appear" >&2
      FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
      exit 13
    }
fi
if [[ "$RECORD_D405" == true ]]; then
  jq -e '([.record_groups.d405[] | select(test("Image"))] | length > 0) and ([.record_groups.d405[] | select(test("CameraInfo"))] | length > 0)' \
    "$RUN_DIR/topic_inventory.json" >/dev/null || {
      echo "D405 image and CameraInfo topics did not appear" >&2
      FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
      exit 14
    }
fi

setsid python3 "${SCRIPT_DIR}/vehicle_collection_monitor.py" \
  --run-dir "$RUN_DIR" --topic-inventory "$RUN_DIR/topic_inventory.json" \
  --warning-free-gb "${FIELD_DISK_WARNING_GB:-100}" \
  --hard-stop-free-gb "${FIELD_DISK_HARD_STOP_GB:-50}" \
  >"$RUN_DIR/logs/monitor.log" 2>&1 &
MONITOR_PID=$!
register_pid monitor "$MONITOR_PID"

STATUS_READY=false
for _ in $(seq 1 40); do
  if [[ -s "$RUN_DIR/collection_status.json" ]] && \
    jq -e '.schema_version == "field_w5_collection_status_v1"' \
      "$RUN_DIR/collection_status.json" >/dev/null 2>&1; then
    STATUS_READY=true
    break
  fi
  sleep 0.25
done
if [[ "$STATUS_READY" != true ]]; then
  echo "runtime collection monitor did not produce status" >&2
  FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
  exit 15
fi
python3 "${SCRIPT_DIR}/vehicle_collection_manager.py" add-runtime-status --run-dir "$RUN_DIR" \
  >"$RUN_DIR/logs/runtime_status_inventory_command.json"

# Advertised topics alone are not evidence that a sensor is delivering data.
# Require observed camera, D405 and MID-360 messages before starting writers.
SENSOR_DATA_READY=false
for _ in $(seq 1 40); do
  CAMERA_OK=true
  MID360_OK=true
  D405_OK=true
  EDDY_OK=true
  MAPPING_OK=true
  if [[ "$RECORD_CORE" == true ]]; then
    jq -e '.topics["/camera/front/camera_info"].count > 0' \
      "$RUN_DIR/collection_status.json" >/dev/null 2>&1 || CAMERA_OK=false
  fi
  if [[ "$RECORD_MID360" == true ]]; then
    jq -e '([.topics | to_entries[] | select(.key | startswith("/livox/")) | select(.value.type | contains("PointCloud2")) | .value.count] | any(. > 0)) and ([.topics | to_entries[] | select(.key | startswith("/livox/")) | select(.value.type | contains("Imu")) | .value.count] | any(. > 0))' \
      "$RUN_DIR/collection_status.json" >/dev/null 2>&1 || MID360_OK=false
  fi
  if [[ "$RECORD_D405" == true ]]; then
    jq -e '([.topics | to_entries[] | select(.key | startswith("/d405/")) | select(.value.type | contains("CameraInfo"))] | length > 0) and ([.topics | to_entries[] | select(.key | startswith("/d405/")) | select(.value.type | contains("CameraInfo")) | .value.count] | all(. > 0))' \
      "$RUN_DIR/collection_status.json" >/dev/null 2>&1 || D405_OK=false
  fi
  if [[ "$RECORD_EDDY" == true ]]; then
    kill -0 "$EDDY_PID" 2>/dev/null || EDDY_OK=false
    # The monitor deliberately subscribes to the raw frame instead of
    # duplicating both high-rate Eddy streams.  The inventory gate above proves
    # that channel_mean is advertised with the expected type, while live raw
    # frames prove the shared publisher loop is delivering both messages.
    jq -e '.topics["/eddy/data"].count > 0 and .topics["/eddy/status"].count > 0' \
      "$RUN_DIR/collection_status.json" >/dev/null 2>&1 || EDDY_OK=false
  fi
  if [[ "$RECORD_MAPPING" == true ]]; then
    jq -e '.topics["/cloud_registered"].count > 0 and .topics["/path"].count > 0
      and .topics["/aft_mapped_to_init"].count > 0' \
      "$RUN_DIR/collection_status.json" >/dev/null 2>&1 || MAPPING_OK=false
  fi
  if [[ "$CAMERA_OK" == true && "$MID360_OK" == true && "$D405_OK" == true && "$EDDY_OK" == true && "$MAPPING_OK" == true ]]; then
    SENSOR_DATA_READY=true
    break
  fi
  sleep 0.5
done
if [[ "$SENSOR_DATA_READY" != true ]]; then
  echo "one or more enabled sensors advertised topics but delivered no monitored messages" >&2
  FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
  exit 16
fi

if [[ "$RECORD_FRONT_VIDEO" == true ]]; then
  # Start the subscriber/muxer before the encoder so it receives the initial
  # SPS/PPS/IDR access units required for a self-contained first segment.
  setsid python3 "${SCRIPT_DIR}/front_h264_recorder.py" \
    --topic /camera/front/image_h264 --output "$RUN_DIR/video/front" \
    --status-file "$RUN_DIR/reports/video_runtime_status.json" \
    --fps "${FIELD_CAMERA_HZ:-10.0}" --segment-sec "${FIELD_VIDEO_SEGMENT_SEC:-300}" \
    --queue-size "${FIELD_VIDEO_QUEUE_SIZE:-16}" \
    >"$RUN_DIR/logs/front_video.log" 2>&1 &
  VIDEO_PID=$!
  register_pid front_video "$VIDEO_PID"
  # Allow DDS matching to complete before the encoder emits its initial SPS,
  # PPS and IDR units. Missing those units makes the first GOP undecodable.
  sleep 2
  setsid ros2 launch hobot_codec hobot_codec_encode.launch.py \
    codec_channel:=1 codec_in_mode:=ros codec_in_format:=bgr8 codec_out_mode:=ros \
    codec_out_format:=h264 codec_sub_topic:=/camera/front/image_raw \
    codec_pub_topic:=/camera/front/image_h264 codec_input_framerate:=10 \
    codec_output_framerate:=10 log_level:=warn \
    >"$RUN_DIR/logs/front_video_encoder.log" 2>&1 &
  VIDEO_ENCODER_PID=$!
  register_pid front_video_encoder "$VIDEO_ENCODER_PID"
  VIDEO_DATA_READY=false
  for _ in $(seq 1 40); do
    if [[ -s "$RUN_DIR/reports/video_runtime_status.json" ]] && \
      jq -e '.encoded_frames > 0 and .write_error_count == 0' \
        "$RUN_DIR/reports/video_runtime_status.json" >/dev/null 2>&1; then
      VIDEO_DATA_READY=true
      break
    fi
    if ! kill -0 "$VIDEO_PID" 2>/dev/null || ! kill -0 "$VIDEO_ENCODER_PID" 2>/dev/null; then
      break
    fi
    sleep 0.5
  done
  if [[ "$VIDEO_DATA_READY" != true ]]; then
    echo "tested hobot_codec path produced no H.264 access unit" >&2
    FIELD_SKIP_FINALIZE=1 "${SCRIPT_DIR}/stop_vehicle_collection.sh" "$RUN_ID" || true
    exit 17
  fi
fi

start_bag() {
  local name="$1" duration="$2" cache="$3" topic_file="$4"
  mapfile -t topics < <(sed '/^[[:space:]]*$/d' "$topic_file")
  if ((${#topics[@]} == 0)); then
    echo "refusing to start empty $name recorder" >&2
    return 1
  fi
  local qos_config="$ROOT_DIR/src/inspection_bringup/config/qos_overrides.yaml"
  if [[ -f "$RUN_DIR/frozen_configs/qos_overrides.yaml" ]]; then
    qos_config="$RUN_DIR/frozen_configs/qos_overrides.yaml"
  fi
  setsid ros2 bag record --storage mcap --output "$RUN_DIR/.partial/$name" \
    --max-bag-duration "$duration" --max-cache-size "$cache" \
    --storage-config-file "$ROOT_DIR/src/inspection_bringup/config/mcap_writer_options.yaml" \
    --qos-profile-overrides-path "$qos_config" \
    "${topics[@]}" >"$RUN_DIR/logs/${name}_recorder.log" 2>&1 &
  local pid=$!
  register_pid "${name}_recorder" "$pid"
}

if [[ "$RECORD_CORE" == true ]]; then
  start_bag core "${FIELD_CORE_SEGMENT_SEC:-300}" 268435456 "$RUN_DIR/logs/core_topics.txt"
fi
if [[ "$RECORD_MAPPING" == true ]]; then
  start_bag mapping "${FIELD_MAPPING_SEGMENT_SEC:-300}" 536870912 \
    "$RUN_DIR/logs/mapping_topics.txt"
fi
if [[ "$RECORD_MID360" == true ]]; then
  { printf '%s\n' /diagnostics; cat "$RUN_DIR/logs/mid360_topics.txt"; } | awk '!seen[$0]++' >"$RUN_DIR/logs/mid360_topics.record.txt"
  start_bag mid360 "${FIELD_MID360_SEGMENT_SEC:-120}" 536870912 "$RUN_DIR/logs/mid360_topics.record.txt"
fi
if [[ "$RECORD_D405" == true ]]; then
  { printf '%s\n' /diagnostics /tf /tf_static; cat "$RUN_DIR/logs/d405_topics.txt"; } | awk '!seen[$0]++' >"$RUN_DIR/logs/d405_topics.record.txt"
  start_bag d405 "${FIELD_D405_SEGMENT_SEC:-60}" 1073741824 "$RUN_DIR/logs/d405_topics.record.txt"
fi
if [[ "$RECORD_EDDY" == true ]]; then
  start_bag eddy "${FIELD_EDDY_SEGMENT_SEC:-300}" 33554432 \
    "$RUN_DIR/logs/eddy_topics.txt"
fi

if [[ "$RECORD_FRONT_RAW_BURST" == true && "$RAW_BURST_MODE" == full ]]; then
  setsid timeout --signal=INT --kill-after=15 "${RAW_BURST_DURATION_SEC}s" \
    ros2 bag record --storage mcap --output "$RUN_DIR/.partial/raw_burst" \
    --max-bag-duration 10 --max-cache-size 1073741824 \
    --storage-config-file "$ROOT_DIR/src/inspection_bringup/config/mcap_writer_options.yaml" \
    --qos-profile-overrides-path "$ROOT_DIR/src/inspection_bringup/config/qos_overrides.yaml" \
    /camera/front/image_raw /camera/front/camera_info \
    >"$RUN_DIR/logs/raw_burst_recorder.log" 2>&1 &
  RAW_PID=$!
  register_pid raw_burst_recorder "$RAW_PID"
fi

setsid python3 "${SCRIPT_DIR}/vehicle_collection_disk_guard.py" \
  --run-dir "$RUN_DIR" --run-id "$RUN_ID" --stop-script "$SCRIPT_DIR/stop_vehicle_collection.sh" \
  >"$RUN_DIR/logs/disk_guard.log" 2>&1 &
GUARD_PID=$!
register_pid disk_guard "$GUARD_PID"

trap - ERR

echo "run_id=$RUN_ID"
echo "run_dir=$RUN_DIR"
echo "status=${SCRIPT_DIR}/status_vehicle_collection.sh $RUN_ID"
echo "stop=${SCRIPT_DIR}/stop_vehicle_collection.sh $RUN_ID"
