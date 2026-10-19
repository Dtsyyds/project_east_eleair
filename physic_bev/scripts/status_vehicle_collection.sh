#!/usr/bin/env bash
set -euo pipefail

DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${1:-${FIELD_RUN_ID:-}}"
MODE="${2:-}"
if [[ -z "$RUN_ID" ]]; then
  echo "usage: status_vehicle_collection.sh RUN_ID [--once]" >&2
  exit 2
fi
STATUS="${DATA_ROOT}/runs/${RUN_ID}/collection_status.json"

render() {
  if [[ ! -f "$STATUS" ]]; then
    echo "status not available yet: $STATUS" >&2
    return 1
  fi
  jq -r '
    "updated=" + (.updated_at // "n/a"),
    "disk_free_GB=" + ((.disk.free_bytes / 1000000000) | tostring)
      + " remaining_h=" + (if .disk.estimated_remaining_sec then (.disk.estimated_remaining_sec / 3600 | tostring) else "n/a" end),
    "camera_input_hz=" + ((.topics["/camera/front/camera_info"].receive_hz // 0) | tostring)
      + " video_hz=" + ((.video.encoded_hz // 0) | tostring)
      + " tracker_hz=" + ((.topics["/vision/front/seam_state"].receive_hz // 0) | tostring),
    "mid360_lidar_hz=" + ([.topics | to_entries[] | select((.key|startswith("/livox/")) and (.value.type|contains("PointCloud2"))) | .value.receive_hz] | max // 0 | tostring)
      + " mid360_imu_hz=" + ([.topics | to_entries[] | select((.key|startswith("/livox/")) and (.value.type|contains("Imu"))) | .value.receive_hz] | max // 0 | tostring),
    "d405=" + ([.topics | to_entries[] | select((.key|startswith("/d405/")) and (.value.type|contains("CameraInfo"))) | (.key + "=" + ((.value.receive_hz // 0)|tostring))] | join(" ")),
    "eddy_hz=" + ([.topics | to_entries[] | select(.value.type == "eddy_driver/msg/EddyFrame" or .value.type == "eddy_driver/msg/EddyChannelMean") | .value.receive_hz] | max // 0 | tostring),
    "writers_MBps=" + ([.writers | to_entries[] | (.key + "=" + (.value.write_MBps|tostring))] | join(" ")),
    "video_queue=" + ((.video.queue_depth // 0)|tostring)
      + " dropped=" + ((.video.queue_drop_count // 0)|tostring)
      + " write_errors=" + ((.video.write_error_count // 0)|tostring)
      + " segment=" + ((.video.current_segment // 0)|tostring),
    "cpu=" + ((.resources.cpu_percent.cpu // 0)|tostring)
      + "% mem_available_GB=" + ((.resources.memory.MemAvailable / 1000000000)|tostring)
      + " snapshots=" + (.failure_snapshot_count|tostring)
  ' "$STATUS"
}

if [[ "$MODE" == "--once" || "${FIELD_STATUS_ONCE:-}" == "1" ]]; then
  render
  exit
fi
while true; do
  clear 2>/dev/null || true
  render || true
  sleep "${FIELD_STATUS_INTERVAL_SEC:-2}"
done
