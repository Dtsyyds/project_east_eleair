#!/usr/bin/env bash
set -euo pipefail

DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
RUN_ID="${1:-${FIELD_RUN_ID:-}}"
[[ -n "$RUN_ID" ]] || { echo "usage: status_mapping_eddy_collection.sh RUN_ID" >&2; exit 2; }
STATUS="$DATA_ROOT/runs/$RUN_ID/collection_status.json"
[[ -f "$STATUS" ]] || { echo "status not available: $STATUS" >&2; exit 1; }
jq -r '
  "updated=" + (.updated_at // "n/a"),
  "camera_info_hz=" + ((.topics["/camera/front/camera_info"].receive_hz // 0)|tostring)
    + " h264_input_hz=" + ((.video.input_hz // 0)|tostring)
    + " h264_written_hz=" + ((.video.encoded_hz // 0)|tostring),
  "cloud_registered_hz=" + ((.topics["/cloud_registered"].receive_hz // 0)|tostring)
    + " age_s=" + ((.topics["/cloud_registered"].last_message_age_sec // -1)|tostring)
    + " max_gap_ms=" + ((.topics["/cloud_registered"].interval_ms.max // -1)|tostring),
  "path_hz=" + ((.topics["/path"].receive_hz // 0)|tostring)
    + " poses=" + ((.topics["/path"].path_pose_count // 0)|tostring)
    + " avg_bytes=" + ((.topics["/path"].average_message_bytes // 0)|tostring)
    + " max_bytes=" + ((.topics["/path"].max_message_bytes // 0)|tostring)
    + " max_gap_ms=" + ((.topics["/path"].interval_ms.max // -1)|tostring),
  "aft_mapped_hz=" + ((.topics["/aft_mapped_to_init"].receive_hz // 0)|tostring)
    + " age_s=" + ((.topics["/aft_mapped_to_init"].last_message_age_sec // -1)|tostring)
    + " max_gap_ms=" + ((.topics["/aft_mapped_to_init"].interval_ms.max // -1)|tostring),
  "eddy_hz=" + ((.topics["/eddy/data"].receive_hz // .topics["/eddy/channel_mean"].receive_hz // 0)|tostring)
    + " age_s=" + ((.topics["/eddy/data"].last_message_age_sec // .topics["/eddy/channel_mean"].last_message_age_sec // -1)|tostring)
    + " max_gap_ms=" + ((.topics["/eddy/data"].interval_ms.max // .topics["/eddy/channel_mean"].interval_ms.max // -1)|tostring),
  "eddy_status_hz=" + ((.topics["/eddy/status"].receive_hz // 0)|tostring),
  "eddy_device=" + ((.topics["/eddy/status"].device_status // {})|tostring),
  "writers=" + ([.writers|to_entries[]|(.key+":"+(.value.bytes|tostring)+"B,"+(.value.write_MBps|tostring)+"MB/s,segment="+(.value.current_segment|tostring))]|join(" ")),
  "video_queue=" + ((.video.queue_depth // 0)|tostring)
    + " drops=" + ((.video.queue_drop_count // 0)|tostring)
    + " write_errors=" + ((.video.write_error_count // 0)|tostring)
    + " segment=" + ((.video.current_segment // 0)|tostring),
  "disk_free_GB=" + ((.disk.free_bytes/1000000000)|tostring)
    + " cpu=" + ((.resources.cpu_percent.cpu // 0)|tostring)
    + "% mem_available_GB=" + ((.resources.memory.MemAvailable/1000000000)|tostring),
  "temperatures_C=" + ([.resources.temperatures[]?.celsius|tostring]|join(",")),
  "processes=" + ([.processes|to_entries[]|(.key+"="+(.value.alive|tostring)+",RSS="+(.value.VmRSS//"n/a"))]|join(" "))
' "$STATUS"
