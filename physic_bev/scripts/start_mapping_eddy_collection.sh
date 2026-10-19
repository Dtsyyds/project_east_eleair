#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/tros/humble/setup.bash ]]; then source /opt/tros/humble/setup.bash; fi
source "$ROOT_DIR/install/setup.bash"
set -u

python3 "$SCRIPT_DIR/vehicle_collection_manager.py" assert-no-active --data-root "$DATA_ROOT" >/dev/null || {
  echo "an active formal collection run already exists" >&2; exit 2;
}
if ps -eo args= | grep -E '[r]eplay_mapping_eddy_collection(\.py|\.sh)' >/dev/null; then
  echo "mapping/Eddy replay is active; collection startup refused" >&2; exit 2
fi
if ros2 node list 2>/dev/null | grep -Eq '^/camera/front$|stable_seam_direct'; then
  echo "a MindVision camera owner already exists; collection startup refused" >&2; exit 2
fi
if ps -eo args= | grep -E '[m]indvision_camera_node|[s]table_seam_direct' >/dev/null; then
  echo "a MindVision SDK owner process already exists; collection startup refused" >&2; exit 2
fi

exec "$SCRIPT_DIR/start_vehicle_collection.sh" "$@" \
  collection_profile:=mapping_eddy record_mapping:=true \
  record_mid360:=false record_d405:=false enable_tracker:=false \
  provenance_enabled:=false record_front_raw_burst:=false auto_save_failures:=false
