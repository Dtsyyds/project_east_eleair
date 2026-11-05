#!/usr/bin/env bash
# Passive A subscriber only: no camera launch, recorder or chassis commands.
set -euo pipefail
tracking_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$tracking_root/deploy/env.sh" s100p
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$tracking_root/config/communication/fastdds_d405_127.xml"
exec "$tracking_root/.runtime/s100p/d405_prototype_venv/bin/python" \
  "$tracking_root/tools/vision/run_d405_node.py" \
  --config "$tracking_root/config/d405/round1_a.yaml" --stream color --depth-scale-m 0.001 \
  --tracking-error-config "$tracking_root/config/d405/weld_tracking_error.yaml" "$@"
