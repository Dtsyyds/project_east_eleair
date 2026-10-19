#!/usr/bin/env bash
set -Eeuo pipefail
SYSTEM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SYSTEM/.." && pwd)"
ACTION="${1:---help}"
if [[ "$ACTION" == --help ]]; then
  echo 'usage: rollback.sh {collect|stop|replay|validate|handoff} ORIGINAL_ARGUMENTS...'
  echo 'Uses isolated pre-task entry copies, baseline core wheel and original ROS installs.'
  exit 0
fi
shift
case "$ACTION" in
  collect) ENTRY=start_vehicle_collection.sh;;
  stop) ENTRY=stop_vehicle_collection.sh;;
  replay) ENTRY=replay_vehicle_collection.sh;;
  validate) ENTRY=validate_vehicle_collection.py;;
  handoff) ENTRY=build_role_handoff.py;;
  *) echo "unknown action: $ACTION" >&2;exit 2;;
esac
source "$SYSTEM/core_env_baseline/bin/activate"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/physic_bev/install/setup.bash"
source "$ROOT/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash"
set -u
export FIELD_PHYSIC_ROOT="$SYSTEM/rollback/physic_bev"
export VISION_ROOT="$SYSTEM/rollback/tank_vision_inspection"
export FIELD_PHYSIC_INSTALL="$ROOT/physic_bev/install"
export FIELD_TANK_INSTALL="$ROOT/tank_vision_inspection/ros2_ws/install_s100p_humble"
if [[ "$ENTRY" == *.py ]]; then
  exec python3 "$FIELD_PHYSIC_ROOT/scripts/$ENTRY" "$@"
fi
exec bash "$FIELD_PHYSIC_ROOT/scripts/$ENTRY" "$@"
