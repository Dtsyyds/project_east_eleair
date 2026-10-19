#!/usr/bin/env bash
set -Eeuo pipefail
SYSTEM_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SYSTEM_WS_ROOT/.." && pwd)"
source "$PROJECT_ROOT/tank_inspection_system/scripts/env.sh"
set +u
source "$PROJECT_ROOT/physic_bev/install_lifecycle/setup.bash"
set -u
SYSTEM_WS_RUNTIME="$PROJECT_ROOT/tank_inspection_system/.runtime"
SYSTEM_WS_BRIDGE="$PROJECT_ROOT/physic_bev/src/system_websocket"
SYSTEM_WS_STAMP="$SYSTEM_WS_RUNTIME/system_websocket_build.stamp"
if [[ ! -f "$SYSTEM_WS_STAMP" ]] || [[ -n "$(find "$SYSTEM_WS_BRIDGE/agv_bridge/src" "$SYSTEM_WS_BRIDGE/agv_bridge/include" -type f -newer "$SYSTEM_WS_STAMP" -print -quit 2>/dev/null)" ]]; then
  colcon --log-base "$SYSTEM_WS_RUNTIME/log/system_websocket" build \
    --base-paths "$SYSTEM_WS_BRIDGE/agv_protocol" "$SYSTEM_WS_BRIDGE/agv_bridge" \
    --build-base "$SYSTEM_WS_RUNTIME/build/wireless" \
    --install-base "$TANK_INSTALL_ROOT/wireless" \
    --packages-up-to agv_bridge --cmake-args -DBUILD_TESTING=ON
  touch "$SYSTEM_WS_STAMP"
fi
set +u
source "$TANK_INSTALL_ROOT/wireless/setup.bash"
set -u
export FASTRTPS_DEFAULT_PROFILES_FILE="$SYSTEM_WS_ROOT/config/fastdds.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python "$SYSTEM_WS_ROOT/src/run_system.py"
