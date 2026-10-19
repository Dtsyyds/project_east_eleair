#!/usr/bin/env bash
set -Eeuo pipefail
SYSTEM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(cd "$SYSTEM/.." && pwd)"
RUNTIME="${TANK_RUNTIME_ROOT:-$SYSTEM/.runtime}"
BUILD_ROOT="${TANK_BUILD_ROOT:-$RUNTIME/build}"
INSTALL_ROOT="${TANK_INSTALL_ROOT:-$RUNTIME/install}"
CORE_ENV="${TANK_CORE_ENV:-$RUNTIME/core_env}"
mkdir -p "$BUILD_ROOT" "$INSTALL_ROOT" "$RUNTIME/log"
# All new dependencies come from pinned local archives; never download during build.
python3 - "$SYSTEM" <<'OFFLINE'
from pathlib import Path
import json,hashlib,subprocess,sys
root=Path(sys.argv[1])/'offline'
manifest=json.loads((root/'dependency_manifest.json').read_text())
for record in manifest['archives']:
    path=root/record['path']
    if not path.resolve().is_relative_to(root.resolve()):raise ValueError('offline archive path escape')
    if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:raise ValueError('offline archive SHA mismatch: '+str(path))
    if path.suffix=='.deb':subprocess.run(['dpkg-deb','-x',str(path),str(root/'sysroot')],check=True)
OFFLINE
if [[ ! -x "$CORE_ENV/bin/python" ]]; then
  python3 -m venv --system-site-packages "$CORE_ENV"
fi
python3 "$SYSTEM/scripts/install_core.py" --venv "$CORE_ENV"
export TANK_RUNTIME_ROOT="$RUNTIME" TANK_BUILD_ROOT="$BUILD_ROOT" TANK_INSTALL_ROOT="$INSTALL_ROOT" TANK_CORE_ENV="$CORE_ENV"
source "$SYSTEM/scripts/env.sh"

# Build the workspace-carried Livox SDK with internal symbol binding. This
# avoids an exit-time ABI collision between its embedded spdlog and ROS 2's
# system libspdlog, without replacing /usr/local/lib or any vendor binary.
LIVOX_SDK_SOURCE="$ROOT/third_party/Livox-SDK2"
LIVOX_SDK_BUILD="$BUILD_ROOT/livox_sdk"
cmake -S "$LIVOX_SDK_SOURCE" -B "$LIVOX_SDK_BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DLIVOX_BUILD_SAMPLES=OFF
cmake --build "$LIVOX_SDK_BUILD" --target livox_lidar_sdk_shared -j"$(nproc)"
LIVOX_SDK_LIBRARY="$LIVOX_SDK_BUILD/sdk_core/liblivox_lidar_sdk_shared.so"
WIRELESS_BUILD_BASE="${FIELD_WIRELESS_BUILD:-$BUILD_ROOT/wireless}"
WIRELESS_LOG_BASE="${FIELD_WIRELESS_LOG:-$RUNTIME/log/wireless}"
if [[ "${1:-}" == --wireless-only ]]; then
  colcon --log-base "$WIRELESS_LOG_BASE" build --base-paths "$ROOT/physic_bev/src/system_websocket/agv_protocol" "$ROOT/physic_bev/src/system_websocket/agv_bridge" --build-base "$WIRELESS_BUILD_BASE" --install-base "$FIELD_WIRELESS_INSTALL" --packages-up-to agv_bridge --cmake-args -DBUILD_TESTING=ON
  exit
fi
colcon --log-base "$RUNTIME/log/cpp" build --base-paths "$ROOT/physic_bev/src/inspection_interfaces" "$ROOT/physic_bev/src/eddy_driver" "$ROOT/physic_bev/src/inspection_sync" "$ROOT/physic_bev/src/inspection_livox_adapter" --build-base "$BUILD_ROOT/cpp" --install-base "$INSTALL_ROOT/cpp" --packages-select inspection_interfaces eddy_driver inspection_sync inspection_livox_adapter
set +u
source "$INSTALL_ROOT/cpp/setup.bash"
set -u
colcon --log-base "$RUNTIME/log/hardware" build \
  --base-paths "$ROOT/physic_bev/src/livox_ros_driver2" "$ROOT/physic_bev/src/realsense-ros" \
  --build-base "$BUILD_ROOT/hardware" --install-base "$INSTALL_ROOT/hardware" \
  --packages-select livox_ros_driver2 realsense2_camera_msgs realsense2_camera \
  --cmake-args \
    -DLIVOX_LIDAR_SDK_LIBRARY="$LIVOX_SDK_LIBRARY" \
    -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$LIVOX_SDK_SOURCE/include"
set +u
source "$INSTALL_ROOT/hardware/setup.bash"
set -u
PHYSIC_PACKAGES=(inspection_tools mindvision_camera_driver inspection_bringup)
if [[ -d "$ROOT/physic_bev/src/fisheye_camera_driver" ]]; then PHYSIC_PACKAGES+=(fisheye_camera_driver); fi
colcon --log-base "$RUNTIME/log/physic" build --base-paths "$ROOT/physic_bev/src/inspection_tools" "$ROOT/physic_bev/src/mindvision_camera_driver" "$ROOT/physic_bev/src/inspection_bringup" "$ROOT/physic_bev/src/fisheye_camera_driver" --build-base "$BUILD_ROOT/physic" --install-base "$INSTALL_ROOT/physic" --packages-select "${PHYSIC_PACKAGES[@]}"
set +u
source "$INSTALL_ROOT/physic/setup.bash"
set -u
colcon --log-base "$RUNTIME/log/vision" build --base-paths "$ROOT/tank_vision_inspection/ros2_ws/src/tank_vision_interfaces" "$ROOT/tank_vision_inspection/ros2_ws/src/tank_vision_ros" "$ROOT/tank_vision_inspection/ros2_ws/src/inspection_rerun" --build-base "$BUILD_ROOT/vision" --install-base "$INSTALL_ROOT/vision" --packages-select tank_vision_interfaces tank_vision_ros inspection_rerun

colcon --log-base "$WIRELESS_LOG_BASE" build --base-paths "$ROOT/physic_bev/src/system_websocket/agv_protocol" "$ROOT/physic_bev/src/system_websocket/agv_bridge" --build-base "$WIRELESS_BUILD_BASE" --install-base "$FIELD_WIRELESS_INSTALL" --packages-up-to agv_bridge --cmake-args -DBUILD_TESTING=ON
