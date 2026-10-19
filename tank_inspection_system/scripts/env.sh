#!/usr/bin/env bash
# Source from wrappers or an interactive shell. Every project path is derived
# from this file so a copied checkout never depends on the directory where an
# older venv or colcon install was generated.
TANK_SYSTEM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EAST_ELEAIR_ROOT="${EAST_ELEAIR_ROOT:-$(cd "$TANK_SYSTEM_ROOT/.." && pwd)}"
TANK_RUNTIME_ROOT="${TANK_RUNTIME_ROOT:-$TANK_SYSTEM_ROOT/.runtime}"
TANK_CORE_ENV="${TANK_CORE_ENV:-$TANK_RUNTIME_ROOT/core_env}"
TANK_INSTALL_ROOT="${TANK_INSTALL_ROOT:-$TANK_RUNTIME_ROOT/install}"
export TANK_SYSTEM_ROOT EAST_ELEAIR_ROOT TANK_RUNTIME_ROOT TANK_CORE_ENV TANK_INSTALL_ROOT
export FIELD_PHYSIC_ROOT="${FIELD_PHYSIC_ROOT:-$EAST_ELEAIR_ROOT/physic_bev}"
export VISION_ROOT="${VISION_ROOT:-$EAST_ELEAIR_ROOT/tank_vision_inspection}"
export FIELD_DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"

# Python venv activation scripts contain an absolute VIRTUAL_ENV and are not
# relocatable. Activate the freshly generated environment without sourcing its
# generated activate script. Commands are invoked as "python -m ...", so stale
# shebangs in a copied legacy venv cannot be selected accidentally.
if [[ -x "$TANK_CORE_ENV/bin/python" ]]; then
  if [[ -n "${VIRTUAL_ENV:-}" && -n "${PATH:-}" ]]; then
    PATH="$(python3 - "$PATH" "$VIRTUAL_ENV/bin" <<'PY'
import sys
print(":".join(part for part in sys.argv[1].split(":") if part != sys.argv[2]))
PY
)"
  fi
  export VIRTUAL_ENV="$TANK_CORE_ENV"
  export PATH="$TANK_CORE_ENV/bin:$PATH"
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/tros/humble/setup.bash && -f /opt/tros/humble/local_setup.bash ]]; then
  source /opt/tros/humble/setup.bash
fi
for prefix in \
  "$TANK_INSTALL_ROOT/cpp" \
  "$TANK_INSTALL_ROOT/hardware" \
  "$TANK_INSTALL_ROOT/physic" \
  "$TANK_INSTALL_ROOT/vision" \
  "$TANK_INSTALL_ROOT/wireless"; do
  if [[ -f "$prefix/setup.bash" ]]; then source "$prefix/setup.bash"; fi
done
set -u
export FIELD_PHYSIC_INSTALL="${FIELD_PHYSIC_INSTALL:-$TANK_INSTALL_ROOT/physic}"
export FIELD_TANK_INSTALL="${FIELD_TANK_INSTALL:-$TANK_INSTALL_ROOT/vision}"
export FIELD_USE_INSTALLED_CODE="${FIELD_USE_INSTALLED_CODE:-1}"
export FIELD_RRD_PYTHON="${FIELD_RRD_PYTHON:-$VISION_ROOT/.venv_rerun/bin/python}"

# Prefer the workspace-carried MindVision 2.1.0.50 SDK without modifying the
# system copy. The ROS driver additionally loads this absolute library path and
# reports it in diagnostics, so its Python binding and native library cannot
# silently diverge.
if [[ -z "${MINDVISION_SDK_ROOT:-}" && -d "$EAST_ELEAIR_ROOT/MindVisionSDK_v2.1.0.50/sdk" ]]; then
  export MINDVISION_SDK_ROOT="$EAST_ELEAIR_ROOT/MindVisionSDK_v2.1.0.50/sdk"
fi
if [[ -n "${MINDVISION_SDK_ROOT:-}" && -f "$MINDVISION_SDK_ROOT/lib/arm64/libMVSDK.so" ]]; then
  export LD_LIBRARY_PATH="$MINDVISION_SDK_ROOT/lib/arm64:${LD_LIBRARY_PATH:-}"
fi

# Local, hash-pinned dependencies; no ROS/SDK system upgrade or network at runtime.
TANK_OFFLINE_PREFIX="$TANK_SYSTEM_ROOT/offline/sysroot"
export TANK_OFFLINE_PREFIX
if [[ -d "$TANK_OFFLINE_PREFIX/opt/ros/humble/share/ament_index" ]]; then
  export AMENT_PREFIX_PATH="$TANK_OFFLINE_PREFIX/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$TANK_OFFLINE_PREFIX/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
fi
# A single generated wireless overlay is authoritative for a deployment.
export FIELD_WIRELESS_INSTALL="${FIELD_WIRELESS_INSTALL:-$TANK_INSTALL_ROOT/wireless}"

# Prefer the locally rebuilt Livox SDK whose embedded spdlog symbols are bound
# internally. The system /usr/local copy remains untouched and available as a
# fallback before the first build.
FIELD_LIVOX_SDK_LIB_DIR="${FIELD_LIVOX_SDK_LIB_DIR:-$TANK_RUNTIME_ROOT/build/livox_sdk/sdk_core}"
export FIELD_LIVOX_SDK_LIB_DIR
if [[ -f "$FIELD_LIVOX_SDK_LIB_DIR/liblivox_lidar_sdk_shared.so" ]]; then
  export LD_LIBRARY_PATH="$FIELD_LIVOX_SDK_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

# Existing RealSense component links SDK SONAME 2.58. Keep its matching library
# visible after all overlays, without installing/upgrading the system SDK.
export FIELD_REALSENSE_LIB_DIR="${FIELD_REALSENSE_LIB_DIR:-$EAST_ELEAIR_ROOT/third_party/librealsense/build/release}"
if [[ -f "$FIELD_REALSENSE_LIB_DIR/librealsense2.so.2.58" ]]; then
  export LD_LIBRARY_PATH="$FIELD_REALSENSE_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi
