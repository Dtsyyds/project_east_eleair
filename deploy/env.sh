#!/usr/bin/env bash
# Source this file. Project roots are derived from this checkout only.
if [[ "${BASH_SOURCE[0]}" == "$0" && "${1:-}" == --help ]]; then
  echo 'Usage: source /path/to/project/deploy/env.sh [s100p|x86|rk3588]'; exit 0
fi
for _inspection_var in PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH VIRTUAL_ENV PATH TANK_RUNTIME_ROOT TANK_INSTALL_ROOT TANK_BUILD_ROOT TANK_CORE_ENV; do
  if [[ "${!_inspection_var:-}" == *"/east_eleair_plus/"* ]]; then
    echo "Refusing inherited old checkout in $_inspection_var; use a clean shell" >&2
    return 2 2>/dev/null || exit 2
  fi
done
unset _inspection_var
export EAST_ELEAIR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export INSPECTION_PLATFORM="${INSPECTION_PLATFORM:-s100p}"
case "${1:-}" in s100p|x86|rk3588) export INSPECTION_PLATFORM="$1";; esac
case "$INSPECTION_PLATFORM" in s100p|x86|rk3588) ;; *) echo "Unknown platform: $INSPECTION_PLATFORM" >&2; return 2 2>/dev/null || exit 2;; esac
export TANK_RUNTIME_ROOT="$EAST_ELEAIR_ROOT/.runtime/$INSPECTION_PLATFORM"
export TANK_BUILD_ROOT="$TANK_RUNTIME_ROOT/build"
export TANK_INSTALL_ROOT="$TANK_RUNTIME_ROOT/install"
export TANK_CORE_ENV="$TANK_RUNTIME_ROOT/venv"
export TANK_SYSTEM_ROOT="$EAST_ELEAIR_ROOT/deploy"
export ROS_LOG_DIR="$TANK_RUNTIME_ROOT/ros_logs"
export TANK_OFFLINE_PREFIX="$TANK_RUNTIME_ROOT/sysroot"
export INSPECTION_CONFIG_ROOT="${INSPECTION_CONFIG_ROOT:-$EAST_ELEAIR_ROOT/config}"
export FIELD_DATA_ROOT="${FIELD_DATA_ROOT:-/media/mcap_data}"
export FIELD_PHYSIC_ROOT="$EAST_ELEAIR_ROOT"
export VISION_ROOT="$EAST_ELEAIR_ROOT"
export FIELD_PHYSIC_INSTALL="$TANK_INSTALL_ROOT"
export FIELD_TANK_INSTALL="$TANK_INSTALL_ROOT"
export FIELD_WIRELESS_INSTALL="$TANK_INSTALL_ROOT"
export FIELD_USE_INSTALLED_CODE=1
export FIELD_RRD_PYTHON="${FIELD_RRD_PYTHON:-$TANK_RUNTIME_ROOT/rerun_venv/bin/python}"
export MINDVISION_SDK_ROOT="${MINDVISION_SDK_ROOT:-$EAST_ELEAIR_ROOT/third_party/mindvision/2.1.0.50}"
export FIELD_LIVOX_SDK_LIB_DIR="$TANK_RUNTIME_ROOT/native/install/lib"
export FIELD_REALSENSE_LIB_DIR="$TANK_RUNTIME_ROOT/native/install/lib"
# Source available underlays and this checkout's overlay, preserving caller nounset.
_inspection_nounset=0
case "$-" in *u*) _inspection_nounset=1; set +u;; esac
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
if [[ "$INSPECTION_PLATFORM" == s100p && -f /opt/tros/humble/local_setup.bash ]]; then source /opt/tros/humble/local_setup.bash; fi
if [[ -f "$TANK_OFFLINE_PREFIX/opt/ros/humble/local_setup.bash" ]]; then source "$TANK_OFFLINE_PREFIX/opt/ros/humble/local_setup.bash"; fi
if [[ -f "$TANK_INSTALL_ROOT/local_setup.bash" ]]; then source "$TANK_INSTALL_ROOT/local_setup.bash"; fi
if ((_inspection_nounset)); then set -u; fi
unset _inspection_nounset
if [[ -x "$TANK_CORE_ENV/bin/python" ]]; then export PATH="$TANK_CORE_ENV/bin:$PATH"; export VIRTUAL_ENV="$TANK_CORE_ENV"; fi
# Only canonical non-ROS libraries and thin application packages are source-importable.
export PYTHONPATH="$EAST_ELEAIR_ROOT:$EAST_ELEAIR_ROOT/packages/inspection_runtime/src:$EAST_ELEAIR_ROOT/packages/emat_core/src:$EAST_ELEAIR_ROOT/packages/tank_vision:$EAST_ELEAIR_ROOT/packages/d405_seam_tracking/src:$EAST_ELEAIR_ROOT/apps/inspection_cli:$EAST_ELEAIR_ROOT/apps/web_console${PYTHONPATH:+:$PYTHONPATH}"
export CMAKE_PREFIX_PATH="$TANK_RUNTIME_ROOT/native/install:$TANK_OFFLINE_PREFIX/usr:$TANK_OFFLINE_PREFIX/opt/ros/humble${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export AMENT_PREFIX_PATH="$TANK_OFFLINE_PREFIX/opt/ros/humble${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
_inspection_arch=x64
case "$(uname -m)" in aarch64|arm64) _inspection_arch=arm64;; esac
export LD_LIBRARY_PATH="$MINDVISION_SDK_ROOT/lib/$_inspection_arch:$TANK_RUNTIME_ROOT/native/install/lib:$TANK_OFFLINE_PREFIX/opt/ros/humble/lib:$TANK_OFFLINE_PREFIX/usr/lib/$(uname -m)-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _inspection_arch
