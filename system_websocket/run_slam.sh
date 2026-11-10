#!/usr/bin/env bash
# 整机启动器内部的 SLAM 服务入口；用户统一运行 system/run_robot.sh。
set -e
SLAM_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_PLATFORM="${INSPECTION_PLATFORM:-s100p}"
SLAM_LIGHT=0
SLAM_ARGS=("$@")
for ((i=0; i<${#SLAM_ARGS[@]}; i++)); do
  case "${SLAM_ARGS[i]}" in
    --platform) SLAM_PLATFORM="${SLAM_ARGS[i+1]:-}" ;;
    --platform=*) SLAM_PLATFORM="${SLAM_ARGS[i]#*=}" ;;
    --help|-h|--plan) SLAM_LIGHT=1 ;;
  esac
done
if [[ "$SLAM_LIGHT" == 0 ]]; then source "$SLAM_PROJECT_ROOT/deploy/env.sh" "$SLAM_PLATFORM"; fi
export PYTHONPATH="$SLAM_PROJECT_ROOT/apps/web_console_slam${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m slam_console "$@"
