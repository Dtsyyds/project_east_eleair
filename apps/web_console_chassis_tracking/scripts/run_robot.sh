#!/usr/bin/env bash
# 扩展版脚本目录入口；复用已有启动链路，不重复维护端口或控制参数。
set -e
ROBOT_CONSOLE_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROBOT_CONSOLE_DIRECTORY/run_robot.sh" "$@"
