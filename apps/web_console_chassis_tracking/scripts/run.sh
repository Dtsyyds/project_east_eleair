#!/usr/bin/env bash
# 保留原三相机启动入口；不启动新增底盘/循迹功能。
set -e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_inspection_project.sh"
exec bash "$INSPECTION_LAUNCH_ROOT/system_websocket/run.sh" "$@"
