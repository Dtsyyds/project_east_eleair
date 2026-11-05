#!/usr/bin/env bash
# 保留原 D405 启动入口；--candidate、--legacy 及其他参数原样转发。
set -e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_inspection_project.sh"
exec bash "$INSPECTION_LAUNCH_ROOT/system_websocket/run_d405.sh" "$@"
