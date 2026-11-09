#!/usr/bin/env bash
# 兼容原 system 入口，统一使用 system_websocket 的整机启动逻辑。
set -e
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/system_websocket/run_robot.sh" "$@"
