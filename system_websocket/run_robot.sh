#!/usr/bin/env bash
# 正式启动入口统一放在 system_websocket；仅使用独立扩展版，不覆盖原版。
set -e
ROBOT_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROBOT_WEB_ROOT:-}" ]]; then
  ROBOT_WEB_DIRECTORY="$ROBOT_WEB_ROOT"
elif [[ -d "$ROBOT_PROJECT_ROOT/apps/web_console_chassis_tracking" ]]; then
  ROBOT_WEB_DIRECTORY="$ROBOT_PROJECT_ROOT/apps/web_console_chassis_tracking"
else
  ROBOT_WEB_DIRECTORY="$(cd "$ROBOT_PROJECT_ROOT/.." && pwd)/web_console_chassis_tracking"
fi
# CLI 显式 --web-root 置于后面，仍可覆盖默认目录；缺失文件由预检明确报错。
exec python3 "$ROBOT_PROJECT_ROOT/tools/robot_session.py" --web-root "$ROBOT_WEB_DIRECTORY" "$@"
