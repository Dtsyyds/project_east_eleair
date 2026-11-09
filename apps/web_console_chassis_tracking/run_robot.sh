#!/usr/bin/env bash
# 兼容旧调用：正式启动逻辑统一转发到巡检工程 system_websocket。
set -e
ROBOT_CONSOLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_INSPECTION_ROOT="${INSPECTION_PROJECT_ROOT:-$(cd "$ROBOT_CONSOLE_ROOT/../.." && pwd)}"
if [[ ! -f "$ROBOT_INSPECTION_ROOT/tools/robot_session.py" && -z "${INSPECTION_PROJECT_ROOT:-}" ]]; then
  ROBOT_INSPECTION_ROOT="$(cd "$ROBOT_CONSOLE_ROOT/.." && pwd)/project_east_eleair_refactor"
fi
if [[ ! -f "$ROBOT_INSPECTION_ROOT/tools/robot_session.py" ]]; then
  echo '缺少完整巡检后台；请设置 INSPECTION_PROJECT_ROOT 指向 project_east_eleair_refactor' >&2
  exit 2
fi
exec bash "$ROBOT_INSPECTION_ROOT/system_websocket/run_robot.sh" --web-root "$ROBOT_CONSOLE_ROOT" "$@"
