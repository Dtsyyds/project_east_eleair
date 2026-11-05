#!/usr/bin/env bash
# 公共路径解析；由 scripts 中的原版入口 source，不启动任何设备。
INSPECTION_CONSOLE_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSPECTION_LAUNCH_ROOT="${INSPECTION_PROJECT_ROOT:-$(cd "$INSPECTION_CONSOLE_DIRECTORY/../.." && pwd)}"
if [[ ! -d "$INSPECTION_LAUNCH_ROOT/system_websocket" && -z "${INSPECTION_PROJECT_ROOT:-}" ]]; then
  INSPECTION_LAUNCH_ROOT="$(cd "$INSPECTION_CONSOLE_DIRECTORY/.." && pwd)/project_east_eleair_refactor"
fi
if [[ ! -d "$INSPECTION_LAUNCH_ROOT/system_websocket" ]]; then
  echo '未找到原巡检启动脚本；请设置 INSPECTION_PROJECT_ROOT 指向完整巡检工程' >&2
  exit 2
fi
