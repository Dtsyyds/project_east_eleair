#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STATE="$ROOT/.runtime/s100p/analysis/d405_realtime_replay_20261103"
PIDFILE="$STATE/server.pid"
mkdir -p "$STATE"
active() {
  [[ -f "$PIDFILE" ]] || return 1
  local replay_pid
  replay_pid="$(cat "$PIDFILE")"
  [[ "$replay_pid" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/$replay_pid/cmdline" ]] || return 1
  tr '\0' ' ' < "/proc/$replay_pid/cmdline" | grep -q 'tools/vision/replay_d405_realtime.py'
}
case "${1:-status}" in
 start)
  shift
  if active; then echo "回放服务已运行；http://192.168.127.10:8772/"; exit 0; fi
  cd "$ROOT"
  nohup setsid "$ROOT/.runtime/s100p/d405_prototype_venv/bin/python" -u "$ROOT/tools/vision/replay_d405_realtime.py" "$@" >> "$STATE/server.log" 2>&1 < /dev/null &
  echo "$!" > "$PIDFILE"
  echo "已启动；http://192.168.127.10:8772/；日志 $STATE/server.log"
  ;;
 stop)
  if active; then
    kill "$(cat "$PIDFILE")"
    for attempt in {1..50}; do
      if ! active; then break; fi
      sleep 0.1
    done
    if active; then echo '进程尚未退出，请检查 server.log'; exit 1; fi
  fi
  rm -f "$PIDFILE"
  echo '回放服务已停止'
  ;;
 status)
  if active; then echo "服务运行中 PID=$(cat "$PIDFILE")"; else echo '服务未运行'; fi
  ;;
 *) echo '用法：start [--center2d] | stop | status。省略 --center2d 为原 A。'; exit 2 ;;
esac
