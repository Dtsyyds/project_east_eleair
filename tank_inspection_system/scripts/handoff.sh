#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo "usage: handoff.sh --run-dir RUN_DIR --profile PROFILE --output-root DESTINATION [其他原参数]"
  echo "canonical entry: physic_bev/scripts/build_role_handoff.py; arguments/exit status/signals are forwarded"
  exit 0
fi
if [[ -z "${FIELD_PHYSIC_ROOT:-}" ]]; then source "$SCRIPT_DIR/env.sh"; fi
exec python3 "$FIELD_PHYSIC_ROOT/scripts/build_role_handoff.py" "$@"
