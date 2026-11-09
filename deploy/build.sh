#!/usr/bin/env bash
set -e
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  exec python3 "$(dirname "${BASH_SOURCE[0]}")/build.py" --help
fi
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
exec python3 "$EAST_ELEAIR_ROOT/deploy/build.py" "$@"
