#!/usr/bin/env bash
set -e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/deploy/env.sh"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo "Usage: $(basename "${BASH_SOURCE[0]}") [existing arguments]. See docs/data and deploy/README.md."
  exit 0
fi
set -euo pipefail
SCRIPT_DIR="$EAST_ELEAIR_ROOT/tools/data"
exec "$SCRIPT_DIR/stop_vehicle_collection.sh" "$@"
