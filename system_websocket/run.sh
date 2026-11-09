#!/usr/bin/env bash
# Keep the familiar one-command launcher; implementation stays in apps/deploy.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export INSPECTION_SYSTEM_CONFIG="${INSPECTION_SYSTEM_CONFIG:-$ROOT/config/communication/cameras.yaml}"
exec bash "$ROOT/apps/web_console/run.sh" "$@"
