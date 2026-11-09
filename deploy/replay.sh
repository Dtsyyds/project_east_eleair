#!/usr/bin/env bash
set -e
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
exec python3 -m inspection_cli replay "$@"
