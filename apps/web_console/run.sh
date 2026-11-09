#!/usr/bin/env bash
set -e
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/deploy/run.sh" "$@"
