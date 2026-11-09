#!/usr/bin/env bash
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/packages/inspection_runtime/scripts/run_d405_lifecycle_validation.sh" "$@"
