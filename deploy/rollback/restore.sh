#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help || $# -ne 2 ]]; then
  echo 'Usage: deploy/rollback/restore.sh COMMIT NEW_DIRECTORY'
  echo 'Creates a detached worktree for rebuilding; does not reset or reuse old installs.'
  [[ "${1:-}" == --help ]] && exit 0 || exit 2
fi
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
git -C "$root" worktree add --detach "$2" "$1"
echo 'Rebuild using the selected checkout deployment instructions and fresh runtime outputs.'
