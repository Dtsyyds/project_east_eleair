#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 || "$1" != /* ]]; then
  echo "Usage: colmap_output.sh /absolute/path/to/new_colmap_directory" >&2
  exit 2
fi
mkdir -p -- "$1/images" "$1/sparse/0"
echo "COLMAP directories ready: $1"
