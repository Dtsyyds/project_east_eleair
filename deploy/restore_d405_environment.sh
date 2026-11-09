#!/usr/bin/env bash
# Restore runtime packages from an explicitly selected local wheel directory.
set -euo pipefail
if [[ $# != 1 || ! -d "$1" ]]; then
  echo "Usage: bash deploy/restore_d405_environment.sh /path/to/offline-wheels" >&2
  exit 2
fi
tracking_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tracking_environment="$tracking_project_root/.runtime/s100p/d405_prototype_venv"
python3 -m venv --system-site-packages "$tracking_environment"
"$tracking_environment/bin/python" -m pip install --no-cache-dir --no-index --find-links "$1" \
  'torch==2.5.1' 'torchvision==0.20.1' 'segmentation-models-pytorch==0.5.0' \
  'numpy==1.26.4' 'opencv-python==4.11.0.86' 'PyYAML==6.0.3'
echo "Python environment restored: $tracking_environment/bin/python"
echo 'The original checkpoint.pt is a separate required artifact; this script does not replace or train a model.'
