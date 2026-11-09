#!/usr/bin/env bash
# Local offline replay: saved held-out probabilities, no training or hardware.
set -euo pipefail
ab_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ab_repo"
ab_output="${1:?Usage: bash run_d405_central_window_ab.sh NEW_OUTPUT}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
"$ab_repo/.runtime/s100p/d405_prototype_venv/bin/python" \
  experiments/vision/scripts/d405_central_window_ab.py --output "$ab_output"
