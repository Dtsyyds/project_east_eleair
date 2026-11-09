#!/usr/bin/env bash
set -euo pipefail
ranking_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ranking_repo"
ranking_output="${1:?Usage: bash run_d405_candidate_ranking_ab.sh NEW_OUTPUT}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
"$ranking_repo/.runtime/s100p/d405_prototype_venv/bin/python" \
  experiments/vision/scripts/d405_candidate_ranking_ab.py --output "$ranking_output"
