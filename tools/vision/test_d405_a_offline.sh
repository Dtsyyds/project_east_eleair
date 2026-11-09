#!/usr/bin/env bash
# Offline only: existing files -> isolated DDS fixture -> passive A subscriber.
set -euo pipefail
test_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$test_repo"
test_output="${1:?Usage: bash tools/vision/test_d405_a_offline.sh NEW_OUTPUT}"
source deploy/env.sh s100p
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
# Match the existing console supervisor's large-image DDS transport. The default
# 512 KiB SHM segment is smaller than one RGB frame and can lose replay inputs.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$test_repo/config/communication/fastdds.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
exec "$test_repo/.runtime/s100p/d405_prototype_venv/bin/python" \
  tools/vision/validate_d405_a_offline.py --output "$test_output"
