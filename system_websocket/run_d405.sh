#!/usr/bin/env bash
# A is the default; --legacy keeps the previous classical/fused entry.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# D405 communication uses eth1 (192.168.127.10); do not advertise eth0.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/config/communication/fastdds_d405_127.xml"
profile=d405_a.yaml
if [[ "${1:-}" == --candidate ]]; then
  profile=d405_candidate.yaml
  shift
fi
if [[ "${1:-}" == --legacy ]]; then
  profile=d405_experiment.yaml
  shift
fi
exec bash "$ROOT/apps/web_console/run.sh" --config "$ROOT/config/communication/$profile" "$@"
