#!/usr/bin/env bash
# Source this script to inspect D405 topics in the running system's DDS domain.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Usage: source deploy/env_d405.sh' >&2
  exit 2
fi

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh" || return $?

export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$EAST_ELEAIR_ROOT/config/communication/fastdds_d405_127.xml"
