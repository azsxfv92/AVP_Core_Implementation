#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DDS_XML="${WS_ROOT}/config/dds/fastdds.xml"

# ROS2 RMW implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Adopt fastdds.xml profile
export FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_XML}"

echo "[INFO] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[INFO] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
