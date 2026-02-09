#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

PKG="avp_core_implementation"
RUN_CMD_DEFAULT="ros2 launch ${PKG} avp_core_implementation.launch.py"
RUN_CMD="${RUN_CMD:-$RUN_CMD_DEFAULT}"

echo "[INFO] WS_ROOT=${WS_ROOT}"
echo "[INFO] BUILD_TYPE=${BUILD_TYPE}"
echo "[INFO] RUN_CMD=${RUN_CMD}"

if [ ! -f "${ROS_SETUP}" ]; then
  echo "[ERROR] ROS setup not found: ${ROS_SETUP}"
  echo "        e.g. export ROS_SETUP=/opt/ros/humble/setup.bash"
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WS_ROOT}/scripts/env_dds.sh"
set -u

cd "${WS_ROOT}"

echo "[INFO] colcon build..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

echo "[INFO] source install/setup.bash"
set +u
source "${WS_ROOT}/install/setup.bash"
set -u

echo "[INFO] Check executables:"
ros2 pkg executables "${PKG}" || true

echo "[INFO] Run"
eval "${RUN_CMD}"
