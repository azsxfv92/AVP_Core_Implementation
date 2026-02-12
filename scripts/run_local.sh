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

# ---- Week4: results dir (runtime artifacts) ----
LOG_DIR="${WS_ROOT}/results/week4"
mkdir -p "${LOG_DIR}"

# ---- ROS2 + DDS env (must be set before any ROS2 processes start) ----
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

# ---- Week4: run log (always save) ----
TS="$(date +%Y%m%d_%H%M%S)"
RUN_LOG="${LOG_DIR}/run_local_${TS}.log"

# ---- Week4: DDS baseline options (off by default) ----
DDS_BASELINE="${DDS_BASELINE:-0}"           # 0=normal run, 1=run+measure+stop
TOPIC="${TOPIC:-/avp/vehicle_state}"        # baseline topic
DURATION="${DURATION:-15}"                  # seconds
STARTUP_WAIT="${STARTUP_WAIT:-2}"           # seconds

echo "[INFO] logging to ${RUN_LOG}"
echo "[INFO] DDS_BASELINE=${DDS_BASELINE} TOPIC=${TOPIC} DURATION=${DURATION}s STARTUP_WAIT=${STARTUP_WAIT}s"

RUN_PID=""

cleanup() {
  if [[ -n "${RUN_PID}" ]]; then
    echo "[INFO] cleanup: stopping process group (pgid=${RUN_PID})"
    kill -TERM -- -"${RUN_PID}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL -- -"${RUN_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "[INFO] Run"

set +e
setsid bash -lc "${RUN_CMD}" > >(tee -a "${RUN_LOG}") 2>&1 &
RUN_PID=$!
set -e

echo "[INFO] RUN_PID=${RUN_PID}"

# Wait a bit for nodes to come up and start publishing
sleep "${STARTUP_WAIT}"

# ---- Week4: DDS baseline measurement (Hz/BW) ----
if [ "${DDS_BASELINE}" = "1" ]; then
  echo "[INFO] DDS baseline start: ${TOPIC}"

  # Save baseline results with timestamp to avoid overwrite
  HZ_OUT="${LOG_DIR}/dds_$(echo "${TOPIC}" | tr '/' '_' | sed 's/^_//')_${TS}_hz.txt"
  BW_OUT="${LOG_DIR}/dds_$(echo "${TOPIC}" | tr '/' '_' | sed 's/^_//')_${TS}_bw.txt"

  set +e
  timeout "${DURATION}" ros2 topic hz "${TOPIC}" | tee "${HZ_OUT}"
  timeout "${DURATION}" ros2 topic bw "${TOPIC}" | tee "${BW_OUT}"
  set -e

  echo "[INFO] DDS baseline done:"
  echo "       - ${HZ_OUT}"
  echo "       - ${BW_OUT}"

  echo "[INFO] stopping RUN_CMD (process group pgid=${RUN_PID})"
  kill -TERM -- -"${RUN_PID}" >/dev/null 2>&1 || true
  sleep 1
  kill -KILL -- -"${RUN_PID}" >/dev/null 2>&1 || true
fi

# Normal mode: keep running until user Ctrl+C.
# Baseline mode: process group was killed above; wait will return.
wait "${RUN_PID}" 2>/dev/null || true
