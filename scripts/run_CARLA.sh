#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT="${HOME}/sim"
REPO_ROOT="${HOME}/avp_core_implementation"
LOG_DIR="${REPO_ROOT}/results/week9"
LOG_FILE="${LOG_DIR}/carla_server_gui.log"

mkdir -p "${LOG_DIR}"

echo "[INFO] CARLA_ROOT=${CARLA_ROOT}"
echo "[INFO] LOG_FILE=${LOG_FILE}"
echo "[INFO] DISPLAY=${DISPLAY:-<empty>}"
echo "[INFO] XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-<empty>}"

if [ ! -f "${CARLA_ROOT}/CarlaUE4.sh" ]; then
  echo "[ERROR] CarlaUE4.sh not found: ${CARLA_ROOT}/CarlaUE4.sh"
  exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
  echo "[ERROR] DISPLAY is empty. This script must be run from Ubuntu GUI desktop terminal."
  exit 2
fi

echo "[INFO] kill previous CARLA process if exists"
pkill -f CarlaUE4 || true
sleep 2

echo "[INFO] start CARLA with GUI"
cd "${CARLA_ROOT}"
./CarlaUE4.sh 2>&1 | tee "${LOG_FILE}"