#!/usr/bin/env bash
set -euo pipefail

# ============================================
# Device: Ubuntu server
# Purpose:
#   Start CARLA server in offscreen mode
#   and save logs for reproducibility
# ============================================

CARLA_ROOT="${HOME}/sim"
LOG_DIR="${HOME}/avp_core_implementation/results/week9"
LOG_FILE="${LOG_DIR}/carla_server.log"

mkdir -p "${LOG_DIR}"

echo "[INFO] CARLA_ROOT=${CARLA_ROOT}"
echo "[INFO] LOG_FILE=${LOG_FILE}"

if [ ! -f "${CARLA_ROOT}/CarlaUE4.sh" ]; then
  echo "[ERROR] CarlaUE4.sh not found at ${CARLA_ROOT}"
  exit 1
fi

echo "[INFO] killing previous CARLA process if exists..."
pkill -f CarlaUE4 || true
sleep 2

echo "[INFO] starting CARLA server..."
cd "${CARLA_ROOT}"
nohup ./CarlaUE4.sh -RenderOffScreen > "${LOG_FILE}" 2>&1 &

echo "[INFO] waiting for server boot..."
sleep 10

echo "[INFO] process check"
ps -ef | grep CarlaUE4 | grep -v grep || true

echo "[INFO] port 2000 check"
ss -ltnp | grep 2000 || true

echo "[INFO] port 2001 check"
ss -ltnp | grep 2001 || true

echo "[INFO] tail log"
tail -n 30 "${LOG_FILE}" || true

echo "[DONE] run_carla_server.sh finished"