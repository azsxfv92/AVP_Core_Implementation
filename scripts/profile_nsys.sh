#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULT_DIR="${ROOT_DIR}/results/"
NSYS_DIR="${RESULT_DIR}/nsys"
LOG_DIR="${RESULT_DIR}/logs"

mkdir -p "${NSYS_DIR}" "${LOG_DIR}"

TS="$(date '+%Y%m%d_%H%M%S')"
CASE_NAME="${CASE_NAME:-default}
TARGET_CMD="${TARGET_CMD:-ros2 launch avp_core_implementation avp_core_implementation.launch.py}"

OUT_BASE="${NSYS_DIR}/week12_${CASE_NAME}_${TS}"
LOG_FILE="${LOG_DIR}/profile_nsys_${CASE_NAME}_${TS}.log"

echo "[INFO] root_dir=${ROOT_DIR}" | tee "${LOG_FILE}"
echo "[INFO] result_dir=${RESULT_DIR}" | tee -a "${LOG_FILE}"
echo "[INFO] case_name=${CASE_NAME}" | tee -a "${LOG_FILE}"
echo "[INFO] target_cmd=${TARGET_CMD}" | tee -a "${LOG_FILE}"
echo "[INFO] nsys_output=${OUT_BASE}.nsys-rep" | tee -a "${LOG_FILE}"

if ! command -v nsys >/dev/null 2>&1; then
  echo "[ERROR] nsys not found. Install Nsight Systems or check PATH." | tee -a "${LOG_FILE}"
  exit 1
fi

nsys profile \
    --trace=cuda,nvtx,osrt \
    --sample=cpu \
    --force-overwrite=true \
    --output="${OUT_BASE}" \
    bash -lc "${TARGET_CMD}" 2>&1 | tee -a "${LOG_FILE}"

echo "[OK] Nsight profile saved: ${OUT_BASE}.nsys-rep" | tee -a "${LOG_FILE}"