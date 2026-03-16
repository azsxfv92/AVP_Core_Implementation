#!/bin/bash
set -euo pipefail

DATE=$(date +%Y%m%d_%H%M%S)
LOG_DIR="results/build_logs"
mkdir -p "${LOG_DIR}"

LOG_FILE="${LOG_DIR}/build_${DATE}.log"

echo "[INFO] Build stated as ${DATE}" | tee "${LOG_FILE}"

YOCTO_ROOT="${HOME}/yocto/poky"
if [ ! -d "${YOCTO_ROOT}" ]; then ehco "[ERROR] YOCTO_ROOT doesn't exist: ${YOCTO_ROOT}" | tee -a "${LOG_FILE}" exit 1
fi

cd "${YOCTO_ROOT}"

# oe-init-build-env set up an environment in order to can use bitbake and build/conf
source oe-init-build-env build >> "${LOG_FILE}" 2>&1

# check if the folders already exists
grep -q 'DL_DIR' conf/local.conf || echo 'DL_DIR ?= "'"${HOME}/yocto_cache/downloads"'"' >> conf/local.conf
grep -q 'SSTAE_DIR' conf/local.conf || echo 'SSTATE_DIR ?= "'"${HOME}/yocto_cache/sstate_cache"'"' >> conf/local.conf

# assigne the build target image
IMAGE_TARGET="core_image_minimal"

echo "[INFO] Building image: ${IMAGE_TARGET}" | tee -a "${LOG_FILE}"

bitbake "${IMAGE_TARGET}" 2>&1 | tee -a "${LOG_FILE}"

echo "[INFO] Build finished at ${date +%Y%m%d_%H%M%S}" | tee -a "${LOG_FILE}"
