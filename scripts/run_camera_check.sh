#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="${HOME}/avp_core_implementation"
CONDA_ROOT="${HOME}/miniconda3"
CONDA_ENV_NAME="carla-python37"

CARLA_EGG="${HOME}/sim/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg"
PY_SCRIPT="${REPO_ROOT}/tools/week9_spawn_camera_check.py"
LOG_DIR="${REPO_ROOT}/results/week9"
LOG_FILE="${LOG_DIR}/camera_check_run.log"

mkdir -p "${LOG_DIR}"

echo "[INFO] REPO_ROOT=${REPO_ROOT}"
echo "[INFO] CONDA_ENV_NAME=${CONDA_ENV_NAME}"
echo "[INFO] CARLA_EGG=${CARLA_EGG}"
echo "[INFO] PY_SCRIPT=${PY_SCRIPT}"
echo "[INFO] LOG_FILE=${LOG_FILE}"

if [ ! -f "${PY_SCRIPT}" ]; then
  echo "[ERROR] Python script not found: ${PY_SCRIPT}"
  exit 1
fi

if [ ! -f "${CARLA_EGG}" ]; then
  echo "[ERROR] CARLA egg not found: ${CARLA_EGG}"
  exit 1
fi

# conda activate works only after sourcing conda.sh
source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV_NAME}"

export PYTHONPATH="${CARLA_EGG}:${PYTHONPATH:-}"

echo "[INFO] python path"
which python

echo "[INFO] python version"
python --version

echo "[INFO] import carla test"
python -c "import carla; print(carla)"

echo "[INFO] running week9 camera check..."
cd "${REPO_ROOT}"
python "${PY_SCRIPT}" 2>&1 | tee "${LOG_FILE}"

echo "[DONE] run_week9_camera_check.sh finished"