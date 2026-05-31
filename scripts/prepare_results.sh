#!/usr/bin/env bash
set -euo pipefail
 
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")//.." && pwd)"
RESULT_DIR="${ROOT_DIR}/results/week12"
NSYS_DIR="${RESULT_DIR}/nsys"
LOG_DIR="${RESULT_DIR}/logs"
OUTPUT_DIR="${RESULT_DIR}/output"

mkdir -p "${RESULT_DIR}" "${NSYS_DIR}" "${LOG_DIR}" "${OUTPUT_DIR}"

RUN_INFO="${RESULT_DIR}/run_info.txt"
METRICS_CSV="${RESULT_DIR}/metrics.csv"
STAGE_CSV="${RESULT_DIR}/e2e_stage_times.csv"

{
    echo "===== Run Info ====="
    echo "date: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "hostname: $(hostname)"
    echo "user: $(whoami)"
    echo "root_dir: ${ROOT_DIR}"
    echo "result_dir: ${ROOT_DIR}"
    echo ""

    echo "==== OS ===="
    echo uname -a
    lsb_release -a 2>/dev/null || true
    echo ""

    echo "==== ROS2 ===="
    echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
    echo "RMW_IMPLEMENTATION=${RAW_IMPLEMENTATION:-unset}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
    echo "FASTDOS_DEFAULT_PROFILES_FILE=${FASTDOS_DEFAULT_PROFILES_FILE:-unset}"
    echo ""

    echo "==== NVIDIA / Jetson Tools ===="
    echo "tegrastats: $(command -v tegrastats || echo not_found)"
    echo "nsys: $(command -v nsys || echo not_found)"
    echo "gst-laubch-1.0: $(command -v gst-launch-1.0 || echo not_found)"
    echo ""

    echo "==== CUDA / TensorRT ===="
    echo "nvcc: $(command -v nsys || echo not_found)"
    nvcc --version 2>/dev/null || true
    echo "trtexec: $(command -v trtexec || echo not_found)"
    echo ""

    echo "==== ROS2 Visible Stat ===="
    ros2 node list 2>/dev/null || echo "ros2 node list failed or ROS2 not running"
    echo ""
    ros2 topic list 2>/dev/null || echo "ros2 topic list failed or ROS2 not running"
    echo ""
} | tee "${RUN_INFO}"

cat > "${METRICS_CSV}" <<EOF
case_name,fps,p50_total_ms,p95_total_ms,p50_encode_ms,p95_encode_ms,drop_rate,notes
EOF

cat > "${STAGE_CSV}" <<EOF
frame_id,recv_ms,pre_ms,infer_ms,post_ms,encode_ms,total_ms,queue_depth,drop_count,drop_reason
EOF

echo "[OK] Prepared Week 12 result package"
echo "[OK] run_info: ${RUN_INFO}"
echo "[OK] metrics: ${METRICS_CSV}"
echo "[OK] stage_times: ${STAGE_CSV}"