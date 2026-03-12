#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# ---- knobs (you can override by env vars) ----
REPORT_SEC="${REPORT_SEC:-2.0}"
DURATION_SEC="${DURATION_SEC:-60}"
QUEUE_SIZE="${QUEUE_SIZE:-10}"
SLOP_MS="${SLOP_MS:-100}"

OUT_DIR="${OUT_DIR:-${WS_ROOT}/results/week5}"
TS="$(date +%Y%m%d_%H%M%S)"

set +u
source "${ROS_SETUP}"
source "${WS_ROOT}/install/setup.bash"
set -u
mkdir -p "${OUT_DIR}"

cleanup() {
  # stop harness first
  pkill -TERM -f "sync_harness_node" 2>/dev/null || true

  # stop launch + avp nodes
  pkill -TERM -f "ros2 launch avp_core_implementation" 2>/dev/null || true
  pkill -TERM -f "avp_main_node|avp_controller_node" 2>/dev/null || true
  sleep 1

  pkill -KILL -f "sync_harness_node" 2>/dev/null || true
  pkill -KILL -f "ros2 launch avp_core_implementation" 2>/dev/null || true
  pkill -KILL -f "avp_main_node|avp_controller_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_topics() {
  local timeout_s="${1:-10}"
  local start
  start="$(date +%s)"

  while true; do
    # topic list가 에러 없이 돌아가고, 두 토픽이 모두 보여야 통과
    if ros2 topic list 2>/dev/null | grep -q "^/avp/vehicle_state$" && \
       ros2 topic list 2>/dev/null | grep -q "^/avp/parking_slot$"; then
      echo "[INFO] Topics are ready."
      return 0
    fi

    if [ "$(( $(date +%s) - start ))" -ge "${timeout_s}" ]; then
      echo "[ERROR] Topics not ready within ${timeout_s}s."
      ros2 topic list 2>/dev/null | head -n 50 || true
      return 1
    fi

    sleep 0.5
  done
}

echo "[INFO] WS_ROOT=${WS_ROOT}"
echo "[INFO] OUT_DIR=${OUT_DIR}"
echo "[INFO] DURATION_SEC=${DURATION_SEC} REPORT_SEC=${REPORT_SEC} QUEUE_SIZE=${QUEUE_SIZE} SLOP_MS=${SLOP_MS}"

# 1) run_local 실행 (백그라운드)
bash -lc "cd '${WS_ROOT}' && ./scripts/run_local.sh" >/dev/null 2>&1 &
RUN_LOCAL_PID=$!

# 2) 토픽 준비될 때까지 대기
wait_for_topics 15

# 3) approx 실행
RUN_ID="w5_${TS}_approx"
echo "[INFO] Run approx (run_id=${RUN_ID})"
timeout "${DURATION_SEC}" ros2 run avp_core_implementation sync_harness_node --ros-args \
  -p policy:=approx \
  -p queue_size:="${QUEUE_SIZE}" \
  -p slop_ms:="${SLOP_MS}" \
  -p report_sec:="${REPORT_SEC}" \
  -p out_dir:="${OUT_DIR}" \
  -p run_id:="${RUN_ID}" || true

# 4) exact 실행
RUN_ID="w5_${TS}_exact"
echo "[INFO] Run exact (run_id=${RUN_ID})"
timeout "${DURATION_SEC}" ros2 run avp_core_implementation sync_harness_node --ros-args \
  -p policy:=exact \
  -p queue_size:="${QUEUE_SIZE}" \
  -p slop_ms:="${SLOP_MS}" \
  -p report_sec:="${REPORT_SEC}" \
  -p out_dir:="${OUT_DIR}" \
  -p run_id:="${RUN_ID}" || true

echo "[DONE] results in: ${OUT_DIR}"
ls -ltr "${OUT_DIR}" | tail -n 10