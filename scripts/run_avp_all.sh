#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Device: Desktop PC (Ubuntu)
# Purpose:
#   One-command startup of the full Week 16 Day 6 AVP pipeline
#   (README "Week 16 Day 6 execution flow" steps 1-5) :
#
#     CARLA server
#       -> Autoware planning simulator (+ initial pose)
#       -> CARLA scenario (hero + obstacle, camera, target_accel)
#       -> [Jetson] trt_infer_node
#       -> detection -> Autoware bridge
#       -> aeb_node
#
#   Every process is started in its own process group, logged under
#   results/week16_bridge/day6_aeb/run_<TS>/, and killed together on Ctrl+C.
#
# Usage:
#   ./scripts/run_avp_all.sh                       # everything on this PC
#   ./scripts/run_avp_all.sh --jetson user@orin    # + start trt_infer_node over ssh
#   ./scripts/run_avp_all.sh --build --record
#   ./scripts/run_avp_all.sh --skip-carla --skip-autoware   # reuse running sim
# ============================================================

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ---- configuration (override via env or flags) ----
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
AUTOWARE_SETUP="${AUTOWARE_SETUP:-${HOME}/autoware/install/setup.bash}"
CARLA_ROOT="${CARLA_ROOT:-${HOME}/sim}"
MAP_PATH="${MAP_PATH:-${HOME}/autoware_map/sample-map-planning}"
# CARLA + rclpy live in the system python3.10, not in conda's python3.
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
PKG="avp_core_implementation"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

RVIZ="${RVIZ:-false}"                       # Autoware rviz
USE_DDS_PROFILE="${USE_DDS_PROFILE:-0}"     # 1 = source scripts/env_dds.sh
BUILD="${BUILD:-0}"
RECORD="${RECORD:-0}"
JETSON_HOST="${JETSON_HOST:-}"              # e.g. minhyuk@192.168.0.31
WAIT_JETSON_SEC="${WAIT_JETSON_SEC:-120}"   # wait for /avp/detections when starting the Jetson by hand
BUILD_TYPE="${BUILD_TYPE:-Release}"

SKIP_CARLA=0 SKIP_AUTOWARE=0 SKIP_SCENARIO=0 SKIP_BRIDGE=0 SKIP_AEB=0

SCENARIO_ARGS="${SCENARIO_ARGS:-}"          # extra --ros-args for the CARLA scenario
BRIDGE_ARGS="${BRIDGE_ARGS:-}"              # extra --ros-args for the detection bridge
AEB_ARGS="${AEB_ARGS:-}"                    # extra --ros-args for aeb_node

JETSON_WS="${JETSON_WS:-~/avp_core_implementation}"
JETSON_CMD_DEFAULT="source /opt/ros/humble/setup.bash; source ${JETSON_WS}/install/setup.bash; \
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; ros2 run ${PKG} trt_infer_node"
JETSON_CMD="${JETSON_CMD:-$JETSON_CMD_DEFAULT}"

usage() {
  sed -n '3,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
  cat <<'USAGE'

Flags:
  --build              colcon build before starting
  --record             also run tools/week16_record_aeb_evidence.py
  --jetson USER@HOST   start trt_infer_node on the Jetson over ssh (needs ssh key auth)
  --no-wait-jetson     do not wait for /avp/detections when the Jetson is started by hand
  --rviz               launch Autoware with rviz
  --dds-profile        source scripts/env_dds.sh (Fast DDS baseline profile)
  --skip-carla         do not start the CARLA server
  --skip-autoware      do not start Autoware / set initial pose
  --skip-scenario      do not start tools/week16_carla_scenario.py
  --skip-bridge        do not start tools/week16_detection_to_autoware_node.py
  --skip-aeb           do not start aeb_node
  -h, --help           show this help
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --build)         BUILD=1 ;;
    --record)        RECORD=1 ;;
    --jetson)        JETSON_HOST="${2:?--jetson needs USER@HOST}"; shift ;;
    --no-wait-jetson) WAIT_JETSON_SEC=0 ;;
    --rviz)          RVIZ=true ;;
    --dds-profile)   USE_DDS_PROFILE=1 ;;
    --skip-carla)    SKIP_CARLA=1 ;;
    --skip-autoware) SKIP_AUTOWARE=1 ;;
    --skip-scenario) SKIP_SCENARIO=1 ;;
    --skip-bridge)   SKIP_BRIDGE=1 ;;
    --skip-aeb)      SKIP_AEB=1 ;;
    -h|--help)       usage; exit 0 ;;
    *) echo "[ERROR] unknown option: $1"; usage; exit 2 ;;
  esac
  shift
done

TS="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${WS_ROOT}/results/week16_bridge/day6_aeb/run_${TS}"
mkdir -p "${LOG_DIR}"

echo "[INFO] WS_ROOT=${WS_ROOT}"
echo "[INFO] LOG_DIR=${LOG_DIR}"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] PYTHON_BIN=${PYTHON_BIN}"

# ---- preflight ----
[ -f "${ROS_SETUP}" ]      || { echo "[ERROR] ROS setup not found: ${ROS_SETUP}"; exit 1; }
[ -x "${PYTHON_BIN}" ]     || { echo "[ERROR] python not executable: ${PYTHON_BIN}"; exit 1; }
if [ "${SKIP_AUTOWARE}" = "0" ] && [ ! -f "${AUTOWARE_SETUP}" ]; then
  echo "[ERROR] Autoware setup not found: ${AUTOWARE_SETUP}"; exit 1
fi
if [ "${SKIP_CARLA}" = "0" ] && [ ! -f "${CARLA_ROOT}/CarlaUE4.sh" ]; then
  echo "[ERROR] CarlaUE4.sh not found at ${CARLA_ROOT}"; exit 1
fi

# ---- environment (before any ROS process starts) ----
set +u
source "${ROS_SETUP}"
[ -f "${AUTOWARE_SETUP}" ] && source "${AUTOWARE_SETUP}"
if [ "${USE_DDS_PROFILE}" = "1" ]; then
  source "${WS_ROOT}/scripts/env_dds.sh"
fi
set -u

cd "${WS_ROOT}"

if [ "${BUILD}" = "1" ]; then
  echo "[INFO] colcon build (${BUILD_TYPE})..."
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    2>&1 | tee "${LOG_DIR}/colcon_build.log"
fi

if [ ! -f "${WS_ROOT}/install/setup.bash" ]; then
  echo "[ERROR] ${WS_ROOT}/install/setup.bash not found. Run with --build first."
  exit 1
fi
set +u
source "${WS_ROOT}/install/setup.bash"
set -u

# ---- process bookkeeping ----
PIDS=()      # pid (== pgid, started with setsid)
NAMES=()
CARLA_STARTED=0

start_proc() {  # start_proc <name> <command string>
  local name="$1" cmd="$2"
  local log="${LOG_DIR}/${name}.log"
  echo "[INFO] start ${name} -> ${log}"
  setsid bash -c "${cmd}" > "${log}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  NAMES+=("${name}")
  sleep 0.5
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[WARN] ${name} exited immediately; see ${log}"
    tail -n 20 "${log}" || true
  fi
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  echo
  echo "[INFO] shutting down..."
  local i
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do
    local pid="${PIDS[$i]}"
    [ -n "${pid}" ] || continue
    if kill -0 "${pid}" 2>/dev/null; then
      echo "[INFO]   stop ${NAMES[$i]} (pgid=${pid})"
      kill -TERM -- -"${pid}" >/dev/null 2>&1 || true
    fi
  done
  sleep 2
  for (( i=${#PIDS[@]}-1; i>=0; i-- )); do
    [ -n "${PIDS[$i]}" ] || continue
    kill -KILL -- -"${PIDS[$i]}" >/dev/null 2>&1 || true
  done
  if [ "${CARLA_STARTED}" = "1" ]; then
    echo "[INFO]   stop CARLA server"
    pkill -f CarlaUE4 >/dev/null 2>&1 || true
  fi
  echo "[INFO] logs: ${LOG_DIR}"
  exit "${rc}"
}
trap cleanup EXIT INT TERM

wait_for() {  # wait_for <desc> <timeout_sec> <shell test>
  local desc="$1" timeout="$2" test_cmd="$3" waited=0
  echo -n "[INFO] waiting for ${desc} "
  while ! eval "${test_cmd}" >/dev/null 2>&1; do
    if [ "${waited}" -ge "${timeout}" ]; then
      echo " TIMEOUT (${timeout}s)"
      return 1
    fi
    echo -n "."
    sleep 2
    waited=$((waited + 2))
  done
  echo " ok (${waited}s)"
  return 0
}

# ============================================================
# 1) CARLA server
# ============================================================
if [ "${SKIP_CARLA}" = "0" ]; then
  echo "[INFO] killing previous CARLA process if exists..."
  pkill -f CarlaUE4 >/dev/null 2>&1 || true
  sleep 2
  start_proc "1_carla_server" "cd '${CARLA_ROOT}' && ./CarlaUE4.sh -RenderOffScreen"
  CARLA_STARTED=1
  wait_for "CARLA RPC port 2000" 120 "ss -ltn | grep -q ':2000 '" || {
    echo "[ERROR] CARLA did not open port 2000. See ${LOG_DIR}/1_carla_server.log"; exit 1; }
else
  echo "[INFO] skip CARLA server (--skip-carla)"
fi

# ============================================================
# 2) Autoware planning simulator + initial pose
# ============================================================
if [ "${SKIP_AUTOWARE}" = "0" ]; then
  start_proc "2_autoware" "ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:='${MAP_PATH}' \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    rviz:=${RVIZ} \
    rviz_respawn:=${RVIZ}"

  wait_for "Autoware simple_planning_simulator" 240 \
    "ros2 node list 2>/dev/null | grep -q simple_planning_simulator" || {
    echo "[ERROR] Autoware did not come up. See ${LOG_DIR}/2_autoware.log"; exit 1; }

  # planning_simulator.launch.xml spawns dummy_perception_publisher, which remaps onto
  # /perception/object_recognition/detection/objects and masks a dead perception source.
  echo "[INFO] removing autoware_dummy_perception_publisher_node"
  for _ in 1 2 3; do
    pkill -9 -f autoware_dummy_perception_publisher_node >/dev/null 2>&1 || true
    sleep 1
  done

  # simple_planning_simulator subscribes to /initialpose3d, not /initialpose.
  echo "[INFO] publishing initial pose"
  "${WS_ROOT}/scripts/set_initial_pose.sh" 2>&1 | tee "${LOG_DIR}/2b_initial_pose.log" || \
    echo "[WARN] set_initial_pose.sh failed; retry it manually."

  if ! wait_for "/localization/kinematic_state" 30 \
    "timeout 5 ros2 topic echo /localization/kinematic_state --once"; then
    echo "[WARN] no kinematic_state yet. Re-run ./scripts/set_initial_pose.sh if TF map->base_link is missing."
  fi
else
  echo "[INFO] skip Autoware (--skip-autoware)"
fi

# ============================================================
# 3) CARLA scenario (hero + front obstacle, camera pub, target_accel apply)
# ============================================================
if [ "${SKIP_SCENARIO}" = "0" ]; then
  start_proc "3_carla_scenario" \
    "cd '${WS_ROOT}' && '${PYTHON_BIN}' tools/week16_carla_scenario.py ${SCENARIO_ARGS}"
  wait_for "/avp/camera/front/compressed" 90 \
    "ros2 topic list --no-daemon 2>/dev/null | grep -q '^/avp/camera/front/compressed$'" || {
    echo "[ERROR] scenario did not publish camera topic. See ${LOG_DIR}/3_carla_scenario.log"; exit 1; }
else
  echo "[INFO] skip CARLA scenario (--skip-scenario)"
fi

# ============================================================
# 4) Jetson inference (optional, over ssh) + detection -> Autoware bridge
# ============================================================
# trt_infer_node runs on the Jetson, not on this PC. Two ways to get it up:
#   --jetson USER@HOST : this script starts it over ssh (needs key auth: ssh-copy-id)
#   otherwise          : start it by hand on the Jetson; this script waits for
#                        /avp/detections so both machines line up in one go.
print_jetson_manual_cmd() {
  echo "         source /opt/ros/humble/setup.bash"
  echo "         source ${JETSON_WS}/install/setup.bash"
  echo "         export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "         ros2 run ${PKG} trt_infer_node"
}

if [ -n "${JETSON_HOST}" ]; then
  # BatchMode: never hang on a password prompt (the process is backgrounded into a log).
  echo "[INFO] ssh preflight: ${JETSON_HOST}"
  if ! ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
       "${JETSON_HOST}" true 2>"${LOG_DIR}/4_jetson_ssh_preflight.log"; then
    echo "[ERROR] cannot ssh to ${JETSON_HOST} without a password."
    sed -n '1,5p' "${LOG_DIR}/4_jetson_ssh_preflight.log" || true
    echo "        set up key auth once:  ssh-keygen -t ed25519 && ssh-copy-id ${JETSON_HOST}"
    echo "        or drop --jetson and run this on the Jetson yourself:"
    print_jetson_manual_cmd
    exit 1
  fi
  if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "${JETSON_HOST}" \
       "test -f ${JETSON_WS}/install/setup.bash" 2>/dev/null; then
    echo "[WARN] ${JETSON_WS}/install/setup.bash not found on ${JETSON_HOST}."
    echo "       build the workspace there, or set JETSON_WS=<path>."
  fi

  start_proc "4_jetson_trt_infer" \
    "ssh -tt -o BatchMode=yes -o ServerAliveInterval=5 -o ServerAliveCountMax=3 '${JETSON_HOST}' \"bash -lc '${JETSON_CMD}'\""
  if ! wait_for "/avp/detections from Jetson" 120 \
    "ros2 topic list --no-daemon 2>/dev/null | grep -q '^/avp/detections$'"; then
    echo "[WARN] no /avp/detections yet. See ${LOG_DIR}/4_jetson_trt_infer.log"
    echo "       check on both sides: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, same subnet, ros2 node list"
  fi
else
  echo "[INFO] JETSON: run this on the Jetson (separate machine, separate terminal):"
  print_jetson_manual_cmd
  if [ "${WAIT_JETSON_SEC}" -gt 0 ]; then
    if ! wait_for "/avp/detections from Jetson (Ctrl+C to abort)" "${WAIT_JETSON_SEC}" \
      "ros2 topic list --no-daemon 2>/dev/null | grep -q '^/avp/detections$'"; then
      echo "[WARN] no /avp/detections yet; continuing anyway."
      echo "       aeb_node will hold the fail-safe brake until detections arrive."
      echo "       check on both sides: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, same subnet, ros2 node list"
    fi
  fi
fi

if [ "${SKIP_BRIDGE}" = "0" ]; then
  start_proc "5_detection_bridge" \
    "cd '${WS_ROOT}' && '${PYTHON_BIN}' tools/week16_detection_to_autoware_node.py ${BRIDGE_ARGS}"
else
  echo "[INFO] skip detection bridge (--skip-bridge)"
fi

# ============================================================
# 5) AEB node (+ optional evidence recorder)
# ============================================================
if [ "${SKIP_AEB}" = "0" ]; then
  start_proc "6_aeb_node" "ros2 run ${PKG} aeb_node ${AEB_ARGS}"
else
  echo "[INFO] skip aeb_node (--skip-aeb)"
fi

if [ "${RECORD}" = "1" ]; then
  start_proc "7_evidence_recorder" \
    "cd '${WS_ROOT}' && '${PYTHON_BIN}' tools/week16_record_aeb_evidence.py"
fi

# ============================================================
# Summary + supervision
# ============================================================
echo
echo "==================== AVP pipeline up ===================="
for i in "${!PIDS[@]}"; do
  printf "  %-22s pid=%-8s log=%s\n" "${NAMES[$i]}" "${PIDS[$i]}" "${LOG_DIR}/${NAMES[$i]}.log"
done
cat <<MON

  monitor:
    ros2 topic echo /avp/aeb/status
    ros2 topic echo /avp/vehicle/target_accel
    ros2 topic hz   /perception/object_recognition/tracking/objects
    rqt_image_view /avp/infer/overlay
  tune (live):
    ros2 param set /aeb_node break_distance 18.0
    ros2 param set /aeb_node lane_half_width 1.0

  Ctrl+C stops every process started above.
=========================================================
MON

while true; do
  sleep 5
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    [ -n "${pid}" ] || continue
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "[WARN] ${NAMES[$i]} (pid=${pid}) exited. Last log lines:"
      tail -n 10 "${LOG_DIR}/${NAMES[$i]}.log" 2>/dev/null || true
      PIDS[$i]=""
    fi
  done
done
