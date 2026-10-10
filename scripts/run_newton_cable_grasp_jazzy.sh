#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${HOME}/newton_ws/.venv-cpu"
ASSET="${HOME}/newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf"
ENDPOINT="${HOME}/ur5_ws/src/newton_ros_bridge/newton_moveit_grasp_endpoint.py"
RUNTIME_CABLE="${HOME}/newton_ws/lessons/segmented_strip/rod_cable_topology.py"

for required in \
  "${VENV_DIR}/bin/python" "${ASSET}" "${ENDPOINT}" "${RUNTIME_CABLE}"; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: missing ${required}; run scripts/bootstrap_school_jazzy.sh first." >&2
    exit 1
  fi
done

source "${REPO_DIR}/scripts/lab_session_env.sh"
source /opt/ros/jazzy/setup.bash
source "${HOME}/ur5_ws/install_jazzy_port/setup.bash"
set -u

if [[ -z "${GRASP_DEVICE:-}" ]]; then
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
    export GRASP_DEVICE="cuda:0"
  else
    export CUDA_VISIBLE_DEVICES=""
    export GRASP_DEVICE="cpu"
  fi
fi
echo "NEWTON_DEVICE ${GRASP_DEVICE}"
export NEWTON_ROBOT_URDF="${ASSET}"
export GRASP_OBJECT_MODEL="rod_cable"
# Radius 3 mm plus the validated 1 mm contact gap places the cable on the floor.
export GRASP_STRIP_CENTER_Z="0.004"
export GRASP_POSITION="center"
export GRASP_FRICTION="1.0"
export GRASP_STRIP_FRICTION="1.0"
export GRASP_GROUND_FRICTION="1.0"
export GRASP_COLLISION_SCOPE="all"
export GRASP_CONTACT_PROXY_INSET="0.001"
export GRASP_SHOW_CONTACT_PROXIES="0"
export GRASP_MAX_ALLOWED_PENETRATION="0.005"
export GRASP_SUBSTEPS="10"
export NEWTON_COMMAND_DT="0.02"
export NEWTON_MAX_HOLD_SECONDS="0.5"
export NEWTON_MAX_COMMAND_QUEUE="4096"

mkdir -p "${HOME}/ur5_ws/run_logs"
ENDPOINT_LOG="${HOME}/ur5_ws/run_logs/jazzy_cable_endpoint_$(date +%Y%m%d_%H%M%S).log"
echo "NEWTON_CABLE_VIEWER_URL http://127.0.0.1:${NEWTON_VIEWER_PORT}/"
echo "CABLE_ENDPOINT_LOG ${ENDPOINT_LOG}"

set +e
"${VENV_DIR}/bin/python" "${ENDPOINT}" 2>&1 | tee "${ENDPOINT_LOG}"
status=${PIPESTATUS[0]}
set -e
exit "${status}"
