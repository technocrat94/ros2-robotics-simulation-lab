#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${HOME}/newton_ws/.venv-cpu"
ASSET="${HOME}/newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf"
ENDPOINT="${HOME}/ur5_ws/src/newton_ros_bridge/newton_moveit_grasp_endpoint.py"

for required in "${VENV_DIR}/bin/python" "${ASSET}" "${ENDPOINT}"; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: missing ${required}; run scripts/bootstrap_school_jazzy.sh first." >&2
    exit 1
  fi
done

source "${REPO_DIR}/scripts/lab_session_env.sh"
source /opt/ros/jazzy/setup.bash
source "${HOME}/ur5_ws/install_jazzy_port/setup.bash"
set -u

export GRASP_FEM_CELLS_X="${GRASP_FEM_CELLS_X:-40}"
export GRASP_FEM_CELLS_Y="${GRASP_FEM_CELLS_Y:-2}"
export GRASP_FEM_CELLS_Z="${GRASP_FEM_CELLS_Z:-2}"
echo "NEWTON_DEVICE ${GRASP_DEVICE}"
export NEWTON_ROBOT_URDF="${ASSET}"
export GRASP_OBJECT_MODEL="fem_cable"
export GRASP_OBJECT_LENGTH="0.40"
export GRASP_OBJECT_WIDTH="0.006"
export GRASP_OBJECT_THICKNESS="0.006"
export GRASP_STRIP_CENTER_Z="0.004"
export GRASP_POSITION="center"
export GRASP_FRICTION="10"
export GRASP_STRIP_FRICTION="10"
export GRASP_GROUND_FRICTION="1.5"
export GRASP_COLLISION_SCOPE="all"
export GRASP_DENSITY="1100.0"
export GRASP_YOUNGS_MODULUS="1000000.0"
export GRASP_POISSON_RATIO="0.45"
export GRASP_SOFT_DAMPING="1000.0"
export GRASP_SOFT_CONTACT_KE="1000.0"
export GRASP_SOFT_CONTACT_KD="10.0"
export GRASP_SOFT_CONTACT_MARGIN="0.005"
export GRASP_FEM_PROXY_INSET="0.001"
export GRASP_MAX_ALLOWED_PENETRATION="0.005"
export GRASP_SUBSTEPS="10"
export NEWTON_COMMAND_DT="0.02"
export NEWTON_MAX_HOLD_SECONDS="0.5"
export NEWTON_MAX_COMMAND_QUEUE="4096"

mkdir -p "${HOME}/ur5_ws/run_logs"
ENDPOINT_LOG="${HOME}/ur5_ws/run_logs/jazzy_fem_cable_endpoint_$(date +%Y%m%d_%H%M%S).log"
echo "FEM_CABLE_MESH ${GRASP_FEM_CELLS_X}x${GRASP_FEM_CELLS_Y}x${GRASP_FEM_CELLS_Z}"
echo "NEWTON_FEM_CABLE_VIEWER_URL http://127.0.0.1:${NEWTON_VIEWER_PORT}/"
echo "FEM_CABLE_ENDPOINT_LOG ${ENDPOINT_LOG}"

set +e
"${VENV_DIR}/bin/python" "${ENDPOINT}" 2>&1 | tee "${ENDPOINT_LOG}"
status=${PIPESTATUS[0]}
set -e
exit "${status}"
