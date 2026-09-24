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
export CUDA_VISIBLE_DEVICES=""
export NEWTON_ROBOT_URDF="${ASSET}"
export GRASP_STRIP_CENTER_Z="0.01"
export GRASP_POSITION="center"
export GRASP_FRICTION="1.5"
export GRASP_COLLISION_SCOPE="all"

exec "${VENV_DIR}/bin/python" "${ENDPOINT}"
