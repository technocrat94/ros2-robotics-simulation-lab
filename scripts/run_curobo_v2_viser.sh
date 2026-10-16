#!/usr/bin/env bash
set -euo pipefail

printf '\033]0;cuRobo V2 + Viser (offline UR5)\007'

REPO_DIR="${HOME}/ur5_ws/src/ur5_moveit_demo"
CUROBO_DIR="${REPO_DIR}/curobo"
CUROBO_ENV="${HOME}/curobo_ros_env"
PYTHON_ENTRY="${REPO_DIR}/docs/experiments/curobo-v2/prototype/ur5_viser_preflight.py"

if [[ ! -d "${CUROBO_DIR}/.git" ]]; then
  echo "ERROR: cuRoboV2 checkout not found at ${CUROBO_DIR}" >&2
  exit 1
fi

if [[ ! -x "${CUROBO_ENV}/bin/python" ]]; then
  echo "ERROR: Python environment not found at ${CUROBO_ENV}" >&2
  exit 1
fi

if ! nvidia-smi --query-gpu=name --format=csv,noheader >/dev/null; then
  echo "ERROR: NVIDIA GPU is unavailable; refusing CPU fallback." >&2
  exit 1
fi

# ROS setup files read optional variables that may legitimately be unset.
set +u
source /opt/ros/jazzy/setup.bash
source "${HOME}/ur5_ws/install_jazzy_port/setup.bash"
source "${HOME}/use_curobo_env.sh"
source "${CUROBO_ENV}/bin/activate"
set -u

export PYTHONPATH="${CUROBO_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export CUROBO_SOURCE_DIR="${CUROBO_DIR}"

cd "${REPO_DIR}"
exec python "${PYTHON_ENTRY}" "$@"
