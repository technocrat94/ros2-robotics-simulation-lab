#!/usr/bin/env bash
set -euo pipefail

printf '\033]0;cuRobo route-crossing dodge\007'

REPO_DIR="${HOME}/ur5_ws/src/ur5_moveit_demo"
CUROBO_DIR="${CUROBO_DIR:-${REPO_DIR}/curobo}"
CUROBO_ENV="${CUROBO_ENV:-${HOME}/curobo_ros_env}"
PYTHON_ENTRY="${REPO_DIR}/docs/experiments/curobo-v2/prototype/curobo_dodge_box_demo.py"

for required in \
  "${CUROBO_DIR}/curobo/__init__.py" \
  "${CUROBO_ENV}/bin/python" \
  "${PYTHON_ENTRY}"; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: missing ${required}" >&2
    exit 1
  fi
done

if ! nvidia-smi --query-gpu=name --format=csv,noheader >/dev/null; then
  echo "ERROR: NVIDIA GPU is unavailable; refusing CPU fallback." >&2
  exit 1
fi

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
