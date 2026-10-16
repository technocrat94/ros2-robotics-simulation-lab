#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
CUROBO_DIR="${CUROBO_DIR:-${REPO_DIR}/curobo}"
CUROBO_ENV="${CUROBO_ENV:-${HOME}/curobo_ros_env}"
PYTHON_ENTRY="${REPO_DIR}/docs/experiments/curobo-v2/prototype/curobo_step_c_pick.py"

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

export GRASP_SCENE_PROFILE="step_c_cable_wall"
export MOVEIT_ROBOT_PADDING_M="0.02"
export MOVEIT_CONTACT_PADDING_M="0.0"
export MOVEIT_PLANNING_TIME_SECONDS="10.0"
export FEM_CABLE_SAFE_PREGRASP="true"

source "${REPO_DIR}/scripts/lab_session_env.sh"
set +u
source /opt/ros/jazzy/setup.bash
source "${ROS_WS}/install_jazzy_port/setup.bash"
source "${HOME}/use_curobo_env.sh"
source "${CUROBO_ENV}/bin/activate"
set -u

export PYTHONPATH="${CUROBO_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
export CUROBO_SOURCE_DIR="${CUROBO_DIR}"
mkdir -p "${ROS_WS}/run_logs"
PICK_LOG="${ROS_WS}/run_logs/curobo_step_c_pick_$(date +%Y%m%d_%H%M%S).log"

cleanup_shadow() {
  ros2 service call /newton/set_trajectory_shadow \
    std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
}
trap cleanup_shadow EXIT

ros2 topic echo /newton/bridge_status --once
ros2 service call /newton/sync_robot_state std_srvs/srv/Trigger "{}"
sleep 1
ros2 run newton_ros_bridge start_state_guard
ros2 service call /newton/set_trajectory_shadow \
  std_srvs/srv/SetBool "{data: true}"

set +e
"${CUROBO_ENV}/bin/python" "${PYTHON_ENTRY}" 2>&1 | tee "${PICK_LOG}"
status=${PIPESTATUS[0]}
set -e

echo "CUROBO_STEP_C_PICK_LOG ${PICK_LOG}"
exit "${status}"
