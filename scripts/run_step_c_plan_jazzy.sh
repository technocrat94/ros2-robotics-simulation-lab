#!/usr/bin/env bash
set -eo pipefail

if [[ "$#" -ne 1 || ( "$1" != "preflight" && "$1" != "c2" ) ]]; then
  echo "Usage: $0 preflight|c2" >&2
  exit 2
fi

CASE="$1"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
PADDING="${MOVEIT_ROBOT_PADDING_M:-0.02}"
TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"
LOG_DIR="${ROS_WS}/run_logs"
mkdir -p "${LOG_DIR}"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing Step C plan evidence from a dirty git tree." >&2
  exit 3
fi

if [[ "${CASE}" == "preflight" ]]; then
  PROFILE="step_c_cable_wall"
else
  PROFILE="step_c_cable_tall_wall"
fi
LOG="${LOG_DIR}/step_c_${CASE}_$(date +%Y%m%d_%H%M%S).log"
RESULT_JSON="${LOG%.log}.result.json"

export GRASP_SCENE_PROFILE="${PROFILE}"
export MOVEIT_ROBOT_PADDING_M="${PADDING}"
source "${REPO_DIR}/scripts/lab_session_env.sh"
source /opt/ros/jazzy/setup.bash
source "${ROS_WS}/install_jazzy_port/setup.bash"
set -u

echo "STEP_C_PLAN_CASE ${CASE}" | tee "${LOG}"
echo "GRASP_SCENE_PROFILE ${PROFILE}" | tee -a "${LOG}"
echo "MOVEIT_ROBOT_PADDING_M ${PADDING}" | tee -a "${LOG}"
set +e
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_newton_object_pose:=false \
  plan_only:=true \
  use_safe_pregrasp_path:=false \
  use_named_start:=false \
  2>&1 | tee -a "${LOG}"
status=${PIPESTATUS[0]}
set -e

if [[ "${CASE}" == "preflight" ]]; then
  if [[ "${status}" -ne 0 ]] || ! grep -q "PLAN_ONLY_SUCCEEDED" "${LOG}"; then
    echo "ERROR: proposed Step C cable pose is not reachable." >&2
    exit 1
  fi
  echo "STEP_C_MOVEIT_REACHABILITY_PASS tested_commit=${TESTED_COMMIT} log=${LOG}"
  exit 0
fi

python3 "${REPO_DIR}/scripts/extract_step_c_result.py" \
  --case c2 --pick-log "${LOG}" --output "${RESULT_JSON}" \
  --tested-commit "${TESTED_COMMIT}" --profile "${PROFILE}" \
  --padding "${PADDING}"
