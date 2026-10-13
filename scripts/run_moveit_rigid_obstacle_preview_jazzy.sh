#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${HOME}/ur5_ws/run_logs"
mkdir -p "${LOG_DIR}"
LOG="${LOG_DIR}/rigid_obstacle_step_b_$(date +%Y%m%d_%H%M%S).log"
RESULT_JSON="${LOG%.log}.result.json"
TESTED_COMMIT="$(git -C "${REPO_DIR}" rev-parse HEAD)"

if [[ -n "$(git -C "${REPO_DIR}" status --porcelain --untracked-files=normal)" ]]; then
  echo "ERROR: refusing Step B result capture from a dirty git tree." >&2
  exit 3
fi

source "${REPO_DIR}/scripts/lab_session_env.sh"
source /opt/ros/jazzy/setup.bash
source "${HOME}/ur5_ws/install_jazzy_port/setup.bash"
set -u

echo "RIGID_OBSTACLE_STEP_B_LOG ${LOG}"
exec > >(tee "${LOG}") 2>&1

echo "STEP_B_NEGATIVE_CONTROL_DISABLED"
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_newton_object_pose:=false plan_only:=true use_safe_pregrasp_path:=false \
  obstacles_enabled:=false object_x:=0.20

echo "STEP_B_ENABLED_SAME_TARGET"
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_newton_object_pose:=false plan_only:=true use_safe_pregrasp_path:=false \
  obstacles_enabled:=true object_x:=0.20

echo "STEP_B_THREE_SPATIAL_STAGE_PREVIEW"
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_newton_object_pose:=false plan_only:=true obstacles_enabled:=true \
  object_x:=0.20 safe_transit_margin:=0.15

echo "STEP_B_TALL_OBSTACLE_NEGATIVE_CONTROL"
set +e
tall_output="$(ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_newton_object_pose:=false plan_only:=true use_safe_pregrasp_path:=false \
  obstacles_enabled:=true object_x:=0.20 \
  obstacle_z:=1.0 obstacle_size_x:=0.10 obstacle_size_y:=4.0 obstacle_size_z:=2.0 2>&1)"
launch_status=$?
set -e
printf '%s\n' "${tall_output}"
if grep -q "Absolute pre-grasp planning failed" <<<"${tall_output}" && \
   ! grep -q "PLAN_ONLY_SUCCEEDED" <<<"${tall_output}"; then
  tall_failed=true
else
  tall_failed=false
fi
echo "TALL_OBSTACLE_LAUNCH_EXIT_STATUS ${launch_status}"
echo "TALL_OBSTACLE_PLANNING_FAILED ${tall_failed}"
if [[ "${tall_failed}" != "true" ]]; then
  echo "ERROR: unreachable tall obstacle silently allowed a plan." >&2
  exit 1
fi

# Last, in the same bringup, prove disabled mode removes the persistent id.
echo "STEP_B_DISABLED_CLEANUP_DEFAULT_PICK"
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_newton_object_pose:=false plan_only:=true obstacles_enabled:=false

python3 "${REPO_DIR}/scripts/extract_rigid_obstacle_plan_result.py" \
  --log "${LOG}" --output "${RESULT_JSON}" --tested-commit "${TESTED_COMMIT}"
