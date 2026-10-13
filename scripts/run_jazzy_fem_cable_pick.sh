#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${HOME}/ur5_ws"
SAFE_PREGRASP="${FEM_CABLE_SAFE_PREGRASP:-true}"

source "${REPO_DIR}/scripts/lab_session_env.sh"
source /opt/ros/jazzy/setup.bash
source "${ROS_WS}/install_jazzy_port/setup.bash"
set -u

mkdir -p "${ROS_WS}/run_logs"
PICK_LOG="${ROS_WS}/run_logs/jazzy_fem_cable_pick_$(date +%Y%m%d_%H%M%S).log"

ros2 topic echo /newton/bridge_status --once
ros2 service call /newton/sync_robot_state std_srvs/srv/Trigger "{}"
sleep 1
ros2 run newton_ros_bridge start_state_guard
ros2 service call /newton/set_trajectory_shadow std_srvs/srv/SetBool "{data: true}"

set +e
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_named_start:=false \
  use_safe_pregrasp_path:="${SAFE_PREGRASP}" \
  safe_transit_margin:=0.050 \
  maximum_pregrasp_joint_travel:=1.600 \
  maximum_wrist_3_travel:=3.250 \
  use_width_calibration:=true \
  object_width_mm:=6 \
  total_compression_mm:=2 \
  uncompensated_approach_distance:=0.105 \
  lift_distance:=0.120 \
  lift_velocity_scale:=0.030 \
  2>&1 | tee "${PICK_LOG}"
status=${PIPESTATUS[0]}
set -e

ros2 service call /newton/set_trajectory_shadow std_srvs/srv/SetBool "{data: false}" || true
echo "FEM_CABLE_PICK_LOG ${PICK_LOG}"
exit "${status}"
