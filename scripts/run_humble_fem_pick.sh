#!/usr/bin/env bash
set -euo pipefail

cd "${HOME}/ur5_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p run_logs
log="run_logs/humble_fem_pick_$(date +%Y%m%d_%H%M%S).log"

ros2 topic echo /newton/bridge_status --once
ros2 service call /newton/sync_robot_state std_srvs/srv/Trigger "{}"
sleep 1
ros2 run newton_ros_bridge start_state_guard
ros2 service call /newton/set_trajectory_shadow \
  std_srvs/srv/SetBool "{data: true}"

set +e
ros2 launch ur5_moveit_demo pick_at_position.launch.py \
  use_named_start:=true \
  use_safe_pregrasp_path:=true \
  safe_transit_margin:=0.050 \
  maximum_pregrasp_joint_travel:=1.600 \
  maximum_wrist_3_travel:=3.250 \
  use_width_calibration:=true \
  object_width_mm:=50 \
  total_compression_mm:=2 \
  uncompensated_approach_distance:=0.105 \
  lift_distance:=0.120 \
  lift_velocity_scale:=0.030 \
  2>&1 | tee "${log}"
status=${PIPESTATUS[0]}
set -e

ros2 service call /newton/set_trajectory_shadow \
  std_srvs/srv/SetBool "{data: false}" || true

echo "LOG_SAVED ${HOME}/ur5_ws/${log}"
exit "${status}"
