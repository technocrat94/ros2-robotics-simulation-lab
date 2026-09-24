# Native Jazzy Newton runbook

This runbook uses native ROS 2 Jazzy and an amd64 Python 3.12 Newton environment. It does not use Docker or ROS 2 Humble.

## One-time reconstruction

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/bootstrap_school_jazzy.sh
```

The script only writes under `~/ur5_ws` and `~/newton_ws`. It creates `~/newton_ws/.venv-cpu`, copies the tracked Newton and bridge source, builds the Jazzy packages in the `*_jazzy_port` output directories, and generates a Newton URDF with resolved mesh paths.

## Ground-grasp experiment

Run exactly one MoveIt bringup. Starting a second bringup in the same ROS domain creates duplicate action servers and can cause `unknown goal response` or preempted execution.

Terminal 1 — MoveIt, ros2_control, and RViz:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Terminal 2 — ROS–Newton adapter:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

Terminal 3 — Newton ground-grasp endpoint and localhost viewer:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_ground_grasp_jazzy.sh
```

Open `http://127.0.0.1:30000`. The script forces CPU execution and supplies the verified ground-scene parameters, including strip center `z=0.01 m`, center grasp, friction `1.5`, and all robot colliders.

Terminal 4 — synchronize and execute:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash

ros2 service call /newton/sync_robot_state std_srvs/srv/Trigger '{}'
ros2 run newton_ros_bridge start_state_guard
ros2 service call /newton/set_trajectory_shadow \
  std_srvs/srv/SetBool '{data: true}'
ros2 launch ur5_moveit_demo pick_at_position.launch.py
```

A successful service response only means the command was sent. Confirm that the guard passes, `/newton/bridge_status` is `OK`, MoveIt paths are complete, and the Newton endpoint's final `MOVEIT_GRASP_RESULT` reports the physical measurements.

Stop each foreground process with `Ctrl+C`. Stopping the Newton endpoint prints the final result JSON.
