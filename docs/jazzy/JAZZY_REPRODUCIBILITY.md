# Reproducible Humble-to-Jazzy FEM Pick

This is the source-based recipe for the verified native ROS 2 Jazzy FEM ground pick on Ubuntu 24.04 amd64. Humble is the behavioral reference; Humble build, install, log, and virtual-environment artifacts are not copied.

## Source of truth

The tracked repository contains the modified C++, Python, shell, launch, Xacro, YAML, and configuration sources. `scripts/bootstrap_school_jazzy.sh` copies the tracked Newton prototypes into the runtime workspaces, regenerates the Jazzy URDF, builds the four ROS packages, and verifies mesh paths.

## Build

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
git fetch origin
git switch jazzy-final-fem-pick
PIP_NO_INDEX=1 ./scripts/bootstrap_school_jazzy.sh
```

## Launch exactly one stack per ROS domain

Terminal 1 — Newton endpoint and viewer:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_ground_grasp_jazzy.sh
```

Terminal 2 — Jazzy MoveIt/controllers:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Terminal 3 — ROS/Newton adapter:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

Open the printed viewer URL, normally `http://127.0.0.1:30000/`.

## Preview, then full pick

Preview is intentionally plan-only and sends no robot command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_fem_preview.sh
```

After `SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED`, execute the complete action:

```bash
./scripts/run_jazzy_fem_pick.sh
```

The full action opens, approaches, closes to about `0.375145 rad`, lifts `0.120 m` at scale `0.030`, and reopens. Timestamped logs are written to `~/ur5_ws/run_logs/`.

## Acceptance

Require bridge `OK`, start-state guard pass, all Cartesian fractions `100.0%`, `ABSOLUTE POSITION PICK SUCCEEDED`, no named-start or `test_configuration` line, and finite Newton state. In the viewer, verify that the strip stays between the fingers during lift and drops only after reopening.

## Troubleshooting

If nothing moves, you probably ran preview. If `execute_action_client_ client/server not ready` appears, stop the duplicate bringup/move_group owned by the current user and keep exactly one stack in the isolated domain.
