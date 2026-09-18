# School Ubuntu Migration Handoff

This document rebuilds the project on the school computer without copying the
home virtual machine. The GitHub repository is the source of truth.

## Required platform

- Ubuntu 22.04 (native installation is preferred)
- ROS 2 Humble
- Git and Internet access
- CPU execution is supported; an NVIDIA GPU is optional

The current home reference system is Ubuntu 22.04 `aarch64`. A school
`x86_64` computer does not need to match the CPU architecture, but it should
use ROS 2 Humble and the pinned Python package versions below.

## Verified state before migration

- Repository baseline: commit `4262654`
- Existing UR5 + Robotiq MoveIt task: verified
- ROS 2 to Newton bridge and start-state guard: verified
- MoveIt-to-Newton kinematic shadow execution: verified
- New absolute-object-position MoveIt node: build and fake-hardware execution
  verified on 2026-09-18
- New combined Newton ground-strip contact endpoint: starts correctly and
  passes start-state synchronization
- Physical contact grasp for the new absolute-position workflow: **not yet
  accepted**. MoveIt success proves planning and controller execution, not that
  Newton lifted the strip.

## 1. Install ROS prerequisites

Install ROS 2 Humble according to the official ROS instructions, then install:

```bash
sudo apt update
sudo apt install -y \
  git curl python3-pip python3-venv python3-rosdep \
  python3-colcon-common-extensions python3-vcstool \
  ros-humble-moveit ros-humble-ur \
  ros-humble-ros2-control ros-humble-ros2-controllers
```

## 2. Clone and bootstrap the workspace

```bash
mkdir -p ~/ur5_ws/src
cd ~/ur5_ws/src
git clone https://github.com/technocrat94/ros2-robotics-simulation-lab.git \
  ur5_moveit_demo
cd ~/ur5_ws/src/ur5_moveit_demo
bash scripts/bootstrap_school_ubuntu.sh
```

The script pins the Robotiq source commit, creates the ROS bridge package,
creates a Python 3.12 Newton environment, copies the experiment sources,
builds the workspace, and generates the Newton-specific URDF with resolved
mesh paths.

## 3. Baseline verification

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select newton_ros_bridge ur5_moveit_demo \
  --symlink-install
ros2 pkg executables newton_ros_bridge
ros2 pkg executables ur5_moveit_demo
```

The expected new executable is `pick_at_position`.

## 4. Run the absolute-position contact experiment

Terminal 1 — ROS, MoveIt, controllers, and RViz:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Terminal 2 — ROS adapter:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

Terminal 3 — Newton contact endpoint:

```bash
cd ~/ur5_ws
NEWTON_VIEWER_PORT=8086 \
GRASP_STRIP_CENTER_Z=0.01 \
GRASP_POSITION=center \
GRASP_FRICTION=1.5 \
GRASP_COLLISION_SCOPE=all \
~/newton_ws/lessons/.venv-cpu/bin/python \
  src/newton_ros_bridge/newton_moveit_grasp_endpoint.py
```

Open `http://127.0.0.1:8086/` on the school computer.

Terminal 4 — synchronize, verify, and execute:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /newton/sync_robot_state std_srvs/srv/Trigger
ros2 run newton_ros_bridge start_state_guard
ros2 service call /newton/set_trajectory_shadow \
  std_srvs/srv/SetBool "{data: true}"
ros2 launch ur5_moveit_demo pick_at_position.launch.py
```

Default object center:

```text
world = (0.4869, 0.10915, 0.0100) m
```

The node treats this as an absolute position. MoveIt performs the IK and global
pre-grasp plan, followed by Cartesian descent and lift. The configured
`0.109 m` offset converts between `tool0` and the midpoint of the fingertips.

## Acceptance evidence

Do not accept the experiment from the final MoveIt success line alone. Check:

1. `start_state_guard` passes before execution.
2. MoveIt reports a complete plan and 100% Cartesian descent/lift.
3. The Newton strip rises while the fingers remain closed.
4. The strip is not penetrating the hand or arm.
5. The strip falls only after the gripper opens.
6. The final report records object height, contact count, and any penetration.

## Parameters the engineer should understand

- `object_x/y/z`: absolute object center in the `world` frame
- `pregrasp_clearance`: vertical distance above the object before descent
- `lift_distance`: vertical distance after grasp closure
- `grasp_center_offset`: `tool0` to fingertip-midpoint distance
- `closed_grip`: Robotiq leader-joint command in radians
- `velocity_scale`: MoveIt speed scaling
- `GRASP_FRICTION`: Newton contact friction coefficient

Change one parameter family at a time. Planning failure, collision failure,
and grasp failure are different diagnoses and must be reported separately.

## Current limitation

Newton contains a physical ground plane, but the MoveIt planning scene does
not yet contain the matching floor collision object. Adding the same floor to
MoveIt is the next safety improvement after the first contact run is diagnosed.
