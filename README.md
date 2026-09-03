# UR5 + Robotiq 2F-85 MoveIt Demo

ROS 2 Humble learning project that combines a UR5 arm and a Robotiq 2F-85 gripper into one robot description, one `ros2_control` bringup, and one sequential MoveIt task.

The current version uses fake hardware and RViz. It does not simulate gravity, contact, friction, or grasped-object physics.

## Verified milestone

On 2026-09-03, the complete task was verified in RViz with all four controllers active. The approach, lift/transport, and automatic-return Cartesian paths each reached 100%, and the process finished cleanly.

```text
UR5 motion planning + Robotiq action control + automatic Cartesian return
                         -> PICK AND PLACE DEMO SUCCEEDED
```

## Portfolio map

- [Learning journey](docs/LEARNING_JOURNEY.md) — how the project progressed from a UTM environment to a verified integrated demo
- [Operations guide](docs/OPERATIONS_GUIDE.md) — repeatable startup, execution, and parameter-editing instructions
- [Troubleshooting record](docs/TROUBLESHOOTING.md) — failures, diagnostic evidence, fixes, and engineering lessons

## Current demo

The C++ node executes this sequence:

```text
open gripper
→ move to test_configuration
→ Cartesian approach
→ close gripper
→ Cartesian lift and transport
→ release
→ automatically return to the work-start pose
```

Stable relative-motion parameters:

```text
Approach:           (+0.03, 0.00, 0.00) m
Lift and transport: (-0.03, 0.00,+0.05) m
Automatic return:  ( 0.00, 0.00,-0.05) m
```

All relative translations use the MoveIt planning frame (`world`), not the gripper's local frame. Orientation is held constant.

## Implemented components

- Combined UR5 + Robotiq Xacro model
- UR-to-Robotiq adapter attached at `tool0`
- Combined `ros2_control` controller configuration
- UR5 `joint_trajectory_controller`
- Robotiq activation and gripper action controllers
- Combined `/joint_states`
- Custom SRDF collision exclusions for expected Robotiq internal contacts
- MoveIt named-target planning for `test_configuration`
- Cartesian approach, transport, and return motions
- Gripper open/close sequencing through `GripperCommand`
- Cartesian completion and excessive-joint-travel guards
- RViz configuration

## Environment

- Ubuntu 22.04 (aarch64, running in UTM)
- ROS 2 Humble
- MoveIt 2
- Universal Robots ROS 2 packages
- Robotiq ROS 2 packages from [`robotiq/ros`](https://github.com/robotiq/ros)

## Workspace layout

This repository is intended to live at:

```text
~/ur5_ws/src/ur5_moveit_demo
```

The Robotiq dependency is a separate repository in the same workspace:

```text
~/ur5_ws/src/ros
```

Do not copy this project's `build`, `install`, or `log` directories into Git.

## Build

From the workspace root:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select ur5_moveit_demo \
  --symlink-install

source install/setup.bash
```

## Run

Open two terminals.

Terminal 1 — launch the robot, controllers, MoveIt, and RViz:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Wait until the four controllers are active and RViz shows the combined robot.

Terminal 2 — run the task:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur5_moveit_demo move_xyz.launch.py
```

A successful run ends with:

```text
PICK AND PLACE DEMO SUCCEEDED
process has finished cleanly
```

## Change task motion

Edit:

```text
src/move_xyz.cpp
```

Search for `APPROACH PARAMETERS` to change the pre-grasp translation:

```cpp
approach_dx = 0.03;
approach_dy = 0.0;
approach_dz = 0.0;
```

Search for `LIFT AND TRANSPORT PARAMETERS` to change the post-grasp translation:

```cpp
transport_dx = -0.03;
transport_dy = 0.0;
transport_dz = 0.05;
```

The return translation is calculated automatically:

```text
Return = -(Approach + Transport)
```

The values are relative translations in metres. Changing them can make a path unreachable or unsafe; do not lower the 99% Cartesian completion requirement to force an incomplete path to execute.

## Gripper settings

The demonstrated fake-hardware values are:

```text
Open position:  0.0
Closed position: 0.7929
Maximum effort: 50.0
```

`max_effort` in fake hardware is not a calibrated physical force. Revalidate position, effort, speed, collision geometry, and safety limits before using real hardware.

## Safety checks

The task rejects:

- Cartesian paths with less than 99% completion
- Empty or malformed trajectories
- Small Cartesian motions whose total movement exceeds 1 rad on any joint
- Rejected, stalled, or timed-out gripper actions

RViz's grid is visual only. It is not a collision floor. A real table, floor, and task objects must be added to the MoveIt Planning Scene before physical deployment.

## Important files

```text
src/move_xyz.cpp                         task sequence and safety checks
launch/ur5_robotiq_bringup.launch.py    combined bringup
launch/move_xyz.launch.py               task-node launch
urdf/ur5_robotiq.urdf.xacro             combined robot model
srdf/ur5_robotiq.srdf.xacro             MoveIt semantics and collisions
config/combined_controllers.yaml         ros2_control controllers
config/controllers.yaml                  MoveIt execution controllers
rviz/view_robot.rviz                     RViz display configuration
```

## Known limitations

- Fake hardware only
- No Gazebo physics
- No object attach/detach
- No floor or table collision object
- Motion offsets are expressed in `world`, not a gripper TCP frame
- MoveIt's current end-effector link is `tool0`, not a dedicated grasp-centre frame

## License

Apache-2.0, matching `package.xml`.
