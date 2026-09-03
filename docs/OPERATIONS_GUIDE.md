# Operations and Parameter Guide

This guide contains only the repeatable operating steps. The default workspace is `~/ur5_ws` in the Ubuntu UTM virtual machine.

## Start the system after boot

### Terminal 1: start the integrated bringup

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Keep this terminal open. Wait for RViz to show the combined UR5 and Robotiq model.

### Terminal 2: verify the controllers

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

All four controllers should be `active`:

```text
joint_state_broadcaster
joint_trajectory_controller
robotiq_activation_controller
robotiq_gripper_controller
```

### Terminal 2: execute the task

```bash
ros2 launch ur5_moveit_demo move_xyz.launch.py
```

A successful run ends with:

```text
PICK AND PLACE DEMO SUCCEEDED
process has finished cleanly
```

## Edit motion parameters

Open the task source:

```bash
nano ~/ur5_ws/src/ur5_moveit_demo/src/move_xyz.cpp
```

### Pre-grasp approach

Search for `APPROACH PARAMETERS`:

```cpp
const double approach_dx = 0.03;
const double approach_dy = 0.0;
const double approach_dz = 0.0;
```

All three axes may be changed. Units are metres; `0.03` means 3 cm.

### Post-grasp lift and transport

Search for `LIFT AND TRANSPORT PARAMETERS`:

```cpp
const double transport_dx = -0.03;
const double transport_dy = 0.0;
const double transport_dz = 0.05;
```

This stage may combine X, Y, and Z translation.

### Automatic return

There is no third displacement to maintain manually. The program calculates:

```text
return_dx = -(approach_dx + transport_dx)
return_dy = -(approach_dy + transport_dy)
return_dz = -(approach_dz + transport_dz)
```

This returns to the Cartesian work-start pose only while all relative stages use the same planning frame and no additional unaccounted motion is inserted.

### Gripper settings

Search for `OPEN GRIPPER` or `CLOSE GRIPPER` and inspect the related `command_gripper()` call.

```text
0.0     open
0.7929  closed
50.0    fake-hardware max_effort
```

For a larger simulated object, test a smaller closed-position value such as `0.4` and adjust gradually. Fake-hardware effort is not a calibrated physical force.

### Arm speed and acceleration

Search for:

```cpp
setMaxVelocityScalingFactor(0.2)
setMaxAccelerationScalingFactor(0.2)
```

`0.2` means 20% of the configured maximum. Physical-hardware validation should begin conservatively and follow the site's safety procedure.

## Rebuild after a source change

Save in nano with `Ctrl+O`, Enter, and exit with `Ctrl+X`.

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select ur5_moveit_demo \
  --symlink-install

source install/setup.bash
```

Keep the integrated bringup running and launch the task again from Terminal 2.

## Operating rules

- `dx`, `dy`, and `dz` currently use the `world` frame, not the gripper frame.
- Change offsets in small increments, initially around 1–3 cm.
- If Cartesian completion is below 99%, improve the pose or reduce the offset; do not lower the guard to force execution.
- The RViz grid is visual only and is not a collision floor.
- Revalidate TCP, collision geometry, speed, acceleration, gripper force, and emergency procedures before physical deployment.

## Shutdown

The task node exits automatically after success. Press `Ctrl+C` in Terminal 1 when the complete bringup should be stopped.
