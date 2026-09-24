# Jazzy port status

Last updated: 2026-09-24 (Asia/Taipei)

## Scope and source state

- Machine: Ubuntu 24.04, native ROS 2 Jazzy, amd64.
- Branch: `jazzy-port`.
- Humble reference baseline visible in history: `3925912`.
- Checked-out HEAD before this Jazzy progress commit: `3925912`.
- The pre-existing school changes from `main` are preserved in `stash@{0}` with message `pre-jazzy-port school changes 2026-09-24`; they were not mixed into this port.
- No Docker Humble environment was started. No Humble build, install, log, or ARM virtual environment was copied.

## Completed

- Adapted the UR5 and Robotiq Xacro to the Jazzy UR description macros.
- Configured both UR5 and Robotiq to use `mock_components/GenericSystem`.
- Adapted the SRDF macro invocation and MoveIt controller, kinematics, and OMPL configuration to Jazzy.
- Replaced the obsolete upstream UR MoveIt launch inclusion with a project-owned MoveIt configuration so the combined UR5 and Robotiq model is used consistently.
- Updated the C++ MoveIt plan member names required by MoveIt 2.12.
- Built the three source packages in independent Jazzy output directories.
- Started robot state publication, ros2_control, MoveGroup, and RViz.
- Executed the `move_xyz` fake-hardware pick-and-place motion successfully.

## Rebuild and run

Build from a new terminal:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
colcon --log-base log_jazzy_port build \
  --build-base build_jazzy_port \
  --install-base install_jazzy_port \
  --symlink-install \
  --packages-select robotiq_controllers robotiq_description ur5_moveit_demo
```

Start MoveIt and RViz in terminal 1:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

After MoveGroup prints `You can start planning now!`, run the demonstration in terminal 2:

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo move_xyz.launch.py
```

## Actual test evidence

The Jazzy build completed on 2026-09-24:

```text
Finished <<< robotiq_controllers
Finished <<< robotiq_description
Finished <<< ur5_moveit_demo
Summary: 3 packages finished
```

At runtime, all required controllers were active:

```text
robotiq_activation_controller  active
joint_trajectory_controller    active
robotiq_gripper_controller     active
joint_state_broadcaster        active
```

MoveGroup loaded the KDL solver and OMPL planning pipeline. RViz reported that it was ready for the `ur_manipulator` planning group. The `move_xyz` run then reported:

```text
UR5 reached 'test_configuration'.
Approach Cartesian path completed: 100.0%
Approach Cartesian motion completed.
Gripper reached position 0.7929
Lift and transport Cartesian path completed: 100.0%
Lift and transport Cartesian motion completed.
Return to work start Cartesian path completed: 100.0%
PICK AND PLACE DEMO SUCCEEDED
```

This evidence establishes native Jazzy MoveIt planning and fake-hardware execution with visible RViz motion. It does not establish Newton contact physics or a successful physical ground grasp.

## Unresolved

- `~/newton_ws` is absent on this machine, so Newton, the ROS–Newton bridge, the segmented strip, and the Newton viewer cannot yet be started here.
- `pick_at_position` has been ported to the Jazzy API and builds, but its end-to-end `/newton/object_pose` workflow has not been tested because Newton is absent.
- The Humble reference still has an unresolved ground-lift failure: bilateral contact exists, but the strip remains on the floor. Jazzy must not treat MoveIt `SUCCEEDED` as proof of a physical grasp.
- Jazzy warns that `ROS_LOCALHOST_ONLY` is deprecated. It is still honored and currently prevents discovery outside this host; migrating the isolation script to the newer discovery settings remains future cleanup.

## Next step

Populate `~/newton_ws` only from the committed Newton source and environment specification, create a fresh amd64 environment there, and verify that the Newton viewer can display the segmented-strip scene before connecting the bridge. Continue ground-grasp force analysis only after the bridge reproduces the Humble baseline.
