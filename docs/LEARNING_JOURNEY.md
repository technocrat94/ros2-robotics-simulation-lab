# Learning Journey: From UTM to an Integrated UR5 and Robotiq Demo

[繁體中文版本](LEARNING_JOURNEY.zh-TW.md)

## Project objective

The goal was not merely to display a robot in RViz. It was to integrate a UR5 arm, a Robotiq 2F-85 gripper, `ros2_control`, and MoveIt into one reproducible system that can execute a guarded pick-and-place-style sequence.

```text
open gripper
-> move to a known work-start configuration
-> Cartesian approach
-> close gripper
-> retreat and lift
-> release
-> automatically return to the work-start pose
```

## 1. Building the development environment

The project runs in an Ubuntu 22.04 aarch64 virtual machine hosted by UTM on macOS. ROS 2 Humble and MoveIt 2 run inside Ubuntu, while the host machine provides the surrounding development environment.

This stage established several fundamentals:

- the roles of `src`, `build`, `install`, and `log` in a ROS workspace;
- why every new shell must source ROS and the workspace overlay;
- the difference between source files and installed package resources;
- why C++ changes require rebuilding and sourcing the new install space.

## 2. Validating the gripper independently

Before integration, the Robotiq subsystem was tested on its own. This reduced the search space: if the gripper controller and action worked independently, later failures were more likely to be in the combined model, MoveIt configuration, or execution mapping.

The following were verified:

- `robotiq_gripper_controller` was active;
- `robotiq_activation_controller` was active;
- the `/robotiq_gripper_controller/gripper_cmd` action server was available;
- position `0.0` opened the gripper;
- position `0.7929` closed the gripper;
- `/joint_states` contained `robotiq_85_left_knuckle_joint`.

## 3. Building one combined robot model

The `ur5_robotiq.urdf.xacro` file combines the UR5, the UR-to-Robotiq adapter, and the 2F-85 gripper in one kinematic tree attached at `tool0`.

URDF/Xacro acts like the robot's anatomical drawing: links are body segments, joints connect them, and transmissions/control interfaces describe how motion is exposed. Keeping the arm and gripper in unrelated descriptions would prevent MoveIt from treating them as one robot.

An early Xacro generation attempt failed with `Undefined substitution argument name`. The missing argument/default was corrected, and the generated URDF was checked for the UR5 `tool0`, the Robotiq base link, and both `ros2_control` definitions before launching the whole system.

## 4. Integrating ros2_control and robot state

The combined bringup uses a consistent controller setup:

- `joint_state_broadcaster`;
- `joint_trajectory_controller`;
- `robotiq_activation_controller`;
- `robotiq_gripper_controller`.

After integration, `/joint_states` contained the six UR5 joints and the gripper joint. This gave MoveIt one coherent view of the complete robot instead of separate arm and gripper state streams.

## 5. Aligning MoveIt's semantic and execution models

URDF describes physical structure, while SRDF describes semantics: planning groups, named states, and collision relationships. The task originally aborted while planning to `test_configuration` and reported MoveIt error `-26`.

The task launch was corrected to use this project's combined robot description, SRDF, kinematics, and controller mapping instead of a semantic model that represented only the original UR robot. Planning then succeeded.

This also demonstrated the distinction between two layers:

- planning success means a valid trajectory was generated;
- execution success means an active controller accepted and completed it.

A trajectory that plans successfully but is immediately aborted during execution still indicates a system integration problem. MoveIt's controller mapping must match the actual active controller.

## 6. Understanding RViz and coordinate frames

RViz initially used a nonexistent `map` fixed frame, so the robot disappeared. Selecting the available `world` frame restored the display.

The orange robot in the MoveIt display represents a goal or planned state, while the grey robot represents the current state. Seeing both is not evidence of two physical robots or a failed execution.

The current `dx`, `dy`, and `dz` offsets use the `world` planning frame. This is the difference between saying “move north” and “move forward”: world directions do not rotate with the gripper. A future tool-relative approach will require a TCP/tool-frame transform.

## 7. Replacing unconstrained pose motion with guarded Cartesian motion

An early pose-target implementation allowed a small end-effector displacement to produce a large joint-space detour, including an apparent full rotation. Reaching the same Cartesian endpoint does not guarantee that the selected inverse-kinematics solution is desirable.

The relative stages were replaced with `computeCartesianPath()`, preserving end-effector orientation and interpolating a straight path. Three guards were added:

1. Cartesian completion must be at least 99%;
2. the generated trajectory must be nonempty and structurally valid;
3. no joint may accumulate more than 1 rad of travel during a small Cartesian stage.

Larger experimental offsets produced only 12.5% or 93.4% completion. The program refused to execute them. This was a successful safety response rather than a failure to be hidden by lowering the threshold.

## 8. Centralizing parameters and calculating the return automatically

The motion parameters were centralized so the task can be edited without searching through multiple execution blocks.

Verified values:

```text
Approach:           (+0.03, 0.00, 0.00) m
Lift and transport: (-0.03, 0.00,+0.05) m
```

The return vector is calculated automatically:

```text
Return = -(Approach + Transport)
       = (0.00, 0.00,-0.05) m
```

This prevents three separate displacement definitions from drifting out of sync, provided that all stages remain in the same planning frame and no unaccounted motion is inserted.

## 9. Verified milestone

The complete flow was verified on 2026-09-03:

```text
four controllers active
move to test_configuration: success
approach Cartesian path: 100.0%
lift and transport Cartesian path: 100.0%
return Cartesian path: 100.0%
gripper open, close, and release: success
PICK AND PLACE DEMO SUCCEEDED
process finished cleanly
```

## Engineering skills demonstrated

- integrating multiple ROS 2 packages into a single bringup;
- inspecting nodes, topics, actions, controllers, and joint states;
- distinguishing model, planning, and execution failures;
- editing URDF/Xacro, SRDF, YAML, Python launch files, and MoveIt C++ code;
- reducing complex failures through layered, reproducible tests;
- rejecting unsafe or incomplete paths rather than forcing execution;
- documenting limitations and preserving stable milestones with Git.

## Next steps

The current milestone validates fake-hardware motion control, not physical grasping. Future work can add:

1. table, floor, and object collision geometry in the Planning Scene;
2. grasped-object attach/detach state;
3. TCP/tool-frame approach commands;
4. NVIDIA Newton Physics or another physics-simulation workflow;
5. hardware-specific force, speed, network, and safety validation.

Explicitly separating verified behavior from future work is part of the engineering result: it communicates both what the prototype proves and what it does not yet prove.
