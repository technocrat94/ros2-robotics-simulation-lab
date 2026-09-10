# Newton–ROS 2 Integration: Verified Bridge and Robot-Model Assessment

Recorded on 2026-09-10. This document describes the first verified integration milestone between ROS 2 Humble and Newton 1.5.1 in the migrated Ubuntu aarch64 VM.

## Objective

Connect Newton to ROS 2 through an observable command-and-feedback path before replacing the existing UR5/Robotiq fake-hardware implementation.

The milestone is intentionally small: a Newton sphere is controlled through ROS services, and Newton's measured state is returned through ROS topics. The UR5/Robotiq model has been expanded and inspected, but has not yet been simulated or controlled in Newton.

## Why the bridge uses two processes

The validated ROS 2 Humble environment uses system Python 3.10.12. The validated Newton environment uses an isolated Python 3.12.14 environment with Newton 1.5.1 and Warp 1.17.0. The inherited Python 3.10 Newton environment could import the top-level package but failed when loading the selected solver.

To preserve both working environments, the implementation separates them:

```text
ROS 2 / Python 3.10
        │ services and topics
        ▼
ROS adapter ── loopback UDP/JSON ── Newton endpoint
                                      │
                                      ▼
                            Newton / Python 3.12
```

This boundary makes dependency failures and stale simulator data observable. It is a project-specific prototype interface, not an official Newton transport protocol.

## Implemented ROS interface

| Interface | Type | Purpose |
|---|---|---|
| `/newton/set_running` | `std_srvs/srv/SetBool` | Start or pause Newton stepping |
| `/newton/reset` | `std_srvs/srv/Trigger` | Restore the initial Newton state |
| `/newton/object_pose` | `geometry_msgs/msg/PoseStamped` | Publish the measured sphere pose |
| `/newton/sim_time` | `std_msgs/msg/Float64` | Publish Newton simulation time |
| `/newton/running` | `std_msgs/msg/Bool` | Publish the current running state |
| `/newton/bridge_status` | `std_msgs/msg/String` | Report `WAITING`, `OK`, or `STALE` |

The first prototype uses loopback UDP ports 15100 and 15101. State messages include a protocol version and monotonically increasing sequence number.

## Verified evidence

The package built successfully with `colcon`, and ROS discovered the `newton_ros_bridge ros_adapter` executable.

With Newton paused at its initial state, ROS received:

- bridge status: `OK`, with a recent packet age and increasing sequence number;
- frame: `world`;
- measured sphere center height: `1.0 m`;
- Newton simulation time: `0.0 s`.

After starting Newton through `/newton/set_running`, then pausing it, ROS received:

- measured sphere center height: approximately `0.200000003 m`;
- simulation time: approximately `3.89583 s`;
- running state: `false`.

The 0.20 m final center height is consistent with the configured 0.20 m sphere radius and ground contact. The run time was longer than the shell's one-second sleep because separate `ros2 service call` processes also incur discovery and command latency while Newton continues stepping. Controlled experiments should therefore use simulation time or state conditions rather than treating wall-clock shell delays as exact simulation durations.

After calling `/newton/reset`, ROS received `z = 1.0 m` and `sim_time = 0.0 s`. After stopping the Newton endpoint, the adapter reported `STALE` instead of presenting the last pose as fresh data.

Together, these observations verify both directions:

```text
ROS command → adapter → Newton
Newton state → adapter → ROS
```

## Important prototype limitation

A successful service response currently means that the ROS adapter sent the UDP command. The endpoint produces an acknowledgement packet, but the current adapter does not use it to confirm completion. Command effects were verified from the returned state. A later version should correlate command IDs with acknowledgements and report a timeout when no acknowledgement arrives.

The bridge does not yet publish `/clock`, load the robot, connect to MoveIt, or replace `ros2_control` fake hardware.

## UR5 and Robotiq model assessment

The existing combined Xacro was expanded without modifying the source model:

```text
Robot name: ur
Links: 24
Joints: 23
Joint types: fixed, revolute
```

The non-fixed joints consist of six independent UR5 joints and six Robotiq joints. In the gripper, `robotiq_85_left_knuckle_joint` is the leader; five joints use URDF `<mimic>` relationships to follow it. The effective command space is therefore six arm joints plus one gripper actuator, or seven independent variables.

The installed Newton 1.5.1 XPBD solver documents that mimic constraints are unsupported. Directly importing the URDF may therefore produce visible geometry while failing to preserve the intended gripper coupling. Import success alone is not evidence of correct robot behavior.

The planned prototype will keep one ROS gripper command and explicitly map the leader value to the five follower targets using each URDF multiplier and offset. Newton will remain responsible for contact and object motion. This mapping must be verified before grasp results are interpreted.

## Next acceptance milestone

1. Inspect and record every Robotiq mimic multiplier and offset.
2. Resolve all URDF mesh resources and import the combined robot into Newton.
3. Verify link and joint-name correspondence between ROS and Newton.
4. Exercise one arm joint and the gripper leader while checking all returned joint states.
5. Ensure that only one component owns authoritative `/joint_states`; the existing fake hardware and Newton must not publish competing robot states.

## Engineering lesson

Integration is established by tracing commands and measured feedback across a defined boundary. A model that loads or looks correct can still be behaviorally wrong when joint coupling, units, timing, or state ownership differ.

## Contribution

Codex implemented the prototype bridge and prepared the documentation. The project owner executed the build and verification commands, supplied the outputs, and participated in interpreting the integration evidence. The document distinguishes completed tests from planned robot integration.

## References

- [Newton Robotics 101](https://newton-physics.github.io/newton/latest/tutorials/01_robotics.html)
- [ROS 2 Humble `sensor_msgs/JointState`](https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/JointState.html)
- [ROS 2 topics, services, and actions](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html)
