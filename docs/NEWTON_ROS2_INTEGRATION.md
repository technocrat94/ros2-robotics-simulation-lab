# Newton–ROS 2 Integration: Verified Bridge and Robot-Model Assessment

> [繁體中文學習筆記](NEWTON_ROS2_INTEGRATION.zh-TW.md) · [Documentation index](NEWTON_ROS2_LEARNING_LOG.zh-TW.md)

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

The endpoint now also exposes a loopback-only Viser view. It deliberately has no motion controls: movement must originate from ROS, while its panel displays Newton simulation time, measured height, running state, the last ROS command, and returned-state sequence. `NEWTON_VIEWER_PORT` selects the port and `NEWTON_BRIDGE_LOG` optionally records the transmitted state as CSV. The viewer supports inspection; the ROS topic and failure-injection evidence remain the acceptance tests.

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

### Verified static URDF import

The combined Xacro was expanded with ROS and imported with Newton 1.5.1 on CPU. The first import produced 24 bodies and 24 Newton joints but zero shapes. This was a useful partial failure: Newton had parsed the kinematic topology, while every `package://ur_description/...` and `package://robotiq_description/...` mesh URI remained unresolved.

The generated Newton-specific URDF replaced those package URIs with the package share paths reported by `ros2 pkg prefix --share`. The ROS source Xacro and generated ROS URDF were left unchanged. A second import produced:

```text
Bodies: 24
Joints: 24
Shapes: 54
```

The model still did not appear until `newton.eval_fk()` initialized the world transforms from the joint coordinates. After that step, the project owner visually confirmed a continuous UR5 assembly, a Robotiq gripper attached to the wrist, and both fingers present.

This milestone proves that the static geometry and assembly can be represented in Newton. It does not prove correct joint motion, mimic coupling, self-collision behavior, contact behavior, dynamics, or ROS control of the robot. The reproducible inspection program is stored in `docs/experiments/newton-ros2-bridge/prototype/newton_robot_viewer.py`.

### Verified ROS-commanded kinematic demonstration

The verified bridge contract was reused with a robot endpoint. Calling `/newton/set_running` from ROS started an eight-second prescribed UR5 and Robotiq sequence in Newton; the endpoint returned 12 revolute-joint names and positions through `/newton/joint_states`, plus the current phase and maximum mimic error. During a paused gripper-close phase, ROS received the six UR5 angles, the Robotiq leader angle, and all five follower angles with `mimic_max_error = 0.0 rad`.

![Newton Viser view of the completed ROS-commanded kinematic sequence](images/newton_ros2_kinematic_demo.png)

**Captured evidence.** The image shows the complete UR5/Robotiq assembly and the endpoint panel after the sequence reached `COMPLETE · PAUSED` at `8.00 s`. The panel records `set_running(true)` as the last ROS command, a reopened leader angle of `0.000 rad`, a maximum mimic error of `0.000000 rad`, and returned-state sequence `49899`.

This demonstrates ROS command delivery, Newton-side FK, explicit URDF mimic mapping, and state return to ROS. The trajectory is prescribed kinematics. It is not evidence of dynamic trajectory tracking, contact, grasping, or a complete control loop. The robot state is published on the namespaced `/newton/joint_states`; it has not replaced the authoritative `/joint_states` used by the existing fake-hardware stack.

#### What the program actually computes

This prototype does not request an end-effector pose and does not solve inverse kinematics. The program defines the joint coordinates directly as functions of simulation time, writes them into Newton's `joint_q`, and calls `newton.eval_fk()` to calculate every link pose in the world frame:

```text
prescribed time t -> joint coordinates q(t) -> forward kinematics -> link poses
```

The initial arm coordinates are `(0, -pi/2, +pi/2, -pi/2, -pi/2, 0)` rad. The eight-second sequence is:

| Time | Phase | Prescribed change |
|---|---|---|
| 0-2 s | `ARM_APPROACH` | shoulder pan: `0 -> +0.55 rad` (`+31.5 deg`); elbow: `+1.571 -> +1.221 rad` (change `-20.1 deg`) |
| 2-4 s | `GRIPPER_CLOSE` | Robotiq leader: `0 -> +0.70 rad` (`+40.1 deg`); five followers are derived from the URDF mimic rules |
| 4-6 s | `WRIST_MOTION` | wrist 3: `0 -> +0.65 -> 0 rad` (maximum `+37.2 deg`) |
| 6-8 s | `RETURN_AND_OPEN` | the changed arm joints return to the initial coordinates and the gripper leader returns to zero |

The conversion is `degrees = radians * 180 / pi`. The approach and return segments use `s = 3u^2 - 2u^3` so their prescribed position curves start and finish with zero slope. This interpolation choice does not model actuator torque or prove dynamically feasible tracking.

Only three arm joints move in this sequence: shoulder pan, elbow, and wrist 3. Shoulder lift, wrist 1, and wrist 2 remain fixed. The test therefore does not establish that every axis behaves correctly. It specifically checks joint indexing and direction for the exercised axes, gripper coupling, FK visualization, and returned ROS state.

#### Relationship to the existing MoveIt demonstration

The earlier MoveIt workflow and this Newton prototype validate different layers:

| Existing MoveIt fake-hardware workflow | Current Newton kinematic prototype |
|---|---|
| accepts a pose or joint objective and uses kinematics and planning components to produce a trajectory | reads a predetermined joint-time sequence; no IK or path planner is called |
| checks the planned robot path against its planning scene | does not yet request a collision-free path |
| sends the trajectory to `ros2_control` fake hardware | writes prescribed joint coordinates directly into Newton |
| validates planning, controller interfaces, and task sequencing | validates the ROS-Newton boundary, imported joint mapping, FK, mimic rules, and feedback |
| does not prove physical object contact | this kinematic mode also does not prove contact or grasping |

The next integration step is to send MoveIt-generated joint trajectory samples to Newton actuator targets and return Newton's simulated joint state as the single authoritative feedback source. Acceptance will require checking names, ordering, units, timing, trajectory tracking error, collision behavior, and state ownership. Directly replacing `/joint_states` before those checks would hide interface faults rather than validate them.

#### Implementation record

The reproducible prototype is committed under `docs/experiments/newton-ros2-bridge/prototype/` and mirrors the operational ROS package used in the VM.

| File | Responsibility |
|---|---|
| `newton_robot_endpoint.py` | Load the resolved URDF, receive commands, prescribe `q(t)`, apply mimic mapping, run FK, update Viser, and transmit state |
| `newton_ros_bridge/ros_adapter.py` | Expose ROS services and topics, translate ROS messages to the versioned UDP/JSON contract, reject invalid state packets, and report freshness |
| `package.xml`, `setup.py`, `setup.cfg` | Declare ROS dependencies and install the `ros_adapter` executable |
| `newton_robot_viewer.py` | Reproduce the earlier static URDF import and FK inspection milestone |

The prescribed trajectory is visible in the endpoint rather than hidden behind the viewer. For example, the approach phase directly changes two generalized coordinates:

```python
s = smoothstep(t / 2.0)
arm[0] += 0.55 * s
arm[2] -= 0.35 * s
```

The endpoint then writes the arm and gripper coordinates, including the five explicit follower mappings, before evaluating FK:

```python
self.q[:6] = arm
self.q[6] = grip
self.q[8] = -grip
self.q[10] = grip
self.q[11] = -grip
self.q[7] = -grip
self.q[9] = grip
self.model.joint_q.assign(self.q)
newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state)
```

The mapping is checked independently of the image. For every follower `i`, the endpoint evaluates

```text
error_i = abs(q_i - (multiplier_i * q_leader + offset_i))
mimic_max_error = max(error_i)
```

and publishes the maximum through `/newton/mimic_max_error`. A zero result verifies the implemented algebra for the transmitted coordinates; it does not verify joint origins, axes, collision geometry, or contact.

On the ROS side, each service command receives a UUID before transmission:

```python
message = {"protocol": PROTOCOL, "id": command_id, "command": command}
self.socket.sendto(json.dumps(message).encode("utf-8"), COMMAND_ADDRESS)
```

State packets are accepted only when both `type == "state"` and the protocol version match. Their joint names and positions become a ROS `sensor_msgs/JointState`. Packet age is measured with a monotonic clock; data older than `0.25 s` is reported as `STALE`. This separates content validity from transport freshness.

The command ID is present in the endpoint acknowledgement, but the adapter does not yet correlate that acknowledgement with the service call. Consequently, the service response means "command transmitted." The returned `/newton/running`, `/newton/demo_phase`, `/newton/joint_states`, and `/newton/sim_time` remain the evidence that the requested state change occurred.

#### Documentation image evidence

The current portfolio image records the prescribed-kinematics milestone and includes the complete model and readable endpoint state in one frame. A later MoveIt-to-Newton milestone should add one separate image showing an executing MoveIt trajectory and the corresponding Newton feedback. A physical-contact milestone should use a short video or GIF only when object motion and slip over time are part of the acceptance evidence. Terminal screenshots are unnecessary because exact commands and machine-readable outputs are preserved as text.

## Next acceptance milestone

1. Verify link and joint-name correspondence between ROS and Newton.
2. Exercise one arm joint and the gripper leader while checking all returned joint states and all five mimic relationships.
3. Check collision geometry and self-collision settings independently of visual geometry.
4. Connect robot commands and feedback through the verified bridge protocol.
5. Ensure that only one component owns authoritative `/joint_states`; the existing fake hardware and Newton must not publish competing robot states.

## Engineering lesson

Integration is established by tracing commands and measured feedback across a defined boundary. A model that loads or looks correct can still be behaviorally wrong when joint coupling, units, timing, or state ownership differ.

## Engineering judgment: what the project owner must understand

Writing every line of integration code is not the main learning objective. The project owner must be able to define the contract, judge the evidence, diagnose the failed boundary, and defend the limits of the conclusion.

| Judgment | Question to answer | Example in this project |
|---|---|---|
| Requirement | What observable result counts as success? | Gripper closure is insufficient; the object must lift and remain supported within a defined slip limit. |
| State ownership | Which component owns the authoritative state? | Newton should own simulated joint and object state after it replaces fake hardware. |
| Interface contract | What names, units, frames, rates, and failure rules cross the boundary? | Joint names must match; revolute joints use radians; object pose uses `world`; stale data must be reported. |
| Model semantics | Does the imported mechanism preserve its intended constraints? | The Robotiq command space is one leader plus five mimic followers, not six independent actuators. |
| Timing | Which clock and time step define the experiment? | Shell delay, ROS time, Newton simulation time, physics `dt`, and real-time factor are different quantities. |
| Evidence | Did the actual state change, or was only a target echoed? | `/newton/object_pose` changed after a ROS command and returned to its initial value after reset. |
| Failure isolation | At which boundary did expected evidence disappear? | `STALE` distinguishes a stopped Newton endpoint from an unchanged but live simulation. |
| Numerical validity | Is the result stable under a reasonable numerical check? | Contact results should be compared after reducing `dt` or increasing solver iterations. |
| Physical validity | What real behavior has and has not been represented? | Rigid contact in Newton is more informative than fake hardware, but it is not calibrated Robotiq hardware. |
| Reproducibility | Can another engineer identify the exact model, versions, configuration, and evidence? | Commit code, parameters, environment versions, observed outputs, and known limitations. |

### Work that can be delegated

Code generation, repetitive configuration edits, command execution, log collection, plotting, and documentation formatting can be delegated to an automation tool. The engineer still reviews the interface and evidence.

### Decisions that remain the engineer's responsibility

- define the task and measurable success criterion;
- choose which component owns commands, time, and state;
- check frames, units, joint names, ordering, and constraints;
- decide whether a test distinguishes competing explanations;
- reject conclusions that exceed the fidelity of the model or the quality of the evidence;
- communicate assumptions, limitations, and operational risk.

### Five-sentence review for a professor

1. State the engineering objective and success criterion.
2. Identify the command path and authoritative feedback path.
3. Explain one important compatibility decision, such as process isolation or mimic handling.
4. Cite the test evidence, including a failure-injection result.
5. State what remains unvalidated and the next acceptance milestone.

## Contribution

Codex implemented the prototype bridge and prepared the documentation. The project owner executed the build and verification commands, supplied the outputs, and participated in interpreting the integration evidence. The document distinguishes completed tests from planned robot integration.

## References

- [Newton Robotics 101](https://newton-physics.github.io/newton/latest/tutorials/01_robotics.html)
- [ROS 2 Humble `sensor_msgs/JointState`](https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/JointState.html)
- [ROS 2 topics, services, and actions](https://docs.ros.org/en/humble/Concepts/Basic/Interfaces-Topics-Services-Actions.html)
