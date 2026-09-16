# Newton Robot Contact-Grasp Experiment

> [繁體中文學習筆記](NEWTON_ROBOT_GRASP_EXPERIMENT.zh-TW.md) · [Documentation index](NEWTON_ROS2_LEARNING_LOG.zh-TW.md)

## Purpose and claim boundary

This experiment tests whether the imported UR5 and Robotiq 2F-85 can lift and release the fast 20-segment strip through Newton collision and friction. The robot follows prescribed joint coordinates and is kinematic; the strip remains dynamic. No fixed joint, attachment constraint, or pose-copy shortcut connects the strip to the gripper.

The result establishes one reproducible **pre-positioned contact lift-and-release** case and one prescribed high-amplitude motion stress test. It is not yet a MoveIt trajectory, an autonomous pickup, a calibrated force-control experiment, or a validation of physical rubber.

![Newton friction grasp surviving a 180-degree wrist motion before release](experiments/newton-robot-grasp/results/newton_180deg_dance_grasp.gif)

The 20-second wall-clock recording begins at lift hold, shows the prescribed shoulder sweep and a maximum wrist offset of approximately 180 degrees, and ends with the strip falling after the gripper opens. The CPU viewer ran slower than simulation time; the programmed dance duration was `1.5 s` of simulation time. [Download the original recording](experiments/newton-robot-grasp/results/newton_180deg_dance_grasp.mov).

![Earlier geometric-capture lift and release at 0.386 rad](experiments/newton-robot-grasp/results/center_grasp_success.gif)

The approximately 4× recording shows the earlier `0.386 rad` configuration carrying and releasing the strip. A later zero-friction control also remained captured at this closure, so the recording is retained as evidence of **geometric capture**, not as evidence of a friction-dependent pinch. The revised quantitative result below uses `0.376 rad` and a matched zero-friction control.

## Model and command sequence

The robot model contains 24 bodies and 54 shapes. All 17 robot collision shapes are enabled, while the robot bodies follow prescribed joint coordinates. The object is the previously compared `0.40 × 0.05 × 0.02 m`, `0.44 kg` segmented strip with 20 rigid links and compliant revolute joints.

The test sequence is:

```text
closed contact hold
→ 0.12 m prescribed robot lift
→ lift hold
→ optional shoulder-sweep and wrist-twist stress motion
→ open gripper
→ release
```

The successful parameters were:

| Parameter | Value |
|---|---:|
| Grasp location | strip center / center-of-mass region |
| Initial strip center height | `0.309 m` |
| Gripper leader angle | `0.376 rad` |
| Contact friction coefficient | `1.5` |
| Lift duration | `0.25 s` |
| Estimated peak lift speed | `0.72 m/s` |
| Estimated peak lift acceleration | `11.52 m/s²` |
| Lift-hold duration | `1.0 s` |
| Stress-motion duration | `1.5 s` |
| Shoulder offset parameter | `0.35 rad` bound |
| Maximum wrist offset | approximately `π rad` / `180°` |
| Collision scope | all robot collision shapes |
| Time step | `1/600 s` |
| XPBD iterations | `30` |

## Why the first apparent successes were rejected

Early near-end tests used leader angles around `0.72–0.74 rad`. Kinematic inspection showed that these settings reduced the nominal fingertip gap to approximately `8–6 mm` around a `50 mm`-wide strip. Visual inspection then showed the strip wedged in or supported by fingertip geometry, and one run passed through robot geometry that had been disabled for diagnostic isolation. Those observations invalidated the initial interpretation of an opposed friction pinch.

The collision geometry was then enabled for the complete robot. A less aggressive `0.36625 rad` closure corresponds to an approximate `48.97 mm` fingertip collision-AABB gap. However, at the original object height of `0.340 m`, the strip aligned with an upper finger section whose approximate gap was `57.62 mm`. The object therefore had no effective opposing contact and fell immediately. This demonstrated that a joint-angle-derived nominal aperture is insufficient: contact also depends on the three-dimensional collision geometry and object pose.

Lowering the strip to `0.309 m` aligned it with the fingertip contact region. Grasping near the end still imposed a large gravity moment. For a uniform `0.44 kg`, `0.40 m` strip, the approximate moment about an end grasp is:

```text
weight = 0.44 * 9.81 = 4.32 N
moment arm = 0.20 m
gravity moment ≈ 4.32 * 0.20 = 0.86 N m
```

Moving the grasp to the center-of-mass region greatly reduced this moment. The subsequent observations refined the mechanism instead of treating every visible lift as the same kind of grasp:

| Closure | Friction | Observation | Interpretation |
|---:|---:|---|---|
| `0.356 rad` | `0` or `1.5` | immediate fall | aperture produced insufficient normal action; increasing friction alone could not help |
| `0.36625 rad` | `0` | immediate fall | no load-bearing geometric lock |
| `0.36625 rad` | `1.5` | descent slowed from the start but never stopped | friction acted, but its limit was below the static weight requirement |
| `0.376 rad` | `0` | immediate fall | no load-bearing geometric lock at the selected working point |
| `0.376 rad` | `1.5` | lifted and released successfully | friction-dependent grasp |
| `0.386 rad` | `0` or `1.5` | remained captured | geometric interlocking / form closure dominated |

The `0.376 rad` pair changes only friction and therefore provides the cleanest mechanism test. The single Newton material coefficient is interpreted as a Coulomb-friction parameter; it is not a separately calibrated real static and kinetic coefficient.

## Measurement correction and quantitative acceptance

The first automated check used the mean height of all 20 strip links. That metric underestimated lift because the two flexible halves sagged even while the center remained carried. It therefore produced a false failure for a center grasp.

The corrected metric tracks the two central links located at the gripper. This is the relevant observable for the question, “Did the grasped region follow the robot?”

```text
grasp-region z after contact hold = 0.276703 m
grasp-region z after lift hold    = 0.396272 m
measured grasp-region rise        = 0.119569 m
commanded robot lift              = 0.120000 m
absolute tracking error           = 0.000431 m
final grasp-region z after release = 0.010000 m
release drop from lift hold       = 0.386272 m
finite state                      = true
```

The predefined programmatic acceptance for this stage is:

```text
abs(measured grasp-region rise - commanded lift) < 0.020 m
abs(grasp-region z after motion - z before motion) < 0.020 m
release drop from lift hold > 0.050 m
all final coordinates are finite
```

The `0.376 rad`, `μ = 1.5`, `0.25 s` lift passed these checks. The strip settled downward by `32.30 mm` during the initial contact hold and then followed the fast lift. This is recorded rather than hidden: the object begins pre-positioned inside a closed gripper and finds a contact configuration before the lift begins.

The matched `μ = 0` control reached the ground during contact hold, produced essentially `0 m` grasp-region rise, and failed with `0.120 m` tracking error. Both the success and failure runs reported a maximum of 2,200 contact candidates. Newton's contact-count buffer is therefore retained only as diagnostic output; it is not proof of load-bearing contact. Acceptance requires the independently simulated grasp region to follow the robot lift and then separate after the commanded opening.

## 180-degree prescribed-motion stress test

After the matched friction control established the grasp mechanism, the successful working point was subjected to a larger motion. The lift and one-second `LIFT_HOLD` were retained; the robot then executed a `1.5 s` smooth, windowed motion combining shoulder motion with a wrist offset reaching approximately `π rad` from nominal before returning to the lift pose.

```text
grasp-region z before stress motion = 0.396272 m
grasp-region z after stress motion  = 0.395392 m
motion-retention error              = 0.000880 m
release drop                        = 0.386272 m
finite state                        = true
motion-retention pass               = true
overall contact/lift/release pass   = true
```

The `0.88 mm` before/after height difference is evidence that the grasp region remained carried through the prescribed motion. The video separately checks that the strip remained between the fingers, did not visibly pass through the arm, and fell only after opening. These claims are intentionally limited: the robot is kinematic, so the test increases the inertial demand on the dynamic strip and contacts but does not prove that a physical UR5 motor can supply the required torque.

`LIFT_HOLD` remains in the test because it isolates failure modes. Loss during lift indicates an acceleration-sensitive failure; loss during stationary hold indicates insufficient static support; loss during the stress motion indicates sensitivity to lateral or torsional loading. It can be set to zero for a short demonstration, but retaining it improves diagnosis.

## Engineering lessons

1. A rendered mesh is not necessarily the collision mesh. Collider-only visualization is required when diagnosing penetration and missed contact.
2. Gripper joint angle is not a complete aperture measurement. The relevant gap must be checked at the object's actual height and contact location.
3. A successful-looking lift can be caused by support, wedging, penetration, or an attachment shortcut. A release test and a deliberately failing comparison help separate these explanations.
4. Grasping near the center of mass reduces gravity moment, but the best grasp point also depends on accessibility, collision clearance, and the downstream task.
5. Validation metrics must measure the phenomenon of interest. Whole-object mean height was useful for strip deformation but inappropriate for judging whether the center grasp followed the robot.
6. Change one variable at a time. The matched `0.376 rad` tests changed only friction, which separated friction-dependent lifting from geometric interlocking.
7. Test speed only after identifying the grasp mechanism. A high-speed success at `0.386 rad` was not a valid friction stress test because the zero-friction case also remained geometrically captured.
8. Separate simulated time from wall-clock time. The CPU viewer can run much slower than real time without changing the configured Newton time step or simulated motion duration.
9. A kinematic stress test validates object/contact response to a prescribed path; actuator torque feasibility requires a dynamic robot model or a separate torque analysis.

## Reproduction

Use the validated Newton Python environment and expanded robot URDF described in the integration record.

```bash
cd docs/experiments/newton-robot-grasp/prototype

GRASP_POSITION=center \
GRASP_COLLISION_SCOPE=all \
GRASP_CLOSED_GRIP=0.376 \
GRASP_STRIP_CENTER_Z=0.309 \
GRASP_FRICTION=1.5 \
GRASP_LIFT_DURATION=0.25 \
python robot_segmented_grasp_batch.py

GRASP_POSITION=center \
GRASP_SHOW_COLLIDERS=1 \
GRASP_COLLISION_SCOPE=all \
GRASP_CLOSED_GRIP=0.376 \
GRASP_STRIP_CENTER_Z=0.309 \
GRASP_FRICTION=1.5 \
GRASP_LIFT_DURATION=0.25 \
python robot_segmented_grasp_simulation.py --start-delay 15

# High-amplitude prescribed-motion stress test.
GRASP_POSITION=center \
GRASP_COLLISION_SCOPE=all \
GRASP_CLOSED_GRIP=0.376 \
GRASP_STRIP_CENTER_Z=0.309 \
GRASP_FRICTION=1.5 \
GRASP_LIFT_DURATION=0.25 \
GRASP_LIFT_HOLD_DURATION=1 \
GRASP_MOTION_MODE=dance \
GRASP_DANCE_DURATION=1.5 \
GRASP_DANCE_CYCLES=1 \
GRASP_DANCE_SHOULDER_AMPLITUDE=0.35 \
GRASP_DANCE_WRIST_AMPLITUDE=3.14159265 \
python robot_segmented_grasp_batch.py

# Matched negative control: change friction only.
GRASP_POSITION=center \
GRASP_COLLISION_SCOPE=all \
GRASP_CLOSED_GRIP=0.376 \
GRASP_STRIP_CENTER_Z=0.309 \
GRASP_FRICTION=0 \
GRASP_LIFT_DURATION=0.25 \
python robot_segmented_grasp_batch.py
```

`robot_segmented_grasp_batch.py` builds the scene, prescribes robot motion, advances Newton contact dynamics, measures the whole strip and grasp region separately, and applies the acceptance criteria. `robot_segmented_grasp_simulation.py` displays the same experiment and can switch between visual geometry and collision geometry.

## Next integration step

The next milestone should preserve this Newton object-response test while replacing the locally prescribed robot coordinates with a ROS 2 trajectory command. MoveIt will plan the robot path; the ROS–Newton bridge will translate the trajectory and return actual simulation state; Newton will remain responsible for contact, gravity, deformation, slip, and release. The current result is the physics baseline against which that integration can be checked.
