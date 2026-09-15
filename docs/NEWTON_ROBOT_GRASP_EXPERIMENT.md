# Newton Robot Contact-Grasp Experiment

## Purpose and claim boundary

This experiment tests whether the imported UR5 and Robotiq 2F-85 can lift and release the fast 20-segment strip through Newton collision and friction. The robot follows prescribed joint coordinates and is kinematic; the strip remains dynamic. No fixed joint, attachment constraint, or pose-copy shortcut connects the strip to the gripper.

The result establishes one reproducible **pre-positioned contact lift-and-release** case. It is not yet a MoveIt trajectory, an autonomous pickup, a calibrated force-control experiment, or a validation of physical rubber.

![UR5 and Robotiq lifting and releasing the segmented strip through Newton contact](experiments/newton-robot-grasp/results/center_grasp_success.gif)

The approximately 4× recording shows the successful configuration: the strip settles between the fingers, follows the commanded lift, remains held during the lift hold, and drops only when the gripper opens. The animation is qualitative evidence; the measured grasp-region trajectory supplies the acceptance evidence.

## Model and command sequence

The robot model contains 24 bodies and 54 shapes. All 17 robot collision shapes are enabled, while the robot bodies follow prescribed joint coordinates. The object is the previously compared `0.40 × 0.05 × 0.02 m`, `0.44 kg` segmented strip with 20 rigid links and compliant revolute joints.

The six-second test is:

```text
closed contact hold
→ 0.12 m prescribed robot lift
→ lift hold
→ open gripper
→ release
```

The successful parameters were:

| Parameter | Value |
|---|---:|
| Grasp location | strip center / center-of-mass region |
| Initial strip center height | `0.309 m` |
| Gripper leader angle | `0.386 rad` |
| Contact friction coefficient | `1.5` |
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

Moving the grasp to the center-of-mass region greatly reduced this moment. With all other settings held fixed, `0.36625 rad` established contact but the operator observed slip during lifting. Increasing only the leader angle to `0.386 rad` produced the successful lift and release. This one-variable comparison supports insufficient normal clamping action as the main cause of the preceding lift failure. It does not calibrate a physical gripper force.

## Measurement correction and quantitative acceptance

The first automated check used the mean height of all 20 strip links. That metric reported only `45.48 mm` of lift because the two flexible halves sagged even while the center remained carried. It therefore produced a false failure for a center grasp.

The corrected metric tracks the two central links located at the gripper. This is the relevant observable for the question, “Did the grasped region follow the robot?”

```text
grasp-region z after contact hold = 0.279712 m
grasp-region z after lift hold    = 0.398397 m
measured grasp-region rise        = 0.118685 m
commanded robot lift              = 0.120000 m
absolute tracking error           = 0.001315 m
final grasp-region z after release = 0.010000 m
release drop from lift hold       = 0.388397 m
finite state                      = true
```

The predefined programmatic acceptance for this stage is:

```text
abs(measured grasp-region rise - commanded lift) < 0.020 m
release drop from lift hold > 0.050 m
all final coordinates are finite
```

The run passed all three checks. The strip also settled downward by `29.29 mm` during the initial contact hold. This is recorded rather than hidden: the object begins pre-positioned inside a closed gripper and finds a supported contact configuration before the lift begins.

Newton's reported contact-count buffer contains collision candidates and is retained only as diagnostic output. It is not used as proof of load-bearing contact. The acceptance instead requires the independently simulated grasp region to follow the robot lift and then separate after the commanded opening.

## Engineering lessons

1. A rendered mesh is not necessarily the collision mesh. Collider-only visualization is required when diagnosing penetration and missed contact.
2. Gripper joint angle is not a complete aperture measurement. The relevant gap must be checked at the object's actual height and contact location.
3. A successful-looking lift can be caused by support, wedging, penetration, or an attachment shortcut. A release test and a deliberately failing comparison help separate these explanations.
4. Grasping near the center of mass reduces gravity moment, but the best grasp point also depends on accessibility, collision clearance, and the downstream task.
5. Validation metrics must measure the phenomenon of interest. Whole-object mean height was useful for strip deformation but inappropriate for judging whether the center grasp followed the robot.
6. Change one variable at a time. Holding geometry, friction, and motion fixed while changing closure from `0.36625` to `0.386 rad` made the result interpretable.

## Reproduction

Use the validated Newton Python environment and expanded robot URDF described in the integration record.

```bash
cd docs/experiments/newton-robot-grasp/prototype

GRASP_POSITION=center \
GRASP_COLLISION_SCOPE=all \
GRASP_CLOSED_GRIP=0.386 \
GRASP_STRIP_CENTER_Z=0.309 \
GRASP_FRICTION=1.5 \
python robot_segmented_grasp_batch.py

GRASP_POSITION=center \
GRASP_SHOW_COLLIDERS=1 \
GRASP_COLLISION_SCOPE=all \
GRASP_CLOSED_GRIP=0.386 \
GRASP_STRIP_CENTER_Z=0.309 \
GRASP_FRICTION=1.5 \
python robot_segmented_grasp_simulation.py --start-delay 15
```

`robot_segmented_grasp_batch.py` builds the scene, prescribes robot motion, advances Newton contact dynamics, measures the whole strip and grasp region separately, and applies the acceptance criteria. `robot_segmented_grasp_simulation.py` displays the same experiment and can switch between visual geometry and collision geometry.

## Next integration step

The next milestone should preserve this Newton object-response test while replacing the locally prescribed robot coordinates with a ROS 2 trajectory command. MoveIt will plan the robot path; the ROS–Newton bridge will translate the trajectory and return actual simulation state; Newton will remain responsible for contact, gravity, deformation, slip, and release. The current result is the physics baseline against which that integration can be checked.
