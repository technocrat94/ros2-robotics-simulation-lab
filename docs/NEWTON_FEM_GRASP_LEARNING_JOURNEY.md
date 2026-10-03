# Newton FEM Ground-Grasp Learning Journey

Status: accepted ROS 2 Humble reference, commit `3546672`.

This is the detailed reasoning record. `HUMBLE_FEM_PICK_SUCCESS.md` remains the
short reproducible recipe; `NEWTON_GROUND_GRASP_DIAGNOSTIC.md` is the failure
index. This document explains how the failures led to the final design.

## 0. The journey began with a simple ground grasp

The project did not begin with a complete FEM solution. It began with a simple
segmented or rigid-like strip on the ground and a Robotiq gripper. That model
answered the first integration questions: can the robot move, can the fingers
close, can Newton receive ROS commands, and can the object respond to contact?
It was a diagnostic model, not the final material representation.

The investigation then grew in controlled steps:

1. **Simple model:** an apparent lift could be friction holding, geometric
   capture, finger support, or penetration. We added zero-friction and angle
   comparisons.
2. **FEM strip:** the tetrahedral deformable strip exposed the soft-contact
   problem: the robot moved while FEM particles did not deform or lift.
3. **Center grasp:** edge grasping created a gravity moment, so the grasp region
   moved toward the center of mass.
4. **MoveIt ground task:** a manually chosen relative offset became Newton's
   measured absolute `world` pose, followed by IK and staged Cartesian motion.
5. **Contact diagnosis:** bilateral contacts and large force magnitudes still
   produced zero lift. A rigid-block control localized the fault to FEM particle
   contact rather than simply IK or friction.
6. **Physics repair:** VBD, full-surface contact, hidden analytic finger
   proxies, and ROS command interpolation supplied stable distance, normal, and
   time-history data.
7. **Kinematic repair:** width calibration and closure-drop compensation
   prevented fingertip entry into the strip or floor; `use_named_start=false`
   removed the last unnecessary detour.
8. **Acceptance:** the arm made no large detour, the FEM strip rose with the
   closed fingers, and it fell only after opening.

Each new model or measurement answered a specific unresolved question from the
previous stage. That sequence is the central engineering story of the project.

## Research question and acceptance boundary

The question was whether MoveIt could use Newton's measured world-frame object
pose to approach a deformable strip, form a real rigid-soft grasp, lift it, and
release it after the gripper opens. MoveIt completion and physics completion are
separate claims:

- Command-layer success: IK, planning, controllers, and the ROS–Newton bridge
  complete the requested trajectory.
- Physics-layer success: the strip rises with the fingers, remains supported,
  does not penetrate the robot or floor, and falls after release.

`ABSOLUTE POSITION PICK SUCCEEDED` proves only the first layer.

## System model

```text
Newton object pose → MoveIt IK and planning → ROS controllers
       → ROS–Newton synchronization, queue, and interpolation
       → Newton gravity, contact, friction, and FEM deformation
       → pose, contact, penetration, lift, and release evidence
```

UR5 and Robotiq are the mechanism. MoveIt determines the desired motion. ROS 2
controllers execute and report joints. The bridge translates state and timing.
Newton evaluates the physical consequence.

## Robotiq kinematics and height compensation

The Robotiq 2F-85 has one commanded leader joint and five mimic joints. The
mimic relationships must be checked; a model can render while having an
incorrect multiplier or joint order.

The fingertips follow an arc. Closing reduces the pad gap and moves the
fingertip midpoint downward. For a 50 mm strip with 2 mm total compression:

```text
target gap = 50 - 2 = 48 mm
0.35 rad → gap 50.716 mm, closure drop 10.109 mm
0.40 rad → gap 45.315 mm, closure drop 11.042 mm
interpolated command = 0.375145 rad
interpolated closure drop = 10.578 mm
compensated approach = 0.105000 - 0.010578 = 0.094422 m
```

This is gripper kinematic calibration: object width determines both the leader
command and the pre-closure tool height.

## FEM mesh and VBD

The strip is `0.40 × 0.05 × 0.02 m`, with `20 × 3 × 2` cells:

- 252 particles/vertices: `(20+1)(3+1)(2+1)`. They carry position, velocity,
  and mass; increasing them changes resolution, not density.
- 600 tetrahedra: `20×3×2×5`. They carry volume mechanics such as stretch,
  shear, and volume change.
- 424 surface triangles: `4(20×3+20×2+3×2)`. They define the visible and
  contact boundary.

The correct solver name is **VBD (Vertex Block Descent)**, not VBM. For a
volumetric deformable body, Newton formulates an implicit time step as a material
and contact energy/constraint problem. It updates local vertex blocks repeatedly,
reducing the energy while satisfying the constraints, then recomputes contact
for the next step. This is different from a single explicit `F=ma` update, but
does not mean forces are irrelevant.

Reference settings were Young's modulus `1,000,000 Pa`, Poisson ratio `0.45`,
density `1,100 kg/m³`, damping `1,000 Pa·s`, 10 VBD iterations, 60 Hz, and 10
physics substeps (`dt=1/600 s`). Finer meshes changed both deflection and CPU
runtime, so resolution must be checked for convergence rather than assumed to be
better.

## SDF, analytic proxies, full-surface contact, and GJK

An SDF (Signed Distance Field) provides a signed distance `phi(p)` from a point
to a surface: positive outside, zero on the surface, negative inside. A simple
geometric interpretation is `penetration=max(0,-phi)` and
`normal≈normalized(gradient(phi))`.

The detailed Robotiq mesh did not provide a reliable CPU particle-to-mesh SDF for
the VBD path. The final solution kept the mesh for appearance, disabled its
unreliable FEM particle collision, and added four hidden analytic box proxies
inside the finger/fingertip regions. The proxies were inset by 1 mm. They provide
stable distance and normal data for full-surface rigid-soft contact.

GJK remains useful for convex rigid-rigid distance/intersection tests, such as
robot links and the floor. GJK success is not equivalent to a continuous
particle-to-surface distance field, so it cannot replace the FEM contact path.

## Failure investigation and decisions

### Why the strip was not our first suspect after MoveIt integration

Before the absolute-position MoveIt task existed, the project had already
lifted a center-grasped segmented strip, retained it during faster motion and an
approximately 180-degree wrist sweep, and released it after opening. A
zero-friction comparison had also shown that at least one working point depended
on friction rather than an attachment constraint.

Given that evidence, it was reasonable to begin the MoveIt failure diagnosis by
checking what had changed: robot start state, object coordinates, approach
height, gripper command, ground collision, trajectory shape, and command timing.
The visual object was still “the strip,” so the initial working assumption was
that the grasp geometry or MoveIt execution was wrong.

The assumption became invalid only after the controls accumulated. The earlier
successful strip was a chain of rigid segments and compliant joints, while the
ground object was a volumetric tetrahedral FEM body. They looked similar but
used different physical contact paths. The earlier success proved robot motion,
gripper kinematics, rigid contact, and a plausible grasp strategy; it did **not**
prove that VBD particles could collide with the original Robotiq mesh.

This distinction was the turning point. It explains why increasing friction and
compression could not repair the FEM run: the missing information was a stable
particle-to-finger distance and normal, not a larger coefficient.

### MoveIt integration sequence and what each step established

1. **Controller and joint-order audit.** The active trajectory controller was
   confirmed to command the six UR5 joints in the expected order. This prevented
   a joint-name mismatch from being mistaken for bad IK.
2. **Start-state guard.** ROS and Newton initially disagreed by about `π/2` at
   the elbow and wrist. The guard made that mismatch visible. Explicit
   synchronization ensured both systems started from the same joint vector
   before trajectory shadowing was enabled.
3. **Relative-motion shadow test.** The existing `move_xyz` task was enlarged so
   motion was visually obvious. All Cartesian segments reached 100%, and the
   Newton shadow error was approximately `5.86×10^-8 rad`. This established that
   the bridge could reproduce MoveIt reference joints accurately.
4. **Absolute object pose.** Newton published `/newton/object_pose` in `world`.
   The new MoveIt node combined that pose with the `tool0`-to-fingertip-midpoint
   offset and used IK for a top-down grasp. This replaced trial-and-error relative
   displacement with a measurable target.
5. **Planning-scene floor and staged preview.** Plan-only, approach-only, and
   grasp-only modes separated path planning from execution. A floor collision
   object and a staged raise/orient/translate/descend route addressed paths that
   crossed the floor or made large joint detours.
6. **Manual gripper checks.** Commands around `0.35–0.40 rad` were compared with
   the visible gap. Contact without lift showed that “the fingers reached the
   object” was not yet the same as “the contact could carry load.”
7. **Closure-drop calibration.** Closing the linkage moved the fingertip midpoint
   downward. Measuring that drop converted the 50 mm object width into both a
   leader angle and a corrected approach height.
8. **Full run and physical rejection.** MoveIt completed, bilateral contact was
   reported, and the gripper lifted, but the strip stayed on the floor. This was
   the first decisive evidence that planning success and contact existence were
   insufficient.
9. **Single-variable diagnostics.** Friction, compression, height, and isolated
   lift were changed separately. None produced retained FEM lift, so the search
   moved from tuning to model representation.
10. **Rigid control and FEM observation.** A same-size rigid body responded,
    while the FEM strip did not indent or was penetrated by the fingers. That
    observation localized the fault to mesh-to-particle contact.
11. **Contact and time-path repair.** Analytic proxies, full-surface VBD contact,
    a bounded command queue, and interpolation supplied the geometry and timing
    required by the FEM solver.
12. **Path cleanup and final acceptance.** After physical lift worked, the final
    controlled change disabled the named start. The grasp still lifted and
    released correctly without the large detour.

### Visual motion was not proof of a grasp

Large closure angles could cause friction holding, geometric capture, finger
support, or penetration. Matched controls separated them: `0.376 rad, μ=1.5`
lifted and released after opening, while the zero-friction control failed;
`0.386 rad` could lift even without friction, indicating geometric capture.

### Edge grasp created a gravity moment

The strip mass was about `0.44 kg` and its weight about `4.32 N`. A roughly
`0.20 m` lever arm produced about `0.86 N·m` of gravity moment. Moving the grasp
region toward the center of mass reduced the tendency to rotate out.

### Contact magnitude did not prove support

A failed run reported bilateral loaded contacts (`25/22`) and force magnitude
sums (`231.335/250.229`), but measured lift was `0 m`. Magnitude did not show
direction: forces could push along the strip, react against the floor, or vanish
when upward motion began. Increasing friction and compression alone did not fix
the failure.

### Rigid control isolated the fault

A same-size rigid block produced a collision response while the FEM strip could
remain undeformed or be crossed by the mesh fingertip. This localized the problem
to the FEM particle/contact representation, rather than immediately blaming IK,
mass, or friction.

### Discrete ROS commands created effective velocity

If Newton jumps from `q_k` to `q_(k+1)`, it sees approximately
`v_effective=(q_(k+1)-q_k)/Δt`. A large jump can cross the surface between
queries. The bridge therefore uses a bounded command queue, `dt=0.02 s`
interpolation, and ten physics substeps per 60 Hz frame.

### MoveIt had an independent path failure

The named `test_configuration` start caused a large detour. Duplicate bringup
instances caused duplicate action servers and `unknown goal response`. The final
route uses raise, orient, horizontal translation, vertical approach, closure,
lift, and release, with `use_named_start=false` and one bringup stack.

## Final accepted run

Only one path variable changed relative to the physically successful grasp:
`use_named_start=true → false`. The FEM, proxies, contact model, calibration,
descent compensation, and lift settings were preserved.

```text
object center                 (0.4869, 0.1093, 0.0110) m
leader command                0.375145 rad
closure drop                 10.578 mm
compensated approach          0.094422 m
lift distance                 0.120 m
lift velocity scale           0.030
pre-grasp / approach / lift   100% / 100% / 100%
lift trajectory               3.517 s
logged MoveIt sequence        about 12.765 s
final message                 ABSOLUTE POSITION PICK SUCCEEDED
```

The operator confirmed no large initial detour, the strip rose with the fingers,
and the strip fell only after opening. This is the accepted Humble reference.
Automatic rise, retention, penetration, and release assertions remain future
work.

## Jazzy migration boundary

The home reference is Ubuntu 22.04, ROS 2 Humble, aarch64 CPU. The school
computer is Ubuntu 24.04, ROS 2 Jazzy, x86_64/amd64, with an NVIDIA GPU. Build
directories, ARM binaries, and virtual environments cannot be copied between
them.

The first Jazzy integrated test rebuilt the URDF/Xacro, SRDF, MoveIt
configuration, controllers, bridge, and a Python 3.12 amd64 Newton environment.
It adapted the controller state topic to
`/joint_trajectory_controller/controller_state` and the state field to
`reference.positions`. It proved build, planning, bridge, and visualization,
but its grasp-region rise was only `0.000630 m`; physical acceptance was false.

That failure led to an architecture audit rather than another friction change.
The audit found that the school runtime did not yet contain every condition from
the Humble success path: the full free tetrahedral strip, VBD construction,
full-surface contact, four analytic particle proxies, particle-collision flags,
bounded queue, command interpolation, and all physics substeps. Those features
were ported while preserving the Jazzy controller and MoveIt API differences.
The Jazzy URDF was regenerated from Jazzy Xacro, and all 34 mesh paths were
verified rather than copying Humble absolute paths or ARM artifacts.

The verified native Jazzy run on 2026-10-02 used an RTX 3080 (`cuda:0`) and
passed both layers:

```text
safe pre-grasp stages              100% each
approach / lift Cartesian paths    100% / 100%
named-start detour                 false
grasp-region lift                  0.117290587 m
release drop                       0.112259318 m
bilateral lift contact samples     229
maximum floor penetration          0.000820466 m (< 0.005 m limit)
finite state                       true
candidate contact grasp pass       true
real-time factor                   0.102285
```

The viewer confirmed that the strip stayed between the fingers during lift and
fell after reopening. This completed the migration: Jazzy success was accepted
only after reproducing the Humble physical behavior, not merely after compiling
or printing a MoveIt success line.

## Engineering skills demonstrated

The project trained evidence-based debugging: separate planning from physics,
separate contact from support, change one variable at a time, test geometry
before material parameters, use rigid and zero-friction controls, choose a metric
that measures the grasp region, and preserve failed logs and rejected hypotheses.

The next implementation should automatically record world-frame `Fx/Fy/Fz`,
grasp-region rise, penetration, retention, release time, and per-run real-time
factor before the final Jazzy comparison.
