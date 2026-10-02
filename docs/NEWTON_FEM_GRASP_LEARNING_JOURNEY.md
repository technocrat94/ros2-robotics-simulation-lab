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

The Jazzy port rebuilt the URDF/Xacro, SRDF, MoveIt configuration, controllers,
bridge, and a Python 3.12 amd64 Newton environment. It adapted the controller
state topic to `/joint_trajectory_controller/controller_state` and the state
field to `reference.positions`. The earlier Jazzy integrated test proved build,
planning, bridge, and visualization, but its grasp-region rise was only
`0.000630 m` and its physical acceptance flag was false.

Therefore the next Jazzy test must start from Humble commit `3546672`, preserve
the Jazzy API differences and user isolation, and compare the same measured
lift, retained contact, penetration, and release criteria.

## Engineering skills demonstrated

The project trained evidence-based debugging: separate planning from physics,
separate contact from support, change one variable at a time, test geometry
before material parameters, use rigid and zero-friction controls, choose a metric
that measures the grasp region, and preserve failed logs and rejected hypotheses.

The next implementation should automatically record world-frame `Fx/Fy/Fz`,
grasp-region rise, penetration, retention, release time, and per-run real-time
factor before the final Jazzy comparison.
