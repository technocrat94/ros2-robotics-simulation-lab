# cuRobo plans, Newton verifies: Step C1 completion

## Claim

Commit `58f572c4e85be355d52ddc7d16a7a45f69f61b4f` replaced only the
arm planner with cuRobo V2. The executed chain was:

```text
cuRobo V2 on cuda:0
  -> FollowJointTrajectory / GripperCommand
  -> existing ros2_control fake hardware
  -> existing trajectory-shadow ROS-Newton bridge
  -> unchanged Newton FEM cable + C1 wall endpoint
```

MoveIt Step C was not rerun for comparison, and no command was sent to real
hardware. The wall, cable mesh, contact physics, grasp calibration, execution
timing, and acceptance rules were unchanged.

## Motion sequence and fixed parameters

All three arm stages were planned independently by cuRobo: start to pre-grasp,
descent to grasp, and world-Z lift. The final pose of each stage became the
start state of the next. Gripper close and open used the existing ROS action.

| Setting | Value |
|---|---:|
| Scene profile | `step_c_cable_wall` |
| Transit padding | `0.020 m` |
| Contact padding | `0.000 m` |
| Cable width / total compression | `6 mm / 2 mm` |
| Calibrated gripper close | `0.7663520720 rad` |
| Compensated descent | `0.0913852610 m` |
| Lift | `0.120 m` |
| Descent / lift execution time | `1.037 s / 3.559 s` |
| Newton FEM cells | `40 x 2 x 2` |
| Newton device | `cuda:0` |

The execution guard measured final tool-position error below one micrometre for
all three stages. It refuses to send a trajectory when any endpoint error
exceeds `0.002 m`.

## Planner evidence

| Metric | Result |
|---|---:|
| Approach planning time | `1.570706 s` |
| Grasp planning time | `0.072951 s` |
| Lift planning time | `0.076487 s` |
| Approach waypoints | `41` |
| Grasp waypoints | `21` |
| Lift waypoints | `41` |
| Maximum tool height on pre-grasp route | `0.375155 m` |
| Minimum fitted-sphere wall distance | `0.034570 m` |

An earlier trial was rejected. cuRobo V2 `plan_grasp()` reported success but
left this robot's grasp phase at pre-grasp height because of the local tool-axis
convention. Newton correctly reported no finger contact and no lift. The final
implementation uses three explicit `plan_pose()` calls and validates their FK
endpoints before execution; it does not alter physics or grasp parameters.

## Newton C1 acceptance

| Metric | Result |
|---|---:|
| `candidate_contact_grasp_pass` | `true` |
| Grasp-region lift | `0.1199769126 m` |
| Release drop | `0.1205940407 m` |
| Left/right loaded soft contacts | `73 / 73` |
| Bilateral lift soft-contact samples | `204` |
| Robot-wall loaded contacts | `0` |
| Robot-wall penetrating contacts | `0` |
| Robot-wall accepted contacts | `0` |
| Raw robot-wall candidates | `83` |
| Newton sampled minimum wall separation | `0.0439252989 m` |
| Object-wall penetration | `0.0 m` |
| Command queue overflow | `0` |
| Finite state | `true` |

The start-state guard passed with maximum error
`1.9436783116e-07 rad`. The command queue drained before result capture. After
the run, bringup and adapter were stopped and the isolated UDP/viewer ports were
clear.

## Evidence files

- `curobo_step_c_pick_20261016_173829.log`: CUDA planning, endpoint guards,
  route metrics, and ROS action execution.
- `jazzy_fem_cable_endpoint_20261016_173748.log`: full Newton contact and phase
  trace.
- `jazzy_fem_cable_endpoint_20261016_173748.result.json`: Newton aggregate
  result.
- `curobo_step_c_c1_20261016_174141.result.json`: combined machine-readable C1
  acceptance; `pass=true`.
- `curobo_step_c_bringup_20261016.log` and
  `curobo_step_c_adapter_20261016.log`: controller and bridge readiness.
