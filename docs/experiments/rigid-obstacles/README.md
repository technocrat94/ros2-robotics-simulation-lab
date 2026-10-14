# Rigid-obstacle milestone: Steps A/B and controlled Step C attempts

Steps A and B validate the obstacle physics, planning controls,
planning-scene cleanup, and Newton endpoint integration.  The latest Step C
attempt restores the verified default cable grasp and completes C0 with the
moved cable.  C1 stops before execution because MoveIt rejects an internally
self-colliding planned path; no complete grasp around an obstacle is claimed.

The single source of truth is
[`config/obstacles.yaml`](../../../config/obstacles.yaml).  Obstacles remain
disabled by default, so the existing grasp scene is unchanged unless a test
explicitly enables them.

## Step A0 — rigid-sphere smoke test

The retained A0 test drops a rigid sphere onto the configured static box.  On
home ARM64 CPU, commit `a6d2af3fa97bfbc2955e97427e6371132ddd2df0` passed:

- maximum penetration: `0.0 m`
- settled and finite: `true`
- real-time factor: `0.686`

Result: [`step_a0_sphere_cpu_arm64.json`](results/step_a0_sphere_cpu_arm64.json)

## Step A1 — FEM-strip contact test

A0 cannot validate VBD soft contact.  A1 therefore drops the FEM strip
(`252` particles, `600` tetrahedra) onto the same box using full-surface
rigid-soft contact.  Its contact law comes from `model.soft_contact_ke`,
`model.soft_contact_kd`, and `model.soft_contact_mu`; the box's rigid-body
`ke/kd` values do not substitute for them.

The run stopped automatically after the one-second center-height range fell
below the settling threshold:

- soft-contact candidates observed: `220`
- maximum particle penetration: `0.000213 m` (`0.213 mm`, below `1 mm`)
- settling time / wall time: `8.717 / 49.681 s`
- finite and settled: `true`

Result: [`step_a1_fem_strip_cpu_arm64.json`](results/step_a1_fem_strip_cpu_arm64.json)

## Step B — MoveIt positive and negative controls

All Step B cases used one unchanged MoveIt bringup and target `x=0.20 m`.
Commit `9695193eb107b31ebaff6d795a16b879071af87e` produced:

| Case | Maximum tool height | Minimum robot-to-box distance | Outcome |
|---|---:|---:|---|
| Box disabled | `0.308737 m` | `-0.060328 m` | route succeeds but intersects the hypothetical box |
| Box enabled, OMPL | `0.401116 m` | `0.003664 m` | route changes and remains clear |
| Box enabled, explicit safe route | `0.451119 m` | `0.012481 m` | all spatial stages reach 100% |
| Tall wall enabled | n/a | n/a | planning fails as required |

This is the required causal control: the target is identical, while the
enabled scene produces a higher route and positive clearance.

A second negative control replaced the box with a wall from `z=0` to `2 m`
and `4 m` across Y.  The requested pose is separated by a barrier taller than
the reachable safe-transit height, and planning failed.  The test rejects any
run that prints `PLAN_ONLY_SUCCEEDED` for this wall.

Finally, after enabled and tall-wall runs, a default disabled run sent a
`CollisionObject::REMOVE` for `grasp_route_box`.  The planning-scene query
reported `confirmed_absent=true`; this prevents an object from persisting in
`move_group` across node runs.

- machine-readable result:
  [`step_b_20261014_021854.result.json`](results/step_b_20261014_021854.result.json)
- complete log:
  [`step_b_20261014_021854.log`](results/step_b_20261014_021854.log)

### Evidence provenance

All paths below are repository-relative.  The commit is the exact source
recorded by the test, not the later documentation commit.

| Evidence | Status | Tested commit | Repository-relative path |
|---|---|---|---|
| Step A0 sphere | accepted | `a6d2af3fa97bfbc2955e97427e6371132ddd2df0` | `docs/experiments/rigid-obstacles/results/step_a0_sphere_cpu_arm64.json` |
| Step A1 FEM strip | accepted | `a6d2af3fa97bfbc2955e97427e6371132ddd2df0` | `docs/experiments/rigid-obstacles/results/step_a1_fem_strip_cpu_arm64.json` |
| Step B controls, run `20261014_021854` | accepted | `9695193eb107b31ebaff6d795a16b879071af87e` | `docs/experiments/rigid-obstacles/results/step_b_20261014_021854.result.json` and `.log` |

The following runs remain in the repository as diagnostic history and must
not be used as acceptance evidence:

| Run | Status | Reason | Retained evidence |
|---|---|---|---|
| `20261014_021734` | superseded / incomplete | The enabled same-target OMPL route was absent, so route metrics and causal comparison were incomplete. | `docs/experiments/rigid-obstacles/results/step_b_20261014_021734.superseded.log` and `.result.json` |
| `20261014_022739` | superseded / incomplete | The run stopped after adding the floor and produced no result JSON. | `docs/experiments/rigid-obstacles/results/step_b_20261014_022739.superseded.log` |

## Newton endpoint integration before Step C

When `enabled=false`, `build_scene()` receives no obstacle, adds no shape, and
the endpoint reports:

- `obstacles_enabled=false`
- `obstacle_shapes=0`
- `maximum_robot_obstacle_contacts=0`
- `object_obstacle_max_penetration_m=0.0`

When enabled, the endpoint reads the same config, creates one static box with
rigid and particle collision enabled, and deliberately adds no robot-obstacle
filter.  The smoke run reports `obstacle_shapes=1`,
`maximum_robot_obstacle_contacts=0`, and
`object_obstacle_max_penetration_m=0.012690`.

That last value is diagnostic rather than a pass claim: the Step B route box
overlaps part of the strip's initial ground pose.  The new metric exposes this
condition before Step C instead of silently hiding it.

- enabled: [`JSON`](results/newton_endpoint_obstacles_enabled_cpu_arm64.json),
  [`log`](results/newton_endpoint_obstacles_enabled_cpu_arm64.log)
- disabled: [`JSON`](results/newton_endpoint_obstacles_disabled_cpu_arm64.json),
  [`log`](results/newton_endpoint_obstacles_disabled_cpu_arm64.log)

## Configuration and CI guards

- `pick_at_position.cpp` has no numeric obstacle fallbacks.  All obstacle
  parameters must be supplied by the launch/config path.
- CI rejects every configured box dimension below `0.01 m`, reducing the risk
  of one-step tunneling.
- `candidate_contact_grasp_pass` now also requires
  `maximum_robot_obstacle_contacts == 0`.  A dependency-free unit check covers
  zero, one, and multiple obstacle contacts.
- The earlier provenance guard in commit `8342375` remains unchanged.

## Step C attempt — cable moved behind a wall

Commit `caa568994c0250c86c1dab3cb0d91fc5b603b0a5` keeps the verified FEM
cable, grasp settings, Jazzy start joints, and Newton `BASE_ARM` unchanged.
The shared profile moves the cable center to `(0.487, -0.180, 0.004) m`.
The approved wall is centered at `(0.487, -0.035, 0.10) m` with size
`(0.20, 0.05, 0.20) m`.  Both Newton and MoveIt read the same profile.

### Geometry preflight

The first `0.30 m` wall was too close to `upper_arm_link` along X.  Shortening
it to `0.20 m` changed the limiting geometry instead of merely moving the
wall in Y:

| Measurement | Result |
|---|---:|
| Closest start-pose link | `ur/robotiq_85_right_finger_link` |
| Closest-shape separation direction | `y` |
| Robot-to-wall distance | `0.049792 m` |
| Clearance after `0.020 m` padding | `0.029792 m` |
| Cable surface-to-wall-face gap | `0.117000 m` |
| Open-finger-to-wall clearance | `0.043792 m` |
| Newton `BASE_ARM` minimum wall distance | `0.084727 m` |
| Straight horizontal sweep intersects padded wall | `true` |

All geometry checks passed.  Because `BASE_ARM` is collision-free, Newton
counts robot-obstacle contacts from simulation start; no post-sync exclusion
is used.  Machine-readable evidence is in
[`step_c_geometry_20261014_035919.result.json`](results/step_c/attempt_20261014_caa5689/step_c_geometry_20261014_035919.result.json).

### Ten-second plan-only checks

Both profiles planned at the original `10 s` limit, so the authorized `30 s`
fallback was not used:

| Profile | Planning time used | Solver time | Maximum EE height | Minimum robot-to-wall distance | Outcome |
|---|---:|---:|---:|---:|---|
| C0, no wall | `10 s` | `0.112 s` | `0.990860 m` | `0.036955 m` | pass, no command sent |
| C1, wall | `10 s` | `0.197 s` | `0.842469 m` | `0.029870 m` | pass, no command sent |

These two random OMPL samples do not prove that the wall route is higher; the
sampled C1 route was lower.  The required C0/C1 comparison therefore remains
an acceptance item for a future complete run.

The earlier no-wall attempt at `04:00:08` timed out while a stale
`pick_at_position` process was still registered.  It is retained as
superseded evidence and was not used for the table.  The clean reruns are
[`no wall`](results/step_c/attempt_20261014_caa5689/step_c_no-wall_20261014_040036.log)
and [`with wall`](results/step_c/attempt_20261014_caa5689/step_c_with-wall_20261014_040042.log).

### C0 stopped before grasp

The full no-wall control reached and executed the absolute pre-grasp pose.
Its vertical Cartesian approach then reached only `89.5%`.  The safety guard
rejected the incomplete path before closure, lift, or contact.  Therefore:

| Case | Result | Lift | Wall contacts | Failure stage |
|---|---|---:|---:|---|
| C0 | fail / retained diagnostic | `0.0 m` | `0` | vertical approach incomplete (`89.5%`) |
| C1 | not run | n/a | n/a | C0 must pass first |
| C2 | not run | n/a | n/a | C0 must pass first |

This failure belongs to the MoveIt approach geometry, not FEM grasp physics:
the gripper never reached the cable.  In accordance with the experiment rule,
C1 and C2 were not run and no contact, material, cable, wall, or grasp
parameter was changed.

- failed C0 result:
  [`step_c_c0_20261014_040130.failed.result.json`](results/step_c/attempt_20261014_caa5689/step_c_c0_20261014_040130.failed.result.json)
- MoveIt log:
  [`jazzy_fem_cable_pick_20261014_040130.log`](results/step_c/attempt_20261014_caa5689/jazzy_fem_cable_pick_20261014_040130.log)
- Newton diagnostic log:
  [`jazzy_fem_cable_endpoint_20261014_040113.log`](results/step_c/attempt_20261014_caa5689/jazzy_fem_cable_endpoint_20261014_040113.log)

## Corrective Step C attempt at `719b925`

The earlier C0 failure was a padding regression rather than a cable-position
failure.  Applying `0.020 m` padding during the vertical contact approach made
the padded fingertips meet the floor before the verified `tool0 z=0.172 m`
target, so the Cartesian fraction stopped at `89.5%`.  Commit
`719b92588c52253ae753ae8ba06534e20a533ee5` restores the default padding to
zero and separates padding by motion phase:

- transit: profile padding (`0.020 m` for Step C);
- contact approach, close, lift, and open: `0.000 m`, matching the verified
  grasp conditions.

The default-profile regression proof used the original cable position and
zero padding for both phases.  It passed with `0.120048 m` lift and
`0.117507 m` release drop.  Its endpoint result is
[`jazzy_fem_cable_endpoint_20261015_015615.result.json`](results/step_c/attempt_20261015_719b925/jazzy_fem_cable_endpoint_20261015_015615.result.json).

The Step C sequence then produced:

| Case | Transit / contact padding | Result | Lift | Release drop | Robot-wall contacts | Maximum EE height | Minimum wall distance |
|---|---:|---|---:|---:|---:|---:|---:|
| Default cable regression | `0.000 / 0.000 m` | pass | `0.120048 m` | `0.117507 m` | `0` | n/a | n/a |
| C0, moved cable without wall | `0.020 / 0.000 m` | pass | `0.120082 m` | `0.117480 m` | `0` | `0.495210 m` | `0.023920 m` (reference wall) |
| C1, moved cable with wall | `0.020 / 0.000 m` | fail before execution | `0` | `0` | n/a | n/a | n/a |
| C2, tall wall | n/a | not run | n/a | n/a | n/a | n/a | n/a |

C1 did not time out.  OMPL returned a 319-state path quickly, but MoveIt's
`ValidateSolution` adapter rejected state 269 as an
`INVALID_MOTION_PLAN`: the padded `forearm_link` collided with
`robotiq_85_left_finger_link`.  No robot command was sent.  Therefore the
authorized 30-second timeout fallback does not apply, and C2 was not run.

- accepted C0 result:
  [`step_c_c0_20261015_020824.result.json`](results/step_c/attempt_20261015_719b925/step_c_c0_20261015_020824.result.json)
- C0 MoveIt and Newton logs:
  [`pick`](results/step_c/attempt_20261015_719b925/jazzy_fem_cable_pick_20261015_020056.log),
  [`endpoint`](results/step_c/attempt_20261015_719b925/jazzy_fem_cable_endpoint_20261015_020035.log)
- retained C1 failure result:
  [`step_c_c1_20261015_021018.failed.result.json`](results/step_c/attempt_20261015_719b925/step_c_c1_20261015_021018.failed.result.json)
- C1 failure evidence:
  [`pick`](results/step_c/attempt_20261015_719b925/jazzy_fem_cable_pick_20261015_021018.log),
  [`MoveGroup`](results/step_c/attempt_20261015_719b925/step_c_c1_bringup_20261015.log),
  [`interrupted endpoint`](results/step_c/attempt_20261015_719b925/jazzy_fem_cable_endpoint_20261015_020956.log)

## Scope boundary

Step C0 is now accepted.  C1 remains blocked at planning validation and C2
remains unrun.  The next change needs an explicit decision about how transit
padding should interact with adjacent-link self-collision checks; no geometry,
physics, grasp parameter, or acceptance threshold was changed in this attempt.
