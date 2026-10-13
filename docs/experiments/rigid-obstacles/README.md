# Rigid-obstacle milestone: corrected Steps A and B plus Newton integration

This milestone stops before Step C.  It validates the obstacle physics,
planning controls, planning-scene cleanup, and Newton endpoint integration
without claiming a complete grasp around an obstacle.

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

## Scope boundary

Step C has not been run.  No complete grasp-with-obstacle success claim is
made by this milestone.
