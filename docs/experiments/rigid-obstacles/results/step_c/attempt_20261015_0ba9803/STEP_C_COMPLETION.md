# Step C completion: C1 cable wall and C2 tall-wall control

## Changes under test

Commit `0ba98034794e444455b2aee99fdceb0eac5fbb51` implements the two
approved acceptance fixes:

- `maximum_robot_obstacle_contacts` counts each contact only when its force is
  positive or its penetration depth is positive;
- `maximum_robot_obstacle_candidate_contacts` retains the raw Newton proximity
  candidate maximum;
- gripper phase detection accepts either an absolute open value (`<=0.05 rad`)
  or an opening of at least `0.15 rad` relative to the first observed value;
- direction-aware phase transitions prevent the opening motion from being
  misclassified as close or release.

Dependency-free unit tests cover separated, loaded, penetrating, and
loaded-plus-penetrating contacts without double counting.  They also cover both
a gripper starting at `0 rad` and one starting at `0.79 rad`.

Commit `b8daea1b5474cefed1fbe37533b6912a40ef4300` adds the guarded
`Safe pre-grasp path is incomplete` message to the C2 planning-failure result
classifier.  A unit test covers all accepted failure messages and rejects a
success message.

No geometry, padding, contact physics, cable physics, or grasp parameter was
changed.

## Live run environment

The final run used a fresh four-window `tmux` session named `stepc` with
`bringup`, `endpoint`, `adapter`, and `pick` windows.  All components were tee'd
to timestamped logs.  Before the final attempt, the six fake-hardware arm joints
were verified against `config/jazzy_fem_initial_positions.yaml` both before and
after endpoint/adapter startup.

Newton reported `device=cuda:0` on the NVIDIA GeForce RTX 3080.  The endpoint
reported ready before the pick, the adapter reported `ROS_ADAPTER_READY`, and
the start-state guard passed with maximum error `1.9436783e-07 rad`.  After C1
and C2, the tmux session and all experiment processes were stopped; UDP ports
20000, 20001, and 30000 were clear.

## C1 result — pass

C1 used profile `step_c_cable_wall`, transit padding `0.020 m`, contact padding
`0.000 m`, and planning time `10 s`.

| Metric | Result |
|---|---:|
| MoveIt absolute-position pick | pass |
| Candidate contact grasp pass | `true` |
| Grasp-region lift | `0.1200761129 m` |
| Release drop | `0.1206058939 m` |
| Maximum end-effector height | `0.375040 m` |
| C0 reference maximum height | `0.351039 m` |
| C1 height above C0 | `0.024001 m` |
| Minimum planned robot-wall distance | `0.025937 m` |
| Raw robot-wall candidate maximum | `82` |
| Loaded-or-penetrating acceptance contacts | `0` |
| Loaded robot-wall contacts | `0` |
| Penetrating robot-wall contacts | `0` |
| Minimum sampled robot-wall separation | `0.0457944989 m` |
| Object-wall penetration | `0.0 m` |

All four safe pre-grasp stages, the vertical approach, and the lift reported
`100.0%`.  The cable was held between both fingers during lift: each side
reached 73 loaded soft contacts, and 234 sampled lift frames had bilateral soft
contact.  The endpoint remained finite, the command queue did not overflow, and
the combined C1 result reports `pass=true`.

The C0 height reference is the accepted entry in
`attempt_20261015_b6b31d4/c1_padding_sweep_summary.json`.

## C2 result — pass

C2 used the unchanged `step_c_cable_tall_wall` profile in plan-only mode.  Its
safe pre-grasp Cartesian path reached only `4.0%`, so the guard rejected the
incomplete path.  The machine-readable checks report:

- `planning_failed=true`;
- `plan_only_did_not_succeed=true`;
- `no_robot_command_sent=true`;
- `pass=true`.

## Evidence files

- `jazzy_fem_cable_endpoint_20261015_185054.log` and `.result.json`: complete
  CUDA Newton C1 run and aggregate metrics.
- `jazzy_fem_cable_pick_20261015_185124.log`: complete successful C1 MoveIt run.
- `step_c_c1_20261015_185422.result.json`: combined C1 acceptance result.
- `step_c_c2_20261015_185547.log` and `.result.json`: complete C2 negative
  control and machine-readable pass result.
- `step_c_bringup_20261015_185013.log` and
  `step_c_adapter_20261015_185013.log`: final fresh bringup and adapter evidence.

Repository CI passed after the metric, phase-detection, and C2-classifier tests
were added.
