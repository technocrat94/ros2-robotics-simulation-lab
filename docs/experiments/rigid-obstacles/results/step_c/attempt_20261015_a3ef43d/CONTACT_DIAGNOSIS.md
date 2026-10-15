# Step C / C1 robot-wall contact diagnosis

## Scope

This run diagnosed the existing C1 wall-contact count without changing geometry,
padding, contact physics, cable physics, or grasp parameters.  It used commit
`a3ef43da8dfa7a2d1ed1e7ce338406153eec102b`, profile
`step_c_cable_wall`, transit padding `0.020 m`, contact padding `0.000 m`, and
Newton on `cuda:0`.

MoveIt completed the staged motion and reported a maximum end-effector height of
`0.375040 m` and a minimum planned robot-wall distance of `0.025938 m`.  The
physical cable did not lift, so this is diagnostic evidence and not a C1 grasp
acceptance result.

## Result

`maximum_robot_obstacle_contacts=82` counts the largest set of broad/narrow-phase
contact candidates present at one instant.  It does not describe 82 impacts.
The instrumentation retained 2,231 per-contact sample records across diagnostic
snapshots.  Every record contains simulation time, inferred task phase, robot
link, signed separation, penetration depth, force vector and magnitude, and the
candidate/loaded/penetrating/real-touch classification.

All 2,231 records are separated proximity candidates:

- combined Newton contact-detection gap: `0.1010000035 m`;
- signed surface separation: `0.0457940782` to `0.1009670013 m`;
- maximum penetration depth: `0.0 m`;
- maximum contact-force magnitude: `0.0 N`;
- loaded, penetrating, and real-touch samples: `0`;
- maximum simultaneous loaded contacts: `0`;
- maximum simultaneous penetrating contacts: `0`.

The positive signed distances prove that the robot surfaces remained at least
`45.794 mm` away from the wall.  A sample is classified as a real touch only
when force is positive or penetration is positive; none met either condition.

## Samples by task phase

The diagnostic emits a snapshot when a phase is first observed or when the
simultaneous candidate count reaches a new peak.  Consequently, sample-row
counts are evidence rows, not time-integrated collision counts.

| Phase | Sample rows | Simulation-time range | Minimum separation | Link at minimum | Maximum penetration | Maximum force |
|---|---:|---:|---:|---|---:|---:|
| transit | 0 | n/a | n/a | n/a | n/a | n/a |
| descend | 0 | n/a | n/a | n/a | n/a | n/a |
| close | 0 | n/a | n/a | n/a | n/a | n/a |
| lift | 16 | `8.635 s` | `0.0780313784 m` | `ur/robotiq_85_left_finger_link` | `0.0 m` | `0.0 N` |
| release | 2,215 | `10.602`-`17.672 s` | `0.0457940782 m` | `ur/robotiq_85_right_finger_link` | `0.0 m` | `0.0 N` |

The raw endpoint log is the per-contact report.  Search it for
`ROBOT_OBSTACLE_CONTACT_SAMPLE`; every matching JSON record is one sample and
includes the required fields.  Twelve robot links appear in those records:
`wrist_2_link`, `wrist_3_link`, `ur_to_robotiq_link`, the Robotiq base, and the
left/right knuckle, inner-knuckle, finger, and fingertip links.

## Force interpretation

Newton defines the normal constraint as the negative of signed surface
separation.  Its VBD rigid-contact kernel skips the force calculation when that
constraint is non-positive and no previous normal multiplier is loaded.  The
run never observed penetration, so each separated record is reported with
`contact_force_source="vbd_zero_for_nonpenetrating_constraint"` and a zero
force.  Force harvesting remains enabled if a penetrating candidate is ever
observed.

## Proposed follow-up — approval required

Change the acceptance metric named `maximum_robot_obstacle_contacts` to count
only contacts for which `penetration_depth_m > 0` or
`contact_force_magnitude_n > 0`.  Add a unit test proving that a positive-gap
candidate is excluded while penetrating and positively loaded contacts are
included.  Keep the raw candidate metric under an explicit diagnostic name so
proximity information is not lost.

This counting-policy change is only proposed.  It was not implemented, pending
approval.

## Evidence

- `jazzy_fem_cable_endpoint_20261015_181650.log`: complete Newton log and all
  2,231 per-contact JSON samples.
- `jazzy_fem_cable_endpoint_20261015_181650.result.json`: endpoint aggregate
  metrics.
- `jazzy_fem_cable_pick_20261015_c1_diagnosis.log`: complete MoveIt run.
- `step_c_c1_20261015_181650.failed.result.json`: combined result; it remains a
  failed acceptance result under the unchanged candidate-count policy.
