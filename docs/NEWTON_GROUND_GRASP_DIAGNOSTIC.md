# MoveIt-to-Newton Ground Grasp Diagnostic

Updated: 2026-10-01

This file is the failure index. The complete teaching timeline is in
[`NEWTON_FEM_GRASP_LEARNING_JOURNEY.md`](NEWTON_FEM_GRASP_LEARNING_JOURNEY.md);
the concise reproducible recipe is in
[`HUMBLE_FEM_PICK_SUCCESS.md`](HUMBLE_FEM_PICK_SUCCESS.md).

## Engineering question

Can MoveIt use Newton's measured object pose to approach a segmented strip on
the ground, close the Robotiq gripper from a width calibration, and lift the
strip through simulated contact rather than an attachment constraint?

## Verified components

The following parts were verified independently on the Ubuntu 22.04 / ROS 2
Humble reference system:

- Newton publishes the strip center as a `PoseStamped` in `world`.
- The MoveIt node consumes that pose and plans an absolute pre-grasp.
- A matching floor collision object prevents MoveIt from planning through the
  ground.
- Plan-only, pre-grasp-only, approach-only, and lift-only modes isolate the
  stages of the sequence.
- The complete ROS fake-hardware sequence can finish with
  `ABSOLUTE POSITION PICK SUCCEEDED`.
- The ROS adapter mirrors controller reference positions into Newton after
  explicit synchronization and trajectory-shadow enablement.
- Newton reports finite strip state, finger-strip contact candidates, loaded
  contacts, contact-force magnitudes, penetration, and measured strip rise.

These results verify planning, command transport, kinematic playback, and
contact instrumentation. They do not yet verify a successful ground pickup.

## Gripper kinematic calibration

The Robotiq leader command was sampled against the distance between the two
finger-link frames. An 85 mm nominal opening was used to estimate the pad gap.
The same samples measured the downward shift of the fingertip midpoint during
closure.

| Leader command (rad) | Estimated gap (mm) | Closure drop (mm) |
|---:|---:|---:|
| 0.00 | 85.000 | 0.000 |
| 0.10 | 75.959 | 3.493 |
| 0.20 | 66.266 | 6.517 |
| 0.30 | 56.017 | 9.041 |
| 0.35 | 50.716 | 10.109 |
| 0.40 | 45.315 | 11.042 |

For a 50 mm object with 2 mm total nominal compression, linear interpolation
returns:

```text
target gap             48.000 mm
leader command          0.375145 rad
closure drop           10.578 mm
```

The approach distance is compensated as
`uncompensated_approach_distance - closure_drop`. This prevents the fingertip
midpoint from being commanded below the intended height merely because the
linkage moves downward while closing. The estimated gap remains a kinematic
calibration and must be validated against the actual collision surfaces.

## Controlled diagnostic results

The initial combined ground-grasp acceptance test did not pass. Increasing a single
parameter family at a time produced the following result:

- friction coefficient `1.5`, `3.0`, and diagnostic value `10.0`: no retained
  lift;
- total nominal compression `2 mm` and `4 mm`: no retained lift;
- manual close at approximately `0.394 rad`: bilateral loaded contact was
  measured before the isolated lift;
- isolated `lift_only` motion: the gripper rose and the strip remained on the
  ground immediately.

One representative closed-state measurement was:

```json
{
  "object_center_m": [0.687, 0.109, 0.010],
  "gripper_leader_rad": 0.394,
  "left_loaded_contacts": 25,
  "right_loaded_contacts": 22,
  "left_contact_force_magnitude_sum": 231.335,
  "right_contact_force_magnitude_sum": 250.229,
  "closed_loaded_contact_z_range_m": [-0.0001, 0.0206],
  "measured_closed_grip_lift_m": 0.0,
  "attachment_constraint": "none"
}
```

The initial clean object center was approximately
`(0.4869, 0.10915, 0.0100) m`. The reported `x=0.687 m` therefore also shows
that the accumulated test state had translated by about 0.20 m and must be
reset before the next controlled comparison.

## Engineering conclusion

The failure cannot be assigned to a lack of collision detection: both fingers
had loaded contacts and nonzero contact-force magnitude. It also was not
corrected by greatly increasing friction or nominal compression.

The current scalar force sums do not establish a valid pinch. They can include
forces that push along the strip, react against the ground, or disappear as
soon as upward motion starts. A contact count or force magnitude is therefore
necessary evidence of contact but insufficient evidence of load-bearing
grasping.

## Next controlled experiment

Reset the Newton endpoint to the clean initial object pose and record the
world-frame absolute contact-force components `|Fx|`, `|Fy|`, and `|Fz|` for
each finger during closure and the first lift frames. The expected closing
direction is across the 50 mm strip width. This test distinguishes:

- longitudinal pushing (`|Fx|` dominant),
- opposing lateral pinch (`|Fy|` dominant), and
- upward friction during lift (`|Fz|` present while contact is retained).

Do not increase friction or compression again until this directional evidence
is available.

## Reproducibility boundary

The validated reference environment is Ubuntu 22.04, ROS 2 Humble, Newton
1.5.1, and CPU execution on `aarch64`. Build artifacts and virtual environments
are not portable. Only source, configuration, calibration data, and measured
results belong in version control.

The school system is Ubuntu 24.04 `x86_64` with ROS 2 Jazzy and an NVIDIA GPU.
After the Humble ground-grasp baseline passes, a separate Jazzy port will
rebuild every package and compare the same machine-readable acceptance
metrics. See `JAZZY_PORT_PLAN.md`.

## Resolution: accepted direct FEM ground pick

The later controlled run preserved the FEM, contact, friction, calibration, and
lift parameters and changed the path variable `use_named_start` from `true` to
`false`. The named `test_configuration` detour disappeared. The safe pre-grasp,
vertical approach, and vertical lift paths all reached `100%`; the leader
command was `0.375145 rad`, and the lift was `0.120 m` at velocity scale
`0.030`.

The operator observed all three physical conditions in the Newton viewer:

1. no large initial arm detour;
2. the FEM strip rose with the closed fingers;
3. the strip fell only after the gripper opened.

The terminal log ended with `ABSOLUTE POSITION PICK SUCCEEDED`. This is now the
accepted Humble reference in commit `3546672`. The earlier measurements remain
valuable because they explain why contact count and force magnitude alone were
rejected as acceptance criteria. Automated rise, retention, penetration, and
release assertions remain future work.
