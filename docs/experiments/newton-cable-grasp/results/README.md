# Newton rod-cable MoveIt results

This directory stores compact, versioned evidence for the experimental
`rod_cable` MoveIt path. The accepted FEM result remains under
`docs/experiments/jazzy-fem-migration/results/` and is not replaced by these
experiments.

## 2026-10-08 first integrated attempt

The first CUDA attempt is **not accepted**. The operator saw the cable remain
on the floor even though MoveIt reported `ABSOLUTE POSITION PICK SUCCEEDED`.

What worked:

- endpoint model was on `cuda:0` with 40 cable bodies and 40 cable shapes;
- Newton supplied the measured object center near
  `(0.4869, 0.1091, 0.0030) m`;
- the 6 mm gap calibration selected `0.748815 rad` and `13.679 mm` closure
  compensation;
- all four safe-pregrasp stages, approach, and lift planned at 100%;
- MoveIt/controller execution completed without an error.

What failed physically:

- the cable grasp region rose only about `9.56 mm`, not the required `80 mm`;
- 71 samples had bilateral loaded rigid contact, but contact disappeared early
  in the commanded 120 mm lift;
- later samples kept the closed gripper moving upward while the cable grasp
  region returned to about `3.0 mm` above the floor;
- the endpoint did not emit its final `MOVEIT_GRASP_RESULT`, so the compact
  record derives failure from the saved contact samples and operator
  observation rather than claiming a completed endpoint acceptance result.

This distinguishes planner success from physical grasp success. Do not raise
friction, damping, or compression merely to force a pass. The next session
should first replay a close-only/early-lift diagnostic and compare the cable
contact point with the finger-pad geometry and vertical grasp height.

Machine-readable summary:

- [`first_moveit_attempt_2026-10-08.json`](first_moveit_attempt_2026-10-08.json)

The original runtime logs remained outside the repository under
`~/ur5_ws/run_logs/`:

- `jazzy_cable_preview_20261008_190026.log`
- `jazzy_cable_pick_20261008_190245.log`
- `jazzy_cable_endpoint_20261008_185916.log`
