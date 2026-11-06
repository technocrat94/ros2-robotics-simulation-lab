# Static obstacle avoidance from the simulated link camera

Check date: 2026-11-06 (Asia/Taipei)

## Verdict

The existing Newton 1.5.1 camera and cuRobo 0.8 source checkout can form an
offline, camera-only static-obstacle planning pipeline without ROS, Newton
physics, nvblox, cuMotion, installation, or a real robot. A single start view
was incomplete and its plan collided with truth. Merging four simulated camera
views produced a conservative cuboid and cuRobo avoided the true obstacle with
`23.93 mm` minimum clearance in all three repeats.

The reproducible entry point is
[`scripts/run_sim_camera_curobo.sh`](../scripts/run_sim_camera_curobo.sh). It
runs [`scripts/newton_camera_static_capture.py`](../scripts/newton_camera_static_capture.py)
in the Newton environment, exchanges a temporary compressed NPZ file, then
runs [`scripts/curobo_camera_static_demo.py`](../scripts/curobo_camera_static_demo.py)
in the cuRobo environment. The temporary file is deleted when the launcher
exits.

## Camera mount and accuracy rerun

The user-matched values in
[`config/sim_camera_mount.yaml`](../config/sim_camera_mount.yaml) are parent
`ur/tool0`, left `0`, up `-0.106 m`, forward `0.056 m`, tilt `0 deg`, and
`status: matched_by_eye`. At this placement neither the compared real view nor
the simulation shows the gripper. The simulated accuracy rerun agrees: robot
pixels were `0.0000%` at all three poses.

| Arm pose | Correct abs median | Correct abs p95 | Negative-control abs p95 | Robot points |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 0.000012 mm | 0.000044 mm | 151.970 mm | 0.0000% |
| Shoulder lift +0.15 | 0.000014 mm | 0.000043 mm | 114.782 mm | 0.0000% |
| Pan +0.25, elbow -0.15 | 0.000019 mm | 0.000069 mm | 188.423 mm | 0.0000% |

All poses pass median `< 1 mm` and p95 `< 5 mm`; treating ray distance as
forward depth is still detected by the negative control. These sub-micrometre
numbers are exact-simulation reconstruction residuals, not real D435 accuracy.

## Why two processes

Direct import checks showed:

| Interpreter | Newton | PyTorch | cuRobo |
| --- | --- | --- | --- |
| `~/newton_ws/.venv-cpu/bin/python` | yes | no | source is discoverable from the checkout, but unusable without PyTorch |
| `~/curobo_ros_env/bin/python` | no | yes | yes |

The environments therefore cannot host the full pipeline together. The
launcher does not alter either environment: Newton writes world points, camera
poses, scan arm poses, and metadata to one NPZ file under `/tmp`; cuRobo reads
it in the second process. No package was installed to merge them.

## Point cloud to cuRobo obstacle

The checked-out source is `v0.8.0-43-g78fd485`. Its `curobo.scene` exports both
`Cuboid` and `VoxelGrid`. `VoxelGrid` explicitly requires an ESDF feature
tensor, so raw occupied points are not the simple voxel input they first
appear to be. Building TSDF/ESDF or using the newer perception mapper would add
an unnecessary mapping system to this static check. Mesh reconstruction was
also rejected because it is more complex and fragile for partial surfaces.

The selected representation is one axis-aligned `Cuboid` fitted from the
camera surface points. Points are downsampled on an `8 mm` grid. The fit uses
the observed min/max with `15 mm` XY padding and `130 mm` upward/downward Z
padding, clamped at the floor. The Z margin deliberately turns partial visible
faces into a conservative solid; it is not truth geometry supplied to cases C
or D. The start image produced 229 points and fit center/dimensions
`[0.4480, 0.2630, 0.1277] / [0.1521, 0.0899, 0.2554] m`. The four-view merge
produced 543 points and fit `[0.4870, 0.2630, 0.1571] /
[0.2300, 0.0900, 0.3142] m`.

The perceived points lie on the true cuboid surface with `0.000020 mm` median,
`0.000095 mm` p95, and `0.000100 mm` maximum absolute surface error. The
remaining difference between fitted and true cuboids comes from visibility and
the stated conservative margins, not point reconstruction error.

## Pipeline and scene

Newton renders 848 x 480 forward depth and shape indices from the matched
link camera. Every capture back-projects the same-frame depth and pose, removes
all robot shape indices, removes ground points at `z <= 0.012 m`, and then
downsamples. The ground is excluded explicitly because turning the visible
floor into a cuboid would put the fixed-base robot in contact with a huge
planner obstacle and could block every plan. This experiment evaluates an
above-floor transit; robot self-collision remains enabled, and the floor is not
silently converted into a perceived obstacle.

The required start is loaded from `config/jazzy_fem_initial_positions.yaml`;
the goal is the `step_c_cable_no_wall` cable pre-grasp computed by
`curobo_dodge_demo.py`'s shared preflight code. Their tool positions are about
`[0.488, 0.116, 0.301]` and `[0.487, 0.410, 0.263] m`.

The true main obstacle is a static cuboid centered at
`[0.487, 0.263, 0.160] m`, dimensions `[0.200, 0.060, 0.300] m`. It lies
between start and goal across the almost constant-X tool route and reaches
`z=0.310 m`. This is not just a geometric assertion: case A's no-obstacle path
penetrates the true cuboid by `75.79 mm`, proving that the scene distinguishes
avoidance from an accidentally clear route.

## A/B/C/D comparison

Each case used the same start and goal for three seeded repeats. “Success” is
cuRobo returning a path; true safety is reported separately. Every trajectory
is scored against the true cuboid with all robot collision spheres, never
against its perceived planner cuboid. Path length is cumulative joint-space L2
distance in radians. Planning and conversion timers are separate.

| Case | Planner obstacle | Plans | Median plan | Min true clearance | Median path length | True-safe? |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| A | none | 3/3 | 63.78 ms | -75.79 mm | 0.9327 rad | no |
| B | exact true cuboid | 3/3 | 70.18 ms | +9.44 mm | 1.2553 rad | yes |
| C | start-camera fit | 3/3 | 69.57 ms | -47.22 mm | 1.1647 rad | no |
| D | four-view merged fit | 3/3 | 70.70 ms | +23.93 mm | 1.3100 rad | yes |

The first A query included a lazy CUDA cost of about 1.48 s; the requested
median remains about 64 ms because the following repeats were warm. Camera
back-projection, robot/ground filtering, and 8 mm downsampling had a median of
`54.51 ms` across the five captures. AABB conversion was `0.062 ms` for the
start cloud and `0.048 ms` for the merged cloud. These timings are measured
separately from planning.

Case C is an important negative result: the start camera sees only a partial
height/width of the wall, so the conservative one-view fit is still incomplete
and the returned path is not truly safe. Case D observes enough complementary
surfaces for the conservative fit to clear truth.

## Blind-spot test

The blind-test cuboid is centered at `[0.487, 0.390, 0.160] m`, dimensions
`[0.160, 0.050, 0.280] m`. It is outside the start-pose camera frustum: Newton
reported exactly zero obstacle pixels and zero perceived points. No perceived
obstacle was therefore sent to the planner. cuRobo returned the same short
path family as case A, and all three paths hit the true blind obstacle; minimum
true clearance was `-69.22 mm` with median planning time `62.97 ms`.

This is the expected limitation of a single eye-in-hand view. No hidden truth,
synthetic fill, or blind-spot fix was added to case C or the blind test.

## Viser and what you should see

The viewer binds to `127.0.0.1:8084`. It plans all cases once, so the selector
can switch between A, B, C, D, and blind spot without rerunning the script.
You should see:

- the UR5/Robotiq at the recorded start pose;
- a red true obstacle with a **Show true obstacle** checkbox;
- cyan perceived cuboids for C, D, or the blind case when one exists;
- the yellow start-camera frustum;
- the selected planned tool path and signed true-clearance text;
- a **Play selected path** button that animates the arm along that case.

The one-second Viser smoke test started on port 8084 and stopped cleanly. The
headless end-to-end launcher also removed its NPZ exchange file. No process was
left running and no media was recorded.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_curobo.sh
```

Use Ctrl-C in that terminal to stop the viewer; the launcher then removes its
temporary exchange file.
