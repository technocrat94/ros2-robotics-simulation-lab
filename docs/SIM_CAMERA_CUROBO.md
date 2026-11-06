# Static obstacle avoidance from the simulated link camera

Check date: 2026-11-06 (Asia/Taipei)

## Verdict

> **Second-pass correction:** the original D verdict below is not accepted as
> camera evidence. Case C's highest observed point was only `z=0.1254 m` and
> D's was only `z=0.1842 m` on a wall whose true top was `z=0.3100 m`. D's
> fitted top of `z=0.3142 m` depended on `130 mm` of Z padding, landing about
> `4 mm` above truth without ever seeing the top. Its old measurements remain
> below as history; the padding-free and look-while-moving rerun is in
> **Second pass**.

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

## Second pass

Check date: 2026-11-06 (Asia/Taipei)

The second pass removes the fixed `130 mm` crutch and uses a persistent Newton
worker controlled by the cuRobo process over JSON lines. Each capture is passed
through a temporary NPZ and immediately deleted. Truth geometry is available
only to case B and offline scoring; C, D, and E construct planner cuboids solely
from observed points. Raw results and the compact run log are in
[`docs/experiments/sim-camera-curobo/results/`](experiments/sim-camera-curobo/results/).

### Planned observation motion

The old four views had been teleported. In this pass every observation move is
planned and every interpolated state is scored against truth. The overview is
chosen without wall coordinates: start from the recorded tool pose, retain its
already downward-looking orientation, translate `+0.20 m` in world Z, then for
the full scan observe at fixed `+0.12 m` and `-0.12 m` world-X offsets at that
height. The arm plans back to the recorded start before the final cable
pre-grasp plan.

For wall tops `0.25` and `0.31 m`, all overview/full-scan motions were clear and
returned to start. On the `0.40 m` wall, the first attempted raise crossed truth
with `-60.71 mm` clearance, so the scan stopped after the start capture. That
case is a scan failure even though cuRobo, unaware of the missing wall height,
still returned a final path.

### Padding sweep at the 0.31 m wall

The new high scan truly sees the top: maximum observed `z=0.310000 m`. The same
2,476 observed points and safe scan path (`+24.94 mm` minimum true clearance)
are used for every row. No setting was tuned after seeing the outcome.

| Z padding | Highest seen | Fitted top | Final plans | Min final true clearance | Whole case |
| ---: | ---: | ---: | ---: | ---: | --- |
| 0 mm | 0.310 m | 0.310 m | 3/3 | +19.47 mm | pass |
| 20 mm | 0.310 m | 0.330 m | 3/3 | +29.29 mm | pass |
| 50 mm | 0.310 m | 0.360 m | 0/3 | no plan | fail/hold |
| 80 mm | 0.310 m | 0.390 m | 0/3 | no plan | fail/hold |
| 130 mm | 0.310 m | 0.440 m | 0/3 | no plan | fail/hold |

The larger paddings are not safer here: their single fitted box covers a start
or goal state, so cuRobo reports `Start or End state in collision`. This is why
the second-pass default is fixed at `20 mm`, not selected per scene.

### Fixed 20 mm padding across wall heights

The full scan and the overview-only scan were run independently. “Scan min”
includes the planned move out and return; a negative value fails the entire
case. “Top seen” allows one `8 mm` downsample voxel of tolerance.

| Scan | True top | Highest seen | Fitted top | Top seen? | Scan min | Final plans | Final min | Result |
| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| Full | 0.25 m | 0.250 m | 0.270 m | yes | +26.44 mm | 3/3 | +29.83 mm | pass |
| Full | 0.31 m | 0.310 m | 0.330 m | yes | +24.94 mm | 3/3 | +29.29 mm | pass |
| Full | 0.40 m | 0.125 m | 0.145 m | no | -60.71 mm | 3/3 | -73.93 mm | fail |
| Overview | 0.25 m | 0.250 m | 0.270 m | yes | +29.94 mm | 3/3 | +29.83 mm | pass |
| Overview | 0.31 m | 0.310 m | 0.330 m | yes | +28.43 mm | 3/3 | +29.29 mm | pass |
| Overview | 0.40 m | 0.125 m | 0.145 m | no | -60.71 mm | 3/3 | -73.93 mm | fail |

For the two reachable heights, the fixed overview alone sees the actual top;
the extra lateral views add surface coverage but are not needed to establish
height. The `0.40 m` result is intentionally not repaired: the arm collides on
the way to the overview before it can take that image.

### Case E: look while moving

E repeats: capture at the current joint state, merge all observations, refit
with fixed `20 mm` Z padding, replan from that state, then execute at most
`0.05 rad` cumulative joint-space path length. Every executed state is scored
against truth. It stops at the goal, on true collision, or on no plan; on no
plan it holds the current state.

| Scene | Cycles | No-plan cycles | First seen | Clearance when first seen | Min executed clearance | Goal? | Outcome | Median capture | Median plan |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- | ---: | ---: |
| Main 0.31 m wall | 3 | 1 | cycle 1 | +29.29 mm | +4.03 mm | no | held on no plan, no hit | 86.14 ms | 1,469.14 ms |
| Existing blind wall | 7 | 1 | cycle 3 | +63.06 mm | +31.96 mm | no | held on no plan, no hit | 34.11 ms | 61.09 ms |

For the main wall, E safely executed two `0.05 rad` portions. Its accumulated
highest seen Z increased from `0.1254` to `0.1532 m`; at cycle 3 it saw
`0.1854 m`, but the resulting partial cuboid left no plan, so it held with
`+4.03 mm` minimum executed clearance. It did not reach the goal.

The blind wall produced exactly zero points in cycles 1 and 2. It was first
visible in cycle 3 as 15 points while the arm remained `63.06 mm` from truth;
the minimum-point threshold delayed a planner cuboid until cycle 4. E continued
safely through cycle 6, then the grown partial box produced no plan in cycle 7.
It held at `+31.96 mm` minimum executed clearance, did not hit the wall, and did
not reach the goal. No truth-based blind-spot fix was added.

Capture time includes Newton rendering, same-frame copies, back-projection,
robot/ground removal, and downsampling. Plan time is separate. Main E's median
is high because two replanners paid fresh lazy CUDA costs; the raw per-cycle
times are retained in JSON.

### Seeded repeats

The three seeds did not provide meaningful route diversity. With 0 mm padding,
all three paths were identical within `1e-5 rad` path length. At 20 mm and the
passing height cases, the first post-warmup path differed only slightly while
repeats 2 and 3 were identical. The 50/80/130 mm cases consistently returned
no path, and the 0.40 m final paths were identical unsafe paths.

### Viser and what you should see

The second-pass viewer requires `127.0.0.1:8084` and refuses to silently move
to another port. It precomputes padding, height, overview, E, and E-blind cases,
so they can be selected without rerunning. You should see:

- the robot, red true obstacle, yellow perceived points, and cyan fitted box;
- a yellow line at highest observed Z and a magenta line at padded fitted top;
- one capture frustum at every observation pose;
- a live frustum following the robot during playback;
- selectable padding, wall-height, overview, E, and blind-E cases;
- E point clouds and fitted boxes growing cycle by cycle during playback;
- live signed true clearance and a true-obstacle visibility checkbox.

Known limits are deliberately unchanged: all perceived points become one box,
so separate obstacles merge; all `z <= 0.012 m` points are deleted, including a
cable; and robot removal uses the simulator shape index, which a real camera
does not provide.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_curobo_second_pass.sh
```

The validated headless run wrote only a 37 KiB JSON and 773-byte log, with no
NPZ committed. During the Viser control smoke test, an unrelated pre-existing
`python` process (PID 837788) owned `127.0.0.1:8084`; it was not stopped. Viser
therefore exercised the complete UI on its automatic fallback `8085` and
stopped cleanly. The final code now checks the actual bound port and refuses
that silent fallback, so it will use exactly 8084 or report that the required
port is busy. All Newton workers started by this check stopped. No media was
recorded.

## Third pass: valid scenes, playback, and explained stops

Check date: 2026-11-06 (Asia/Taipei)

This pass applies the requested validity gate before running a case: every
collision sphere must have at least `30 mm` true clearance at both endpoints,
and the no-obstacle trajectory must penetrate truth. Invalid cases remain in
the JSON and viewer as `INVALID_SCENE`, but are excluded from pass/fail.
Start, goal, and interior along-path clearances are separate fields in
[`third_pass.json`](experiments/sim-camera-curobo/results/third_pass.json).

The loaded model puts `tool0` at `[0.4880, 0.1160, 0.3010] m`. Newton FK from
the same loaded URDF puts the `wrist_1_link` origin at
`[0.5731, 0.1574, 0.3833] m`; this differs in X/Y from the reviewer's hand
estimate `[0.525, 0.256, 0.383] m`, while agreeing in Z. Clearance decisions
use every cuRobo collision sphere, not either link-origin estimate.

### Scene-validity results

All three mandated new main walls use the original XY footprint. All three
are invalid by the literal endpoint threshold: their goal clearance is
`29.833 mm`, just `0.167 mm` below `30 mm`. They do block the route, but that
does not override the endpoint rule.

| Main top | Start | Goal | Interior no-obstacle path | Status |
| ---: | ---: | ---: | ---: | --- |
| 0.18 m | +38.436 mm | +29.833 mm | -40.614 mm | INVALID_SCENE |
| 0.22 m | +34.531 mm | +29.833 mm | -46.673 mm | INVALID_SCENE |
| 0.26 m | +34.531 mm | +29.833 mm | -64.353 mm | INVALID_SCENE |

Consequently the 0/20 mm padding sweep, overview planning, and E-main run are
marked skipped; running them would violate the instruction to use valid
scenes. The historical 0.25/0.31/0.40 rows above remain unchanged as second-
pass measurements, not third-pass passes. Under the new strict endpoint gate,
the old main footprint also has only about `29.83 mm` goal clearance; the
0.40 m case is additionally invalid because its start is already in collision
(`-60.71 mm`). The old blind wall is invalid because it contains the goal.

The deterministic blind-wall search therefore shifted Y, as requested, and
also narrowed/offset X to avoid the endpoint while still intersecting a robot
link on the route. The selected cuboid is centered at
`[0.567, 0.320, 0.130] m`, dimensions `[0.050, 0.040, 0.240] m`. It has
`+96.860 mm` start clearance, `+35.963 mm` goal clearance, and the baseline
path penetrates it by `15.690 mm`. The start render contains exactly zero
obstacle pixels and zero perceived points.

### E reuse, timing, and stop reason

E owns one warmed planner. Its preallocated `camera_obstacle` pose is updated
through the collision checker and its dimensions through cuRobo 0.8's
`CuboidData.update_dims`; the graph buffer is reset without rebuilding or
rewarming the planner. Whole-cycle time covers capture, conversion, merge,
fit, in-place scene update, and plan. In the validated playback run the first
cycle was `77.41 ms`, subsequent cycles had an `82.45 ms` median, capture had
a `15.62 ms` median, and planning had a `64.46 ms` median.

Blind E reached neither the obstacle nor a fitted-box planner stop early: it
saw zero points in every cycle, so no fitted box existed and the per-cycle
current/goal-to-fit clearances are correctly `null`. It followed the
no-obstacle replans and collided with truth in cycle 9 at `-9.746 mm`.
`final_stop_reason` is `TRUE_COLLISION`; `goal_reached` is false. Thus no stop
was caused by a fitted box covering the current state or goal.

### Playback cause and verification

The no-op playback was an uncaught background-thread exception.
`clearance_series()` returned shape `[N, 1]`; formatting
`clearances[index]` with `+.4f` raised
`TypeError: unsupported format string passed to numpy.ndarray.__format__`.
The function now returns one scalar per joint state, playback explicitly casts
the displayed value, and every worker exception is printed to stderr and
shown in a **Playback error** GUI field.

The Viser smoke test invoked the registered click callback for padding,
standard scan, overview, E-main, E-blind, and explicit invalid-scene case
types from non-main callback threads. Every callback changed both the commanded
robot joints and the live frustum, and no callback raised. The live frustum now
reads [`config/sim_camera_mount.yaml`](../config/sim_camera_mount.yaml) rather
than embedding its mount pose. The verification used port 8085; the normal
command below still requires the requested `127.0.0.1:8084`.

What you should see:

- invalid main-wall cases clearly labelled `INVALID_SCENE`, with the reason in
  the **Case status / no-plan reason** field;
- the valid, narrow blind wall in red and no yellow perceived points;
- a live white camera frustum that follows the arm using the YAML mount;
- E-blind playback advancing the robot until the measured truth collision;
- live signed clearance plus a visible **Playback error** field that remains
  `none` on a successful playback.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_curobo_second_pass.sh
```

The run stopped its Newton worker and Viser server. No ROS, Newton physics,
nvblox, cuMotion, real robot, install, or media path was used.

## Fourth pass: camera side correction and completed matrix

Check date: 2026-11-06 (Asia/Taipei)

This pass uses one named `25 mm` endpoint-clearance threshold and rotates the
experiment start and goal by pi about tool Z. The mount YAML is unchanged.
The loaded model confirms the reviewer's offsets: at the mount-match pose,
camera minus tool flange is `[+0.106, 0.000, -0.056] m`; at the original
start it is `[-0.106, 0.000, -0.056] m`. The new camera therefore sits away
from the base.

`wrist_3_joint` changes from `-5.830452` to `-2.688859 rad`, within its
`[-2 pi, +2 pi]` model limits. The 112 collision spheres before and after the
flip match with `0.000 mm` maximum bidirectional center deviation, which is
inside the `2 mm` symmetry criterion.

| Pose | Flange before/after (m) | tool0 before/after (m) | Camera before (m) | Camera after (m) |
| --- | --- | --- | --- | --- |
| Start | `[0.488, 0.116, 0.301]` / same | `[0.488, 0.116, 0.301]` / same | `[0.382, 0.116, 0.245]` | `[0.594, 0.116, 0.245]` |
| Goal | `[0.487, 0.410, 0.263]` / same | `[0.487, 0.410, 0.263]` / same | `[0.381, 0.410, 0.207]` | `[0.593, 0.410, 0.207]` |

The actual imported Python package reports `0.8.0.post1.dev43`; its checkout
is `v0.8.0-43-g78fd485`. The earlier third-pass `cuRobo 1.5.1` label was
therefore wrong and has been corrected to cuRobo 0.8.

### Scene validity and completed cases

All three same-XY scenes are valid at `25 mm`. Start, goal, and interior
no-obstacle clearance are kept separate; the negative interior values prove
that every scene blocks the baseline route.

| Wall top | Start | Goal | No-obstacle interior | Status |
| ---: | ---: | ---: | ---: | --- |
| 0.18 m | +33.844 mm | +34.531 mm | -42.505 mm | VALID |
| 0.22 m | +29.834 mm | +34.531 mm | -44.659 mm | VALID |
| 0.26 m | +29.834 mm | +34.531 mm | -64.692 mm | VALID |

Every A and B row and every final scan plan ran three seeded repeats. A
collided with truth in all repeats. B used exact truth as its planner input;
no other case did. The standard and overview scans saw each true wall top and
returned to start without collision or planner failure.

| Top | A min | B min | Standard scan min | Overview scan min | 0 mm final min | 20 mm/standard final min | Overview final min |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.18 m | -42.505 mm | +9.869 mm | +24.929 mm | +33.844 mm | +16.089 mm | +29.454 mm | +29.386 mm |
| 0.22 m | -44.659 mm | +9.488 mm | +21.451 mm | +27.437 mm | +16.269 mm | +27.418 mm | +27.236 mm |
| 0.26 m | -64.692 mm | +9.392 mm | +24.798 mm | +24.798 mm | +16.049 mm | +27.713 mm | +27.556 mm |

All tabled B, padding, standard, and overview final plans returned `3/3`; their
per-repeat records contain separate start, goal, and interior clearance plus a
planner-failure field. The only completed planner call that failed was E at
the `0.22 m` wall, cycle 3: `planner_failure_status=TRAJOPT_FAILED`, with the
more specific stop classification `FITTED_BOX_COVERS_CURRENT_STATE`.

### Look while moving and timing

| Scene | Cycles | First seen | Clearance then | Minimum executed | Final stop | Goal? |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| Main 0.18 m | 12 | 1 | +33.844 mm | +20.388 mm | `MAX_CYCLES_REACHED` | no |
| Main 0.22 m | 3 | 1 | +29.834 mm | +1.819 mm | `FITTED_BOX_COVERS_CURRENT_STATE` | no |
| Main 0.26 m | 2 | 1 | +29.834 mm | -0.860 mm | `TRUE_COLLISION` | no |
| Blind | 2 | never | n/a | -7.459 mm | `TRUE_COLLISION` | no |

Whole-cycle timing includes capture, conversion, merge, fit, in-place scene
update, and plan. Main scenes had a fitted obstacle in every cycle: median /
p95 were `103.75 / 114.34 ms`, `110.14 / 110.58 ms`, and
`129.62 / 129.67 ms` for wall tops `0.18`, `0.22`, and `0.26 m`. They had no
no-box cycles. The blind run never formed a box; its two no-box cycles measured
`88.58 / 91.64 ms` median / p95, with no with-box sample. Thus the old `82 ms`
number is not presented as fitted-box performance.

The new blind cuboid is centered at `[0.647, 0.260, 0.380] m`, dimensions
`[0.050, 0.040, 0.060] m`. Its start, goal, and baseline-interior clearances
are `+31.239`, `+138.973`, and `-28.895 mm`, respectively. The start capture
has zero obstacle pixels and points. A and one-view C each returned `3/3` and
hit truth at `-28.895 mm`; C had no fitted box. E also never saw the obstacle,
so `first_seen_cycle` and its associated clearance are `null`; it collided in
cycle 2 and did not reach the goal.

Raw measurements are in
[`fourth_pass.json`](experiments/sim-camera-curobo/results/fourth_pass.json)
and the compact summary is in
[`fourth_pass.log`](experiments/sim-camera-curobo/results/fourth_pass.log).

### Viewer and what you should see

The viewer now defaults to `B god view wall top 0.18`, the first valid case.
Its automated port-8085 smoke test exercised B, padding, standard, overview,
E-main, blind C/E, and the explicit baseline; every callback changed both the
robot and live frustum without an error. Invalid scenes, when present, show
their status with no path and playback disabled. The normal command still
requests `127.0.0.1:8084` exactly.

What you should see:

- the camera in front of the flange, with its live world position shown as text;
- the valid B case selected initially, with a safe path around the red truth box;
- all three valid wall heights under padding, standard, overview, B, and E selectors;
- E at `0.22 m` holding on its fitted-box stop and E at `0.26 m` reporting the measured collision;
- the unseen blind obstacle with no one-view points and E colliding before first sight;
- **NO OBSTACLE BASELINE (expected truth collision)** as an explicit selector;
- any invalid case showing status text, no path, and disabled playback.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_curobo_second_pass.sh
```

The validated run stopped its Newton worker and Viser server. No ROS, Newton
physics, nvblox, cuMotion, real robot, install, or media path was used.

## Fifth pass: real-time look while moving, with no scan

Check date: 2026-11-06 (Asia/Taipei)

This pass removes the scan and overview moves. Each experiment motion is only
the repeated `capture -> merge -> fit -> update -> replan -> advance` loop,
using one warmed planner and an 80-cycle cap. The cap was not the stop reason
in any case. The fixed speed rule advances at most `0.020 rad` when the current
state is within `0.050 m` of the fitted box and `0.050 rad` otherwise; it is
identical for every scene.

The camera-front start was raised by `0.20 m` in world Z with the same tool
orientation. Its solved joints are `[-2.688859, -2.095384, -0.718405,
1.242993, 4.712389, -2.688859] rad`, its tool is at `[0.488000, 0.116000,
0.501000] m`, and its camera is at `[0.594000, 0.116000, 0.445000] m`.
`wrist_3_joint` stayed exactly at the flipped seed (`0.000000 rad` change).

The supported cuRobo 0.8 wrist-limit method used here is to load the existing
YAML into `RobotCfg`, change the in-memory `KinematicsParams.joint_limits.position`
tensor before `MotionPlannerCfg.create`, and give that same `RobotCfg` to IK,
TrajOpt, and PRM. The effective wrist-3 range is `[-5.830452, +0.452733] rad`,
the raised start plus/minus pi. No source robot configuration was changed.

Unknown image-border extent is conservative and camera-only. A top-border
touch extends the fitted box to the current camera height, a bottom touch to
the floor, and a left/right touch by a fixed `0.100 m` in the corresponding
camera-image direction. The extension is recomputed from the latest frame, so
it shrinks when that frame shows the edge. Fit padding remains `0.015 m` in X/Y
and `0.020 m` in Z. The raw cycle records contain the active flag and direction
for every cycle. Extension was active in cycles 1--11 and 13 for both 0.22 and
0.26 m, inactive in cycle 12, and active in cycle 1 for 0.30 m.

### Raised-start validity and first view

All requested walls pass the `25 mm` endpoint check and collide with the
no-obstacle path, so no taller fallback was needed.

| Wall top | Start clearance | Goal clearance | Baseline interior | First-frame points | Seen vertical fraction | Border touch |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0.22 m | +137.295 mm | +34.531 mm | -34.233 mm | 761 | 98.157% | top |
| 0.26 m | +103.503 mm | +34.531 mm | -46.567 mm | 833 | 98.452% | top |
| 0.30 m | +75.095 mm | +34.531 mm | -46.567 mm | 853 | 98.665% | top, right |

The first-frame observed Z ranges were `0.013870--0.220000 m`,
`0.013870--0.260000 m`, and `0.013870--0.300000 m`, respectively.

### Loop results

| Wall top | Goal | Cycles | Final stop | Min truth clearance | Median / p95 whole cycle | Planner failure |
| ---: | --- | ---: | --- | ---: | ---: | --- |
| 0.22 m | no | 13 | `FITTED_BOX_COVERS_CURRENT_STATE` | +137.295 mm | 108.17 / 774.16 ms | `TRAJOPT_FAILED` |
| 0.26 m | no | 13 | `FITTED_BOX_COVERS_CURRENT_STATE` | +103.503 mm | 113.10 / 378.66 ms | `TRAJOPT_FAILED` |
| 0.30 m | no | 1 | `NO_RESULT_IK_OR_GRAPH_FAILED` | +75.095 mm | 767.95 / 767.95 ms | `NO_RESULT_IK_OR_GRAPH_FAILED` |

All walls were first seen in cycle 1, at clearances `+137.295`, `+103.503`,
and `+75.095 mm`. No executed E path collided with truth. The measured result
is nevertheless not successful goal-reaching: the conservative fitted extent
eventually covered the current state for the lower two walls, while the first
plan failed for the tallest wall.

| Case | pan | lift | elbow | wrist 1 | wrist 2 | wrist 3 | Max wrist-3 delta | Tool path / straight line |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| E, 0.22 m | 0.141142 | 0.290931 | 0.129250 | 0.105933 | 0.035513 | 0.093307 | 0.093302 | 0.229507 / 0.170718 m (1.3444x) |
| E, 0.26 m | 0.151511 | 0.294923 | 0.111163 | 0.112243 | 0.035261 | 0.095360 | 0.095356 | 0.229692 / 0.173383 m (1.3248x) |
| E, 0.30 m | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 / 0 m (n/a) |
| No-obstacle baseline | 0.419341 | 0.363572 | 0.005257 | 0.358316 | 0 | 0.419341 | 0.419340 | 0.384134 / 0.378260 m (1.0155x) |

Joint travel and wrist delta are in radians. The baseline has 41 states and no
planner failure; the E rows have 96, 97, and 1 states, respectively.

The short blind search checked 27 deterministic floor-standing walls. Every
candidate stood on the floor. None was simultaneously endpoint-valid,
route-blocking, and unseen in the raised first frame: valid route blockers had
413 or 484 first-frame points. Therefore no floating block and no blind E run
were created; first-seen cycle, clearance then, stop reason, and goal result
are all n/a for this search outcome.

Raw measurements are in
[`fifth_pass.json`](experiments/sim-camera-curobo/results/fifth_pass.json) and
the compact console record is in
[`fifth_pass.log`](experiments/sim-camera-curobo/results/fifth_pass.log).

### Viewer and what you should see

The normal viewer binds only to `127.0.0.1:8084`. Its selector contains the E
wall cases and **NO OBSTACLE BASELINE** only, and defaults to the first valid
case, `E wall top 0.22`. The callback smoke test moved the robot and frustum
for both moving E cases and the baseline without a playback error.

What you should see:

- the raised camera-front start, live frustum, and current camera points;
- `E wall top 0.22` selected initially;
- the ordinary fitted box in cyan and an unknown-extended box in orange;
- current cycle, stop reason, and truth clearance in the scene text;
- E at 0.22 and 0.26 m moving safely before their conservative-box stops;
- E at 0.30 m reporting its first-cycle planner failure without moving;
- **NO OBSTACLE BASELINE** following its direct 41-state path;
- an empty Playback error field during successful playback.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_curobo_fifth_pass.sh
```

The validated run stopped its Newton worker and Viser server. No ROS, Newton
physics, nvblox, cuMotion, real robot, install, or media path was used.

## Sixth pass: corrected camera-border extent

Check date: 2026-11-06 (Asia/Taipei)

Correction to Fifth pass: its image-top touch was incorrectly treated as world
up. That made a `0.22 m` wall produce a `0.445 m` fitted top, then allowed the
box to follow the rising camera to about `0.612 m`. Those fifth-pass stops are
kept as historical results but are superseded by this corrected run.

Every touched image border now uses its outward image direction transformed by
the same-frame camera pose and projected into world XY. The axis-aligned fit is
extended `0.030 m` horizontally in that direction; a border touch never changes
height. Normal height is the highest accumulated observed point plus `0.020 m`.
If the same-frame highest point is within `0.030 m` of the camera height, the
only exception is the fixed raised-start camera height plus `0.100 m`, or
`0.545000 m`. It never follows the current camera. The exception did not
activate in these three scenes.

The raised start, goal, wrist-3 range `[-5.830452, +0.452733] rad`, fixed speed
rule, one warmed planner, 80-cycle cap, and scene geometry are unchanged. The
blind search was not repeated: the accepted fifth-pass result remains 27
floor-standing candidates with no candidate simultaneously valid,
route-blocking, and unseen.

### Raised-start validity and first view

| Wall top | Start clearance | Goal clearance | Baseline interior | First-frame points | Seen vertical fraction | Border touch |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0.22 m | +137.295 mm | +34.531 mm | -34.233 mm | 761 | 98.157% | top |
| 0.26 m | +103.503 mm | +34.531 mm | -46.567 mm | 833 | 98.452% | top |
| 0.30 m | +75.095 mm | +34.531 mm | -46.567 mm | 853 | 98.665% | top, right |

### Loop results and stop diagnostics

| Wall top | Goal | Cycles | Final stop | Min truth clearance | Median / p95 whole cycle | Planner failure |
| ---: | --- | ---: | --- | ---: | ---: | --- |
| 0.22 m | yes | 25 | `GOAL_REACHED` | +28.030 mm | 137.70 / 148.81 ms | none |
| 0.26 m | yes | 27 | `GOAL_REACHED` | +28.127 mm | 134.62 / 147.89 ms | none |
| 0.30 m | no | 1 | `FITTED_BOX_COVERS_GOAL_STATE` | +75.095 mm | 774.17 / 774.17 ms | `NO_RESULT_IK_OR_GRAPH_FAILED` |

| Wall top | Final current-to-fit | Final goal-to-fit | Planner status | Classification |
| ---: | ---: | ---: | --- | --- |
| 0.22 m | +18.733 mm | +19.531 mm | success | not covering goal |
| 0.26 m | +19.404 mm | +15.173 mm | success | not covering goal |
| 0.30 m | +51.511 mm | -6.023 mm | `NO_RESULT_IK_OR_GRAPH_FAILED` | box covering goal |

The 0.30 m failure is specifically the fitted box covering the unchanged goal,
not an upward-chasing box and not an unexplained IK failure. Its top is only
`0.320000 m` (observed `0.300000 m` plus padding).

| Case | pan | lift | elbow | wrist 1 | wrist 2 | wrist 3 | Max wrist-3 delta | Tool path / straight line |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| E, 0.22 m | 0.419334 | 0.363215 | 0.151057 | 0.358132 | 0.058571 | 0.419057 | 0.419052 | 0.420365 / 0.378041 m (1.1120x) |
| E, 0.26 m | 0.434098 | 0.358473 | 0.210314 | 0.356105 | 0.077116 | 0.415529 | 0.415524 | 0.444818 / 0.375292 m (1.1853x) |
| E, 0.30 m | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 / 0 m (n/a) |
| No-obstacle baseline | 0.419341 | 0.363572 | 0.005257 | 0.358316 | 0 | 0.419341 | 0.419340 | 0.384134 / 0.378260 m (1.0155x) |

Joint travel and wrist delta are in radians. The no-obstacle path's maximum
tool height is `0.501000 m`, effectively no climb from its start. The 0.22 m
case also never climbs. The 0.26 m case peaks at `0.502586 m`, a `1.587 mm`
climb caused by its collision-free fitted-obstacle detour; the fit stays at
`0.280000 m`, so this is not camera/box positive feedback. The 0.30 m case does
not move.

### Box top and tool height by cycle

Tool height is the capture/start-of-cycle value; each cycle's recorded end
height is the next row's start, with final ends `0.263290 m` and `0.267144 m`
for the 0.22 and 0.26 m cases. Raw JSON includes both start and end values.

| Cycle | 0.22 m box / tool (m) | 0.26 m box / tool (m) | 0.30 m box / tool (m) |
| ---: | ---: | ---: | ---: |
| 1 | 0.2400 / 0.5010 | 0.2800 / 0.5010 | 0.3200 / 0.5010 |
| 2 | 0.2400 / 0.4972 | 0.2800 / 0.5022 | n/a |
| 3 | 0.2400 / 0.4929 | 0.2800 / 0.5025 | n/a |
| 4 | 0.2400 / 0.4884 | 0.2800 / 0.5016 | n/a |
| 5 | 0.2400 / 0.4813 | 0.2800 / 0.4998 | n/a |
| 6 | 0.2400 / 0.4736 | 0.2800 / 0.4957 | n/a |
| 7 | 0.2400 / 0.4655 | 0.2800 / 0.4899 | n/a |
| 8 | 0.2400 / 0.4538 | 0.2800 / 0.4867 | n/a |
| 9 | 0.2400 / 0.4473 | 0.2800 / 0.4831 | n/a |
| 10 | 0.2400 / 0.4406 | 0.2800 / 0.4791 | n/a |
| 11 | 0.2400 / 0.4336 | 0.2800 / 0.4748 | n/a |
| 12 | 0.2400 / 0.4266 | 0.2800 / 0.4677 | n/a |
| 13 | 0.2400 / 0.4194 | 0.2800 / 0.4600 | n/a |
| 14 | 0.2400 / 0.4122 | 0.2800 / 0.4517 | n/a |
| 15 | 0.2400 / 0.3989 | 0.2800 / 0.4430 | n/a |
| 16 | 0.2400 / 0.3859 | 0.2800 / 0.4343 | n/a |
| 17 | 0.2400 / 0.3735 | 0.2800 / 0.4178 | n/a |
| 18 | 0.2400 / 0.3620 | 0.2800 / 0.4020 | n/a |
| 19 | 0.2400 / 0.3514 | 0.2800 / 0.3871 | n/a |
| 20 | 0.2400 / 0.3361 | 0.2800 / 0.3735 | n/a |
| 21 | 0.2400 / 0.3235 | 0.2800 / 0.3613 | n/a |
| 22 | 0.2400 / 0.3086 | 0.2800 / 0.3443 | n/a |
| 23 | 0.2400 / 0.2974 | 0.2800 / 0.3302 | n/a |
| 24 | 0.2400 / 0.2861 | 0.2800 / 0.3185 | n/a |
| 25 | 0.2400 / 0.2746 | 0.2800 / 0.3049 | n/a |
| 26 | n/a | 0.2800 / 0.2912 | n/a |
| 27 | n/a | 0.2800 / 0.2796 | n/a |

The complete per-cycle records also include border flags, projected world XY
directions, current/goal fit clearances, planner status, and stop reason in
[`sixth_pass.json`](experiments/sim-camera-curobo/results/sixth_pass.json).
The compact console record is
[`sixth_pass.log`](experiments/sim-camera-curobo/results/sixth_pass.log).

### Viewer and what you should see

The normal viewer remains on `127.0.0.1:8084`, contains only the three E cases
and **NO OBSTACLE BASELINE**, and defaults to `E wall top 0.22`. The viewer now
shows box-top height and live tool height during playback in addition to cycle,
stop reason, frustum, points, clearance, and Playback error. Callback smoke
tests moved the robot and frustum for both successful E paths and the baseline,
with no playback error.

What you should see:

- a horizontal orange extension in the projected direction of every touched border;
- constant box tops of 0.240, 0.280, and 0.320 m rather than a box chasing the camera;
- the 0.22 and 0.26 m cases descending around the wall and reaching the goal;
- the 0.30 m case remaining at start and reporting that its fit covers the goal;
- live box-top and tool-height text changing with the selected playback cycle;
- **NO OBSTACLE BASELINE** following its direct 41-state path;
- an empty Playback error field during successful playback.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_curobo_sixth_pass.sh
```

The validated run stopped its Newton worker and Viser server. No ROS, Newton
physics, nvblox, cuMotion, real robot, install, or media path was used.
