# Moving-rod arm-camera experiment

Check date: 2026-11-12 (Asia/Taipei)

This experiment shuttles the UR5 between the raised camera-front start and the
goal for 20 simulated seconds while a floor-standing rod crosses the route. The
rod is a vertical cylinder with radius `0.025 m`, top at `z=0.35 m`, and center
at `y=0.263 m`. Its X position reflects between `0.30` and `0.70 m`. The Newton
renderer is built once and the kinematic rod body pose is changed for each
capture; Newton physics is not used.

One warmed Sixth-pass planner supplies the raised start, wrist-3 constraint,
fixed advance-speed rule, and corrected border rule. The arm is stop-and-go: it
does not move while computing. One simulated clock advances the rod by measured
capture/fit/plan time and by the executed trajectory duration, so planning does
not give the rod free stopped time. Truth is used only for scoring, except that
case B deliberately supplies the current true rod bounding box to the planner.

## Camera memory and cases

Points inside the current camera frustum are replaced by the current frame.
Points outside the frustum remain eligible for no more than `1.0 s` of
simulated time. Every raw cycle records `stale_points_used`; the maximum counts
were 139/148/129 for A, 61/175/109 for B, and 134/115/144 for E at
`0.05/0.10/0.20 m/s`. The zero-compute and blind controls reached maxima of 288
and 138 stale points.

A supplies no rod obstacle to the planner, B supplies the current true box, and
E supplies only the box fitted from camera points and retained points. All A
cases collided, so the moving scene is a meaningful collision test.

## Measured results

Clearance is signed truth clearance; negative values are penetration. First
seen is reported as simulated time / rod-to-arm distance. Cycle timings include
measured compute and executed duration.

| Case | Speed | Cycles | Reaches | Collisions | Min clearance | Stopped, no plan | Median / p95 | First seen | Planner failures |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A | 0.05 m/s | 63 | 4 | 3 | -48.360 mm | 0.000 s | 314.12 / 474.28 ms | 0.000 s / 190.179 mm | none |
| B | 0.05 m/s | 85 | 2 | 2 | -42.621 mm | 4.483 s | 239.57 / 401.25 ms | 0.000 s / 190.179 mm | `TRAJOPT_FAILED` |
| E | 0.05 m/s | 82 | 3 | 3 | -37.947 mm | 4.537 s | 257.39 / 366.19 ms | 0.000 s / 190.179 mm | `TRAJOPT_FAILED` |
| A | 0.10 m/s | 66 | 4 | 8 | -59.624 mm | 0.000 s | 315.27 / 443.17 ms | 0.000 s / 190.179 mm | none |
| B | 0.10 m/s | 92 | 2 | 5 | -37.684 mm | 4.602 s | 211.34 / 363.29 ms | 0.000 s / 190.179 mm | `NO_RESULT_IK_OR_GRAPH_FAILED`, `TRAJOPT_FAILED` |
| E | 0.10 m/s | 80 | 3 | 3 | -42.969 mm | 2.714 s | 257.02 / 367.82 ms | 0.000 s / 190.179 mm | `TRAJOPT_FAILED` |
| A | 0.20 m/s | 68 | 5 | 9 | -39.480 mm | 0.000 s | 313.60 / 466.88 ms | 0.000 s / 190.179 mm | none |
| B | 0.20 m/s | 89 | 2 | 9 | -41.687 mm | 5.045 s | 235.06 / 375.08 ms | 0.000 s / 190.179 mm | `TRAJOPT_FAILED` |
| E | 0.20 m/s | 83 | 3 | 9 | -50.796 mm | 4.545 s | 237.09 / 365.88 ms | 0.000 s / 190.179 mm | `TRAJOPT_FAILED` |

No E speed survived: the fastest surviving camera-only speed is therefore
`none`. The god-view B cases also collided. B knows the rod's current box but
has no prediction of its future position while compute and execution advance
the shared clock; its result is not a collision-free upper bound.

At `0.10 m/s`, removing compute latency produced 224 cycles, 4 reaches, 5
collisions, -50.506 mm minimum clearance, 3.500 s stopped without a plan, and
163.84 / 343.03 ms median/p95 cycle time. The counted-latency E run produced 80
cycles, 3 reaches, 3 collisions, -42.969 mm, 2.714 s stopped, and 257.02 /
367.82 ms. Zeroing compute time improved throughput but did not make this
controller safe.

The blind `0.10 m/s` rod began at `x=0.30 m` behind/outside the current camera
view. It was first seen at `0.940 s`, only `70.802 mm` from the arm. The case
completed 75 cycles and 3 reaches, collided 4 times, reached -53.561 mm minimum
clearance, stopped without a plan for 3.275 s, and measured 291.48 / 381.28 ms
median/p95. Its planner failure was `TRAJOPT_FAILED`. This is reported as
observed; no blind-entry fix was added.

Raw per-cycle data are in
[`moving_rod.json`](experiments/sim-camera-curobo/results/moving_rod.json), and
the compact record is in
[`moving_rod.log`](experiments/sim-camera-curobo/results/moving_rod.log).

## Viewer and what you should see

The normal viewer binds to `127.0.0.1:8084`. Playback follows simulated time,
including the stop-and-go compute interval, and the callback smoke test covers
the A `0.10 m/s`, E `0.10 m/s`, and blind E cases. Viewer runs do not write the
tracked result files; evidence is regenerated only with the explicit
`--save-results` flag.

What you should see:

- the true vertical rod moving back and forth across the route;
- the UR5 shuttling between the raised start and goal;
- current camera points, the fitted camera-only box, and the live frustum;
- the live simulated depth image panel changing with each capture;
- simulated time, rod speed, current case, cycle, stop reason, and truth clearance;
- an empty/`none` Playback error field during successful playback.

Known limits, intentionally not solved here: one axis-aligned box represents all
retained points; the floor is cut at world `z=0.012 m`; robot pixels are selected
by simulator shape index; and the eye-in-hand camera sees only in front of and
below the gripper.

Exact one-line command:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_moving_rod.sh
```

For a headless evidence rewrite, add `--headless --save-results`. No ROS,
Newton physics, nvblox, cuMotion, real robot, install, or media path is used.


## Second run

Check date: 2026-11-12 (Asia/Taipei). This section supersedes the first-run
scene and results above. The original `moving_rod.json` and `.log` are preserved.

### Scene selection and validity

Final scene status: `VALID_SCENE`. Radius remains `0.025 m`; lane Y is `0.245 m`, X reflects over `[0.3, 0.7]`, and top height is `0.250 m`, below the raised-start camera.

The complete endpoint sweep uses 1 mm spacing and subtracts 0.5 mm from the minimum sampled signed clearance. Cylinder distance is 1-Lipschitz in X, so this bounds intermediate positions too. Start/goal lower bounds are `104.233/51.458 mm`, both above 25 mm. Scoring uses the existing robot collision spheres against the true cylinder, not mesh contact physics.

The old lane fails the endpoint sweep and is recorded as `INVALID_SCENE`.
Every rejected geometric candidate is retained in `scene_preflight.rejected`.
A first endpoint-safe, blind-entry candidate (Y=0.220, X=0.30–0.70,
top=0.250 m) failed the timed A control at 0.20 m/s (zero collisions);
its measured A summaries are retained in `moving_rod_v2_scene_search.json`
as `INVALID_SCENE`. A second candidate (Y=0.245, X=0.30–0.70, top=0.250 m)
was endpoint-safe but A at 0.05 m/s missed the rod when starting X was 0.70 m.
The final regular initial X is 0.60 m, moving toward decreasing X, shared by A/B/E
at every speed; blind entry starts at X=0.30 m moving toward increasing X.
Its full sweep is unchanged. Both rejected timed configurations are preserved.
The final search requires at least 30 mm baseline-path
intersection depth as a selection margin and then checks all three actual A
controls. This margin is a scene-selection choice, not a planner safety margin.

The blind-view prefilter explicitly converts the Viser camera orientation back
to Newton's -Z sensor orientation. The first attempt omitted Viser's pi-X
conversion; the actual renderer caught the mismatch (4198 rod pixels), and the
run stopped. The final scene must pass the renderer's zero-initial-rod-pixel
check before running any case. No camera frames are suppressed.

### Prediction and retreat rules

B uses true instantaneous velocity; E differences the fitted box centers from
the two most recent nonempty **fresh** camera frames in simulated time. It does
not differentiate the retained cloud and never reads truth velocity. If there
are not two observations, or the current fresh frame is empty, velocity is zero.
Both modes sweep the current box along `2 * velocity * (expected_compute +
expected_segment)`: add the absolute displacement to dimensions and half the
signed displacement to its center. Initial expected compute and segment are
0.20 s each (0.40 s horizon); later values use the preceding measured whole
compute time and the preceding executed segment duration. The zero-latency
control excludes compute from the prediction horizon and the simulated clock.
At the initial horizon, B extends X by 0.040/0.080/0.160 m at
0.05/0.10/0.20 m/s. Each cycle records its actual velocity, horizon, and box;
these are estimates, not guaranteed bounds on future computation or reversals.

On target-planning failure or less than 30 mm predicted-box clearance, try the
raised start; if that plan fails, try a pose 0.10 m above the current tool with
its orientation preserved. Each cycle records both attempted targets and their
failure statuses; summary counts distinguish raised-start selection,
straight-up selection, and all-failed holding. Escape does not count as reaching
a task target. A remains the no-obstacle control, with no prediction or escape.
The shared clock includes all attempted planning time, during which the arm
holds. Renderer startup and warmup occur outside the case clock.

### Measured second-run results

Collision episodes retain the original continuous counting. The phase columns
below count colliding samples while joints change / while they hold; an episode
may span both. `collision_episodes_by_onset_phase` separately records the phase
where each episode begins. Compute-time and no-plan collisions count as holding;
constant states in an executed trajectory also count as holding.

| Case | Speed | Reaches | Episodes | Moving / holding samples | Min clearance mm | Escape raised / up / all failed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A | 0.05 | 4 | 3 | 36 / 49 | -36.306 | 0 / 0 / 0 |
| B | 0.05 | 3 | 0 | 0 / 0 | 10.905 | 13 / 0 / 0 |
| E | 0.05 | 3 | 1 | 14 / 14 | -6.768 | 7 / 0 / 1 |
| A | 0.10 | 5 | 2 | 36 / 41 | -26.144 | 0 / 0 / 0 |
| B | 0.10 | 3 | 1 | 0 / 43 | -33.951 | 2 / 0 / 7 |
| E | 0.10 | 3 | 2 | 5 / 62 | -34.102 | 8 / 0 / 12 |
| A | 0.20 | 4 | 3 | 22 / 39 | -35.183 | 0 / 0 / 0 |
| B | 0.20 | 2 | 4 | 0 / 69 | -20.676 | 13 / 0 / 15 |
| E | 0.20 | 3 | 4 | 5 / 76 | -32.824 | 5 / 0 / 15 |
| E zero latency | 0.10 | 5 | 2 | 22 / 68 | -28.390 | 3 / 0 / 73 |
| E blind entry | 0.10 | 3 | 2 | 13 / 40 | -20.408 | 6 / 0 / 14 |

Fastest surviving tested speeds (zero collisions and at least one task reach): B=`0.05`, E=`None` m/s. All three A controls collide: `True`. These are outcomes of one measured-timing run, not certified speed limits.

Blind entry: initial rod pixels=`0`; first detection at `0.955 s`, true arm clearance `88.755 mm`.

Actual prediction horizons and X extensions (min / median / max) are:

| Case | Horizon s | Added X extent m |
| --- | --- | --- |
| B god view 0.05 m/s | 0.1962 / 0.3127 / 0.6020 | 0.0196 / 0.0313 / 0.0602 |
| E camera only 0.05 m/s | 0.2117 / 0.3095 / 0.6058 | 0.0000 / 0.0263 / 0.0775 |
| B god view 0.10 m/s | 0.2209 / 0.3105 / 0.6064 | 0.0442 / 0.0621 / 0.1213 |
| E camera only 0.10 m/s | 0.2175 / 0.3206 / 0.5915 | 0.0000 / 0.0521 / 0.1423 |
| B god view 0.20 m/s | 0.1949 / 0.3174 / 0.5935 | 0.0779 / 0.1270 / 0.2374 |
| E camera only 0.20 m/s | 0.2102 / 0.3195 / 0.6085 | 0.0000 / 0.0929 / 0.2613 |
| E camera only 0.10 m/s zero compute latency | 0.0500 / 0.2000 / 0.5000 | 0.0000 / 0.0335 / 0.2930 |
| E blind entry 0.10 m/s | 0.1632 / 0.3182 / 0.6010 | 0.0000 / 0.0480 / 0.1429 |

### Viewer: what you should see

- Red true rod moving along the final lane, with the arm shuttling.
- Larger magenta camera points against the white background.
- Orange observed box and a separate translucent blue predicted box.
- `ESCAPE: raised_start`, `ESCAPE: straight_up`, or `ESCAPE: all_failed`
  when a retreat is active; all-failed means the arm holds.
- Live depth panel, camera frustum, shared simulated time, and clearance.
- `Playback error: none` after successful playback.

Exact one-line run command (uses this user's isolated port):

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && source scripts/lab_session_env.sh && scripts/run_sim_camera_moving_rod.sh --port "$NEWTON_VIEWER_PORT"
```

The normal viewer does not write results. Evidence regeneration requires
`--save-results` and writes only `moving_rod_v2.json` and `.log`. The existing
8084 listener was left untouched; this run used the free per-user port 30000.
GPU runtime checks cover the complete case matrix and worker shutdown; viewer
callback checks cover A, E, and blind entry. CPU checks cover reflected motion
and directional prediction-box containment; `scripts/ci_checks.py` passes.

Prediction and retreat still can fail after entering an inflated obstacle box;
constant-velocity prediction also does not anticipate reflections. Camera
visibility, box estimation, and planner/controller delay remain distinct
failure sources. This kinematic experiment does not validate physical contact,
real depth hardware, or real-robot safety.


## MPC prototype

Check date: 2026-11-12 (Asia/Taipei). Source parent:
`15c5a543c0c7ff26233b619c0f05b57e08974c92`; local cuRobo 0.8 checkout:
`78fd485fa82d9b9a063fb4985e371814587e666a`. The accepted v2 source,
worker, evidence and shared configurations are unchanged. No installation,
ROS, Newton physics, hardware, nvblox, cuMotion, or media artifacts are used.

### Read-only API survey

The public `ModelPredictiveControl` and `ModelPredictiveControlCfg` aliases
are in `curobo/curobo/model_predictive_control.py`. They expose `MPCSolver`
and `MPCSolverCfg` in `curobo/curobo/_src/solver/solver_mpc.py` and
`solver_mpc_cfg.py`. Call `setup(JointState)` once, then
`update_goal_tool_poses(GoalToolPose, run_ik=False)` for Cartesian tracking.
`optimize_action_sequence(current_state)` re-optimizes on every call.
A preallocated `camera_obstacle` cuboid is changed in-place each step using
`scene_collision_checker.update_obstacle_pose`, `data.cuboids.update_dims`,
and `enable_obstacle`. This preserves the collision storage used by CUDA graphs.

On `NVIDIA GeForce RTX 3080`, ten warmed solves after two discarded warmups measured median `19.359 ms`, p95 `21.121 ms`, or `51.65 Hz` at the median. These are measured solver wall times, not advertised GPU throughput.

The collision-start probe placed a 50 mm cube over the final collision sphere; signed clearance was `-37.663 mm`. Both setup and optimization returned, with finite velocities and `success=false`. MPC accepts the colliding state; this does **not** establish escape or safety. Unlike MotionPlanner, setup validates tensor shape rather than rejecting a colliding start. In this checkout the automatic infeasibility/deceleration branch in `_solve_impl` is guarded by `and False`, despite the configuration option. No library code was changed. Since usable MPC exists, the conditional no-MPC patch was not applied.

### Controller and shared clock

The scene and endpoints are loaded directly from the accepted v2 evidence:
Y=0.245 m, X=0.30–0.70 m, radius=0.025 m, top=0.250 m, regular initial
X=0.60 m moving toward decreasing X. The complete sweep and wrist-3 limit
are retained. The tool goal and raised start are unchanged; a reach uses v2's
position tolerance. Targets alternate only after an actual reach.

The plant is an **ideal joint-velocity integrator**, not a rigid-body simulator:

- Optimize from a snapshot of current joints and the held joint velocity.
- During the measured sensing/box-update/optimization/output-transfer time,
  integrate the **previous** command, moving the rod on the same clock.
- At completion, issue a latency-indexed MPC velocity and execute the remainder
  of the nominal 25 ms control period. If compute overruns, there is no extra
  free execution period: the new command runs during the next computation.
- Clamp commands to existing joint-velocity limits and positions to existing
  joint bounds. Acceleration feedback is zero during each held command;
  velocity changes are instantaneous. Acceleration, jerk and actuator dynamics
  are not validated.

Optimization knots are 0.100 s apart with four interpolation samples; the
nominal command interval is 0.025 s. CUDA graphs remain enabled, with default
200 warm / 300 cold LBFGS iterations and 30 mm collision activation.
`action_sequence.dt` is reported as 0.100 s in this checkout, so the adapter
explicitly uses the configured interpolation interval. The returned four-state
window starts at the **current-state boundary**, whose velocity is the current
velocity. The adapter skips that boundary and selects the first non-stale
future sample, clamped to the last available sample on overrun.

An initial diagnostic incorrectly reissued boundary index zero: all six runs
had zero reaches and almost no motion. A direct position/velocity inspection
exposed the boundary semantics. That adapter error was corrected before the
saved result below; a CPU regression test covers it. The final artifacts store
capture joints, joints after computation, old/new velocities and counted time.
All 4800 computation intervals satisfy `q_after_compute = q_capture +
old_velocity * counted_time` within 1e-6 rad; 4794 intervals show joint motion.
There were no position-limit clamps in the saved run. Thus zero collisions
below are not produced by stopping the arm during computation.

B receives only the true current cuboid. Its prediction variant uses the
unchanged v2 directional stretch: `2 * velocity * (previous measured compute +
0.025 s)`, initialized with expected compute 0.025 s. The truth rod reflects;
the linear prediction does not anticipate reflection. Camera E would use the
same v2 fresh-frame velocity and retained-cloud fit if the B gate passed.
Truth scoring and B replay image rendering are offline and excluded from the
control clock, matching v2's exclusion of truth scoring. B does not need camera
rendering to control; E's camera capture would be included in compute time.
Goal-switch updates are counted in the following control computation.

### Direct comparison with v2 B

Each row covers 20 simulated seconds. Moving/holding are collision **episodes
classified by onset phase**. Period timings include compute and execution;
MPC's solver-only and whole-compute timings are separately recorded in JSON.
The v2 period columns use its saved `whole_cycle_median_ms` / p95. The algorithms
and command profiles differ: this is a controller comparison, not a matched
path/speed ablation.

| Controller | Rod m/s | Reaches | Episodes moving / holding | Min clearance mm | Period median / p95 ms | Rate Hz |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| v2 B prediction + escape | 0.05 | 3 | 0 / 0 | 10.905 | 312.601 / 399.853 | 3.35 |
| MPC B current box | 0.05 | 0 | 0 / 0 | 45.250 | 25.000 / 25.000 | 40.00 |
| MPC B + prediction | 0.05 | 0 | 0 / 0 | 46.997 | 25.000 / 25.000 | 40.00 |
| v2 B prediction + escape | 0.10 | 3 | 0 / 1 | -33.951 | 284.221 / 377.383 | 3.40 |
| MPC B current box | 0.10 | 0 | 0 / 0 | 6.072 | 25.000 / 25.000 | 40.00 |
| MPC B + prediction | 0.10 | 0 | 0 / 0 | 13.685 | 25.000 / 25.000 | 40.00 |
| v2 B prediction + escape | 0.20 | 2 | 0 / 4 | -20.676 | 281.466 / 359.611 | 3.50 |
| MPC B current box | 0.20 | 0 | 0 / 0 | 4.074 | 25.000 / 25.000 | 40.00 |
| MPC B + prediction | 0.20 | 0 | 0 / 0 | 13.084 | 25.000 / 25.000 | 40.00 |

All six MPC B runs were collision-free, but **none reached either task target**.
The prototype therefore does not demonstrate a better shuttle controller.
Survival retains the v2 requirement of zero collisions **and at least one
reach**; no MPC B variant passed at 0.10 m/s, so the conditional E run was
not performed. This configuration shows continuous clearance-preserving motion
with insufficient task progress. It does not establish whether the remaining
cause is goal tracking, local avoidance, integration/tuning, or the 20 s horizon.
An obstacle-free reach control is the next diagnostic before claiming superior
avoidance or tuning the task further.

For near-rod behavior, compute signed cylinder/sphere clearance at both states
with the rod fixed at its new location. Their difference / dt measures the
**robot's** contribution: positive means away, negative means toward. The full
change, including rod motion, gives relative clearance rate. Count samples when
either endpoint is within 50 mm; +/-0.1 mm/s is the stationary band.

| MPC B variant | Rod m/s | Away / toward / stationary samples within 50 mm | Mean robot / relative clearance rate m/s |
| --- | ---: | ---: | ---: |
| current box | 0.05 | 90 / 0 / 0 | +0.01320 / -0.00005 |
| prediction | 0.05 | 75 / 0 / 0 | +0.01486 / +0.00007 |
| current box | 0.10 | 272 / 59 / 2 | +0.00165 / -0.00017 |
| prediction | 0.10 | 196 / 83 / 4 | +0.00253 / -0.00013 |
| current box | 0.20 | 319 / 65 / 0 | +0.00269 / +0.00089 |
| prediction | 0.20 | 116 / 91 / 2 | -0.00208 / +0.00099 |

A positive robot contribution can coexist with decreasing relative clearance
because the rod also moves. In particular, the prediction variant is not always
moving away even when it avoids collision. Neither a positive mean nor zero
collisions alone establishes task success.

### Viewer: what you should see

- Selectable B cases for both prediction choices and all three rod speeds.
- Controller name and measured achieved step rate, with shared simulated time.
- Continuous joint-state replay at the saved integration samples, rather than
  freezing the arm while the controller computes.
- Red true rod, larger magenta camera points, orange current box, and blue
  prediction box when prediction is enabled.
- Camera frustum and depth image refreshed at about 5 Hz from offline captures.
- Arm motion that preserves clearance but does not complete the shuttle task;
  the reach count in the evidence is zero.
- `Playback error: none` after callback verification.

Exact one-line viewer command (reads saved traces and cannot save results):

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && NEWTON_VIEWER_PORT=30001 scripts/run_sim_camera_moving_rod_mpc.sh --replay
```

The wrapper sources `lab_session_env.sh`; its default per-user port 30000 was
already occupied by PID 1244750 and was left untouched. The explicit personal
override 30001 was checked free before use. The program refuses Viser's silent
fallback to a different port. Replay and `--save-results` are mutually exclusive.
Normal runs also do not write evidence unless `--save-results` is explicit.
Headless regeneration:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_moving_rod_mpc.sh --headless --save-results
```

Raw evidence is `moving_rod_mpc.json` with per-step clocks and sampled traces;
`moving_rod_mpc.log` contains compact summaries. Viewer verification and
before/after evidence hashes are in `moving_rod_mpc.viewer.log`. CPU checks
cover old-command integration, continuity, joint bounds, zero-duration steps,
MPC boundary indexing, and replay write protection. All project CI checks pass.
The prototype remains a kinematic experiment, with no physical-contact or
hardware-safety acceptance.


## MPC tuned

Check date: 2026-11-12 (Asia/Taipei). Source parent:
`ad2bf29` (full parent hash is recorded in the session handoff).
This is a new configuration and script beside the accepted v2 and original MPC
prototype. Those scripts, their worker, old evidence, library code, shared
configs and the FEM grasp path are unchanged. The same CUDA environment and
ideal continuous velocity plant are used; no installation, ROS, Newton physics,
real robot, nvblox, cuMotion or media artifacts are involved.

### No-rod reference before tuning

All obstacles are disabled, with the identical raised start, Cartesian goal,
and 6 mm positional acceptance. MotionPlanner executes its returned path at
its own 25 ms interpolation timing. Planning/setup is reported separately and
excluded from path time. MPC includes the same measured-compute/shared-clock
execution as the original prototype and stops at its first measured reach.

| No-rod controller | Time to enter 6 mm | Final sampled error | Max commanded joint speed | Max update-sampled acceleration |
| --- | ---: | ---: | ---: | ---: |
| v2 MotionPlanner path | 0.875 s | 4.743 mm at first reach | 0.724449 rad/s | 3.347397 rad/s² |
| Original MPC | 11.800 s | 5.819 mm | 0.051068 rad/s | 0.030044 rad/s² |
| Frozen tuned MPC | 1.125 s | 3.728 mm | 0.616654 rad/s | 3.596537 rad/s² |

The deadline is twice MotionPlanner's reach time: `1.750 s`. Tuned MPC passed in `1.125 s`, with no joint-position or velocity clipping and zero infeasible steps in this no-rod run. All six joint speed bounds remain +/-3.141593 rad/s. MotionPlanner planning time, excluded from the reference path clock, was `0.060790 s`.

The acceleration values are explicitly `abs(v_new-v_old) / control-period`
maxima, recorded per joint. The plant has instantaneous velocity switches and
no acceleration limiter: its ideal continuous-time acceleration at a switch
has no finite peak. These discrete numbers are diagnostics, not hardware
acceleration or jerk guarantees.

Tool distance to the original goal over time is stored at every step. The
following table interpolates within the saved samples and shows `ended` after
a reference run finishes (it does not invent a held-goal continuation).

| Sim time s | MotionPlanner distance mm | Original MPC distance mm | Tuned MPC distance mm |
| ---: | ---: | ---: | ---: |
| 0.000 | 378.260 | 378.261 | 378.261 |
| 0.250 | 337.380 | 377.547 | 328.926 |
| 0.500 | 194.562 | 375.390 | 229.463 |
| 0.750 | 45.289 | 371.909 | 112.649 |
| 0.875 | 4.743 | 369.709 | 65.877 |
| 1.000 | 0.000 | 367.231 | 30.583 |
| 1.125 | ended | 364.479 | 3.728 |
| 1.750 | ended | 347.154 | ended |
| 2.000 | ended | 338.814 | ended |
| 5.000 | ended | 209.733 | ended |
| 10.000 | ended | 36.114 | ended |
| 11.800 | ended | 5.819 | ended |

### Loaded settings, cause and no-rod selection evidence

The actual loaded config is captured for every candidate. The local cuRobo
checkout is unchanged (`78fd485fa82d9b9a063fb4985e371814587e666a`).
`ModelPredictiveControlCfg.create` loads `mpc/lbfgs_mpc.yml` and
`mpc/transition_bspline_mpc.yml`. `solver_mpc_cfg.py` overrides knot timing and
`solver_mpc.py:enable_tool_pose_tracking` applies the non-terminal factor.

| Setting | Original | Frozen | Interpretation |
| --- | ---: | ---: | --- |
| Optimization knot interval | 0.100 s | 0.020 s | Changes forecast time scale and differentiation/retimed costs |
| Non-terminal tool pose factor | 0.001 | 0.3 | Gives the near-term goal substantially more cost |
| Interpolation steps | 4 | 4 | MPC sample interval changes from 0.025 to 0.005 s |
| Executor period | 0.025 s | 0.025 s | Same shared clock and previous-command execution |
| B-spline control points / state horizon | 16 / 81 | 16 / 81 | Same shape, shorter nominal time span |
| Velocity scale | 1.0 | 1.0 | No artificial slow-speed multiplier |
| Tool pose weights | [5000, 200] | unchanged | Loaded optimizer pose costs |
| Velocity / acceleration / jerk regularization | [0.01, 10000, 10] | unchanged | Acceleration smoothing is retained |
| Retime regularization weights | true | true | Knot-time changes also rescale derivative costs |
| Terminal pose cost tolerance | [0, 0] | unchanged | No terminal deadband at 6 mm |
| Solver position metric / task reach | 5 mm / 6 mm | unchanged | Tolerance was not loosened |
| Warm / cold LBFGS iterations | 200 / 300 | unchanged | GPU solve work is comparable |

This rules out a binding joint-speed cap or GPU delay as the source of the
original slow approach: the no-rod original commands were far below limits.
The tested cause is the **cost/time configuration**: a long knot interval plus
weak near-term tracking allows a slow receding-horizon approach. Changing only
acceleration regularization did not fix it. Shortening the knot interval alone
improved progress but did not meet the 6 mm/deadline requirement; changing only
the pose factor also failed, and large factors could become infeasible.
The two-setting combination passes. This is causal ablation evidence for that
combination, not proof that one scalar is the unique cause. Knot timing also
changes retimed derivative costs, so its effect cannot be attributed solely
to horizon duration.

All candidate tests were **no-rod only**. Candidate outcomes are non-monotonic,
so failure and infeasibility records are preserved rather than inferred from
weight magnitude. Failed candidates were censored at deadline + 25 ms; `none`
below means no reach within that interval, not never reaching eventually.

| Acceleration reg. | Non-terminal factor | Knot dt s | Time to reach s | Last error mm | Passed |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1000 | 0.001 | 0.100 | none | 346.366 | False |
| 100 | 0.001 | 0.100 | none | 347.445 | False |
| 10 | 0.001 | 0.100 | none | 351.168 | False |
| 1 | 0.001 | 0.100 | none | 350.020 | False |
| 10000 | 0.01 | 0.100 | none | 345.886 | False |
| 10000 | 0.1 | 0.100 | none | 296.966 | False |
| 10000 | 1 | 0.100 | none | 120.694 | False |
| 10000 | 10 | 0.100 | none | 529.479 | False |
| 10000 | 2 | 0.100 | none | 88.177 | False |
| 10000 | 3 | 0.100 | none | 830.792 | False |
| 10000 | 5 | 0.100 | none | 709.526 | False |
| 10000 | 0.001 | 0.030 | none | 140.537 | False |
| 10000 | 0.001 | 0.020 | none | 26.979 | False |
| 10000 | 0.001 | 0.050 | none | 239.009 | False |
| 10000 | 0.001 | 0.040 | none | 207.149 | False |
| 10000 | 0.001 | 0.010 | none | 146.295 | False |
| 10000 | 0.1 | 0.020 | none | 155.147 | False |
| 10000 | 0.01 | 0.020 | none | 45.371 | False |
| 10000 | 0.3 | 0.020 | 1.125 | 3.728 | True |

The result is frozen in the single `NO_ROD_SELECTED_MPC` block named
`mpc_no_rod_dt_0.02_nonterminal_0.3` in the new script. The rod runner refuses
settings that differ from the no-rod selection and embeds the reference file's
SHA-256. That file was finalized before any tuned rod test; no settings were
changed based on rod outcomes. The adapter derives command indexing from the
configured MPC interpolation interval and retains the accepted boundary-skip
rule, previous-command execution and 25 ms executor period.

### Frozen rod results compared with v2 B

All cases last 20 simulated seconds on the accepted fair scene. Collision
columns are episodes classified by onset phase, not colliding sample counts.
Timings are full control periods (compute plus executed remainder); solver and
compute timings are also retained separately. The v2 rows use their saved
whole-cycle timings.

| Controller | Rod m/s | Reaches | Collisions moving / holding | Min clearance mm | Period median / p95 ms | Rate Hz |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| v2 B prediction + escape | 0.05 | 3 | 0 / 0 | 10.905 | 312.601 / 399.853 | 3.35 |
| Tuned B current box | 0.05 | 7 | 0 / 0 | 16.173 | 25.000 / 25.000 | 40.00 |
| Tuned B prediction | 0.05 | 7 | 0 / 0 | 15.271 | 25.000 / 25.000 | 40.00 |
| v2 B prediction + escape | 0.10 | 3 | 0 / 1 | -33.951 | 284.221 / 377.383 | 3.40 |
| Tuned B current box | 0.10 | 3 | 2 / 0 | -65.425 | 25.000 / 25.000 | 40.00 |
| Tuned B prediction | 0.10 | 3 | 1 / 0 | -80.846 | 25.000 / 25.000 | 40.00 |
| v2 B prediction + escape | 0.20 | 2 | 0 / 4 | -20.676 | 281.466 / 359.611 | 3.50 |
| Tuned B current box | 0.20 | 3 | 3 / 0 | -31.444 | 25.000 / 25.000 | 40.00 |
| Tuned B prediction | 0.20 | 6 | 0 / 0 | 1.657 | 25.000 / 25.000 | 40.00 |

The tuned controller now reaches targets. Both variants pass 0.05 m/s;
current-box B fails 0.10 and 0.20, while prediction B fails 0.10 but passes
0.20 in this single measured-timing run. Thus the fastest *tested* surviving
speed is 0.20 m/s with prediction; this is **not** a monotonic safe-speed bound
or a guarantee of passing every lower speed. Rod/robot phase and the nonlinear
controller matter. The frozen configuration was not retuned after these runs.

Neither B variant passed 0.10 m/s, so the explicit 0.10 gate for camera E
failed: **no E speed was run**, including 0.05 and 0.20. There are no measured
worst camera frames to report. The E path is implemented to log, per capture,
missing box, missing vertical/XY coverage, missing top, and XYZ/norm center
offset against the true rod box; this scoring occurs after command selection
and is never E planner input. It scores the actual fitted box including point
memory, before prediction. Worst-height, worst-width and worst-offset frames
and missing-box counts are retained when E is authorized by the gate. CPU
fixtures verify partial/shifted and missing boxes; no camera experiment is
claimed from those fixtures.

| Tuned case | Max joint speed rad/s | Max update-sampled acceleration rad/s² | Near-rod away / toward / stationary samples |
| --- | ---: | ---: | ---: |
| Tuned B current box 0.05 m/s | 0.705627 | 6.207813 | 124 / 97 / 0 |
| Tuned B prediction 0.05 m/s | 0.834949 | 5.418647 | 58 / 61 / 0 |
| Tuned B current box 0.10 m/s | 3.007632 | 21.103233 | 369 / 144 / 4 |
| Tuned B prediction 0.10 m/s | 1.598878 | 6.854234 | 495 / 78 / 1 |
| Tuned B current box 0.20 m/s | 2.745994 | 16.209421 | 285 / 63 / 0 |
| Tuned B prediction 0.20 m/s | 0.951364 | 6.264689 | 89 / 96 / 0 |

All commanded speeds stayed within the existing joint bounds without velocity
clipping, and no position clamps occurred. Acceleration is still unbounded in
the ideal plant between sampled commands. Near-rod direction uses the original
prototype's robot-only change in signed clearance with the rod fixed, while
full relative rates also include rod motion. Near-rod starts when clearance is
within 50 mm. Infeasible finite MPC commands are executed as in the accepted
prototype; collisions at higher speeds show that continuous motion alone is
not a safety solution.

### Viewer: what you should see

- Selectable **Tuned B** cases for current box/prediction and all three speeds.
- Controller name, measured step rate, shared simulated time and truth clearance.
- Arm motion that reaches and shuttles, unlike the original zero-reach prototype.
- Red rod, magenta points, orange observed box and blue prediction box.
- Camera frustum and offline depth replay; images do not affect god-view control.
- Collision examples at 0.10 m/s; the viewer is evidence, not a safety claim.
- `Playback error: none` after both 0.10 m/s callback checks.

Exact one-line viewer command (read-only saved replay, using the per-user port):

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_moving_rod_mpc_tuned.sh --replay
```

The wrapper sources `lab_session_env.sh`; port 30000 was checked free and used.
The existing 30001 listener (PID 1248594) was left untouched. Replay cannot be
combined with save or reference flags. Hashes of all old/new evidence files
are checked around replay; the proof is in `moving_rod_mpc_tuned.viewer.log`.

Exact no-rod regeneration, then frozen rod-evidence regeneration:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_moving_rod_mpc_tuned.sh --reference --save-results
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_moving_rod_mpc_tuned.sh --headless --save-results
```

The named block must match the no-rod selection before the second command runs.
Evidence is `moving_rod_mpc_tuned.reference.json`, `moving_rod_mpc_tuned.json`
and `.log`. Four CPU tests cover the no-rod qualification gate, partial/offset
box coverage, missing/worst frames and replay write protection. Project CI,
all six GPU B cases, old-command integration audit and viewer callbacks pass
execution checks. Those checks do not convert colliding cases into acceptance.
The remaining boundary is collision avoidance at normal task speed, then E's
conditional perception test; no contact-physics or hardware claim is made.
