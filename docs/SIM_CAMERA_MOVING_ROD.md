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
