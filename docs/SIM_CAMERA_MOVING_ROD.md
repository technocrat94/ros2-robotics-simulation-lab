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
