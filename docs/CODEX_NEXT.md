# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

**Standing rule: fail fast on setup bugs, never run a broken batch to the end.**
- Before any batch over 10 runs: smoke set first (A and B, speed 0.05, one phase). If A has any floor
  contact, A never reaches without the rod, or B does not succeed, STOP and report.
- During a batch, stop at once (keep the checkpoint) only for setup-bug signals: any A or B run with
  floor contact; floor depth > 0.20 m in any run; the first 3 B runs all fail; any non-finite value;
  a run > 3x its expected wall time.
- E and *-nopred failures (rod or floor contact) are results, not bugs: score them as failed and continue.
- Before starting, print the expected total wall time.

**Standing rule: develop with few runs, batch only at final validation (from 2026-11-19).**
- While building or fixing: run only what is needed to see the change, at most 1 run per group that
  matters (usually just A + E, or only E). No repeats, no lanes x speeds grids, no score tables.
- End every task at a checkpoint: that short run, the Viser command (replay and/or interactive) and
  3 lines on what to look at. Then stop.
- A batch (> 10 runs) is final validation only: run it only when the user says the feature is done
  and asks for the batch by name. Never propose or start one on your own.
- The fail-fast rules above apply only to that final batch.

## Local pre-merge status

No pending task

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: none. v7 checkpoint PASSED (2026-11-20). Wait for the user.

08ff7ba reviewed. Long-retract scene, gates: A0 reach, A touches, B reach 4.375 s / 0 contacts / bar 52.8 mm,
E (both cameras) reach 6.725 s / 0 bar and floor contacts / bar 34.3 mm, floor 8.46 mm.
Next (only when the user says): planner (MotionGen) + MPC to go over the bar instead of waiting; then the
"Next time" display items; then re-identification for prediction. Do not start anything.

### Previous task (kept for reference)
#### v7 — run E only (B passed) (TIME LIMIT 5 min) — 2026-11-20

df3eba8 reviewed. B PASSED as intended: reached at 4.375 s, 0 contacts, closest BAR clearance 0.053 m. The
8.5 mm minimum is the FLOOR (low path), which is fine (floor activation 0.01 m, safety plane 0.005 m). My task
wording "min clearance >= 0.02 m" meant the bar; clarify that in the report fields (bar_min vs floor_min).
Do: run E on the same scene, both cameras, 15 s. SUCCESS = >= 1 reach and 0 bar contacts (floor contacts must be
0 too). Report contacts, reaches, first reach time, bar min clearance, floor min clearance; if a bar contact,
add the forensic rows. Commit, push, leave the existing --interactive running (E, both cameras), URL + 3 lines.

### Previous task (kept for reference)
#### v7 — let the arm pass when the bar pulls back (TIME LIMIT 10 min) — 2026-11-20

45d92d6 reviewed. The path works (A0 reaches, no thrash), the bar is in the way (A touches), but B never
reaches: it stops in front of the bar (min 12.6 mm, 0 contacts). That is the MPC local minimum: with a 0.4 s
horizon it will not take a detour that first moves away from the goal. User decision: option A today (pass by
timing); planner + MPC (going over the bar) is the next step.
Do, keep everything else from 45d92d6:
1. Longer retract: inner tip x 0.50 <-> 0.75 (stroke 0.25 m) at 0.10 m/s, so when out the gap to the tool line
   (0.25 m) exceeds gripper radius + obstacle activation 0.07 m + margin.
2. Gates in order, stop at the first failure: A (bar ignored, must touch), B (success + min clearance
   >= 0.02 m), E both cameras (success). A0 already passed, skip it. Run 15 s each (one full in/out = 5 s).
3. Short table (contacts, reaches, min clearance, time of first reach), commit, push, leave the existing
   --interactive running (E, both cameras), URL + 3 lines. If any step takes > 3 min, stop and report where.

### Previous task (kept for reference)
#### v7 — longer path that the arm can actually drive (TIME LIMIT 10 min) — 2026-11-20

326c25d reviewed. The new goal (0.50, 0.50, 0.25) is the problem, not the bar: even A (bar ignored) reached 0
times and thrashed, and B reached 0 times. IK solved the goal on a different arm branch (joint 1: start -2.92 vs
goal +0.63 rad, elbow flipped): 0.707 m from the base with the gripper pointing down is at the edge of the
comfortable workspace, so MPC cannot drive there.
Do:
1. Goal = the original, validated v4-v7 goal (unchanged scene goal, tool ~ (0.50, 0.45, 0.24)).
   Start = tool (0.50, -0.05, 0.25), same orientation: path ~0.50 m, low and straight.
   Start joints by IK seeded from the validated goal joints (v3 REFERENCE trace end) so both ends are on the SAME
   branch: every joint |start - goal| < 2.0 rad and the same elbow sign. Floor clearance >= 0.02 m. Else stop.
2. Bar on the line: lane y = midpoint of start/goal tool y (~0.20), z = midpoint tool z (~0.245), x axis, inner
   tip x 0.50 <-> 0.625 at 0.10 m/s.
3. Gates in order, stop at the first failure: A0 = no bar at all, must reach >= 1 and not thrash (the path itself
   works); A = bar ignored, must touch; B (success + min clearance >= 0.02 m); E both cameras (success).
4. Short table, commit, push, leave the existing --interactive running (E, both cameras), URL + 3 lines.
If any step takes > 3 min, stop and report where.

### Previous task (kept for reference)
#### v7 — longer low straight path, bar exactly on it (TIME LIMIT 10 min) — 2026-11-20

User: make the path longer and make sure the bar is on it. Keep everything else from 4fbf078.
1. Start and goal tool poses: same tool orientation as now, x = 0.50, z = 0.25, start y = 0.00, goal y = 0.50
   (0.50 m straight, low). Start joints by cuRobo IK seeded from the current raised_start; check every
   non-excluded sphere >= 0.02 m above the floor at both poses. If IK fails, stop and report.
2. Bar exactly on that line: lane y = 0.25 (midpoint), z = 0.25, x axis, tip reciprocating x 0.50 <-> 0.625 at
   0.10 m/s (the tool line is x = 0.50, so the inner tip crosses it). >= 0.10 m from start and goal by
   construction (0.25 m).
3. Gates in order, stop at the first failure: A (must touch), B 10 s (success + min clearance >= 0.02 m),
   E 10 s both cameras (success). Skip E empty (mask unchanged, passed in 4fbf078).
4. Short table, commit, push, leave the existing --interactive running (E, both cameras), URL + 3 lines.
If any step takes > 3 min, stop and report where.

### Previous task (kept for reference)
#### v7 — bar in the MIDDLE of the path, not at the goal; then B/E — 2026-11-20

4fbf078 reviewed. Search worked; the rule picked y 0.36, z 0.23, which A hits at 1.33 s right before arriving:
the bar sits next to the goal (goal tool y ~0.45, z ~0.24), so it blocks the goal itself and B could not
reach (0 reaches, 3 contacts). That is an unfair scene, not a control/perception failure.
Lane table: in the middle of the path (y 0.24-0.34) the arm never goes below ~0.24 m, so the lowest
obstructing bar there is z ~0.25 (y 0.28: z 0.25, clearance -0.021 m).
Do:
1. Lane: y = 0.28, z = 0.25 (lowest obstructing in the middle of the path, from the table). Add a check: the
   bar (any phase) stays >= 0.10 m from the goal tool position and from the start tool position.
2. Gate A on this lane (must touch). Then E empty 10 s (both cameras), B 10 s (success + min clearance >= 0.02 m),
   E 10 s (success). Same settings as 4fbf078. Stop at the first failed gate and report.
Commit, push, leave the existing --interactive running (E, both cameras, this lane), URL + 3 lines. Stop.

### Previous task (kept for reference)
#### v7 — place the bar ON the arm's path by computation, then B/E (gate first) — 2026-11-20

838b298 reviewed: gate A failed correctly. With the bar ignored, the arm passed 20.6 mm from it (t = 1.45 s,
bar tip x 0.52, y 0.28, z 0.15): the arm goes above the bar, so z 0.15 is too low to be in the way.
Do not guess a new spot. Compute it from the recorded A trajectory (no new run needed for the search):
1. Save A's swept robot spheres for the whole 10 s run (the run is deterministic).
2. Grid search lanes offline against that sweep: y in [0.20, 0.36] step 0.02, z in [0.12, 0.30] step 0.01, same
   reciprocating bar (tip in to x 0.50, out 0.125 m, 0.10 m/s, x axis, starts outer). For each lane compute
   A's signed clearance to the moving bar over time. Choose the LOWEST z (user wants it low) whose minimum
   clearance <= -0.02 m (clearly in the way), tie-break y closest to 0.28. Save the table.
3. Rerun A on that lane to confirm it touches the bar (gate). If not, report and stop.
4. Then the rest of the previous task on that lane, unchanged: E empty 10 s both cameras (arm mask check),
   B 10 s (success + min clearance >= 0.02 m), E 10 s both cameras (success). Arm mask 15 mm + URDF base volume,
   obstacle activation 0.07 m, RGB brightened, no ESDF slice, white removed pixels.
Report the chosen lane + the 4 results. Commit, push, leave the existing --interactive running (E, both
cameras, this lane), URL + 3 lines. Stop. Do not do "Next time".

### Previous task (kept for reference)
#### v7 — measured arm mask per part + MPC reaction distance (3 runs) — 2026-11-20

91af079 reviewed. Scene is fair (B succeeded), E empty passed, display fixes done. Two findings:
1. Calibration: every MOVING link sticks out of its spheres by <= 15 mm (upper_arm max 14.9 mm, forearm 13.0,
   gripper links <= 4). The 51 mm max comes ONLY from ur/base_link_inertia (the fixed base, poorly covered by
   spheres). Using the overall max (51 mm) erased the bar again (fixed bar px 1125 -> 963 at the contact).
   Fix: distance_threshold = 0.015 (moving links, measured). The fixed base is removed by COMPUTED geometry,
   not a hand-made box: take ur/base_link_inertia's collision/visual geometry from the robot URDF at its
   fixed world pose, use its bounding cylinder/box + 1 cm, and drop depth points inside it each frame (the base
   never moves; same on the real robot). Verify with the calibration pixels: 0 base pixels left outside.
   Save the computed base volume in the calibration json.
2. Perception is now correct: in the 1 s before the E contact the ESDF matched the truth within ~1 cm
   (0.138/0.138, 0.087/0.094, 0.039/0.037 m) while the ARM drove toward the bar at 0.40-0.48 m/s. B passed only
   by 1.1 cm at the same moment (4.93 s). So it is control: MPC feels obstacles only within
   optimizer_collision_activation_distance = 0.03 m and treats them as static over its 0.43 s horizon.
   Required distance = closing speed x reaction time + bar travel over the horizon
   = 0.5 m/s x (1/30 + 0.025) s + 0.10 m/s x 0.43 s ~= 0.072 -> use 0.07 m for obstacles. Keep the floor's
   own activation at 0.01 (FloorMaskedScene) unchanged.
3. Display (user: do it now): the RGB panels are dark because Newton renders without lights/textures.
   Make them readable for display only (per-frame normalisation / gamma, or a simple headlight if Newton's
   sensor supports it). Does not affect mapping.
4. Scene change (user request): remove the Dance and the vertical rod. One horizontal bar only, at ONE fixed
   spot, lower: tip lane y = 0.28 (middle of the start -> goal path), z = 0.15 m, axis along x entering from +x.
   It just reciprocates in place: tip goes in to x = 0.50 and back out 0.125 m, at 0.10 m/s, continuously
   (no re-aiming, no teleport; starts already at the outer position). Use this scene for B and E.
   Add one run (A) with the bar NOT given to the MPC at all: it must touch the bar, otherwise the bar is not in
   the way and the test means nothing (report and stop).
Runs, 1 each: (A) bar ignored, (a) E empty 10 s both cameras (arm mask check), (b) B reciprocating bar 10 s, (c) E reciprocating bar 10 s both cameras.
SUCCESS as before (start -> goal, 0 bar contacts). Pass: (a) <= 5 voxels near the arm, reversals <= 2;
(b) success and min clearance >= 0.02 m; (c) success. If (b) fails, stop and report (control, not perception).
Report a table + E forensic rows if any contact. Commit, push, leave the existing --interactive running (E,
both cameras), URL + 3 lines. Stop. Do not do "Next time".

### Next time (user: postpone)
0. Interactive, like the v6 interactive: GUI selector for mode B / E (switch without
   restarting), Dance on/off button, single Poke button, draggable bar and vertical-rod gizmos, rod on/off,
   both cameras on in E, live status (mode, contacts, reaches, min clearance). Same scene rules as above
   (no teleport, bar z 0.20 by default; dragging may move it anywhere).
a. The red area is the reference example's ESDF slice: red = far from anything, and with nothing observed
   it is the 10000 m "no site" value, so the whole plane goes red. Hide it by default behind a GUI checkbox
   "顯示距離切片 (ESDF)"; when shown, clip the colour at 0.30 m so only near-obstacle distances are coloured;
   legend line: 藍 = 障礙物內部, 白 = 貼近障礙物, 紅 = 遠離 (>= 0.30 m).
   The user saw it lying ON THE FLOOR (our grid_center is z = 0, so the example puts the slice at z = 0,
   and the floor is removed from the map, so it is all "far"). When shown, put it at a GUI slider height,
   default 0.20 m (bar height), never at the floor.
b. Dark images: Newton renders RGB without lights (load_textures=False). RGB is not used for mapping.
   For display only, brighten it (normalise per frame) or replace the RGB panel with a shaded depth
   image; say which.
d. The red square the user saw is the 2 x 2 m map extent (grid_center = robot base, so half the map is under
   the floor/behind the robot). Draw the map bounds as a thin box outline + legend line; consider centring the
   map on the workspace (e.g. (0.45, 0.2, 0.5)) with the same size.
c. Per camera show TWO depth panels side by side: "原始深度" (raw, whole scene) and "建圖用深度" (after
   depth limits, floor removal and RobotSegmenter). Removed pixels are drawn WHITE, not black (user request);
   legend says white = removed (floor, arm, too near/far).

### Previous v7 task (kept for reference)
#### v7 = follow NVIDIA's official cuRobo pipeline with a fixed camera (checkpoint only) — 2026-11-20

The user does not want us to invent this. cuRobo ships an official example that does exactly this (not
nvblox; cuRobo's own warp mapper): `curobo/examples/reference/live_volumetric_mapping_mpc.py`
(github.com/NVlabs/curobo). Pipeline there: fixed RGB-D camera -> `FilterDepth` -> `Mapper`
(`MapperCfg`: voxel_size 0.005, esdf_voxel_size 0.03, truncation 4 x voxel, decay_factor 0.3,
num_cameras) -> `mapper.integrate(CameraObservation)` every frame -> `mapper.compute_esdf()` ->
`ModelPredictiveControlCfg.create(scene_model=Scene(voxel=[voxel_grid]), ...)`. Robot pixels are removed
with `curobo/_src/perception/robot_segmenter.py` (`RobotSegmenter`, robot spheres vs depth points).
Multi-camera: `num_cameras` + stacked CameraObservation (see curobo/tests/_src/perception/mapper/test_multi_camera.py).
Step 0 (stop if it fails): in the school env, check `from curobo.perception import Mapper, MapperCfg, FilterDepth`
and `RobotSegmenter` import. If missing, STOP and report the local cuRobo version; do not install or update
anything and do not touch curobo/.
Then copy that example's structure as closely as possible; only replace the RealSense input with our
Newton render. New files only: scripts/curobo_v7_mapper.py, scripts/run_sim_camera_v7.sh,
config/sim_fixed_camera.yaml (placeholder pose: position (0.50, -0.70, 1.00), looking at (0.50, 0.25, 0.15),
same intrinsics as the wrist camera; do not touch sim_camera_mount.yaml).
1. Inputs: fixed camera (+ wrist camera as the 2nd camera, num_cameras=2). Robot removed by RobotSegmenter
   (not Newton shape ids). Same UR5 MPC settings as v6 (our robot yml, v3 fixed rules), scene = mapper voxel
   grid + the floor cuboid. Keep NVIDIA's defaults; list every value you had to change and why.
2. Same scenes as v6: Dance bar (aimed at visible path) and the still vertical rod (Sofa). No v6 box rules.
3. Viewer like the example (ESDF slice / occupied voxels) + both frustums, floor footprints, depth panels,
   legend.
4. Checkpoint, 2 runs only (E, Dance+still sequence): `--cameras fixed` and `--cameras both`. Report:
   contacts, reaches, min clearance, integrate ms, esdf ms, how long a voxel stays after the bar leaves
   (decay), and whether the arm thrashes. Commit, push, leave --interactive running, URL + 3 lines. Stop.

### Spec kept from v4 (unchanged rules)
: floor, unseen rod top, visible prediction (started 2026-11-19)

v3 (56ea2c3) is reviewed. B 50/50, E 10/50, repeats identical, so 1 repeat is enough now.
Claude checked the 10 E collision onsets (repeat 1): 4 had no box for the whole last 0.3 s (rod out of
view); 6 had a box but its top was 0.05-0.17 m while the rod is 0.25 m, and it lagged 3-5 cm in X.
The arm hit the part it never saw. Also the MPC world has no floor (only `camera_obstacle`), so the
arm may pass through the floor and scoring would not count it.

Keep every v3 rule unchanged except the items below. New files `*_v4*`; do not change v3 code or results.

1. Floor: add a static floor cuboid to the MPC world (top at z=0, or just below if base spheres touch
   it; say which). Score floor contact for all links except the base as a collision too. Check that
   start, goal and raised start are clear.
2. Principle for E: what the camera has not confirmed is treated as occupied.
   a. Unseen top: today the top is observed + 20 mm unless the top point is near camera height. Change
      for the rod case: if the fresh cluster touches any image border, set the box top to the fixed
      `fixed_unknown_top_m` (0.545 m, fixed world value, never camera-relative). Log how often.
   b. Out of view: today memory points stay where they were seen for up to 1 s, so the box stays behind
      while the rod moves away (Claude found box X 0.405-0.468 vs rod X 0.378 at an E collision).
      Instead, while the rod is not seen, grow the remembered box in X and Y by 0.20 m/s x time since
      last seen (0.20 = assumed max rod speed, state it), keep the top rule from 2a, drop after 1 s as
      before. Log the grown size.
   Do not add active "go and look" behaviour; MPC still only tracks the goal.
3. Make prediction visible (replay and interactive):
   - observed box (orange) and predicted swept box (blue) with a legend text in the GUI;
   - a ghost rod where E thinks the rod will be at +0.43 s, and an arrow for estimated velocity;
   - GUI text: estimated vs true rod speed;
   - a checkbox to hide/show prediction in replay.
4. Prediction on/off groups so the report can show why it matters:
   A, B, B-nopred, E, E-nopred. Speeds 0.05, 0.10, 0.20 m/s, 5 phases, 1 repeat, A once (75 runs).
   INVALID_SCENE rule as before. Quiet background run, one line per case, do not read the full log.

Report table per group and speed: success/valid, collision runs, worst clearance, floor contacts,
how often 2a and 2b fired (E), and whether E still has collisions with no box or a box not covering the rod. Add section "v4" to `docs/SIM_CAMERA_MOVING_ROD.md` and a Chinese
table to the zh-TW report with replay and interactive commands. Reply with hash, table, commands.
