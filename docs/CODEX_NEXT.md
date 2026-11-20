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

## Task: v7 — make "seen empty" actually clear, and stop erasing the bar near the arm (2 runs) — 2026-11-20

3f91a1d reviewed. B (no decay) had 0 contacts, but the retracted tip voxels were NEVER cleared (130/130
still there after 4.57 s): B is "keep forever", partly safe only because of phantoms. Two causes in our code:
1. Floor pixels are set to depth 0 (`z <= .012` -> 0). Depth 0 is skipped by integration, so rays that hit
   the floor carve no free space. Behind the bar there is mostly floor, so the vacated space is never
   "seen empty". Fix: do not zero floor pixels; for display keep them, for mapping replace each floor pixel's
   depth by the depth where that ray reaches z = -0.20 m (a fake surface 20 cm under the floor; clip to
   depth_maximum). The ray then carves all free space above the floor and adds no surface near the arm.
   Make sure the ESDF/collision still has no voxels at z > -0.15 from the floor.
2. RobotSegmenter default distance_threshold = 0.05 m erases every depth point within 5 cm of the arm
   spheres, so the bar disappears exactly when it gets close (A: fixed-camera bar pixels 1774 -> 1302 at the
   closest approach). Use distance_threshold = 0.015 (sphere fit + depth noise).
Keep decay 1.0 / frustum 1.0 (config B). Runs, 1 each:
- Empty scene 10 s, both cameras: voxels within 5 cm of the arm (must stay ~0), reaches, reversals.
- Dance 6 s, both cameras: contacts, reaches, min clearance, retracted-tip voxels cleared and max stale
  time (target: cleared within ~0.3 s of being seen through), bar pixels before/after RobotSegmenter at the
  closest approach.
Commit, push, leave --interactive running (both cameras), URL + 3 lines. Stop. Do not do "Next time".

### Next time (user: postpone, display only)
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
