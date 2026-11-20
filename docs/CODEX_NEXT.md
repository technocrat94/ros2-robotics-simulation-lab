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

## Task: v6 interactive — prediction readout + legend check (~10 min) — 2026-11-20

d3e2978 reviewed (camera view is back). User report: the predicted box looks >= 1.5x the bar length after
the bar moves, and the arm thrashes when the box is pushed onto it; colours were confusing.
Claude changed scripts/curobo_v6_memory.py (20 GPU-free tests pass; do not change these rules):
- `clip_sweep`: the swept (MPC) box stops 2 cm before the arm's current spheres (CameraMemory.boxes).
- Memory fills are now purple = STATIC, yellow = MOVING (blue/orange were already used); full Chinese
  legend `LEGEND` in the GUI.
Do:
1. Add a live GUI readout per obstacle: observed length, predicted length, estimated speed, true speed
   (from the gizmo/dance motion), sweep kept fraction. Same numbers per frame in the replay rows.
2. Check the legend shows at the top of the GUI in --interactive and --replay, and every listed colour
   matches what is drawn (fix colours to match the legend, not the other way round).
3. --interactive --viewer-smoke: Dance 10 s, then drag-like motion of the bar at 0.3 m/s for 2 s; print
   max predicted/observed length ratio and max |estimated - true| speed for both. No batch.
4. Commit, push, leave --interactive running for the user, give the URL + 3 lines on what to look at. Stop.

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
