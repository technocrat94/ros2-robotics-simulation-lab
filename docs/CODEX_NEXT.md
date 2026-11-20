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

## Task: v6 rerun 2 (checkpoint only, ~10 min) — 2026-11-20

3b6a1aa reviewed; camera layers are back, thanks. Findings from the checkpoint file:
- Car: the bar was seen only 0.03-0.47 s. Its MOVING box then slid at constant velocity for 8.5 s straight
  through the workspace (x 0.57 -> -0.55), a phantom that pushed the arm; the real bar's later pokes were
  never seen and E hit it twice (min clearance -43 mm).
- Sofa: STATIC box kept for 8.6 s as intended, but 0 reaches.
- `obstacle_pixels` is always 0 for the bar (wrong metadata key).
- Car scene bug (found by the user): Dance aims the bar tip at the gripper itself (`dance_center` uses the
  tool x/y/z). The camera looks down, so a bar at the camera's own height is never in view. That tests
  nothing. Fix in a v6-only Dance (do not change curobo_moving_bar_poke_demo.py): aim at the arm's PATH,
  i.e. the point 0.15 m ahead of the tool on the straight line tool -> goal (the goal if closer), with
  z = clip(that z, 0.15, tool z - 0.12) so it is below the camera. Log per poke whether the tip at its
  closest point projects inside the depth image.
Claude changed one rule (11 GPU-free tests pass): a MOVING box out of view is kept only for MOVING_MEMORY_S
(= the 0.5 s velocity window), then cleared_by `moved_on`. STATIC is unchanged. Do not change rules. Do:
1. Fix the bar `obstacle_pixels` log.
2. Rerun `--smoke` (Car E, Sofa E) and add one Sofa B run (truth box) to check the goal is reachable
   with the rod there. If Sofa B also has 0 reaches, move the rod (still seen at start, leaves the view)
   until B reaches, then rerun Sofa E.
3. Report per run: contacts, reaches, min clearance, cleared_by events with times; for Car also each
   poke's target, whether it was in view, and its seen interval. If a poke is aimed at the path but never
   seen, report it (do not tune it away).
4. Replay + interactive commands, 3 lines on what to look at. Commit, push, stop.

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
