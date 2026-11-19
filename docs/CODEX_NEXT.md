# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

**Standing rule: fail fast, never run a broken batch to the end.**
- Before any batch over 10 runs: run a smoke set first (A and B, speed 0.05, one phase). If A has any
  floor contact or B does not succeed, STOP and report. Do not start the batch.
- During a batch, stop at once and report (keep the checkpoint) if any of these happen: floor depth
  > 0.02 m in any run; the first 3 B runs all fail; any non-finite value; a run takes > 3x its expected
  wall time.
- Before starting, print the expected total wall time. Do not wait silently for a known-bad result.

## Local pre-merge status

Pending: v4.1 task below

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: v4.1 fix the floor, then rerun v4 (2026-11-19)

v4 (1880b81) reviewed: all 75 runs fail, even A, with nonbase links ~0.8-1.0 m under the floor.
Claude's reading (check it): start, raised start and goal all have the SAME 3.12 mm floor clearance,
so the closest sphere is one that never moves up or down, almost surely `shoulder_link` (rotates only
about joint 1 at base height). It sits inside the 0.03 m collision activation distance at every pose,
so floor cost is always on. And the floor slab is only 0.1 m thick: a sphere pushed past its middle gets
a gradient DOWN and the arm dives through. Also Claude's rough FK of saved traces shows v3 B/E and two
11/13 rate runs already dipped below z=0 when there was no floor.

1. Confirm which link(s) give the 3.12 mm. Treat links that cannot move vertically (base + shoulder_link)
   as base: exclude them from floor cost and floor scoring. Say the final list.
2. Floor slab thick: top z=0, thickness 2 m (centre z=-1), so the gradient always points up.
3. Preflight: every scored sphere at start/raised start/goal has floor clearance >= 0.03 m. If not,
   report and stop (do not move the goal without asking).
4. No-rod A reach must pass with no floor contact before the batch. Never tune with the rod.
5. Rescore old saved traces for floor contact with the same nonbase list (no rerun): rates (7527503)
   and v3. Report runs with floor contact and deepest depth per group.
6. Rerun the v4 batch exactly as specified below (75 runs), same table, viewer, commands.

Reply: hash, link list, preflight clearances, old-run floor rescoring, v4 table.

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
