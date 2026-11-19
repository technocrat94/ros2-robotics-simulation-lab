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

**Standing rule: stop at a checkpoint for the user (from 2026-11-19).**
Every task ends at a checkpoint the user can check by eye: a short smoke run plus a Viser command
(replay and/or interactive) and 3 lines on what to look at. Never start a batch (> 10 runs) in the same
task; the batch is its own later task after the user says the checkpoint looks right.

## Local pre-merge status

Pending: horizontal bar checkpoint (v6 memory on hold)

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: horizontal bar scene, checkpoint only (2026-11-19)

User wants an NVIDIA-style demo with a HORIZONTAL bar (like a stick held across), dodged by the arm.
Camera mount stays fixed. Keep v5 code (floor, safety plane, accel limits, start-up look, gates) and
dd7ff91 viewer fix (also check other numpy scalars passed to Viser in the files you touch).

1. New scene option: horizontal bar, radius 0.02 m, length 0.60 m, along world X, centre X=0.50.
   Scripted motion: the bar moves along Y between 0.05 and 0.55 at height Z=0.35 m, crossing the arm's
   start-to-goal path (tool goes Y 0.12 -> 0.45). Check A collides; if not, report the nearest
   height/lane that makes A collide and stop.
2. E perception: v5 rules, but turn OFF the "raise top to 0.545 m" rule for this bar (it is meant for
   vertical rods); instead, if the bar cluster touches an image border, extend the box along X to the
   full 0.60 m bar length centred on the seen part. Log when it fires.
3. Interactive mode: user drags the bar in X, Y and Z (gizmo), groups A/B/E switch cleanly as before.
4. Smoke only: B and E, scripted motion, speed 0.10 m/s, 20 s, one start phase. NO batch.
Reply: hash, smoke results (reach, rod/floor contact, worst clearance), the replay and interactive commands,
and 3 lines on what the user should look at. Then STOP.

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
