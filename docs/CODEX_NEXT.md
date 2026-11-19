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

Checkpoint ready for visual review at Z=0.20 m (v6 memory on hold).

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: final poke batch + dodge-dance demo (2026-11-19)

682ea7b accepted (Z=0.20 m, E 0 contacts in smoke). The user is away for dinner and asked for one last
experiment that combines today's work, so this task may run the batch without a further checkpoint.
Keep everything from 682ea7b: floor slab + exclusions, floor 10 mm / bar 30 mm activation, safety plane,
accel limits, start-up look, height memory + robot cap, bar border extension, 8-point bar minimum,
prediction, viewer fixes. Fail-fast rule at the top applies.

1. Batch (bar Z=0.20 m, 20 s each, 1 repeat): lanes Y = 0.20, 0.28, 0.36; poke speeds 0.05, 0.10, 0.20 m/s;
   two start offsets (poke starts at t=0 and at t=2.0 s). Groups A (validity), B, E, E-nopred = 72 runs.
   A lane/speed/offset with no A contact is INVALID_SCENE for all groups. Print expected wall time first.
2. Table per group and speed: success/valid, bar-contact runs, floor-contact runs, worst clearance,
   safety-plane activations; for E hits: no box / box not covering. Short Chinese table in the zh-TW report.
3. Interactive "Dance" button (the user's dodge-dance demo): every 3 s, poke from +X toward the gripper's
   CURRENT Y and Z (Z clamped to 0.15-0.30 m), 0.10 m/s, retract, repeat until pressed again. Works in A/B/E
   and E-nopred; switching stops the old loop as before. Keep the Poke button and XYZ drag.
4. Replay: group / lane / speed / offset dropdowns, depth panel, boxes, ghost, arrow.
5. Interactive vertical cylinder (to verify the unseen-top rule): add a second draggable obstacle, a
   vertical rod (radius 0.025 m, height selectable 0.25 / 0.50 m, standing on the floor), with a toggle to
   enable it next to the bar. E applies the v5 rod rules to it: height memory, raise to 0.545 m only if the
   top was never seen AND not under the arm, robot cap. GUI text for the rod: "top seen" / "assumed tall"
   / "capped by arm", plus the box top height. Small check (not part of the batch): one E run where the
   rod is placed so only its lower part is in view; confirm the box top becomes 0.545 m, and one where the
   arm is above it; confirm it is capped. Report both.
Reply: hash, table, replay + interactive commands, 3 lines on what to watch. Then stop (user will check).

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
