# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

## Local pre-merge status

No pending task

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: make the moving-rod scene fair, then add prediction and escape (branch `experiment/newton-link-depth-camera`, time box 90 min)

Review of 81505e3. Housekeeping, depth panel, no-write viewer and the shared clock are accepted. The results do not test the camera yet: B (god view)
also collides at 0.05 m/s. Claude's reading of moving_rod.json, three design faults in Claude's spec:
- The goal state is inside the rod's sweep: target-to-box clearance reaches -0.025 m in B. Some collisions cannot be avoided while the task requires
  parking at the goal.
- No escape: once the rod is close, the current state is inside the box, TRAJOPT_FAILED, and the arm holds still while the rod drives through it
  (B 0.05 m/s: 33 held cycles, -42.6 mm while holding).
- No prediction: the planner sees where the rod is, not where it will be after compute plus execution.

Same rules as the last task. Edit scripts/curobo_camera_moving_rod.py and keep everything accepted above.

1. Fair scene: before running, sweep the rod over its whole X range and require at least 25 mm true clearance to the start and goal states at every rod
   position, and that A still collides. If the current lane fails, search lane Y, X range and top height (keep radius 0.025 m, top below the start camera) and
   state the chosen one. Report any lane that fails as INVALID_SCENE.
2. Prediction, same rule for B and E: estimate rod velocity (B from truth, E only from the box centres of recent frames), and stretch the box along that
   velocity by speed x (expected compute + one executed segment) x 2. State the numbers.
3. Escape: if the plan to the current target fails, or the current state is within 30 mm of the box, plan to a retreat pose instead: the raised start, or
   if that also fails, 0.10 m straight up from the current tool. Hold still only if all fail. Count each escape and say which it used.
4. Rerun A, B, E at 0.05, 0.10, 0.20 m/s, the zero-latency control at 0.10, and the blind entry at 0.10, on the fair scene. Split collisions into "while
   moving" and "while holding". Report the fastest speed B survives and the fastest speed E survives.
5. Viewer: same as now, plus the predicted box in a separate colour and an ESCAPE label when it is active. Do not rewrite tracked files.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Keep moving_rod.* as history and save to
moving_rod_v2.*. Append a "Second run" section to docs/SIM_CAMERA_MOVING_ROD.md with a plain "what you should see" list and the exact one-line run command.
Stop every process you started. Commit by path, push, reply in 5 lines with the hash and the run command.
