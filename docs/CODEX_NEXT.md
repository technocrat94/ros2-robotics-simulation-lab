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

## Task: success rates for tomorrow's report, no new tuning (branch `experiment/newton-link-depth-camera`, time box 75 min)

Review of c5bb696: the no-rod tuning procedure and frozen block are accepted. Single runs are not enough: tuned B prediction passed 0.20 m/s with +1.7 mm
but failed 0.10 m/s, which points to rod phase luck. The user reports to the senior tomorrow and needs rates, not single runs. Change no controller or
perception setting: use the frozen `mpc_no_rod_dt_0.02_nonterminal_0.3` block, the v2 fair scene and the shared clock exactly. Same rules as before.

1. Phases: 5 rod initial phases per speed, fixed before running: initial X 0.36, 0.44, 0.52, 0.60, 0.68 m, moving toward decreasing X. Check A collides in
   each one. Keep phases where A does not collide in the table, but mark them INVALID_SCENE and exclude them from rates.
2. Cases, 20 simulated seconds each, at 0.05 and 0.10 m/s:
   - A: no planner obstacle
   - B: tuned MPC, god view, prediction
   - E: tuned MPC, camera only, prediction, v2 point memory
   E is run at both speeds regardless of the old 0.10 gate; the user asked for it, so state that the gate was lifted for this report.
3. Report per case and speed: success rate (zero collisions and at least one reach, over valid phases), mean reaches, total collision episodes, worst
   clearance, and max joint speed and acceleration. Also give the per-phase table.
4. For E, log per frame how far its box is from the true rod box (missing height, missing width, offset), and list the three worst frames with time and phase.
5. Viewer through the per-user port: the cases above, selectable by speed, phase and case. Must not rewrite tracked files.
6. Write a one-page Traditional Chinese summary docs/SIM_CAMERA_MOVING_ROD_REPORT.zh-TW.md for the senior: the question, the setup in plain words, the rate
   table, what works, what fails and why, and the limits (ideal velocity plant with no acceleration limit, simulated noise-free camera, one rod, one box,
   simulator shape index for robot pixels, camera sees only in front of and below the gripper). Only state measured numbers.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Save to
docs/experiments/sim-camera-curobo/results/moving_rod_rates.*. Stop every process you started. Commit by path, push, reply in 5 lines with the hash and the run
command.
