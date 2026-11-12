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

## Task: make MPC reach its goal at a normal speed, then retest the rod (branch `experiment/newton-link-depth-camera`, time box 90 min)

Review of 0abadef: the MPC adapter, shared clock and no-write viewer are accepted. MPC B never collided, but never reached a target. Claude computed the tool
path from the saved trace (MPC B current box 0.10 m/s): 37.8 cm from the goal at 0 s, 16.4 cm at 10 s, 1.0 cm at 20 s. It crossed the rod lane above the rod
(z about 0.43 m) at about 4.5 s. Average tool speed is about 2 cm/s and slows near the goal, so it never enters the 6 mm tolerance. The user confirmed that in the
viewer the simulated time runs normally while the arm moves slowly. Compute is about 20.8 ms per 25 ms step, so this is controller behaviour, not hardware.
Same rules as the last task. Do not change library code or the accepted v2 and MPC prototype behaviour; add a new tuned configuration beside them.

1. No-rod reference: with no obstacle, time a start-to-goal reach for (a) the v2 MotionPlanner path at its own timing and (b) the current MPC settings.
   Report time to enter 6 mm, and the tool distance to goal over time.
2. Find the cause: read the MPC config actually loaded and list the settings that limit speed or final convergence (for example velocity scaling, goal cost
   weights, horizon, convergence or terminal terms). Say which one causes the slow final approach, with evidence from step 1.
3. Tune with no rod only: change as few settings as possible until MPC reaches within 6 mm in no more than twice the v2 reach time and stays within joint
   velocity limits. Report max joint velocity and max joint acceleration of the result, since the plant has no acceleration limit. Then freeze the settings in
   one named config block. Never tune against rod results.
4. Rerun MPC B, current box and prediction, at 0.05, 0.10 and 0.20 m/s, with the frozen settings. Same table as the MPC prototype plus the v2 B rows.
5. If B survives 0.10 m/s (zero collisions and at least one reach), run camera E at every speed B survived, with the same controller. Also log per frame how far
   E's box is from the true rod box (missing height, missing width, offset), so partial views can be found. Report the worst frames.
6. Viewer through the per-user port, the new cases selectable. Must not rewrite tracked files.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Save to
docs/experiments/sim-camera-curobo/results/moving_rod_mpc_tuned.*. Append an "MPC tuned" section to docs/SIM_CAMERA_MOVING_ROD.md and a short note in
docs/SIM_CAMERA_MOVING_ROD.zh-TW.md, each with a plain "what you should see" list and the exact one-line run command. Stop every process you started. Commit
by path, push, reply in 5 lines with the hash and the run command.
