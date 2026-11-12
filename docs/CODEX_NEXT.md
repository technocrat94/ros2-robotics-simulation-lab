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

## Task: does cuRobo MPC dodge the rod better? (branch `experiment/newton-link-depth-camera`, time box 90 min)

Review of 2f7fcda: the fair scene, prediction, escape and viewer are accepted. The failure is the controller. In B and E the predicted box covers the current
arm state, every plan is refused (start in collision, TRAJOPT_FAILED), the arm freezes and the rod drives through it (B 0.10 cycles 41-45, E 0.10 cycles
25-29). Stop-and-go plus whole-path replanning cannot push away continuously. Same rules as before: no install, no sudo, no real robot, no Newton physics,
no ROS, no nvblox, no cuMotion, no media. Do not change the accepted moving-rod v2 script behaviour, the FEM grasp path or shared configs. Use
scripts/lab_session_env.sh and its per-user port; do not stop processes you did not start.

1. Survey, read only: in the project's cuRobo 0.8 checkout (`curobo/`), find the MPC (model predictive control) API. Report the class and file, how it gets a
   goal, how obstacles are updated each step, its step rate on this GPU, and whether it can start from a state that is already in collision. If there is no
   usable MPC, say so, and do step 3 instead of step 2.
2. MPC prototype, new script scripts/curobo_moving_rod_mpc.py, on the v2 fair scene and the same shared clock. Use god view only: B's true rod box,
   with and without the v2 prediction stretch. Continuous control: the arm keeps executing the previous command while the next step is computed; count
   that compute time on the clock. Same shuttle task, speeds 0.05, 0.10, 0.20 m/s. Report reaches, collision episodes split into moving and holding, min true
   clearance, step time median and p95, and direction of motion relative to the rod when it is within 50 mm.
3. Only if there is no MPC: the patch. When the current state is inside the box, plan against the current box without prediction, and choose the retreat that
   increases distance from the rod. Same report.
4. Compare directly with v2 B in one table. Only if MPC B survives 0.10 m/s: run camera E once at 0.10 m/s with the same controller and add it to the table.
5. Viewer through the per-user port: same view as v2, plus the controller name and step rate, and selectable cases. Must not rewrite tracked files.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Save to
docs/experiments/sim-camera-curobo/results/moving_rod_mpc.*. Append an "MPC prototype" section to docs/SIM_CAMERA_MOVING_ROD.md and a short note in
docs/SIM_CAMERA_MOVING_ROD.zh-TW.md, each with a plain "what you should see" list and the exact one-line run command. Stop every process you started. Commit
by path, push, reply in 5 lines with the hash and the run command.
