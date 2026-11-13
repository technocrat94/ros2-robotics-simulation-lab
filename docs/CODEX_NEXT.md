# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

## Local pre-merge status

Pending: v3 task below

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: rod dodge v3 with fixed realistic rules (2026-11-13)

The latency split (effbe9a) is reviewed and closed. Its lesson: rules were not fixed, so every run
exposed a new gap (no acceleration limit, 3.14 rad/s bursts, 9 mm prediction, single runs that flip).
Now fix all rules once, then run A/B/E. MPC only (no planner) in this task.

New files: `scripts/curobo_moving_rod_v3.py`, `scripts/run_sim_camera_moving_rod_v3.sh`. Reuse old code
by import; do not change old behaviour or old results.

Rules (state each in the report with where the value came from):
1. Plant: joint velocity limit from the cuRobo robot config, and a joint acceleration limit. Take it
   from the robot config / UR description if present; if none exists use 3.0 rad/s^2 and say it is an
   assumption. Clamp the commanded velocity change per step to that limit. Give the same limit to the
   MPC if its config supports it.
2. Timing: control period 25 ms, counted compute fixed at 25 ms for every group (not measured).
   Rod observation updates at 30 Hz for both B and E (E: new camera frame every 33.3 ms; between frames
   reuse the last one). Still log real measured compute.
3. Rod prediction: stretch the box along estimated velocity over the full MPC horizon plus one
   observation period (read the horizon length in seconds from the MPC config and log it).
4. MPC: start from the frozen config. If the no-rod reach with the new plant fails or takes over 10 s,
   retune on no-rod only (same method as before), then freeze. Never tune with the rod.
5. Groups A, B, E. Speeds 0.05 and 0.10 m/s, phases as before, 20 s per case. A runs once per
   phase (validity only); B and E run 5 repeats each = 110 cases. INVALID_SCENE as before.
   Run the batch in the background with checkpoints; print one short line per case and the final
   table only. Do not read the full log back (save tokens).

Report per group and speed: success count / total runs, collision runs, worst and median worst clearance,
max joint speed and max acceleration (check they stay within the limits), real compute median/p95.
Say plainly whether E is close to B. No cause analysis in this task.

Simulation version (the user wants to watch it):
- `--replay`: dropdowns group / speed / phase / repeat; robot, rod, magenta camera points, observed and
  predicted boxes, depth panel, sim time, live clearance and joint speed.
- `--interactive`: user drags the rod; A/B/E switch stops the old loop first (as before); same v3 rules.

Save `docs/experiments/sim-camera-curobo/results/moving_rod_v3.{json,log}`. Add section "v3 fixed rules"
to `docs/SIM_CAMERA_MOVING_ROD.md` and a short Chinese table to `docs/SIM_CAMERA_MOVING_ROD_REPORT.zh-TW.md`
with both commands. Do not run interactive while measuring.

Reply with the pushed hash, the table, and the two commands.
