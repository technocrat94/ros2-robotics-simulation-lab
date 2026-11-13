# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

## Local pre-merge status

Pending: latency split task below

## Working tree note

Leave the deleted 05_c2_tall_wall_refused.png and the untracked curobo/ untouched. Stage files by
explicit path only. No git clean/reset/restore/stash/add -A.

## Task: split E failures into perception vs latency (2026-11-13)

Problem: E fails more than B, but E also computes ~50 ms per step vs B ~19 ms, and the clock counts
real compute. We cannot tell if E fails from what the camera sees or from reacting later.

Do: add `scripts/curobo_moving_rod_latency.py` and `scripts/run_sim_camera_moving_rod_latency.sh`.
Reuse `curobo_moving_rod_rates.py` / `curobo_moving_rod_mpc_tuned.py` code by import; do not change
their behaviour or their saved results.

- One new option in the case loop: `counted_latency_s`. When set, use it instead of measured
  `compute_s` for: the clock advance during compute, the command index, `expected_compute`
  (prediction horizon), and `compute_counted_s`. Still record the real measured compute per step.
  Real compute longer than the counted value just pauses the simulated world. Nothing else changes:
  same frozen MPC config, same rod, phases, speeds, 20 s per case, same success and INVALID_SCENE rule.
- Groups (all with prediction, v2 point memory for E):
  A20 (validity only), B20, B50, E20, E50 — the number is counted latency in ms.
- Run 2 speeds x 5 phases x 5 groups = 50 cases. If A20 does not collide in a phase, that phase is
  INVALID_SCENE for all groups.
- How to read it (write this in the report): B50 vs B20 = cost of latency with perfect perception;
  E20 vs B20 = cost of perception at equal latency; E50 should be close to the old E result.
- `--replay` viewer like the rates one, with group dropdown A20/B20/B50/E20/E50 + speed + phase,
  camera points in the same strong colour, depth image panel.
- Save `docs/experiments/sim-camera-curobo/results/moving_rod_latency.{json,log}`.
- Add section "Latency split" to `docs/SIM_CAMERA_MOVING_ROD.md` and a short table to
  `docs/SIM_CAMERA_MOVING_ROD_REPORT.zh-TW.md`: success/valid, collisions, worst clearance, real
  compute median/p95 per group and speed. Also check: is each fixed-latency group repeatable? Rerun
  B20 and E20 at 0.10 m/s X=0.60 once more and report if success or worst clearance changed.
- Do not run the interactive demo while measuring (GPU timing).

Reply with the pushed hash and the 5-group table.
