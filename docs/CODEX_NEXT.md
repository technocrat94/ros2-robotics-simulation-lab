# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task (TIME BOX: 45 minutes): cuRobo-only moving-wall replanning demo (branch `experiment/moving-obstacle`)

No Newton, no ROS execution. Build on docs/experiments/curobo-v2/prototype/ur5_viser_preflight.py
(new script, do not change the accepted one).

1. cuRobo scene = profile step_c_cable_wall (same start pose, cable target, wall size).
2. Move the wall along x: center x = 0.487 + 0.10*sin(2*pi*t/8), t in seconds.
3. Loop for ~30 s: every 0.2 s update the wall pose in the cuRobo scene and replan from the current
   simulated robot state to the pre-grasp over the wall; advance the robot along the latest plan.
4. Show it live in the cuRobo Viser viewer (wall + robot + current planned path); give the URL.
5. Log each replanning time (ms) and success; print mean / p95 / max and success rate at the end.
6. Record a 30 s screen video (1280x720) to docs/experiments/curobo-v2/media/moving_wall_replan.mp4
   and a GIF copy < 5 MB.

If something blocks for more than 10 minutes, stop and report what works so far.
Commit, update docs/CODEX_STATUS.md (3 lines with the timing numbers), push, reply with the hash.
