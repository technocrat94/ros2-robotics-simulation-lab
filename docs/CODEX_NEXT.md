# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task (TIME BOX 45 min): cuRobo dodging demo — the arm really moves (branch `experiment/curobo-dodge`)

Review of moving_wall_replan.py: the arm advanced only ONE 25 ms waypoint per 0.2 s replan, so it
moved at 1/8 speed and looked like a line being drawn; the wall slid along its own length, so the
route barely changed. Make a new script (keep moving_wall_replan.py unchanged), cuRobo only, no ROS/Newton:

1. Arm shuttles continuously between the start pose (tool over y=0.116) and the pre-grasp over the
   cable (y=0.410); when a goal is reached, switch to the other one.
2. Obstacle: a CYLINDER (radius 0.05 m, height 0.25 m, vertical) crossing the arm's route
   (if cuRobo V2 has no cylinder primitive, use the closest supported shape and say which).
   Two modes, selected by a flag:
   a. `--mode random --seed N`: random but smooth motion generated at runtime (e.g. random
      waypoints inside x 0.40-0.58, y 0.15-0.38, z 0.30-0.45, interpolated, speed <= 0.15 m/s).
      Print the seed and log the obstacle pose every cycle.
   b. `--mode interactive`: the user drags the cylinder in the Viser viewer with a transform gizmo;
      its pose is read every cycle.
   Proof that nothing is pre-scripted: run random mode with 3 different seeds and report that the
   obstacle paths and the arm paths differ, all with minimum distance > 0.
3. Every 0.2 s: update the cube pose, replan from the current state to the current goal, then
   advance the robot along the new plan by 0.2 s of trajectory time (real-time motion).
4. Every cycle log: replan time (ms), success, and the minimum robot-to-cube distance from cuRobo's
   collision checker. At the end print mean/p95/max replan time, success rate, overall minimum
   distance (must stay > 0), and number of goal switches.
5. Live in the Viser viewer (cube + robot + current path). Do NOT record video or screenshots;
   leave it running in the stepc tmux session and give the user the URL.

If blocked > 10 minutes, stop and report what works. Commit code + JSON/log, update
docs/CODEX_STATUS.md (3 lines with the numbers), push, reply with the hash.
