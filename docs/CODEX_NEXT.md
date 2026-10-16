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
2. Obstacle: a 0.10 m cube that crosses the arm's route: it moves back and forth along y between
   0.15 and 0.38 at tool-route height (z ~0.40), period ~6 s. Remove the static Step C wall.
3. Every 0.2 s: update the cube pose, replan from the current state to the current goal, then
   advance the robot along the new plan by 0.2 s of trajectory time (real-time motion).
4. Every cycle log: replan time (ms), success, and the minimum robot-to-cube distance from cuRobo's
   collision checker. At the end print mean/p95/max replan time, success rate, overall minimum
   distance (must stay > 0), and number of goal switches.
5. Live in the Viser viewer (cube + robot + current path). Do NOT record video or screenshots;
   leave it running in the stepc tmux session and give the user the URL.

If blocked > 10 minutes, stop and report what works. Commit code + JSON/log, update
docs/CODEX_STATUS.md (3 lines with the numbers), push, reply with the hash.
