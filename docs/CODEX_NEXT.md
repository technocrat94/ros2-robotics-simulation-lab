# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: moving obstacle with cuRobo replanning (branch `experiment/moving-obstacle`)

Base: main with the accepted cuRobo C1 (cuRobo -> ros2_control -> bridge -> Newton).
Work in milestones; STOP after each one and report (5 lines) before the next.

M1 Newton only: make the Step C wall a kinematic body that moves on a prescribed path
   (e.g. slides along x, amplitude 0.10 m, period 8 s; parameters in config/obstacles.yaml under a
   new profile `moving_wall`, default profiles unchanged). No ROS. Accept: motion follows the
   prescription, the wall does not get pushed by contacts, finite state.
M2 Publish the wall pose from the Newton endpoint on a ROS topic (e.g. /newton/obstacle_poses) at
   >= 20 Hz with timestamps. Accept: rate and latency measured.
M3 Plan-only: cuRobo updates its scene from that topic and replans; show in the viewer that the
   route changes as the wall moves. Log replanning time per update.
M4 Closed loop: execute with replanning (replace the running trajectory when the current one
   becomes unsafe). Accept: candidate_contact_grasp_pass=true and loaded/penetrating robot-wall
   contacts == 0; also report the minimum robot-wall distance over the run.

Rules: keep the static profiles and all accepted behaviour unchanged; wall speed must stay low
enough that the planner can react (report the speed used). Use the stepc tmux session.
Commit evidence per milestone, update docs/CODEX_STATUS.md, push, reply with the hash.
