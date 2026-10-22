# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

No pending task
## Task: finish sim-time sync + real dodging (branch `experiment/newton-moving-wall`)

Claude ran the integration on the home VM (commits 1092a71, 9bdbd3c): with use_sim_time the pick
passed (lift 0.120 m, loaded wall contacts 0) and the ROS->Newton backlog fell 552 -> 96 commands
(~11 s -> ~1.9 s lag). Runner used: Newton endpoint first, wait ENDPOINT_READY, ros_adapter,
then bringup use_sim_time:=true, open gripper, run_jazzy_fem_cable_pick.sh use_sim_time:=true
dynamic_obstacle_replanning:=true (profile moving_wall). Wall amplitude is now 0.25 m.

Do (you run everything; report in 5 lines):
1. Find the component still on wall time (likely the ros2_control update loop / fake hardware or
   the adapter's command pacing). Goal: backlog stays <= 5 commands during the whole pick.
2. Apply transit padding 0.02 to the moving_wall profile in the pick script and bringup
   (same rule as step_c_*).
3. Dodging was never triggered: MoveIt planned against the config wall pose and the wall happened
   to be clear. Make MoveIt use the live /newton/obstacle_poses pose, and choose the motion phase so
   the wall moves INTO the planned route during transit. Accept only if the log shows at least one
   stop-and-replan event, candidate_contact_grasp_pass=true, loaded wall contacts 0, and report the
   minimum robot-wall distance.
No video. Commit evidence, update docs/CODEX_STATUS.md, push, reply with the hash.
