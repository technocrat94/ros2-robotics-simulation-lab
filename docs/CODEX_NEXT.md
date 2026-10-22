# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: Newton moving wall — CODE ONLY (branch `experiment/newton-moving-wall`)

Token rules: read only this file and the files you edit. Do NOT run Newton/ROS simulations,
record media, or write long docs — Claude runs and validates on the home VM. If blocked > 10 min,
stop. Reply in 3 lines with the pushed hash.

Write the code for:
1. config/obstacles.yaml: new profile `moving_wall` = step_c_cable_wall plus
   `motion: {axis: x, amplitude_m: 0.10, period_s: 8.0}`. Existing profiles unchanged (CI too).
2. Newton (robot_segmented_grasp_batch.py + newton_moveit_grasp_endpoint.py): when the profile has
   `motion`, the wall is a kinematic body whose pose is set every substep from SIM time
   (center + amplitude*sin(2*pi*t/period)); contacts must not push it. Static profiles: identical
   behaviour to today.
3. Endpoint publishes wall pose with the sim timestamp on the existing state socket so
   ros_adapter republishes it as /newton/obstacle_poses (geometry_msgs/PoseArray) and also publishes
   /clock from Newton sim time.
4. pick_at_position: optional `use_sim_time` passthrough in launch files (default false), and an
   optional replanning mode: subscribe /newton/obstacle_poses, update the MoveIt collision object,
   and if the remaining trajectory comes within padding of the wall, stop and replan from the current
   state (default off).
5. Unit tests for the wall pose function and a CI check that default profiles are unchanged.
Run scripts/ci_checks.py only. Commit, push.
