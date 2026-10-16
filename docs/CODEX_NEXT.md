# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: cuRobo plans, Newton verifies — C1 wall + FEM cable (branch `experiment/curobo-v2-viser`)

MoveIt Step C is already accepted; do not redo a MoveIt comparison.
Goal: replace only the PLANNER with cuRobo and keep everything after it unchanged:
cuRobo plan -> ros2_control (FollowJointTrajectory to joint_trajectory_controller) -> existing
ROS-Newton bridge (sync, start-state guard, trajectory shadow) -> Newton endpoint (FEM cable,
wall, result JSON, metrics).

1. Use profile `step_c_cable_wall` from config/obstacles.yaml for the cuRobo scene (already read by
   ur5_viser_preflight.py) and the same start pose, cable target and gripper width calibration.
2. Sequence: cuRobo plan start -> pre-grasp over the wall; then descend, close, lift, open
   (cuRobo or the existing Cartesian stages, your choice — state which).
3. Log cuRobo planning time per segment and the route max height / min wall distance.
4. Accept exactly as C1: candidate_contact_grasp_pass=true, loaded/penetrating robot-wall
   contacts == 0. GPU cuda:0. Use the stepc tmux session so the user can watch.

Do not change wall, cable, physics, grasp parameters or acceptance rules. Commit evidence,
update docs/CODEX_STATUS.md (5 lines), push, reply with the hash.
Next (not now): moving obstacle with cuRobo replanning.
