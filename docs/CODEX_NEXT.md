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

## Task: cuRobo avoids a static obstacle using only the simulated camera (branch `experiment/newton-link-depth-camera`, time box 90 min)

Rules: no install, no sudo, no ROS, no real robot, no Newton physics, do not record media, do not stop processes
you did not start. Leave `curobo/` and `~/isaac_ros_ws` untouched. No nvblox, no cuMotion. Decided already.
Time and moving obstacles are out of scope here. New script(s) under scripts/, report in new docs/SIM_CAMERA_CUROBO.md.

1. Mount: set config/sim_camera_mount.yaml to left 0, up -0.106, forward 0.056, tilt 0, status matched_by_eye (the
   file may already hold this locally). The user compared it with the real camera: neither view shows the gripper.
   Rerun the accuracy table and negative control at this mount and report them with the robot point percentage.
2. Environments: Newton runs in ~/newton_ws/.venv-cpu, cuRobo in ~/curobo_ros_env (see
   scripts/run_curobo_v2_viser.sh). Check whether one can import the other. If not, use two processes with a simple
   file exchange. Do not install anything to merge them.
3. Input format: read the cuRobo 0.8 source and pick the simplest supported way to turn world points into
   obstacles (for example voxels or cuboids). Say what you chose, why, and what you rejected.
4. Pipeline: Newton depth, world cloud, remove robot pixels with the shape index, deal with the ground explicitly
   (say how, and why the floor does not block every plan), convert, give to cuRobo. Start from the start pose and
   cable pre-grasp in docs/experiments/curobo-v2/prototype/curobo_dodge_demo.py. Choose one static obstacle that
   blocks the direct route and state why it does.
5. Compare the same start and goal in four cases, several seeds or repeats each:
   - A no obstacle in the planner (the path must truly collide with the obstacle, otherwise the scene proves nothing)
   - B god view: the true obstacle given directly
   - C camera, one view from the start pose
   - D camera, a short scan of 3 or more arm poses, clouds merged
   Score every path against the TRUE obstacle, never the perceived one: success, minimum true clearance, path length.
   Also report perceived-to-true surface error and timing medians for conversion and planning, measured separately.
6. Blind spot test: put the obstacle where the start-pose view cannot see it and run case C. Report what happens,
   even if the arm hits it. This is an expected limit, not a failure to hide. Do not add a fix.
7. Viser on 127.0.0.1:8084: robot, true obstacle (with hide checkbox), perceived obstacles, camera frustum, the
   planned path, a case selector A/B/C/D plus blind spot, and playback of the arm along the path. Show true clearance
   as text. The user must be able to see each case without rerunning the script.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of the cause, and commit what
works. Include a plain "what you should see" list and the exact one-line run command in the report. Stop every process
you started. Commit by path, push, reply in 5 lines with the hash and the run command.
