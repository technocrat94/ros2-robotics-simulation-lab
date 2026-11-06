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

## Task: valid scenes, working playback, explained stops (branch `experiment/newton-link-depth-camera`, time box 90 min)

The user saw the wall overlap the UR5 start pose in Viser, and "Play selected case" does nothing. Same rules as the last
task: no install, no sudo, no real robot, no Newton physics, no ROS, no nvblox, no cuMotion, no media. Planner input never
uses truth except case B. Edit scripts/curobo_camera_second_pass.py and related scripts.

Review of 1a9963c. Claude's hand FK of the start pose: flange (0.488, 0.116, 0.301), wrist_1 centre (0.525, 0.256, 0.383),
directly above the wall footprint. Confirm these from the model:
- A 0.40 m wall contains the wrist_1 axis, so the start is already in collision. That row is an invalid scene, not a
  perception limit. The -60.71 mm is the start state, not the raise.
- `+29.29 mm` at the 0.31 m wall equals the start-state clearance, so it says nothing about how well the path dodges.
- The camera is at about z = 0.245 at the start, below the 0.31 m top, which is why the start view misses it.
- The blind wall spans x 0.407-0.567, y 0.365-0.415, z up to 0.30. The goal tool position (0.487, 0.410, 0.263) is inside it,
  so no path exists. The blind A, C and E results are invalid.

1. Playback: find the real cause. Reproduce by invoking the registered click callback for each case type, on the same
   non-main thread as the viewer, and check that the robot joint state and frustum actually change. Catch thread exceptions and
   show them in a GUI text field plus stderr. Report the cause. Add this check to the verification run.
2. Scene validity: before any case runs, compute true clearance (all collision spheres) of the start and goal states.
   Require at least 30 mm at both, and require that the no-obstacle path truly collides. Otherwise label it `INVALID_SCENE` in
   the JSON and Viser, and exclude it from pass/fail. Do not count it as a perception failure.
3. New scenes: main wall tops 0.18, 0.22, 0.26 m at the same XY. Blind wall: new position that blocks the route, is unseen
   from the start and passes the validity check. If a height is invalid, report it. If none is valid, shift the wall in Y and
   state it. The 0.25/0.31/0.40 rows stay in the report as history, with 0.40 and the old blind results marked invalid.
4. Clearance reporting: report start, goal and along-path clearance separately, so the start pose cannot hide the path result.
5. Stop reasons: save the planner's failure status for every failed plan in the JSON, including the padding sweep. Add the
   clearance of the current state to the fitted box at each E cycle. State whether stops were caused by the box covering the
   current state or goal. Do not guess; show the values.
6. Planner reuse in E: keep one planner and update the obstacle in place, as curobo_dodge_demo.py does. If box size cannot be
   updated in place, say so and use the cheapest supported way. Time the whole cycle: capture, fit, update, plan, with cold
   start reported apart.
7. Rerun padding 0/20 mm, overview, E main, E blind on valid scenes. Report whether E reaches the goal. If it stops, report it; do
   not add a fix for reaching it.
8. Viewer: frustum must read config/sim_camera_mount.yaml instead of the hard-coded values in camera_pose_for_q. Show the
   INVALID_SCENE label and any no-plan reason as text.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Save raw results
to docs/experiments/sim-camera-curobo/results/ under a new name, keep second_pass.* as history. Append a "Third pass" section to
docs/SIM_CAMERA_CUROBO.md with a plain "what you should see" list and the exact one-line run command. Stop every process you
started. Commit by path, push, reply in 5 lines with the hash and the run command.
