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

## Task: simulated depth to world point cloud, visible in Viser (branch `experiment/newton-link-depth-camera`, time box 60 min)

Build on scripts/newton_link_depth_camera_benchmark.py. Same scene, same rules: no install, no sudo, no ROS,
no cuRobo, no real robot, GPU `cuda:0`, Python `~/newton_ws/.venv-cpu/bin/python`. Do not record media. Do not
stop processes you did not start. New script: scripts/newton_sim_depth_cloud_viser.py.

1. Camera: 848x480, vertical FOV 58 deg, using `forward_depth_image` (not `depth_image`). State how
   `camera_fovs` is interpreted. Keep the current mount and label it a placeholder, not our real mount.
2. Point cloud: back-project depth with the pinhole model, then transform to world with the camera pose
   from the same frame. Return time on the same state, so image and pose cannot differ.
3. Robot pixels: if the sensor has a shape-index output, mark points that hit the robot and count them. If not,
   say so and use a simple alternative (render without the robot shapes). Explain the 0.183064 m minimum depth
   seen in both poses of the last report: what does the camera actually hit there?
4. Accuracy, at 3 or more different arm poses, excluding robot points: signed distance from each point to the
   nearest true surface (box, sphere, cylinder, ground, computed analytically from the scene values).
   Report median, p95 and max per surface. Target: median < 1 mm, p95 < 5 mm. If missed, report it; do not tune
   the test to pass.
5. Negative control: repeat step 4 once treating `depth_image` as forward depth. The error must clearly rise. If it
   does not, the test cannot detect this mistake: say so.
6. Timing, median over 120 or more frames, measured separately: depth render, copy to host, point cloud
   conversion, Viser update. Also confirm the loaded URDF is a UR5, not a UR5e, by shoulder height
   (UR5 about 0.0892 m, UR5e about 0.1625 m).

Viser view (bind 127.0.0.1, port 8083, print the URL). The user must be able to see all of this:
- the robot, and the true objects as solid shapes
- the camera frustum at its live pose, and the live depth image as a panel
- the world point cloud on top of the objects, robot points in red, the rest coloured by height
- 6 joint sliders, so moving the arm moves the camera while the cloud stays on the objects
- checkboxes to hide the objects, hide the cloud and hide robot points; a text readout of current point count
  and median error
Keep it on the current pose when idle; do not spin the GPU at full rate.

Append a "Point cloud and Viser" section to docs/SIM_CAMERA_CHECK.md with the results, including a plain
"what you should see" list and the exact one-line run command. Stop every process you started; the user will
open the viewer. Commit by path, push, reply in 5 lines with the hash and the run command.
