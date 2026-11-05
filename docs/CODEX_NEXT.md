# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.

**Standing rule for every task:** a task is done only when the commit is on GitHub. Commit, run
`git push origin <branch>`, then check that `git ls-remote origin <branch>` equals `git rev-parse HEAD`.
"Everything up-to-date" means nothing new was pushed; it is not success. Reply with the pushed hash.

## Task: D435 check + cuMotion start test (branch `experiment/d435-cumotion`, time box 45 min)

Safety, shared machine: never command a real robot. Do not start a real UR driver or anything that
connects to a robot IP; use fake hardware only. No sudo, install, reboot, or system change. Do not
build or edit anything inside ~/isaac_ros_ws; if a rebuild seems needed, stop and report. Check
`nvidia-smi` first and do not stop other users' processes. Record no video or screenshots.

Write the facts to docs/D435_CUMOTION_CHECK.md, including the exact commands that worked.

### Part 1: D435 (the user just plugged it in)
1. `lsusb`, `ls /dev/video*`, `rs-enumerate-devices -s`: model, serial, firmware, USB type.
   USB type must be 3.x; if it reports 2.x, stop and tell the user to change the cable or port.
2. Start `realsense2_camera` with depth, color and pointcloud enabled in ROS domain 20 for about 30 s.
   Report `ros2 topic hz` for depth, color and pointcloud, plus the frame ids. Then stop it.
3. Tell the user how to view it: only one program can open the camera at a time, so leave the
   camera free and give the exact commands for `realsense-viewer` and for RViz with the pointcloud.

### Part 2: cuMotion start test (no camera needed, no robot motion)
1. In a shell sourcing Jazzy then ~/isaac_ros_ws/install/setup.bash, confirm the cuMotion packages resolve.
   Record which ROS distro it was built against, from the install or build files.
2. Find a UR5 or closest UR robot description/XRDF in isaac_ros_cumotion_robot_description, and
   report what it uses. Do not edit it; if none fits, stop and report.
3. Start the cuMotion planner node. Report whether it starts, reaches ready, and uses the GPU.
   If it does, send one plan-only request with fake hardware and report planning time and success.
4. On failure, paste the exact error and your best read of the cause (version mismatch, missing
   dependency, cuRobo 0.7.7 vs the project's 0.8.0). Do not try fixes beyond environment sourcing.

Stop all processes you started. Commit, push, reply in 5 lines with the hash.
