# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: READ-ONLY survey for the lab 3D camera + cuRobo package (branch `experiment/camera-survey`)

Goal: find out what exists before anything is installed. A senior said cuRobo maps to a package that
connects the lab's 3D camera; likely NVIDIA Isaac ROS cuMotion + nvblox, but this is unconfirmed.

Rules: shared lab machine. No sudo, no install, no reboot, no driver or system change, no stopping
other people's processes, and do not read other users' home directories. Do not start simulations.

Collect, and write the facts to docs/CAMERA_SURVEY.md (mark anything not found as "not found"):
1. Machine: lsb_release -a; uname -m; echo $ROS_DISTRO; ls /opt/ros; nvidia-smi (driver, CUDA, GPU, memory).
2. Camera: lsusb; ls /dev/video*; rs-enumerate-devices -s if present; any camera vendor tool found on PATH.
3. ROS packages: ros2 pkg list after sourcing Jazzy, filtered for
   realsense|zed|orbbec|azure|kinect|nvblox|cumotion|isaac|curobo.
4. Isaac ROS: docker images and docker ps filtered for isaac|nvblox|curobo (only if docker works
   without sudo); ls -d /opt/*isaac* ~/*isaac* ~/workspaces/isaac_ros* 2>/dev/null.
5. cuRobo: the version and path our Python environment imports (`scripts/run_curobo_v2_viser.sh` shows which env).
6. From official NVIDIA docs if you have web access: which Isaac ROS release supports ROS 2 Jazzy and
   which cameras cuMotion/nvblox officially support. Give the URLs. If no web access, say so.

Finish with a short "open questions for the senior" list. Commit, push, reply in 5 lines with the hash.
