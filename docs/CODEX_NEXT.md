# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

No pending task
## Task: READ-ONLY follow-up, existing Isaac folders + camera (branch `experiment/camera-survey`)

Same rules as the first survey: no sudo, install, reboot, or system change; do not build, launch, or
modify anything in these folders. Append the facts to docs/CAMERA_SURVEY.md under "Follow-up".

1. For ~/isaac_ros_ws, ~/Isaac+ROS, ~/isaacsim_curobo, ~/isaac_ros_venv: owner, last-modified date,
   top-level contents. For isaac_ros_ws: list src/ packages, whether build/ and install/ exist, and
   any git remote and branch/tag (this shows the Isaac ROS release). Note whether cumotion or nvblox
   packages are in src/ or install/.
2. Who created them is unknown: report dates only, do not guess.
3. If a camera is now plugged in: lsusb, ls /dev/video*, rs-enumerate-devices -s (model, serial,
   firmware, USB type). If not, say "still not connected".
Do not stream video or start realsense2_camera. Commit, push, reply in 5 lines with the hash.
