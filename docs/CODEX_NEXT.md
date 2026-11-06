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

## Task: adjustable camera mount in Viser (branch `experiment/newton-link-depth-camera`, time box 45 min)

The real D435 sits to the upper left of the gripper, not at the current placeholder. You cannot see the robot, so
do not guess the true values: give the user controls to match it by eye. Same rules as before: no install, no
sudo, no ROS, no cuRobo, no real robot, do not record media, do not stop processes you did not start.
Edit scripts/newton_sim_depth_cloud_viser.py.

1. New file config/sim_camera_mount.yaml with `parent_link: ur/tool0`, `left_m`, `up_m`, `forward_m`, `tilt_deg`,
   and `status: unmeasured_initial_guess`. Initial guess: left 0.05, up 0.08, forward 0.03, tilt 15. The script
   reads it; no mount value stays hard-coded.
2. Define the axes from the loaded model and state them in the report:
   - forward: flange toward fingertips (confirm the tool0 axis from the model)
   - left/right: the finger opening axis, measured from the finger links
   - up: perpendicular to both; left and up as seen when looking from flange toward fingertips
   `forward_m` is measured from the flange face. `tilt_deg` turns the optical axis from parallel-to-forward toward
   the gripper centre line; 0 means parallel. Image up stays on the `up` side.
3. Viser: add a "Camera mount" folder with 4 sliders (left and up -0.20 to 0.20 m, forward -0.10 to 0.20 m, tilt
   -60 to 60 deg). Moving one recaptures and updates camera, depth, cloud and readouts. Add a "Save mount" button
   that writes the yaml, keeps the file format, and sets `status: matched_by_eye`. Draw labelled LEFT, UP and
   FORWARD arrows at the gripper, with a checkbox to hide them, so the directions cannot be misread.
   Also read out the current mount values and the robot point percentage.
4. Accuracy: run the existing accuracy table and negative control at the initial guess and at one other mount, 3
   arm poses each. Same targets. Report robot point percentage and the nearest hit link at each mount.
5. Correct the record in docs/SIM_CAMERA_CHECK.md:
   - Mark the first half (benchmark section) as having rendered no target objects, so its 0.2277/0.2610 ms and
     its depth values do not describe the stated scene.
   - The report blames eval_fk. Test whether `q[6:] = 0.0` in the old script is the real cause, because it also
     zeroes free-joint coordinates of the objects. State what the test showed. Say what a moving obstacle must do to
     avoid this.
   Do not change the old benchmark's measured numbers.

Append an "Adjustable mount" section to docs/SIM_CAMERA_CHECK.md, including what the user should do and the
exact one-line run command. Slider and button callbacks must actually be exercised by the check, not just the
capture path; say how you did it. Stop every process you started. Commit by path, push, reply in 5 lines with the
hash and the run command.
