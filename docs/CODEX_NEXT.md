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

## Task: remove the 130 mm crutch, then look again while moving (branch `experiment/newton-link-depth-camera`, time box 90 min)

Review of efa26cb: the D result is not accepted. Its fitted top is 0.3142 m, of which 0.130 m is FIT_PADDING_Z_M, so the
camera's highest seen point was about 0.184 m on a 0.310 m wall. The top was never seen, and the padding lands 4 mm above
truth. Rules: no install, no sudo, no real robot, no Newton physics, no ROS, no nvblox, no cuMotion, no media. Leave
`curobo/` and `~/isaac_ros_ws` untouched. Static obstacles only. Planner input may never use truth geometry except case B.
Keep two processes: a persistent Newton worker over a pipe or local files is fine.

1. Correct docs/SIM_CAMERA_CUROBO.md: mark the D verdict as depending on 130 mm Z padding, and state the highest seen z
   for C and D. Keep the old numbers as history.
2. Padding sweep: rerun case D with Z padding 0, 20, 50, 80, 130 mm. Then, with one fixed setting, run wall heights
   0.25, 0.31, 0.40 m. Report highest seen z, fitted top, true clearance and plan result for each. Do not tune to pass.
3. Overview pose: add one fixed pose, chosen without using the wall position, that raises the camera and looks down at the
   workspace. State how it was chosen. Repeat the height table with padding 20 mm. Say whether the wall top is seen.
4. Scan safety: score every scan pose and the motion between poses against the true obstacle. A scan that collides counts
   as a failure of that case, even if the later path is safe. Plan these moves; do not teleport without saying so.
5. Look while moving (new case E): repeat this cycle: capture at the current arm pose, merge into the accumulated
   cloud, refit, replan from the current state, advance a short fixed part of the path (state the length). Use padding 20
   mm. Score every executed state against truth. Stop at the goal, on a true collision (the planner is not told), or on
   no plan (hold position and report it). Report cycles, cycles without a plan, min true clearance, whether goal reached,
   and median capture and plan times per cycle.
6. Blind spot: run case E on the existing blind obstacle. Report at which cycle it is first seen and how far the arm
   was from it then, and whether it stopped or hit it. Report it plainly even if it hits. No added fix.
7. Repeats: state whether the seeded repeats gave different paths or identical ones.
8. Viser on 127.0.0.1:8084:
   - draw the perceived points, and show seen height and padding as separate parts (two colours or a marked line)
   - frustum follows the arm during playback; draw a frustum for every capture pose
   - add cases for padding, wall height, overview, E and E blind spot, each with playback, without rerunning
   - in E, show each cycle's points and box as they grow during playback; show true clearance live
9. Results: save raw JSON and logs to docs/experiments/sim-camera-curobo/results/. Keep them small (no NPZ).

Known limits, list but do not solve: all points fit into one box, so several obstacles merge; anything below z = 0.012 m is
deleted, including a cable; robot removal uses the simulator's shape index, which a real camera lacks.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Append
a "Second pass" section to docs/SIM_CAMERA_CUROBO.md with a plain "what you should see" list and the exact one-line run
command. Stop every process you started. Commit by path, push, reply in 5 lines with the hash and the run command.
