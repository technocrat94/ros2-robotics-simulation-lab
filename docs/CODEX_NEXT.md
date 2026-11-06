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

## Task: camera on the correct side, then actually run the skipped cases (branch `experiment/newton-link-depth-camera`, time box 90 min)

Review of 6a37031. Playback fix and in-place planner update are accepted. Two problems remain. Same rules as the last task:
no install, no sudo, no real robot, no Newton physics, no ROS, no nvblox, no cuMotion, no media. Planner input never uses truth
except case B. Do not change config/jazzy_fem_initial_positions.yaml, config/obstacles.yaml, or anything the FEM grasp uses.

Problem 1, all main cases were skipped: goal clearance 29.833 mm failed Claude's 30 mm rule by 0.167 mm. That number was
arbitrary. Use 25 mm from now on, in one named constant.

Problem 2, the camera is on the wrong side. The mount in tool0 is unchanged, but wrist_3 differs by about 180 deg between the poses.
Claude's FK: BASE_ARM, where the user matched the mount, gives camera minus flange about (+0.106, 0, -0.056) m, away from the base.
START_ARM gives about (-0.106, 0, -0.056) m, toward the base. The user says the camera belongs in front (away from the base).

1. Confirm those two offsets from the loaded model and put them in the report. Do not edit the mount yaml.
2. For this experiment only, rotate both start and goal 180 deg about the tool Z axis (wrist_3 by pi, staying inside joint limits).
   Use a clearly named constant in this script. Check that the gripper is symmetric enough for this, and report the flange and tool
   positions before and after. Report the new camera world position at start and at goal.
3. Recompute the scene validity with the new start and goal at 25 mm. Main wall tops 0.18, 0.22, 0.26 m at the same XY. If one is
   invalid, report it and shift only if none is valid.
4. Run everything that was skipped: padding 0 and 20 mm, overview, standard scan, B god view, and E main, on every valid scene. Report
   start, goal and along-path clearance separately, and planner failure status on each failed plan.
5. Blind wall: repeat the search with the new camera. It must be valid, block the route, and be unseen from the start. Run case A, one-view
   C and E. Report first seen cycle, clearance then, final stop reason, and goal reached.
6. E timing: report whole-cycle median and p95 separately for cycles with an obstacle box and without one. The 82 ms in the last
   report came from cycles with no box.
7. The report says "cuRobo 1.5.1" while the checkout was v0.8.0-43. Check which is right and correct it.
8. Viewer: an invalid case must not play the no-obstacle path, because it looks like the planner went through the wall. Show the text status
   and no path. Default to the first valid case. Show the camera world position as text. Add a "no obstacle baseline" case with an explicit
   name, so the collision the avoidance prevents is still visible.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Save raw results
to docs/experiments/sim-camera-curobo/results/ under a new name, keep older ones as history. Append a "Fourth pass" section to
docs/SIM_CAMERA_CUROBO.md with a plain "what you should see" list and the exact one-line run command. Stop every process you started.
Commit by path, push, reply in 5 lines with the hash and the run command.
