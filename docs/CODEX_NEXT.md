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

## Task: real-time look-while-moving with no scan (branch `experiment/newton-link-depth-camera`, time box 90 min)

Direction change from the user: the goal is real-time cuRobo planning from the camera, not a scan before each move. Scans were only a
check that the chain works, and that result stands. Static obstacles only in this task. Same rules: no install, no sudo, no real robot, no
Newton physics, no ROS, no nvblox, no cuMotion, no media. Planner input never uses truth. Do not change
config/jazzy_fem_initial_positions.yaml, config/obstacles.yaml, the mount yaml, the URDF, or anything the FEM grasp uses.

1. Loop only: the only motion is the loop of capture, merge, fit, update, replan, advance. No scan, no separate overview move. Keep one
   warmed planner. Set the cycle cap to 80, so a stop is never just the cap.
2. Raised start: use the camera-front start raised 0.20 m in world Z (the old overview pose, same tool orientation) as the start. Same goal.
   Solve it with wrist_3 close to the flipped value. Report joints, camera position, and how much of the wall the first frame sees.
3. Wrist limit: for this experiment only, limit wrist_3 in the planner to the start value +/- pi. Find the supported cuRobo 0.8 way and say what it
   was. If there is none, reject paths that break the limit and report it.
4. Unknown is not free: if non-floor, non-robot points touch the image border, the object's extent past that border is unknown. Extend the box
   in that direction: upward to the current camera height, sideways by a fixed margin you state. Shrink it back once a later frame shows the
   edge inside the image. Use camera data only. Report for each cycle whether extension was active.
5. Speed: state one simple rule that reduces the advance per cycle when the current state is close to the fitted box. Do not tune it per scene.
6. Scenes: wall tops 0.22, 0.26, 0.30 m at the same XY, with the 25 mm validity check using the raised start, including that the no-obstacle path
   collides. If it flies over the wall, that scene is INVALID_SCENE; report it, and then try taller walls only up to where the goal stays valid.
7. Report per scene: reached goal, cycles, final stop reason, min true clearance, whole-cycle median and p95. Also: joint travel per joint, max wrist_3
   change from start, tool path length over straight-line distance, and the same numbers for the no-obstacle path. Save planner failure status.
8. Blind test: a wall standing on the floor that blocks the route and is unseen in the first frame. If none exists at this camera height, try only a
   short search, report that, and do not use a floating block. Report first seen cycle, clearance then, stop reason, and goal reached.
9. Viewer on 127.0.0.1:8084: only these cases plus NO OBSTACLE BASELINE. Default to the first valid E case. Colour the box differently when extension is
   active, and show the current cycle and stop reason as text. Keep the live frustum, points, clearance and Playback error field.

Older results stay as history. Do not rerun them. If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit
what works. Save raw results to docs/experiments/sim-camera-curobo/results/fifth_pass.*. Append a "Fifth pass" section to docs/SIM_CAMERA_CUROBO.md with a
plain "what you should see" list and the exact one-line run command. Stop every process you started. Commit by path, push, reply in 5 lines with the hash
and the run command.
