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

## Task: dodge a moving rod using only the arm camera (branch `experiment/newton-link-depth-camera`, time box 120 min)

The senior wants something like NVIDIA's demos where a stick is pushed into the workspace and the arm avoids it. Static walls passed in Sixth pass;
this is the first test where each frame shows something new. Same rules as before: no install, no sudo, no real robot, no Newton physics, no ROS, no
nvblox, no cuMotion, no media. Planner input never uses truth except case B. Do not change config/jazzy_fem_initial_positions.yaml,
config/obstacles.yaml, the mount yaml, the URDF, or anything the FEM grasp uses. New script scripts/curobo_camera_moving_rod.py and launcher
scripts/run_sim_camera_moving_rod.sh; reuse the Sixth pass code (raised start, wrist_3 limit, speed rule, border rule, one warmed planner).

1. Housekeeping first:
   - Viewer runs must never rewrite tracked files. Results go to the repo only with an explicit --save-results flag.
   - Add the live simulated depth image panel back to the viewer, as scripts/newton_sim_depth_cloud_viser.py had.
   - In docs/SIM_CAMERA_CUROBO.md, note that run_sim_camera_curobo_fifth_pass.sh now runs the corrected rule, so Fifth pass is reproduced only at 67966b4.
2. Rod: a vertical cylinder, radius 0.025 m, standing on the floor, top 0.35 m (below the start camera). It moves along world X at constant speed,
   back and forth between x 0.30 and 0.70 m, at y 0.263 (across the arm route). Move it in Newton by its body pose each capture, not by rebuilding
   the model, and mind the old `q[6:] = 0.0` trap. The planner sees only the camera box.
3. Arm task: shuttle between the raised start and the goal for 20 s of simulated time. Count reaches, true collisions, min true clearance, and time
   spent stopped with no plan.
4. Old points: points inside the current camera view are replaced by this frame. Points outside it are kept for at most 1.0 s of simulated time,
   then dropped. Report how many stale points the box used each cycle.
5. Time: one simulated clock. Each cycle the rod moves for the cycle's measured compute time plus the duration of the executed segment, so the planner gets
   no free time. The arm does not move while computing (stop and go); say so in the report. Also run once with compute time counted as zero, to show
   what latency costs.
6. Cases, each at rod speeds 0.05, 0.10 and 0.20 m/s, same seeds:
   - A no obstacle in the planner (must collide, or the scene proves nothing)
   - B god view: the rod's true bounding box each cycle, same clock
   - E camera only, with rules 4 and 5
7. One blind case at 0.10 m/s: the rod enters from behind the camera, outside its view. Report what happens, even if it hits. Add no fix.
8. Report per case: reaches, collisions, min true clearance, stopped time, whole-cycle median and p95, and the rod-to-arm distance when the rod was first
   seen. Save planner failure status. State the fastest speed E survives.
9. Viewer on 127.0.0.1:8084: true rod, camera points, fitted box, live frustum, depth panel, sim time, rod speed, current case. Playback plays the cycles
   in simulated time. Keep the Playback error field.

Known limits, list but do not solve: one box for all points; floor cut at z 0.012 m; robot pixels from the simulator shape index; the camera only sees
in front of and below the gripper.

If a step blocks you for over 20 minutes, stop, report the exact error and your read of it, and commit what works. Save raw results to
docs/experiments/sim-camera-curobo/results/moving_rod.*. Write docs/SIM_CAMERA_MOVING_ROD.md with a plain "what you should see" list and the exact one-line
run command. Stop every process you started. Commit by path, push, reply in 5 lines with the hash and the run command.
