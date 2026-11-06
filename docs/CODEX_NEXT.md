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

## Task: fix the unknown-extent rule (branch `experiment/newton-link-depth-camera`, time box 45 min)

Review of 67966b4. The raised start, wrist limit, speed rule and no-collision results are accepted. The stops come from Claude's rule in the last task, not
from your implementation. It treated the image top border as world up, but this camera looks straight down, so image top is a horizontal direction. In
fifth_pass.json the wall top is seen at 0.22 m in cycle 1, yet the box top is 0.445 m (camera height) and climbs to 0.612 m while the tool rises from z 0.501 to 0.627 m:
camera rises, box rises, arm flees. Cycle 12 had no extension, top 0.24 m, current clearance +264 mm; cycle 13 jumped back to 0.605 m and failed.

Same rules as the last task. Edit scripts/curobo_camera_fifth_pass.py and keep its start, goal, wrist limit, speed rule, 80-cycle cap and scenes (tops 0.22, 0.26, 0.30 m).

1. Replace the rule. For each touched image border, compute that border's outward direction in world coordinates from the same-frame camera pose, and project it
   to the horizontal plane. Extend the box horizontally that way by a fixed 0.03 m. Border touches never raise the box.
2. Height comes from observed points plus 20 mm padding only. Exception: if the highest obstacle point is within 0.03 m of the camera height, the camera may be
   below the top. Then set the top to one fixed world height, the raised start camera height + 0.10 m. It must never follow the current camera, so it cannot chase the arm.
3. Check the old failure is gone: in each scene, report box top per cycle and tool height per cycle. The tool must not climb more than the no-obstacle path does without a stated reason.
4. Report the same tables as Fifth pass. For any stop, show current-to-fit and goal-to-fit clearance and the planner status. For the 0.30 m first-cycle failure, state which it was: box
   covering goal, or something else. Do not change the goal.
5. Do not repeat the blind search. Keep 27 candidates, none valid, as the result.
6. Viewer: same cases, on 127.0.0.1:8084. Add the box top height and tool height as text during playback.

If a scene still fails, report it plainly, do not add another rule. If a step blocks you for over 20 minutes, stop and report the exact error. Save raw results to
docs/experiments/sim-camera-curobo/results/sixth_pass.*. Append a "Sixth pass" section to docs/SIM_CAMERA_CUROBO.md, with a correction note on Fifth pass and the exact one-line run command.
Stop every process you started. Commit by path, push, reply in 5 lines with the hash and the run command.
