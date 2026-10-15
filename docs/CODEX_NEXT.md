# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: Step C / C1 wall-contact diagnosis (branch `experiment/step-c-cable-wall`)

Context: C0 passed. C1 grasps and routes higher than C0, but Newton reports 81-83 robot-wall
contacts for padding 20 / 25 / 27.5 mm (30 mm stops on elbow travel 1.639 > 1.600 rad).
The count barely changes with padding, so these may be proximity candidates inside the contact
margin/gap, or they may happen in the contact phase (padding 0).

Diagnose before any change. For the robot-wall contacts, report per contact sample:
sim time and task phase (transit / descend / close / lift / release), robot link, penetration depth
or separation distance, and contact force. Distinguish real touches (force > 0 or penetration > 0)
from proximity candidates inside the contact margin/gap.

If they are only candidates: propose counting loaded/penetrating contacts only in
`maximum_robot_obstacle_contacts` (with a unit test) and stop for approval.
Do not change geometry, padding, or physics. Commit evidence; report in 5 lines.
