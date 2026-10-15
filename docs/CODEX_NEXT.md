# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: fix two acceptance-metric bugs, then finish C1 and C2 (branch `experiment/step-c-cable-wall`)

Review of fe3855f by Claude:

1. APPROVED: `maximum_robot_obstacle_contacts` counts only loaded or penetrating contacts
   (force > 0 or penetration > 0); keep the raw candidate count as a separate diagnostic field.
   Add a unit test (CI). The 82 C1 wall contacts were all candidates (min separation 45.8 mm).

2. BUG, your "cable did not lift" is wrong: in attempt_20261015_a3ef43d the grasp region rose to
   0.124 m (MOVEIT_CONTACT_SAMPLE grasp_region_z_m). lift_started stayed false because the gripper
   in Newton started near 0.79 rad and only opened to 0.511, so `open_seen` (grip <= 0.05) never
   fired before the grasp. Fix: `open_seen` also becomes true when grip <= (first observed grip - 0.15).
   Runs that start open (grip 0) are unchanged. Add a unit test for both cases.

3. Re-run C1 on cuda:0 (fresh bringup, wait for ENDPOINT_READY, clean up after). Accept:
   candidate_contact_grasp_pass=true, loaded/penetrating robot-wall contacts == 0, route higher than C0.
   Then C2 (tall wall must fail planning, no robot command).

Do not change geometry, padding, physics, or grasp parameters. Commit evidence, update
docs/CODEX_STATUS.md (5 lines), push, and reply with the commit hash.
