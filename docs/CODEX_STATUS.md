C1 diagnosis is complete: all 82 reported robot-wall contacts were proximity candidates, not collisions.
Across 2,231 samples, the minimum separation was 45.794 mm; penetration, force, and real-touch counts were all zero.
MoveIt completed its motion, but the cable did not lift, so C1 remains unaccepted; the run used cuda:0 and changed no physics or grasp parameters.
The complete evidence is in `docs/experiments/rigid-obstacles/results/step_c/attempt_20261015_a3ef43d/`; evidence commit: `dd112a3`.
Proposed next step, awaiting approval: count only loaded or penetrating contacts for acceptance and add a unit test while retaining raw candidate diagnostics.
