cuRobo V2 replaced only the Step C arm planner; ros2_control, the ROS-Newton bridge, FEM cable, wall, physics, grasp calibration, and acceptance rules remained unchanged.
Commit `58f572c4e85be355d52ddc7d16a7a45f69f61b4f` passed on `cuda:0`: approach/grasp/lift planning took `1.570706 / 0.072951 / 0.076487 s`.
The pre-grasp route reached `0.375155 m` and kept `0.034570 m` fitted-sphere clearance from the wall; all three FK endpoint errors were below `0.5 um`.
Newton reported `candidate_contact_grasp_pass=true`, `0.1199769126 m` lift, `0.1205940407 m` release drop, and zero loaded or penetrating robot-wall contacts.
Complete logs and JSON are in `docs/experiments/curobo-v2/results/step_c/attempt_20261016_58f572c/`; next milestone is moving-obstacle cuRobo replanning, not part of this task.
