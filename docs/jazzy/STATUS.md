# Jazzy port status

Last updated: 2026-10-02 (Asia/Taipei)

## 2026-10-02 direct FEM ground-pick verification

Native Jazzy was visually verified in the Newton viewer after starting from the Humble-success IK pose:

- `START_STATE_GUARD pass=true`, maximum error `1.9436783e-07 rad`; bridge status `OK`.
- Target came from `/newton/object_pose`; safe pre-grasp, approach, and lift Cartesian paths were all `100.0%`.
- No named-start or `test_configuration` detour appeared; MoveIt ended with `ABSOLUTE POSITION PICK SUCCEEDED`.
- Newton FEM grasp-region height rose from about `0.01095 m` to `0.12830 m` (`0.11735 m` lift); the viewer showed the strip rising between the fingers and releasing after reopening.
- Full logs are retained under `/home/aisc216/ur5_ws/run_logs/`; the machine-readable summary is `docs/experiments/jazzy-fem-migration/results/jazzy_fem_pick_success_2026-10-02.json`.

## Scope and source state

- Machine: Ubuntu 24.04, native ROS 2 Jazzy, amd64.
- Branch: `jazzy-final-fem-pick`.
- Humble source of truth: `origin/main@61aae20`; verified Humble FEM baseline: `3546672`.
- Previous verified Jazzy commit: `abccf2b39ab0f87abf6667981cd1e91efba43e5a`.
- The pre-existing school changes from `main` remain preserved in `stash@{0}` and were not mixed into this port.
- No Docker Humble environment was started. No Humble build, install, log, or ARM virtual environment was copied.

## Completed

- Ported the UR5 and Robotiq Xacro, SRDF, MoveIt configuration, launch files, and C++ plan API to Jazzy.
- Built and ran the native Jazzy MoveIt fake-hardware stack with all four controllers active.
- Executed the `move_xyz` fake-hardware pick-and-place motion successfully in RViz.
- Created a fresh Python 3.12 amd64 environment at `~/newton_ws/.venv-cpu` with Newton 1.5.1, Warp 1.17.0, Viser 1.0.26, and NumPy 2.5.3.
- Generated `~/newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf` from the Jazzy Xacro with all package mesh paths resolved.
- Built `newton_ros_bridge` on Jazzy and discovered its `ros_adapter` and `start_state_guard` executables.
- Ported the adapter from the Humble controller topic and `desired.positions` field to Jazzy's `/joint_trajectory_controller/controller_state` and `reference.positions`.
- Displayed the UR5, Robotiq, and 20-segment strip in the localhost Newton Viser viewer.
- Ran the complete measured-object-pose MoveIt–Newton ground-grasp workflow.

## Rebuild and run

Use the idempotent Jazzy bootstrap:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/bootstrap_school_jazzy.sh
```

Use [NEWTON_RUNBOOK.md](NEWTON_RUNBOOK.md) for the verified four-terminal launch sequence. Run only one `ur5_robotiq_bringup.launch.py` instance per ROS domain.

## Actual test evidence

The verified 2026-10-02 direct Jazzy run started from the Humble-success IK pose and used the measured target from `/newton/object_pose`.

```text
bridge_status=OK
START_STATE_GUARD pass=true maximum_error_rad=1.9436783e-07
safe_pregrasp raise/orient/translate/descend=100.0%
approach_cartesian_fraction=100.0%
lift_cartesian_fraction=100.0%
ABSOLUTE POSITION PICK SUCCEEDED
named_start_detour=false
finite_state=true
grasp_region_lift_m=0.1173476921
```

The viewer was checked by the user: the strip rose between the fingers and released after reopening. The machine-readable summary and repo-relative raw logs are indexed in `docs/experiments/jazzy-fem-migration/results/README.md`.

## Reproducibility status

- The source branch is `jazzy-final-fem-pick`; Humble reference code remains available in `origin/main`.
- `scripts/bootstrap_school_jazzy.sh` rebuilds the Jazzy workspace from tracked source and verifies 34 mesh paths.
- `docs/jazzy/JAZZY_REPRODUCIBILITY.md` is the English runbook; `JAZZY_REPRODUCIBILITY.zh-TW.md` is the Chinese translation.
- Use exactly one bringup, one move_group/controller_manager, one adapter, and one endpoint per isolated ROS domain.
