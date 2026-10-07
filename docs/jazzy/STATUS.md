# Jazzy port status

Last updated: 2026-10-08 (Asia/Taipei)

Jazzy is the primary platform. `main` is the source of truth; the verified
state is released as `v0.2.0-fem-grasp`.

## Accepted result — 2026-10-02 17:29 run

The rerun kept the endpoint alive through release settling and printed a
complete `MOVEIT_GRASP_RESULT`
(`docs/experiments/jazzy-fem-migration/results/jazzy_fem_pick_result_2026-10-02.json`):

```text
candidate_contact_grasp_pass=true
grasp_region_lift_m=0.1173
release_drop_m=0.1123
minimum_strip_bottom_z_m=-0.00082
bilateral_closed_soft_contact_samples=34
bilateral_lift_soft_contact_samples=229
finite_state=true
device=cuda:0 (RTX 3080), real_time_factor=0.102
```

## 2026-10-02 first direct FEM ground-pick verification (visual)

Native Jazzy was visually verified in the Newton viewer after starting from the Humble-success IK pose:

- `START_STATE_GUARD pass=true`, maximum error `1.9436783e-07 rad`; bridge status `OK`.
- Target came from `/newton/object_pose`; safe pre-grasp, approach, and lift Cartesian paths were all `100.0%`.
- No named-start or `test_configuration` detour appeared; MoveIt ended with `ABSOLUTE POSITION PICK SUCCEEDED`.
- Newton FEM grasp-region height rose from about `0.01095 m` to `0.12830 m` (`0.11735 m` lift); the viewer showed the strip rising between the fingers and releasing after reopening.
- The machine-readable summary is `docs/experiments/jazzy-fem-migration/results/jazzy_fem_pick_success_2026-10-02.json`; repo-relative logs are indexed in `docs/experiments/jazzy-fem-migration/results/README.md`.

## Scope and source state

- Machine: Ubuntu 24.04, native ROS 2 Jazzy, amd64, NVIDIA RTX 3080.
- Source: `main` (the former `jazzy-final-fem-pick` branch was fast-forwarded into `main` at `7f4d715`, tag `jazzy-fem-grasp-v1`).
- Humble baseline: tag `humble-fem-baseline` (`3546672`), CPU.
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

- The source is `main`; Humble reference code remains available at tag `humble-fem-baseline`.
- `scripts/bootstrap_school_jazzy.sh` rebuilds the Jazzy workspace from tracked source and verifies 34 mesh paths.
- `docs/jazzy/JAZZY_REPRODUCIBILITY.md` is the English runbook; `JAZZY_REPRODUCIBILITY.zh-TW.md` is the Chinese translation.
- Use exactly one bringup, one move_group/controller_manager, one adapter, and one endpoint per isolated ROS domain.
