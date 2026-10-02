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
- Branch: `jazzy-port`.
- Humble reference baseline visible in history: `3925912`.
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

The MoveIt-only demonstration reached the named configuration, completed all three Cartesian paths at `100.0%`, operated the gripper, and ended with `PICK AND PLACE DEMO SUCCEEDED`.

Newton CPU execution passed a 20-segment headless simulation with finite state. The Viser server accepted a browser connection at `http://127.0.0.1:30000`.

The integrated Jazzy bridge reported:

```text
object pose = (0.48689985, 0.10915001, 0.00999984) m in world
START_STATE_GUARD pass=true maximum_error_rad=5.6182855e-08
trajectory shadow enabled
bridge status = OK
trajectory_shadow_error = 5.6578260e-08 rad
mimic_max_error = 0.0 rad
```

The absolute-position task consumed `/newton/object_pose`, completed its global pre-grasp plan and both Cartesian paths at `100.0%`, closed and reopened the gripper, and ended with `ABSOLUTE POSITION PICK SUCCEEDED`.

Newton's final physical result was:

```text
closed_seen=true
lift_started=true
release_seen=true
grasp_region_lift_m=0.0006303657
minimum_strip_bottom_z_m=-5.5157e-07
maximum_contact_count=1900
finite_state=true
candidate_contact_grasp_pass=false
```

This reproduces the Humble boundary: planning, controller execution, ROS–Newton transport, state synchronization, trajectory shadow, contact generation, and visualization work; the dynamic ground pickup still does not retain the strip during lift.

## Unresolved

- The physical ground grasp still fails. MoveIt success is not physical grasp success.
- Contact instrumentation still reports aggregate magnitude. The next experiment must record per-finger world-frame `|Fx|`, `|Fy|`, and `|Fz|` during closure and the first lift frames before changing friction or compression again.
- `ROS_LOCALHOST_ONLY` is deprecated in Jazzy but remains honored. Migrating the isolation script to the newer discovery settings is future cleanup.
- The run intentionally forced CPU execution. FEM/GPU testing remains deferred until the segmented reference passes.

## Next step

Add per-finger `|Fx|`, `|Fy|`, and `|Fz|` measurements to `newton_moveit_grasp_endpoint.py`, rerun one clean ground-grasp trial, and save the machine-readable force trace before changing any contact parameter.
