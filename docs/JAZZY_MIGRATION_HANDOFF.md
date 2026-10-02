# Jazzy FEM Grasp Migration Handoff

## Verified outcome — 2026-10-02

The direct native Jazzy FEM ground-pick was executed and visually confirmed in the Newton viewer. It starts from the Humble-success IK pose, avoids the named-start detour, uses `/newton/object_pose`, lifts the FEM grasp region by approximately `0.11735 m`, and releases after reopening. The software and physical acceptance layers are **PASS** for this run.

The reproducible English guide is `docs/jazzy/JAZZY_REPRODUCIBILITY.md`; the Traditional Chinese translation is `docs/jazzy/JAZZY_REPRODUCIBILITY.zh-TW.md`.

Last updated: 2026-10-01 (Asia/Taipei)

## Status

**INCOMPLETE — DO NOT CLAIM JAZZY GRASP PASS.**

The Humble source bundle has been audited and its successful FEM/contact/queue architecture has been ported into the school Jazzy workspace. The Jazzy MoveIt sequence completes, ROS-to-Newton shadow playback is accurate, the FEM topology and material values are correct, and CUDA execution on the RTX 3080 is active. However, the free FEM strip has not yet been physically lifted and released in Jazzy. The latest full run completed the robot motion while the strip center remained at floor height.

No commit or push has been made. Show the user the final changed-file list and validation evidence before either action.

## Source bundle and integrity

Bundle:

`/home/aisc216/Downloads/humble_fem_grasp_success_2026-10-01/humble_fem_grasp_success_2026-10-01`

Before editing, these files were read:

- `README.md`
- `metadata/environment.txt`
- `metadata/git_state.txt`
- `metadata/source_files.txt`

`sha256sum -c CHECKSUMS.sha256` passed all 29 entries. The archived Humble result did not contain the exact clean quantitative success log, so Jazzy still requires a fresh measured lift/release result.

## Environment and repository

- Workspace: `/home/aisc216/ur5_ws`
- Repository: `/home/aisc216/ur5_ws/src/ur5_moveit_demo`
- Branch: `jazzy-port`
- Baseline commit: `e50f068e9b4ee62956c23d5ca98bac9dcae16dae`
- OS: Ubuntu 24.04.4 LTS
- Architecture: x86_64
- ROS: Jazzy
- Python: 3.12.3
- Newton: 1.5.1
- Warp: 1.17.0
- NumPy: 2.5.3
- Viser: 1.0.26
- GPU: NVIDIA GeForce RTX 3080, 10 GiB
- NVIDIA driver: 580.173.02
- CUDA device used by the current Jazzy run: `cuda:0`
- GPU smoke-test toolkit/driver reported by Warp: toolkit 12.9, driver 13.0

The virtual-environment directory is still named `.venv-cpu`, but that is only a directory name. The Jazzy launch script now explicitly selects `GRASP_DEVICE=cuda:0`.

## Architecture audit

| Successful Humble feature | Initial Jazzy status | Current status |
|---|---|---|
| Full 20 x 3 x 2 tetrahedral strip | MISSING | PRESENT: 252 particles, 600 tetrahedra |
| Density 1100 kg/m^3 | MISSING | PRESENT |
| Young's modulus 1 MPa | MISSING | PRESENT |
| Poisson ratio 0.45 | MISSING | PRESENT |
| Free object, `fix_left=false` | MISSING | PRESENT |
| No attachment constraint | Framework present | PRESENT; attachment is none |
| `SolverVBD` | MISSING | PRESENT, 10 iterations |
| Full-surface rigid-soft contact | MISSING | PRESENT |
| Hidden analytic finger/fingertip boxes | MISSING | PRESENT: four proxies |
| Visual finger meshes remain visible | PRESENT | PRESERVED |
| Finger mesh particle collision disabled | MISSING | PRESENT |
| Proxy `COLLIDE_PARTICLES` enabled | MISSING | PRESENT |
| Bounded command queue | MISSING | PRESENT: capacity 4096 |
| ROS sample interpolation | MISSING | PRESENT: command dt 0.02 s |
| Physics substeps | Partial | PRESENT: 10 |
| Jazzy controller-state API | INCOMPATIBLE with Humble bundle | Existing Jazzy `controller_state/reference` path preserved |
| MoveIt Plan fields | INCOMPATIBLE | Ported to Jazzy fields without Humble underscore suffix |
| Cartesian time parameterization | INCOMPATIBLE | Uses Jazzy `TimeOptimalTrajectoryGeneration` |
| Humble absolute URDF paths | INCOMPATIBLE | URDF regenerated from Jazzy xacro; 34 mesh paths verified |

## Preserved physics/configuration

- Object model: `GRASP_OBJECT_MODEL=fem_strip`
- FEM cells: `20 x 3 x 2`
- Particles: 252
- Tetrahedra: 600
- Total mass: approximately 0.44 kg
- Density: 1100 kg/m^3
- Young's modulus: 1,000,000 Pa
- Poisson ratio: 0.45
- Soft damping: 1000
- Solver: `SolverVBD`
- VBD iterations: 10
- Substeps: 10
- Particle radius: 0.001 m
- Soft contact stiffness: 1000
- Soft contact damping: 10
- Soft contact margin: 0.005 m
- Robot, strip, and ground friction: 1.5
- Proxy inset: 0.001 m
- Maximum allowed penetration: 0.005 m
- Command sample interval: 0.02 s
- Maximum identical-command hold: 0.5 s
- Queue capacity: 4096
- Attachment constraint: none
- `fix_left=false`
- Lift distance: 0.12 m
- Lift velocity scale: 0.03

## Changed files and reasons

- `docs/experiments/newton-robot-grasp/prototype/robot_segmented_grasp_batch.py`
  - Ported the full FEM strip, VBD-compatible material construction, hidden analytic proxies, collision flags, mass scaling, and selectable `GRASP_DEVICE`.
- `docs/experiments/newton-ros2-bridge/prototype/newton_moveit_grasp_endpoint.py`
  - Ported the bounded queue/interpolation/substep endpoint and added quantitative FEM contact, lift, release, penetration, finite-state, mimic-error, and queue metrics.
- `src/pick_at_position.cpp`
  - Ported staged pregrasp and slow lift behavior; adapted Humble MoveIt API fields to Jazzy and uses `TimeOptimalTrajectoryGeneration`.
- `launch/pick_at_position.launch.py`
  - Added ported grasp arguments and correctly passes `joint_limits`/`robot_description_planning` into the pick node. This fixed Jazzy Cartesian time-parameterization failure.
- `scripts/bootstrap_school_jazzy.sh`
  - Synchronizes runtime prototypes, regenerates the Jazzy URDF, rejects Humble and `/home/yuhao` paths, parses the XML, and verifies every mesh.
- `scripts/run_newton_ground_grasp_jazzy.sh`
  - Exports the exact FEM/contact/queue settings and now selects RTX 3080 with `CUDA_VISIBLE_DEVICES=0` and `GRASP_DEVICE=cuda:0`.
- `docs/experiments/jazzy-fem-migration/results/`
  - Contains layered validation logs listed below.

Runtime copies generated by bootstrap:

- `/home/aisc216/ur5_ws/src/newton_ros_bridge`
- `/home/aisc216/newton_ws/lessons/segmented_strip`
- `/home/aisc216/newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf`

## Build and validation evidence

### Build

`PIP_NO_INDEX=1 ./scripts/bootstrap_school_jazzy.sh` completed successfully. Four packages built:

- `robotiq_controllers`
- `robotiq_description`
- `newton_ros_bridge`
- `ur5_moveit_demo`

The generated URDF contains 34 verified mesh paths and no Humble or `/home/yuhao` path.

### FEM topology/import

Validated:

- 252 particles
- 600 tetrahedra
- approximately 0.44 kg total mass
- 24 robot bodies
- 58 robot shapes including four hidden FEM proxies
- all four proxies use particle collision
- original Robotiq finger meshes do not use particle collision
- 252 free particles and zero fixed particles

### Endpoint idle validation

The instrumented endpoint ran with finite values. Observed floor penetration was approximately `1.74e-5 m`, below the `0.005 m` bound.

### ROS/MoveIt validation

- Newton object pose feedback received.
- Start-state synchronization passed with maximum initial error approximately `5.25e-8 rad`.
- The direct all-Cartesian safe pregrasp path from the default high pose produced 0% and was rejected.
- Direct OMPL pregrasp from the default pose timed out.
- Existing Humble-supported route via named `test_configuration`, followed by OMPL absolute pregrasp, succeeded.
- Approach and lift Cartesian paths both completed at 100%.
- After passing Jazzy joint-limit parameters to the pick node:
  - approach duration: 1.213 s at velocity scale 0.15
  - lift duration: approximately 4.124 s at velocity scale 0.03
  - fake-hardware task ended with `ABSOLUTE POSITION PICK SUCCEEDED`

### CPU full-run diagnostic

The CPU endpoint was finite and had no queue overflow, but ran around 0.04x realtime. An interrupted diagnostic result was not a grasp pass and retained pending commands.

### GPU validation and current performance

Warp recognizes and uses:

`cuda:0 NVIDIA GeForce RTX 3080 (10 GiB, sm_86)`

The first CUDA run compiled and cached collision, XPBD, rigid VBD, and particle VBD kernels. The isolated GPU smoke result remained finite and simulated 16.8167 s in 207.017 s including first-time compilation.

A later live full-run sample on 2026-10-01 recorded:

- endpoint wall elapsed: 2728 s
- simulated time: 266.0333333331 s
- realtime factor: `266.0333333331 / 2728 = 0.0975x`
- equivalent: approximately `10.25 real seconds per simulated second`
- GPU utilization at sample: 49%
- GPU memory in use: 1464 MiB
- endpoint CPU use: approximately one full CPU core
- shadow error: `1.1645e-7 rad`
- object center z: `0.0109815225 m`

The arm reached the final lifted robot pose and the gripper reopened, but the object remained at its initial floor height. This is definitive evidence that the latest run did **not** physically lift the FEM strip. GPU acceleration improves throughput but does not fix the contact outcome.

## Result interpretation

Current layer status:

- Robot import: PASS
- Jazzy-specific URDF mesh paths: PASS
- FEM topology/material/free boundary: PASS
- VBD/full-surface initialization: PASS
- Hidden analytic proxies/flags: PASS
- Bridge synchronization: PASS
- MoveIt preview/kinematic execution: PASS using named start plus OMPL pregrasp
- Queue/interpolation/substeps: PRESENT and observed
- Finite state: PASS in completed diagnostics
- Bounded penetration: PASS in idle/smoke diagnostics
- Bilateral loaded contact during closure: NOT YET PROVEN
- Bilateral loaded contact during lift: NOT YET PROVEN
- Physical strip lift: FAIL in latest observed full run
- Release drop after a successful lift: NOT TESTED
- Overall Jazzy grasp: **NO PASS**

## Logs

All logs are under:

`docs/experiments/jazzy-fem-migration/results/`

Important files:

- `fem_endpoint_instrumented_idle.log`
- `preview_pose_only.log`
- `preview_guard.log`
- `preview_plan_only.log`
- `preview_ompl_plan_only.log`
- `named_pregrasp_execute.log`
- `approach_plan_only.log`
- `moveit_full_fake_fixed.log`
- `full_fem_endpoint.log`
- `full_fem_pick.log`
- `full_fem_guard.log`
- `full_fem_sync.log`
- `full_fem_shadow.log`
- `gpu_smoke_endpoint.log`
- `gpu_smoke_endpoint_second.log`

No validated success video exists yet.

## Clean startup commands

Use four terminals. In terminals 1, 3, and 4 first run:

```bash
cd /home/aisc216/ur5_ws/src/ur5_moveit_demo
source /opt/ros/jazzy/setup.bash
source /home/aisc216/ur5_ws/install_jazzy_port/setup.bash
source scripts/lab_session_env.sh
```

Terminal 1:

```bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Terminal 2:

```bash
cd /home/aisc216/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_ground_grasp_jazzy.sh
```

Terminal 3:

```bash
ros2 run newton_ros_bridge ros_adapter
```

Terminal 4:

```bash
T=std_srvs/srv/Trigger
ros2 service call /newton/sync_robot_state "$T" "{}"
ros2 run newton_ros_bridge start_state_guard
B=std_srvs/srv/SetBool
S=/newton/set_trajectory_shadow
ros2 service call "$S" "$B" "{data: true}"
P=pick_at_position.launch.py
A=use_named_start:=true
C=use_safe_pregrasp_path:=false
ros2 launch ur5_moveit_demo "$P" "$A" "$C"
```

## Clean rebuild

```bash
cd /home/aisc216/ur5_ws/src/ur5_moveit_demo
PIP_NO_INDEX=1 ./scripts/bootstrap_school_jazzy.sh
```

Do not copy Humble `build/`, `install/`, `log/`, or a Humble Python virtual environment.

## Next session: exact continuation point

1. Read this file and inspect `git status`; do not overwrite the current Jazzy changes.
2. Reproduce one clean GPU full run with endpoint output redirected to a new timestamped log.
3. Stop the endpoint only after `MOVEIT_COMMAND_QUEUE_DRAINED` and sufficient post-release settling, then save the complete `MOVEIT_GRASP_RESULT` line as standalone JSON.
4. Use its left/right candidate and loaded soft-contact metrics to locate the failure:
   - if both candidate counts are zero, audit proxy/body transforms and target alignment;
   - if candidates exist but loaded counts are zero, audit activation-depth interpretation/contact flags;
   - if bilateral loaded contact exists during closure but disappears before lift, audit command timing/interpolation while preserving the grasp behavior;
   - do not alter FEM material, proxy geometry, or grasp target merely to force a pass.
5. Capture visual evidence only after metrics show bilateral loaded closure/lift, at least 0.08 m strip lift, bounded penetration, and at least 0.05 m post-open drop.
6. Update this document with the final result JSON/log/video paths.
7. Show the user the changed-file list and validation evidence before commit or push.

## Safety and scope

All work remained under `/home/aisc216`. Isolated tests used separate ROS domains and UDP ports. Existing user and real-robot processes were not killed. No commit or push was performed.

