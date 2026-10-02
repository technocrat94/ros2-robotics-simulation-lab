# Jazzy FEM Grasp Migration Handoff

## Verified outcome — 2026-10-02

The direct native Jazzy FEM ground-pick was executed and visually confirmed in the Newton viewer. It starts from the Humble-success IK pose, avoids the named-start detour, uses `/newton/object_pose`, lifts the FEM grasp region by approximately `0.11735 m`, and releases after reopening. The software and physical acceptance layers are **PASS** for this run.

The reproducible English guide is `docs/jazzy/JAZZY_REPRODUCIBILITY.md`; the Traditional Chinese translation is `docs/jazzy/JAZZY_REPRODUCIBILITY.zh-TW.md`.

Last updated: 2026-10-02 (Asia/Taipei)

## Status

**VERIFIED — native Jazzy FEM ground-pick passed.**

The direct Jazzy run starts from the Humble-success IK pose, uses `/newton/object_pose`, avoids named-start/test-configuration detours, closes the Robotiq gripper, lifts the FEM grasp region by approximately `0.11735 m`, and releases after reopening. The user visually confirmed the strip stayed between the fingers during lift and dropped after reopening.

The Humble reference is `origin/main@61aae20` with the verified FEM baseline at `3546672`. Jazzy-specific source, scripts, configuration, raw logs, JSON, and English/Chinese runbooks are tracked in this branch.

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

The verified direct Jazzy run on 2026-10-02 used `cuda:0` on an NVIDIA RTX 3080 and retained the physics/grasp parameters from the Humble source. Its grasp-region lift was `0.1173476921 m`; the standalone endpoint result records simulated time, wall time, real-time factor, bilateral contact counts, release drop, minimum strip bottom z, and `candidate_contact_grasp_pass`.

## Result interpretation

Current verified layer status:

- Robot import and Jazzy mesh paths: PASS.
- FEM topology/material/free boundary and VBD/full-surface contact: PASS.
- Bridge synchronization and start-state guard: PASS.
- Direct safe pre-grasp, approach, and lift Cartesian paths: PASS at `100.0%`.
- No named-start or `test_configuration` detour: PASS.
- Newton finite state and release-after-reopen evidence: PASS.
- Physical strip lift: PASS in the 2026-10-02 viewer-confirmed run.

The complete result JSON and repo-relative logs are indexed in `docs/experiments/jazzy-fem-migration/results/README.md`.

## Logs

The key preview, bridge, endpoint, MoveIt, and successful release records are listed in `docs/experiments/jazzy-fem-migration/results/README.md`. The standalone 2026-10-02 `MOVEIT_GRASP_RESULT` JSON is the authoritative quantitative result for this run.

## Clean rebuild

```bash
cd /home/aisc216/ur5_ws/src/ur5_moveit_demo
PIP_NO_INDEX=1 ./scripts/bootstrap_school_jazzy.sh
```

Do not copy Humble `build/`, `install/`, `log/`, or a Humble Python virtual environment.

## Safety and scope

All work remained under `/home/aisc216`. Isolated tests used separate ROS domains and UDP ports. Existing user and real-robot processes were not killed. No commit or push was performed.

