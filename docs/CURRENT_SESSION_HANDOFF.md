# Current Session Handoff

Updated: 2026-09-24

## Recovery rule

This file is durable project memory. Verify the remote branch, exact commit,
working tree, machine, ROS distribution, and active processes before changing
anything. Never depend on chat history as the only record.

## Platform decision

The home UTM system remains the reference until the current ground-grasp
acceptance test passes:

- Ubuntu 22.04, `aarch64`, ROS 2 Humble;
- Newton 1.5.1 in a Python 3.12 CPU environment;
- fake-hardware MoveIt execution plus Newton contact validation.

The school computer is Ubuntu 24.04 `x86_64`, ROS 2 Jazzy, Docker-capable, and
has an NVIDIA GPU. After the Humble baseline passes and is tagged, create a
separate Jazzy port. Once the port reproduces the reference metrics, Jazzy
becomes the primary platform for new work and Humble remains a frozen baseline.
Do not copy `build`, `install`, `log`, ARM binaries, or Python virtual
environments between machines. See `docs/JAZZY_PORT_PLAN.md`.

## Verified milestones

1. UR5 and Robotiq MoveIt fake-hardware task builds and executes.
2. The ROS–Newton bridge returns state, exposes stale data, and supports
   explicit start-state synchronization.
3. UR5/Robotiq imports into Newton with 24 bodies, 24 Newton joints, 54 shapes,
   and verified mimic mapping.
4. FEM and compliant-joint strip studies produced convergence and runtime
   evidence; the segmented model is the fast integration reference.
5. A pre-positioned, elevated center grasp lifted, retained, stress-moved, and
   released the segmented strip without an attachment constraint.
6. MoveIt trajectory shadow reproduced the verified Cartesian task in Newton
   after the start-state guard passed.
7. The absolute-position node consumes `/newton/object_pose`, adds a MoveIt
   floor, plans a top-down pre-grasp, isolates individual stages, and executes
   successfully against fake hardware.
8. The gripper width calibration maps a requested gap to leader angle and
   compensates for the fingertip midpoint's downward motion during closure.
9. The combined Newton endpoint reports loaded contacts, force magnitudes,
   penetration, lift, release, finite state, and a machine-readable result.

## Verified Humble ground-pick reference

On 2026-10-01 the current UTM source completed the accepted FEM ground-pick
sequence. MoveIt used the measured Newton object pose, executed the safe
four-stage pre-grasp route, descended, closed to `0.375145 rad`, lifted
`0.120 m` at velocity scale `0.030`, and reopened. The operator observed the
strip rise and release; MoveIt reported `ABSOLUTE POSITION PICK SUCCEEDED`.

The exact source copies, physics environment, run scripts, and measurements
are versioned in `docs/HUMBLE_FEM_PICK_SUCCESS.md`. Treat that record as the
Humble reconstruction source. Earlier scalar-friction failures remain useful
diagnostic history but are no longer the current boundary.

## Next validation

Reproduce this reference on the school computer under ROS 2 Jazzy without
changing the physics or grasp parameters. Then add automated per-run checks
for strip rise, bounded penetration, retention during lift, and release after
opening.

## Runtime lessons

- `ros_adapter` resets trajectory shadow to disabled when restarted. Call
  `/newton/sync_robot_state`, then enable `/newton/set_trajectory_shadow`.
- `newton_moveit_grasp_endpoint.py` must initialize the arm from `BASE_ARM`.
  Initializing all robot coordinates to zero lays the UR5 across the ground
  before the first ROS synchronization.
- A MoveIt success message proves planning/controller completion only. Ground
  grasp acceptance requires measured strip rise, retained bilateral contact,
  bounded penetration, and release after opening.

## Reconstruction sources

- `scripts/bootstrap_school_ubuntu.sh` reconstructs the Humble reference from
  tracked source.
- `docs/experiments/newton-ros2-bridge/prototype/` contains the bridge package
  and Newton endpoints copied by the bootstrap script.
- `docs/experiments/newton-robot-grasp/prototype/` contains the segmented
  contact model.
- `scripts/gripper_kinematics_calibration.py` and
  `scripts/gripper_width_solver.py` reproduce the gripper mapping.
- `config/gripper_kinematics_calibration.csv` is the versioned calibration
  table.
- `docs/experiments/newton-ros2-bridge/results/ground_grasp_diagnostic_2026-09-24.json`
  records the latest controlled evidence.

## Shared school computer boundary

Operate only inside directories owned by the project user. Inspect process and
port ownership before stopping anything. Never use broad process or delete
commands. At session end, update this file, preserve verified evidence, then
run `codex logout` and `codex login status`.
