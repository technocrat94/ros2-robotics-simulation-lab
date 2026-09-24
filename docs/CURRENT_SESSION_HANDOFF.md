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

## Current unverified boundary

The dynamic ground pickup has not passed. A representative manual close at
about `0.394 rad` reported left/right loaded contacts `25/22` and force-
magnitude sums `231.335/250.229`, but a 30 mm isolated lift left the strip on
the ground immediately.

Friction trials `1.5`, `3.0`, and `10.0`, and nominal compression trials
`2 mm` and `4 mm`, did not produce retained lift. These results rule out
continuing to tune only scalar friction or nominal compression. The scene was
also no longer clean: object center `x` had moved from about `0.487 m` to
`0.687 m` after accumulated tests.

## Single next experiment

Restart the Newton endpoint to restore the clean object pose. Extend contact
instrumentation to record per-finger absolute world-frame `|Fx|`, `|Fy|`, and
`|Fz|` during closure and the first lift frames. This will distinguish
longitudinal pushing, opposing lateral pinch, and upward friction. Do not
increase friction or compression again before collecting that evidence.

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
