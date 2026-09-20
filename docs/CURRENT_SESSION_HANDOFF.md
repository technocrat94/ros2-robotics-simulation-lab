# Current Session Handoff

Updated: 2026-09-20

## Recovery rule

This file is durable project memory. Verify the remote branch and current
commit before trusting any recorded commit ID. Never depend on a previous chat
session as the only record of work.

## School computer

- User observed: `aisc216`
- Host OS: Ubuntu 24.04.4 LTS
- Architecture: `x86_64` / `amd64`
- Host ROS: Jazzy
- Docker is available and the user belongs to the `docker` group.
- The original verified project uses Ubuntu 22.04, ROS 2 Humble, and Newton
  1.5.1. Preserve that software contract in an isolated environment rather
  than replacing the host ROS installation.
- The account may be shared. Work only within `$HOME/ur5_ws`,
  `$HOME/newton_ws`, and `$HOME/.config/yuhao_robotics` and follow
  `docs/SCHOOL_COMPUTER_HANDOFF.md`.

## Verified project milestones

1. UR5 and Robotiq MoveIt fake-hardware task builds and executes.
2. The versioned ROS 2–Newton bridge returns state and reports stale data when
   the Newton endpoint stops.
3. The UR5/Robotiq URDF imports into Newton with the gripper mimic relationships
   preserved.
4. FEM and compliant-joint soft-strip representations were measured and
   compared. The segmented model was selected for fast robot integration while
   FEM remains the higher-fidelity reference.
5. A centred, pre-positioned Newton contact grasp lifted and retained the strip,
   including a 180-degree prescribed-motion stress test, and released it after
   the gripper opened.
6. A start-state guard detected the original 90-degree elbow and wrist-2
   mismatch. After synchronization, MoveIt shadow execution reproduced the
   complete task in Newton with a final maximum arm difference near
   `3.4e-8 rad`.
7. An absolute-object-position MoveIt node was built and verified with fake
   hardware. The combined Newton contact endpoint starts and synchronizes.

## Current boundary

The combined absolute-position MoveIt plus Newton contact grasp has not yet
passed visual and numerical acceptance. A successful MoveIt plan alone does
not prove that the flexible strip was lifted by valid contact.

## Two-computer development decision

The home UTM environment remains the canonical development and teaching
baseline. It already matches Ubuntu 22.04, ROS 2 Humble, MoveIt, and Newton
1.5.1, so integration logic should be completed and tested there with the
fast segmented strip.

The school RTX computer is the high-compute validation target. Do not rewrite
the project for the host's ROS 2 Jazzy installation at this stage. Reproduce
the pinned Ubuntu 22.04 / ROS 2 Humble contract in an isolated Docker
environment, check GPU passthrough, run the same repository commit, and then
replace only the strip physics backend with the higher-resolution FEM model.

The robot-side interface must remain unchanged across both backends:

- same `world` frame, SI units, object dimensions, initial pose, and ground;
- same MoveIt joint trajectory and gripper commands;
- same bridge protocol and returned timestamps;
- comparable measurements for lift height, slip/release time, penetration,
  deformation, finite state, and runtime.

The segmented model and FEM model are two numerical representations of the
same experiment. Higher-resolution FEM is not automatically "real rubber".
A physical claim additionally requires calibrated density, Young's modulus,
Poisson ratio, damping, friction, dimensions, and comparison with measured
hardware data.

## Immediate objectives

### Home baseline

Complete the combined absolute-position MoveIt plus segmented-strip contact
grasp. Accept it only when the strip lifts, remains between the fingers without
penetration, and drops after the gripper opens. Preserve the trajectory and
machine-readable metrics as the reference case.

### School GPU validation

Reproduce the pinned Humble/Newton environment safely without modifying the
host Jazzy installation. First prove that the same segmented reference case
runs from the same commit. Then enable the FEM backend and compare it against
the reference under identical commands. Change one model or material variable
at a time and record the evidence.

## Next session start

1. Read `docs/SCHOOL_CODEX_PROMPT.md` and
   `docs/SCHOOL_COMPUTER_HANDOFF.md`.
2. Confirm whether `aisc216` is a personal or shared Linux account and confirm
   the laboratory's Docker policy.
3. Run read-only checks for user, repository, branch, commit, working tree,
   ROS distribution, Docker, and owned project processes.
4. If the repository is not present, clone the GitHub repository into
   `$HOME/ur5_ws/src/ur5_moveit_demo` without touching existing unknown files.
5. Continue with the isolated Ubuntu 22.04 / ROS 2 Humble bootstrap only after
   the environment boundary is understood. Do not begin a Jazzy port.
6. Verify the segmented reference before enabling GPU FEM; this separates
   environment failures from physics-model differences.

## Required session close

Update this file and the relevant English and Traditional Chinese records,
run checks, commit and push verified work, then remind the owner to exit Codex
and run `codex logout` followed by `codex login status`. Never store or commit
authentication credentials.
