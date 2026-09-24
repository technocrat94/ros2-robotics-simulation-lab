# ROS 2 Jazzy Port Plan

Updated: 2026-09-24

## Decision

The home Ubuntu 22.04 / ROS 2 Humble system remains the reference until the
ground-grasp acceptance test passes. The school Ubuntu 24.04 `x86_64` system
will then receive a clean ROS 2 Jazzy port. After the port reproduces the
reference results, Jazzy becomes the primary platform for new development and
Humble remains a frozen comparison baseline.

## Source boundary

Transfer source code, URDF/Xacro, SRDF, configuration, launch files,
calibration data, documentation, and machine-readable results. Never transfer
`build/`, `install/`, `log/`, Python virtual environments, or architecture-
specific binaries from the home `aarch64` VM.

## Port sequence

1. Tag the verified Humble reference commit.
2. Create a separate `jazzy-port` branch and fresh Jazzy workspace.
3. Resolve dependencies with Jazzy packages and rebuild from source.
4. Verify UR5/Robotiq description and mimic relationships.
5. Verify ros2_control controller names, joint order, actions, and fake hardware.
6. Verify MoveIt planning and the absolute-position stages without Newton.
7. Recreate the Newton Python environment for `x86_64`; do not copy the ARM venv.
8. Verify bridge synchronization, stale-state detection, and trajectory shadow.
9. Run the same segmented-strip ground-grasp acceptance metrics.
10. Enable GPU/FEM work only after the segmented reference matches.

## Compatibility contract

The Humble and Jazzy runs must use the same world frame, SI units, object
dimensions, initial pose, ground height, trajectory, gripper command, bridge
protocol, and acceptance thresholds. A successful launch or MoveIt result is
not enough; compare measured lift, retained contact, penetration, release,
finite state, and runtime.

## Shared-laboratory boundary

On the school computer, operate only inside directories owned by the project
user. Inspect process and port ownership before stopping anything. Keep ROS
domain and Newton ports scoped per user, and log out of Codex at the end of
each shared-computer session.
