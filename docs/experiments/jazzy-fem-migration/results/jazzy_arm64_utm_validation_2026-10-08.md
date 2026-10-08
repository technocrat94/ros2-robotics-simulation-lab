# Jazzy ARM64 UTM FEM validation — 2026-10-08

The new Ubuntu 24.04 ARM64 UTM environment successfully built and ran the ROS 2 Jazzy, MoveIt, ROS bridge, Newton, UR5, Robotiq, and FEM ground-pick pipeline from `origin/main@a467c5f3fbaea2a89ba642678726aaf2d78daea8`.

## Acceptance evidence

- ROS distribution: Jazzy; architecture: `aarch64`; Newton device: CPU.
- Workspace build succeeded for all four local packages.
- The generated Newton URDF resolved and verified 34 mesh paths.
- Bridge status was `OK` and the start-state guard passed with a maximum joint error of `1.94e-7 rad`.
- The safe pre-grasp stages (`raise`, `orient`, `translate`, `descend`) each planned 100%.
- Cartesian approach and lift each completed 100%.
- MoveIt reported `ABSOLUTE POSITION PICK SUCCEEDED` without the old named-start detour.
- Newton recorded 368 FEM contact samples. The grasp region rose from `0.010981 m` to `0.128460 m`, a measured lift of `0.117479 m`.
- Both fingers contacted the FEM strip in 256 samples. Peak soft-contact counts were 33 on the left and 37 on the right.
- The CPU endpoint accumulated a maximum command backlog of 411, then drained it completely. This proves correctness on ARM CPU, while also showing that it runs slower than real time.

## Engineering interpretation

MoveIt success alone only proves that the commanded robot trajectory completed. The Newton measurements add the missing physical evidence: the deformable grasp region rose about 117.5 mm while bilateral soft contacts remained present. Together these results verify the complete control-to-physics pipeline.

The large command backlog is a performance observation, not a failed grasp. On this CPU-only VM, ROS can finish sending commands before Newton finishes simulating them. Validation must therefore wait for `MOVEIT_COMMAND_QUEUE_DRAINED` before judging the result.

## Portability fixes under review

Two small source changes were required for the local ARM VM:

1. The Jazzy bootstrap now rejects stale `/opt/ros/humble/` paths without rejecting valid `/home/<user>/...` mesh paths.
2. The Jazzy Newton runner selects `cuda:0` when an NVIDIA GPU is available and falls back to `cpu` otherwise.

These changes preserve the school RTX 3080 path while allowing the Mac UTM environment to run the same project on CPU.

## Logs

- `logs/jazzy_fem_preview_20261008_165515.log`
- `logs/jazzy_fem_pick_20261008_165538.log`
- `logs/jazzy_fem_endpoint_20261008_165353.log`

The complete numeric record is in `jazzy_arm64_utm_validation_2026-10-08.json`.
