# Step C presentation media

These presentation captures were recorded from commit
`61bc7a45be35ea3cdb5532afb164ec587f99368d` in the `stepc` tmux session.  Every
Newton run used the unchanged school-GPU profile (`cuda:0`, NVIDIA GeForce RTX
3080, `80 x 2 x 2` FEM cells) and the fixed grasp camera at
`(1.15, -1.35, 0.90)`, pitch `-22.2 deg`, yaw `118.6 deg`.  The MP4 files are
1280x720 H.264/yuv420p.  No geometry, padding, physics, grasp, or acceptance
parameter was changed.

The named clips are the original wall-clock captures.  The `_2x` and `_6x`
files are presentation copies made only by changing video timestamps; they do
not rerun or alter the simulation.

| File | Caption / what it shows | Device | Speed | Duration | Key numbers |
|---|---|---|---:|---:|---|
| [`01_fem_cable.mp4`](01_fem_cable.mp4) | Deformable FEM cable resting on the ground in the idle default scene; no ROS motion command | `cuda:0` | 1x | 10.0 s | 729 particles, 1600 tetrahedra |
| [`01_fem_cable_2x.mp4`](01_fem_cable_2x.mp4) | Sped-up copy of the idle FEM cable clip | `cuda:0` | 2x | 5.1 s | Same unchanged scene |
| [`02_cable_grasp_default.mp4`](02_cable_grasp_default.mp4) | Default position, no wall: approach, close, lift, and release | `cuda:0` | 1x | 149.9 s | lift `0.119899 m`; release drop `0.120357 m`; route height `0.350758 m`; wall contacts n/a |
| [`02_cable_grasp_default_6x.mp4`](02_cable_grasp_default_6x.mp4) | Sped-up default grasp | `cuda:0` | 6x | 25.1 s | Same physical run and measurements |
| [`03_c0_moved_cable.mp4`](03_c0_moved_cable.mp4) | C0, cable at `y=0.410 m`, no wall: complete pick | `cuda:0` | 1x | 171.3 s | lift `0.119951 m`; release drop `0.120466 m`; route height `0.351039 m`; wall contacts n/a |
| [`03_c0_moved_cable_6x.mp4`](03_c0_moved_cable_6x.mp4) | Sped-up C0 control | `cuda:0` | 6x | 28.7 s | Same physical run and measurements |
| [`04_c1_wall_crossing.mp4`](04_c1_wall_crossing.mp4) | C1 main clip: raise over the visible `0.16 m` wall, descend, grasp, lift, and release | `cuda:0` | 1x | 197.7 s | lift `0.119893 m`; release drop `0.120327 m`; route height `0.375040 m`; loaded wall contacts `0` |
| [`04_c1_wall_crossing_6x.mp4`](04_c1_wall_crossing_6x.mp4) | Sped-up C1 main clip | `cuda:0` | 6x | 33.1 s | raw wall candidates `82`; loaded and penetrating wall contacts `0` |
| [`05_c2_tall_wall_refused.png`](05_c2_tall_wall_refused.png) | C2 terminal evidence: tall-wall safe path reached only `4.0%`, so planning was refused | plan-only; no Newton device used | n/a | still | `planning_failed=true`; `robot_command_sent=false`; pass `true` |

All nine files are below 10 MB.  C0, C1, and C2 passed their existing
acceptance checks.  The accelerated copies are the recommended presentation
versions; retain the 1x files when the slower wall-clock behavior itself needs
to be explained.
