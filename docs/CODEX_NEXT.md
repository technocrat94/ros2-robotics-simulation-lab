# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: presentation stills for Steps A, B, C (branch `main`)

Save PNGs (1280x720) to docs/experiments/rigid-obstacles/media/ and add them to its README
(file, what it shows, commit, source). Do not change any parameter.

- A: `A0_sphere_on_box.png`, `A1_fem_strip_on_box.png` — Newton viewer of the existing Step A drop
  scripts, final settled state (strip resting on the box).
- B: `B_disabled_route.png`, `B_enabled_route.png`, `B_tall_wall_refused.png` — RViz with the planned
  trajectory shown, same target (x=0.20), box visible; disabled crosses the box, enabled goes over,
  tall wall refused (terminal is fine for the last one).
- C: extract frames with ffmpeg from the existing 1x videos (no rerun):
  `C1_over_wall.png` (gripper above the wall), `C1_grasp.png` (fingers closed on cable),
  `C1_lift.png` (cable lifted), `C0_grasp.png`.

Commit, update docs/CODEX_STATUS.md (3 lines), push, reply with the hash.
