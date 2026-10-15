# Next task for Codex

Written by Claude (reviewer). Read this file at the start of a session after `git pull`.
When the task below is done, replace this file's task section with "No pending task" in the same commit
as your evidence.

## Task: record presentation videos (branch `experiment/step-c-cable-wall`)

Step C is accepted (C0, C1, C2). The user presents tomorrow. Record these clips with the Newton
viewer (Viser) using the same fixed grasp camera for every clip; run each exactly as the accepted
evidence (same commit, profiles, GPU). Run in the `stepc` tmux session so the user can watch.

1. `01_fem_cable.mp4` — the FEM cable alone lying on the floor (deformable model, ~10 s).
2. `02_cable_grasp_default.mp4` — default cable position, no wall: approach, close, lift, release.
3. `03_c0_moved_cable.mp4` — C0: cable at y=0.410, no wall: full pick.
4. `04_c1_wall_crossing.mp4` — C1 (main clip): arm raises over the 0.16 m wall, descends, grasps,
   lifts, releases; the wall must be visible.
5. `05_c2_tall_wall_refused.png` (or short mp4) — C2: RViz/terminal showing the plan is refused
   (safe path 4.0%, no robot command).

Recording: ffmpeg x11grab (or OBS) of the browser viewer, 1280x720, H.264. The simulation runs
slower than real time: also export a sped-up copy and write the speed factor in the caption.
Keep each file < 10 MB. Save to docs/experiments/rigid-obstacles/media/ with a README listing
each clip: file, what it shows, commit, device, speed factor, and the key numbers
(lift, release drop, route height, loaded wall contacts).

Do not change any parameter. Update docs/CODEX_STATUS.md (5 lines), push, reply with the hash.
