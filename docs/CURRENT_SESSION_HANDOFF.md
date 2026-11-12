# Current Session Handoff

Updated: 2026-11-12

## Recovery rule

This file is durable project memory. Verify the remote branch, exact commit,
working tree, machine, ROS distribution, and active processes before changing
anything. Never depend on chat history as the only record.

## Source of truth and platform decision

`main` on GitHub is the single source of truth. Every machine syncs from it
before work starts; experimental work happens on short-lived branches cut from
`main` and merged back after review.

- Primary platform: Ubuntu 24.04 `x86_64`, ROS 2 Jazzy (school computer, RTX
  3080). Newton defaults to CPU; `GRASP_DEVICE=cuda:0` selects the GPU.
- Frozen baseline: Ubuntu 22.04 `aarch64` in UTM, ROS 2 Humble, Newton 1.5.1
  on CPU. Tag `humble-fem-baseline` (`3546672`). Latest `main` no longer
  targets Humble; check out the tag to rebuild it.
- Research baseline release: `v0.2.0-fem-grasp`. Earlier tag
  `jazzy-fem-grasp-v1` (`7f4d715`) marks the Jazzy acceptance merge.

Do not copy `build`, `install`, `log`, ARM binaries, or Python virtual
environments between machines.

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

## Verified Jazzy ground pick (primary)

On 2026-10-02 native Jazzy reproduced the pick with unchanged physics and
grasp parameters and printed a complete `MOVEIT_GRASP_RESULT`:
`candidate_contact_grasp_pass=true`, lift `0.117 m`, release drop `0.112 m`,
minimum strip bottom `-0.8 mm`, bilateral soft contact in 34 closure and 229
lift samples, real-time factor `0.102` on `cuda:0`. Evidence:
`docs/experiments/jazzy-fem-migration/results/` and `docs/jazzy/STATUS.md`.

## Next direction

The research baseline is frozen. New work should not change the verified grasp
path without a new acceptance run. Candidate next steps, still to be
confirmed with the lab:

1. Cable model: convert the segmented strip into a thin multi-axis cable and
   validate sag, bending, and contact against cylindrical pegs.
2. Scripted single-peg cable routing through the existing ROS–Newton pipeline.
3. Headless, ROS-free, GPU-parallel copies of the scene for robot learning;
   ROS and MoveIt remain the deployment and demonstration path.

## Active cable experiment (not yet accepted)

Branch `experiment/fem-cable` contains the rod-cable experiment.
The 40-segment, 6 mm rod candidate passed its 30 s CUDA cantilever check with
the original damping (`candidate_rod_cable_pass=true`). A separate
`GRASP_OBJECT_MODEL=rod_cable` path, small-gap gripper calibration, CUDA-only
endpoint guard, plan-only script, execution script, and bilateral rigid-contact
acceptance have been implemented without changing the accepted FEM mode.

Static scene construction reproduces 40 cable bodies, 40 shapes, total mass
`0.0124407075 kg`, center height `0.004 m`, and initial surface height
`0.001 m`. The 6 mm pad-gap solution is `0.748815 rad` with `13.679 mm`
approach compensation. CI, shell/Python syntax checks, runtime synchronization,
and the Jazzy build pass.

The first integrated CUDA attempt on 2026-10-08 is **not accepted**. Preview,
safe pre-grasp, approach, and lift paths reached 100%, and MoveIt printed
`ABSOLUTE POSITION PICK SUCCEEDED`, but the operator saw that the cable was not
retained. Endpoint samples show 71 bilateral loaded-contact samples and only
about `9.56 mm` maximum grasp-region rise before contact disappeared during the
commanded 120 mm lift. The endpoint did not emit a final result JSON. Evidence
is in `docs/experiments/newton-cable-grasp/results/`.

Next session should reproduce a close-only/early-lift diagnostic and compare
finger-pad contact geometry with vertical grasp height before changing any
friction, damping, compression, or cable stiffness. Do not call the cable grasp
accepted until `MOVEIT_GRASP_RESULT` provides measured lift, release,
bilateral loaded contact, finite state, and
`candidate_contact_grasp_pass=true`.

## Continuous integration

`.github/workflows/ci.yml` runs `scripts/ci_checks.py` without GPU, ROS, or
Newton. It guards syntax, configuration parsing, license consistency, and the
gripper-width solver result used by the verified pick.

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

- `scripts/bootstrap_school_jazzy.sh` rebuilds the primary Jazzy workspace;
  `docs/jazzy/JAZZY_REPRODUCIBILITY.md` is the runbook.
- `scripts/bootstrap_school_ubuntu.sh` reconstructs the Humble reference from
  the `humble-fem-baseline` tag.
- `docs/experiments/newton-ros2-bridge/prototype/` contains the bridge package
  and Newton endpoints copied by the bootstrap script.
- `docs/experiments/newton-robot-grasp/prototype/` contains the segmented
  segmented and FEM strip scene builder.
- `scripts/gripper_kinematics_calibration.py` and
  `scripts/gripper_width_solver.py` reproduce the gripper mapping.
- `config/gripper_kinematics_calibration.csv` is the versioned calibration
  table.
- `docs/experiments/jazzy-fem-migration/results/jazzy_fem_pick_result_2026-10-02.json`
  records the latest accepted machine-readable result.

## Shared school computer boundary

Operate only inside directories owned by the project user. Inspect process and
port ownership before stopping anything. Never use broad process or delete
commands. At session end, update this file, preserve verified evidence, then
run `codex logout` and `codex login status`.


## 2026-11-12 moving rod second run

Machine: school Ubuntu 24.04.4 x86_64, user `aisc216`, installed ROS Jazzy;
this task is ROS-free CUDA cuRobo planning plus Newton camera rendering,
without Newton physics. Branch: `experiment/newton-link-depth-camera`.
Exact source parent after fetch/switch/pull:
`9cc276ccd2fcf69757b84ceb8e624ba9df57f6c5`. Resolve the delivered implementation commit with
`git log -1 --format=%H -- scripts/curobo_camera_moving_rod.py` after syncing.
The initial worktree was dirty; all unrelated result modifications, deleted
wall image, and untracked `curobo/` have been preserved and excluded from commit.

The final lane is Y=0.245 m, X=0.30–0.70 m, top=0.250 m, radius=0.025 m.
Regular initial X=0.60 m moves toward decreasing X; blind entry starts at
0.30 m moving toward increasing X. Endpoint sweep lower bounds are
104.233/51.458 mm; the renderer confirms zero initial blind-entry rod pixels.
All A speeds collide in the saved final run. Prediction is shared by B/E;
retreat tries raised start then 0.10 m up. Viewer adds a blue prediction box,
ESCAPE status, and larger magenta points. Use free per-user port 30000;
the occupied 8084 listener was left untouched.

Fastest surviving tested speeds: B=0.05, E=None m/s (None means none survived). These are single measured-timing observations, not guaranteed speed limits.

Evidence: `docs/experiments/sim-camera-curobo/results/moving_rod_v2.json`
and `.log`; rejected timed configurations are in
`moving_rod_v2_scene_search.json`. Original `moving_rod.*` is unchanged.
Geometric crossing alone did not ensure timed A collisions: two initial
configurations were rejected. Corrected a Viser-to-sensor pi-X frame mismatch;
the actual renderer caught the invalid blind-entry check. Historical evidence
is documented in the Second run section of `docs/SIM_CAMERA_MOVING_ROD.md`.

Changed files: moving-rod planner/viewer and renderer worker, new CPU contract
test, v2 result files, English second-run record, Traditional Chinese note,
and this handoff. Tests: Python/shell/config CI passed; three CPU tests passed
(prediction containment, reflected clock, viewer save guard and cleanup);
full 11-case GPU matrix completed 20 simulated seconds per case;
three non-main-thread viewer playback callbacks passed. Started renderer,
planner, viewer, and calibration processes are stopped at task completion.

Remaining limits: velocity estimation from partial boxes, unpredicted rod
reflections, variable compute latency, and escape plans rejected once the
current state lies inside an inflated box. This is kinematic sphere/cylinder
scoring, not contact physics or hardware safety validation. Next single step:
review the B/E per-cycle divergence and hold-phase collisions before changing
another controller parameter. Do not alter the accepted physical grasp path.

Resume/view command (does not save tracked evidence):

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && source scripts/lab_session_env.sh && scripts/run_sim_camera_moving_rod.sh --port "$NEWTON_VIEWER_PORT"
```

Only `--save-results` rewrites `moving_rod_v2.*`. On leaving Codex, run
`codex logout`, then `codex login status`, and confirm no account is logged in.
