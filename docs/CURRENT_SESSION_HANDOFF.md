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


## 2026-11-12 MPC prototype

Source parent: `15c5a543c0c7ff26233b619c0f05b57e08974c92`, branch
`experiment/newton-link-depth-camera`. Same school Ubuntu 24.04.4/Jazzy
installation and RTX 3080; no ROS or Newton physics was started.
Local cuRobo checkout: `78fd485fa82d9b9a063fb4985e371814587e666a`.
Resolve the implementation commit with
`git log -1 --format=%H -- scripts/curobo_moving_rod_mpc.py` after syncing.

Added independent MPC prototype, isolated-port wrapper, CPU contract tests,
`moving_rod_mpc.json` / `.log` and viewer verification record. Updated the
English comparison and Chinese learning note. Accepted v2 source/worker,
original and v2 artifacts, shared configs and FEM grasp path are unchanged.
Existing dirty result files, deleted wall image and untracked cuRobo checkout
remain untouched and unstaged.

Read-only survey found ModelPredictiveControl/MPCSolver. Collision-start setup
returns finite commands with infeasible status; no recovery guarantee. Six B
runs completed 20 simulated seconds at about 40 Hz, with and without v2-style
prediction. All had zero collisions but zero task reaches, so none passed the
full task and camera E was conditionally skipped. Continuous old-command
execution was numerically checked in all 4800 compute intervals. Near-rod
robot/relative clearance rates are recorded. Default Cartesian-only tracking
and an ideal velocity integrator are not proven equivalent to a hardware plant.

Failed diagnostic: selecting MPC command-window index zero reissued the current
state boundary and caused essentially no motion. A source/output inspection
identified this; the final adapter selects the next non-stale velocity, with
latency indexing. A CPU regression protects this boundary. No cuRobo changes.
The fallback deceleration branch is disabled by `and False` in the local library.

Tests: four CPU contract tests, project CI, six GPU B cases, sampled-clock
integration audit, two 0.10 m/s viewer playback callbacks, and evidence-hash
checks around read-only replay. Port 30000 was occupied by an existing Python
PID 1244750; it was not stopped. Personal override 30001 was checked free and
used for the viewer. All probe, experiment, renderer and viewer processes
started by this task are stopped at completion.

Single next diagnostic: obstacle-free MPC goal reaching with the same executor,
before attributing zero collisions to better avoidance or tuning the controller.
Remaining limits include incomplete task progress, ideal velocity commands,
zero acceleration feedback, local goal tracking, linear rod prediction, and
sphere/cylinder scoring. Do not call this an accepted shuttle controller.

Read-only resume:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && NEWTON_VIEWER_PORT=30001 scripts/run_sim_camera_moving_rod_mpc.sh --replay
```

Exit Codex, run `codex logout` and `codex login status`, and confirm logged out.


## 2026-11-12 MPC tuned configuration

Source parent: `ad2bf29bd4afba1e6b26c3b3e6c237b318966806`, branch `experiment/newton-link-depth-camera`.
Same school Ubuntu 24.04.4 / Jazzy installation, user aisc216 and RTX 3080;
this task started no ROS or Newton physics. Local cuRobo is unchanged.
Resolve the implementation commit with
`git log -1 --format=%H -- scripts/curobo_moving_rod_mpc_tuned.py` after syncing.

No-rod reference: v2 MotionPlanner enters 6 mm in 0.875 s at its own trajectory
timing (planning excluded), original MPC in 11.8 s. The new single named block
`NO_ROD_SELECTED_MPC` changes optimization_dt from 0.100 to 0.020 s and the
non-terminal pose factor from 0.001 to 0.3, keeping acceleration regularization
10000 and all limits unchanged. It reaches in 1.125 s, below the 1.75 s gate,
with peak commanded speed 0.616654 rad/s and peak update-sampled acceleration
3.596537 rad/s², no velocity clips or position clamps. The ideal velocity plant
still has instantaneous switching and no finite continuous acceleration peak.

All tuning was no-rod only; failed single-parameter candidates, including
infeasible high pose factors, are retained. The successful two-setting
interaction supports a cost/time-scale diagnosis rather than a GPU or speed-cap
explanation; it does not identify a unique causal scalar. Reference SHA-256 is
embedded in the rod result and must match the frozen source block before runs.

Frozen rod results: current box/prediction at 0.05 each reach 7 with no collisions.
At 0.10 they reach 3 with 2/1 collisions. At 0.20 they reach 3/6 with 3/0
collisions. Prediction's fastest tested survivor is 0.20, but it failed 0.10:
never claim a monotonic safe-speed interval. Since the explicit B-at-0.10 gate
failed, no E cases ran. E per-frame height/width undercoverage and center-offset
logging is implemented and CPU-tested; no measured worst frames exist yet.

Added new tuned script, isolated-port wrapper, four CPU tests, reference and
rod evidence plus viewer hash proof; appended English and Chinese records.
Accepted v2/original MPC scripts, workers, evidence, library code, shared configs
and FEM grasp path are unchanged. Existing dirty result files, deleted wall
image and untracked cuRobo checkout remain preserved and excluded from commit.

Tests: four CPU contracts, project CI, six 20 s GPU B cases, old-command
integration audit, frozen reference-hash check and two 0.10 m/s viewer callbacks.
Read-only replay preserved old/new evidence hashes. Port 30000 was free and
used; the existing 30001 listener PID 1248594 was left untouched. Its renderer
child PID 1248601 (PPID 1248594, started 18:18:34 before this task) was also
preserved. All reference, rod, renderer and viewer processes started by this
task are stopped at completion.

Next single diagnostic: inspect infeasible-command execution and true collision
on the frozen 0.10 m/s traces before proposing a new controller change.
The remaining boundary is robust avoidance at normal task speed; camera E is
conditional and unmeasured. Do not alter the frozen block using rod outcomes.

Read-only resume:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && scripts/run_sim_camera_moving_rod_mpc_tuned.sh --replay
```

Exit Codex, run `codex logout` and `codex login status`, confirm logged out.


## 2026-11-12 fixed-phase report and interactive groups

Source parent `8f7e4f2c02629ab5ed0d047c0b4e557b8331be18`, branch
`experiment/newton-link-depth-camera`. Same school Ubuntu 24.04.4/Jazzy,
user aisc216 and RTX 3080; no ROS, physics, install or hardware operation.
Resolve the implementation commit with
`git log -1 --format=%H -- scripts/curobo_moving_rod_rates.py` after syncing.

Completed 30 cases: fixed initial X=0.36/0.44/0.52/0.60/0.68, decreasing X,
0.05/0.10 m/s, 20 simulated seconds, A/B/E with the frozen tuned MPC.
A has no obstacle input and is scored against truth. B/E use prediction; E uses
the unchanged v2 memory. The prior 0.10 B gate was explicitly lifted for this
report, so all ten E cases ran. No tuning or phase substitution occurred.
AST audit confirms batch control/perception/clock identical to the tuned
controller except A's obstacle-disable branch.

Valid-phase success rates: 0.05 B=5/5, E=3/5; 0.10 B=4/4, E=1/4.
The 0.10, X=0.36 condition is INVALID_SCENE because A did not collide;
all its A/B/E data remain but are excluded from aggregate rates and extrema.
A rates are 0/5 and 0/4 by the scene-validity definition. Full mean reaches,
episodes, min clearance, joint speeds/accelerations and per-phase table are in
`moving_rod_rates.json`/`.log` and the English record. All 19776 step clocks
were audited; E has one geometry-error record per capture. Valid E frames
include 946 missing boxes out of 3401; worst-three ranking is explicit.
One-page senior report: `docs/SIM_CAMERA_MOVING_ROD_REPORT.zh-TW.md`.

Steps 1–6 were saved and the speed/phase/case replay verified before live work.
Then added tuned-script `--interactive` dispatch and an independent lifecycle
manager: planar drag, same frozen controller, clean stop/join/worker exit,
raised-start reset, start new group. Renderer protocol adds optional Y only;
old default-Y behavior is regression-tested. Frozen settings, tuned factory
and run_case, v2/original MPC source, old results, library/shared configs/FEM
path are unchanged. Existing dirty results, deleted wall image and untracked
cuRobo remain untouched and excluded from commit.

Live smoke took 14.160289 s, using actual programmatic GUI callbacks
A→E→B→A, three switches. Maximum owned GPU loops=1; all four reset errors=0.
E renderer PID 1255421 exited with code 0 before B started.
All owned threads/workers stopped; the old user viewer PID 1251897 and renderer
1251905 on port 30000 were preserved. New viewers used personal override 30001.
Live mode never writes evidence; all 11 old/new result hashes were unchanged
through replay and smoke. Live rates include UI/capture costs and wall-clock
plant motion; short smoke rates are not a long-duration performance benchmark.

Tests: CPU denominator/invalid-phase/stall contracts; five live lifecycle,
geometry, Y-protocol and write-guard tests; existing four tuned contracts;
project CI, 30-case GPU batch, 19776 interval audit, three replay callbacks,
14.16 s live smoke and evidence-hash checks. Verification files:
`moving_rod_rates.viewer.log` and `moving_rod_rates.live.log`.

Limits: ideal velocity plant without an acceleration limiter, noise-free camera,
one rod/one AABB, simulator shape-index masking, forward/downward view,
finite infeasible MPC commands and measured-timing variability. Acceleration
numbers are update-sampled Δv/Δt, not bounded continuous physical peaks.
Next single step: present the one-page descriptive rates and inspect E's
per-frame missing/partial boxes alongside collision timing before proposing
another experiment; do not tune against these results silently.

Read-only phase replay:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && NEWTON_VIEWER_PORT=30001 scripts/run_sim_camera_moving_rod_rates.sh --replay
```

Interactive live demo (no writes):

```bash
cd ~/ur5_ws/src/ur5_moveit_demo && NEWTON_VIEWER_PORT=30001 scripts/run_sim_camera_moving_rod_mpc_tuned.sh --interactive
```

Exit Codex, run `codex logout` and `codex login status`, and confirm logged out.

## 2026-11-13 — fixed-latency perception split

Pulled reviewer task at `a2eab3a` on `experiment/newton-link-depth-camera`.
Implemented imported rates-loop reuse with optional `counted_latency_s`; default
measured-clock behavior remains identical. Shared viewer accepts optional group
labels while default A/B/E behavior remains unchanged. No tuned controller,
renderer, library, frozen configuration or old saved evidence was changed.

Ran 50 cases: speeds .05/.10, phases .36/.44/.52/.60/.68, groups
A20/B20/B50/E20/E50, each 20 simulated seconds, plus B20/E20 repeats at .10 X=.60.
No interactive demo ran during measurement. Fixed clock affects advance, index,
expected compute, counted compute and remainder pacing; measured time is retained.
20 ms gives a 25 ms step; 50 ms gives 50 ms, with horizons 45/75 ms and capped
command index 3. Real compute can pause the simulated world.

A20 marks only .10 X=.36 invalid. Valid-only success counts (slow, fast):
A20 0/5,0/4; B20 4/5,3/4; B50 5/5,4/4; E20 2/5,2/4; E50 4/5,2/4.
The comparison is non-monotonic in latency and includes cadence/horizon effects.
E50 does not quantitatively reproduce old E (3/5,1/4). Repeats are not in rates.
B20 repeat stays successful but clearance changes +8.7509→+6.6620 mm;
E20 changes success→failure, +15.1808→−66.1993 mm. Fixed latency does not make
this workflow fully repeatable; remaining variation is not yet isolated. Do not
claim causal fractions, all-group repeatability or hardware safety from this run.

Saved new `moving_rod_latency.{json,log}` with all cases, repeats and per-phase
status. Full 33,600-step audit passes fixed time/horizon/index, previous-velocity
integration, 20 s duration, camera-frame counts and no joint-position clamping.
Four latency CPU/evidence tests, original rates test, four tuned contracts and
five live lifecycle contracts pass. AST comparison verifies that removing only
the optional-latency additions reconstructs the prior rates run_case exactly.
Traditional Chinese report and English "Latency split" section contain the table,
real measured median/p95 (pooled valid steps), repeat differences and limitations.

Run a new explicit measurement only when needed:
`scripts/run_sim_camera_moving_rod_latency.sh --headless --save-results`
Read-only five-group replay:
`NEWTON_VIEWER_PORT=30001 scripts/run_sim_camera_moving_rod_latency.sh --replay`

Preserve the unrelated deleted wall image, modified old second/third/fourth/sixth
pass results and untracked curobo checkout. No global process cleanup or library
modification. Next step is to inspect/reset the remaining solver/perception state
and establish repeatability before making stronger causal claims or tuning.
Exit Codex, run `codex logout`, then `codex login status`, and confirm logged out.

Final verification: all 50 frame sets rendered after measurement; five group
callbacks passed on non-main threads, with moving robot/rod and 98 depth
frames each. Viewer port 30001 and owned camera worker closed. Replay writes
no results; all prior result hashes and the new JSON/log hashes verified intact.
Project CI passes. Evidence includes moving_rod_latency.viewer.log and
moving_rod_latency.validation.log. No owned processes remain running.

## 2026-11-13 — v3 fixed-rule MPC rod dodge

Task source parent `dcb62f8` on `experiment/newton-link-depth-camera`.
All old controller/renderer/library code and old evidence were preserved.
New v3 entry point imports the original MPC factory, joint-state/plant helpers,
scene, perception, worker and renderer. New viewer/live modules retain the
existing primitives and serialized lifecycle, with v3-specific scheduling.
No ROS, planner, Newton physics, install or shared process cleanup was used.

Rules frozen before the rod batch: config joint speed limits ±3.141592741 rad/s;
config acceleration limits ±10 rad/s² (YAML cspace max_acceleration=10 and scale=1,
not a 3.0 fallback); hard command delta ≤0.25 rad/s every 25 ms, with the same
robot config passed into MPC. Plant remains an ideal held-velocity discrete plant;
feedback acceleration between command updates is zero, command Δv/25 ms is audited.
Control/counting period 25 ms, exact n/30-second B/E observations using the plant
pose at that timestamp, latest fit and velocity held between observations.
MPC horizon from actual config (81−1)*.02/4=.4 s; single swept-box extension over
.4+1/30=.433333 s (no legacy doubling). Real compute remains logged and can pause
the simulated world. Full no-rod reference passes in 1.075 s; the existing frozen
MPC block was retained, no rod-based tuning. Reference includes robot-config SHA.

Original 150-run task completed before remote revision 569433e; final primary set is 110 cases: 2 speeds ×5 phases ×(A once + B/E five repeats), each 20 simulated
seconds/800 control steps. Fresh MPC instance and robot/point/velocity state per
case. No interactive demo during measurement. No phases are INVALID_SCENE under
the single-A validity rule; 5 A and 25 B/E runs per speed enter the table. Success requires
zero collisions and at least one reach. B is 25/25 at each speed; E is 5/25 at each;
E is not close to B in this batch. No failure-cause attribution is made.

B speed=0.05: success=25/25, collision_runs=0, worst/median_worst_mm=17.68/27.29, max_v=2.679810, max_delta_v_dt=10.000000.
E speed=0.05: success=5/25, collision_runs=20, worst/median_worst_mm=-86.81/-19.20, max_v=3.073118, max_delta_v_dt=10.000000.
B speed=0.10: success=25/25, collision_runs=0, worst/median_worst_mm=17.83/37.71, max_v=2.789972, max_delta_v_dt=10.000000.
E speed=0.10: success=5/25, collision_runs=20, worst/median_worst_mm=-26.22/-18.25, max_v=1.972038, max_delta_v_dt=10.000000.

All original 120,000 steps pass (88,000 primary +32,000 supplemental A) fixed-clock/horizon/index, exact observation timestamps,
held-frame age, speed/command-acceleration bounds, previous-velocity integration,
no joint-position clamps and five-repeat validity checks. JSON retains all
control-boundary states, commands, clocks, boxes, observations and all repeats;
dense collision scoring is unchanged. Redundant fit metadata is omitted; numeric
serialization uses eight decimal places. File is below GitHub's 100 MiB limit.
Seven v3 tests (including all saved data and occupied-port preflight), five existing live lifecycle tests, four
tuned tests and one rates test pass; project CI passes.

New evidence prefix: docs/experiments/sim-camera-curobo/results/moving_rod_v3.
Reference: moving_rod_v3.reference.json. English "v3 fixed rules" and the Chinese
report list all rule sources, full six-row table, limits and both watch commands.
Use only the explicit new prefix for future measurement; never overwrite old results.

Replay:
`NEWTON_VIEWER_PORT=30002 scripts/run_sim_camera_moving_rod_v3.sh --replay`
Interactive:
`NEWTON_VIEWER_PORT=30002 scripts/run_sim_camera_moving_rod_v3.sh --interactive`
Both from ~/ur5_ws/src/ur5_moveit_demo. Live mode uses fixed simulated steps,
so UI shows separate simulation/wall time; user drag is an external input.

Leave unrelated old second/third/fourth/sixth pass JSON/log modifications, the
deleted wall image and untracked curobo checkout untouched. Stage only explicit
paths. Next single step: present the fixed-rule table and let the user watch the
saved repeats or interactive v3; do not add cause analysis or tune against the rod.
Exit Codex, run codex logout, then codex login status, and confirm logged out.

Final watch verification: all 150 frame sets rendered, then actual non-main
playback callbacks for A repeat 1, B repeat 3, E repeat 5 at .10 X=.60 passed,
100 depth frames each. Initial requested port 30001 was occupied by the user's
latency replay PID 1314241, so the guard correctly refused alternate binding;
that process was preserved. Replayed successfully on explicitly selected 30002.
A preflight check now detects occupied ports before expensive frame rendering.

Interactive A→E→B→A smoke passed in 16.775633 s with three switches, maximum one
owned GPU loop, zero reset error, speed/command-acceleration and exact observation
count checks passed. E camera PID 1323255 exited code 0 before B began. All owned
threads/workers/viewers stopped; existing user latency replay remains untouched.
Programmatic GUI callback/gizmo smoke, not manual browser acceptance. See
moving_rod_v3.viewer.log, moving_rod_v3.live.log and moving_rod_v3.validation.log.
No watch mode wrote results; old and new evidence hashes verified unchanged.

Merged remote 569433e without touching unrelated work. Keep 110 primary cases and 40 extra A runs under extra_A_runs (excluded from all main metrics). Future build loop runs A only on repeat 1, B/E on repeats 1..5; one short completion line per case and final table. Viewer forces repeat 1 for A and disables that repeat selector. Live gate requires 110 primary cases. No B/E result, frozen controller, physical rule or measured trace changed.

Final revised-schema verification: A-only-repeat-1 replay callbacks pass; interactive 110-case gate and lifecycle smoke pass in 17.016493 s. All owned processes exited. Prior evidence unchanged; revised JSON/log/reference hashes unchanged through both checks.
