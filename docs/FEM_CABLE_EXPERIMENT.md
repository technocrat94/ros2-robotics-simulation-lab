# FEM Cable Experiment

Status: experimental, not yet integrated with the accepted FEM ground-pick baseline.

This experiment converts the `0.40 × 0.05 × 0.02 m` FEM strip into a
`0.40 × 0.006 × 0.006 m` square-section cable approximation. The first goal is
not robot grasping. It is to establish correct mesh topology, mass, CUDA
placement, settling behavior, and length preservation before adding floor and
gripper contact to the ROS–Newton workflow.

## Representation

The structured grid uses `40 × 2 × 2 = 160` volumetric cells. Grid vertices
are shared, so it contains `(40+1)(2+1)(2+1) = 369` particles. Newton splits
each cell into five tetrahedra, giving 800 tetrahedra. `fix_left=True` fixes
one `3 × 3` particle face, leaving 360 dynamic particles.

## Newton 1.5.1 nodal-mass correction

For this `add_soft_grid` path, each dynamic particle initially receives one
cell-volume mass. Passing the physical density directly therefore produced
`360 / 160 = 2.25` times the intended mass: `0.03564 kg` instead of
`rho LWT = 0.01584 kg`.

The material density remains `1100 kg/m³`. The numerical density passed to
Newton is corrected by

```text
grid_density = material_density × cell_count / dynamic_particle_count
             = 1100 × 160 / 360
             = 488.888... kg/m³
```

This is a discretization compensation, not a claim that the material has a
physical density of `488.888 kg/m³`.

## Early observations before the mass correction

Both runs used `cuda:0`, 40 longitudinal cells, 10 VBD iterations, and
`dt = 1/600 s`.

| Duration | Damping | Last-1-s tip range | Final tip deflection | Interpretation |
|---:|---:|---:|---:|---|
| 4 s | 100 Pa·s | 0.27194 m | 0.30459 m | finite but strongly oscillating |
| 12 s | 1000 Pa·s | 0.000949 m | 0.49928 m | settled numerically, but physically suspect |

The second run passed the predefined 4 mm settling threshold, yet its tip
nearly reached the floor even though a 0.40 m cable fixed at 0.50 m cannot do
so without substantial stretch. This demonstrates that finite and settled do
not by themselves establish physical validity.

After correcting the nodal mass, a third 12 s run proved that mass was not the
only problem. Expected and actual mass agreed within `1.47e-7` relative error,
but the FEM centerline still reached 64.1% maximum strain, ended at 32.7%
strain, contacted the floor in 135 samples, and failed the 4 mm settling
threshold with a 12.44 mm final-one-second range.

## Model decision: solid FEM to split-stiffness rod

The required cable behavior is axially stiff but flexible in two-axis bending
and twist. A single soft-solid material couples those behaviors and the first
FEM approximation elongated excessively. Newton 1.5.1 provides `add_rod()`,
which represents a cable as capsule bodies joined by cable joints with
independent stretch, shear, bend, twist, and damping terms.

The first rod experiment uses a circular 6 mm diameter, 40 segments, stiff
`1e6 N/m` stretch and shear terms, and much smaller bend/twist terms. These are
provisional engineering parameters, not measured cable properties. The rod is
added as a new experiment; it does not replace the diagnostic FEM model or the
accepted `fem_strip` grasp baseline.

`add_rod()` assigns mass from overlapping capsule volumes. The input density
is therefore scaled from the physical `1100 kg/m³` to `785.714 kg/m³`, so the
40-capsule total equals the circular cable mass
`rho*pi*r^2*L = 0.0124407 kg`.

The first rod run kept its root exactly fixed and remained finite. Final
extension was 0.573%, far below the FEM result, although transient extension
reached 2.42% and the last-one-second tip range was still 52.15 mm. Its minimum
surface height was 99.5 mm, proving that it did not reach the floor.

The initially reported 947 rigid contacts were not ground contacts. Newton's
builder default rigid contact gap is 0.1 m, enormous relative to a 6 mm cable,
so many non-neighbor cable segments became contact candidates. The rod model
now sets a 1 mm gap explicitly and reports ground and cable-self contacts
separately. The next run keeps stiffness and damping unchanged so this contact
configuration change can be evaluated independently.

The repeated run with a 1 mm gap produced zero ground contacts and zero
self-contacts. Its motion metrics were identical to the first rod run, proving
that the earlier 947 entries were inactive candidates rather than forces.
Removing them reduced wall time from 54.9 s to 44.1 s (about 20%) without
changing the trajectory. The remaining failures are therefore isolated to
dynamic settling and transient joint extension.

The simulation now accepts `--damping-scale`. It multiplies all four damping
terms while leaving mass, geometry, contact, and every stiffness unchanged.
The next controlled trial uses a scale of 4.0; only if transient extension
still exceeds 1% after settling is improved will stretch stiffness be changed.

At scale 4.0, the 12 s last-one-second range fell from 52.15 mm to 31.61 mm,
while final extension stayed below 1%. This confirms that damping accelerates
decay, but it does not establish that four times the provisional damping is a
better physical cable. Without a measured decay curve, the next experiment
returns to scale 1.0 and extends observation to 30 s.

The acceptance logic also now separates startup shock from settled behavior.
Global maximum extension remains in the result as a diagnostic, but the
cantilever equilibrium candidate uses the maximum extension during the final
one-second window. Per-second tip ranges expose whether oscillation is actually
decaying instead of hiding the history behind one final value. Dynamic robot
manipulation will need a separate transient-strain acceptance test later.

The 30 s scale-1 run passed: the per-second tip range decreased monotonically
from 397.49 mm to 3.98 mm, final-window maximum extension was 0.534%, the root
errors remained zero, and `candidate_rod_cable_pass=true`. This validates the
original damping for the provisional cantilever test; scale 4 is not adopted.

## Diagnostics and acceptance order

`fem_cable_simulation.py` reports expected and actual mass, centerline length,
maximum centerline strain, minimum particle surface height, ground soft-contact
count, fixed-end error, finite state, and settling range. The next CUDA run
must validate mass and length preservation before the model is made free or
introduced to the gripper.

The integration progression is:

1. Correct cantilever mass and measure strain. (complete)
2. Create a free cable resting on an explicitly configured floor. (implemented)
3. Extend the Robotiq pad-gap mapping below the old 45.32 mm limit. (implemented)
4. Verify the MoveIt route with a plan-only preview. (next run)
5. Attempt a slow lift and release with bilateral rigid-contact acceptance.
6. Add peg contact and routing only after the grasp is measurable.

The accepted `fem_strip` mode remains unchanged so every cable result can be
compared with the frozen `v0.2.0-fem-grasp` baseline.

The follow-up `rod_cable_topology.py` and `rod_cable_simulation.py` record
capsule/joint counts, corrected mass, effective connection length, maximum
joint gap, root error, contact count, settling, and a provisional candidate
pass. The initial cantilever should not reach the floor because its 0.40 m
length starts at 0.50 m.

## Reproduction

```bash
cd ~/ur5_ws/src/ur5_moveit_demo/docs/experiments/newton-soft-strip/prototype

~/newton_ws/.venv-cpu/bin/python fem_cable_topology.py

CUDA_VISIBLE_DEVICES=0 ~/newton_ws/.venv-cpu/bin/python \
  fem_cable_simulation.py \
  --device cuda:0 \
  --cells-x 40 \
  --duration 12 \
  --damping 1000 \
  --start-delay 1 \
  --port 8084
```

The virtual-environment name `.venv-cpu` is historical. The simulation
refuses non-CUDA devices, allocates the Newton model explicitly on the chosen
CUDA device, and reports that device in its result JSON.

Run the rod candidate separately:

```bash
~/newton_ws/.venv-cpu/bin/python rod_cable_topology.py

CUDA_VISIBLE_DEVICES=0 ~/newton_ws/.venv-cpu/bin/python \
  rod_cable_simulation.py \
  --device cuda:0 \
  --duration 30 \
  --damping-scale 1 \
  --start-delay 1 \
  --port 8085
```

The MoveIt integration and four-terminal safety sequence are documented in
[`CABLE_MOVEIT_PICK.zh-TW.md`](CABLE_MOVEIT_PICK.zh-TW.md).

The first integrated run on 2026-10-08 was not accepted. Every MoveIt path
completed, but bilateral loaded contact disappeared early in the lift and the
grasp region rose only about 9.56 mm. This is retained as the baseline for the
next contact-geometry and grasp-height diagnostic rather than hidden by tuning
friction or damping.
