# Newton Soft-Strip Modeling Experiments

Recorded on 2026-09-11 and extended on 2026-09-14. These experiments establish and compare two measurable soft-strip representations in Newton 1.5.1 before coupling the object to the UR5/Robotiq and MoveIt workflow.

## Engineering objective

The objective was to determine whether a gravity-loaded, cantilevered rubber-like strip could be represented with Newton's tetrahedral FEM model and whether its predicted equilibrium tip deflection was sufficiently insensitive to refinement along the strip length.

The experiment separates four questions that a convincing animation alone cannot answer:

1. Does the generated mesh have the intended dimensions, topology, and fixed boundary?
2. Does the simulation remain finite and preserve the fixed end?
3. Has the oscillation become small enough for equilibrium values to be compared?
4. Does the measured tip deflection converge as the longitudinal mesh is refined?

## Model definition

| Property | Value |
|---|---:|
| Geometry | `0.40 × 0.05 × 0.02 m` |
| Initial height | `z = 0.50–0.52 m` |
| Boundary condition | complete left Y–Z face fixed |
| Young's modulus, `E` | `1,000,000 Pa` |
| Poisson ratio, `nu` | `0.45` |
| Density | `1,100 kg/m³` |
| FEM damping used in convergence runs | `1,000 Pa·s` |
| Solver | Newton `SolverVBD` |
| Physics step, `dt` | `1/600 s` |
| Solver iterations per substep | `10` |

The values describe a rubber-like numerical test material; they were not calibrated from a physical specimen. In particular, the damping value was selected for the numerical experiment and must not be presented as measured material data.

Newton receives the Lamé parameters derived from `E` and `nu`:

```text
mu     = E / (2(1 + nu))
lambda = E nu / ((1 + nu)(1 - 2 nu))
```

For the selected values, `mu = 344,827.586 Pa` and `lambda = 3,103,448.276 Pa`. The ratio `lambda/mu = 9` reflects the near-incompressible choice. Near incompressibility limits volume change; it does not make the strip rigid or prevent bending. It also makes the numerical system harder to solve as `nu` approaches `0.5`.

## Mesh and boundary verification

Only the number of cells along the 0.40 m X direction was changed. The Y and Z resolutions remained `3 × 2`.

| X cells | Particles | Tetrahedra | Surface triangles | Fixed particles |
|---:|---:|---:|---:|---:|
| 20 | 252 | 600 | 424 | 12 |
| 40 | 492 | 1,200 | 824 | 12 |
| 80 | 972 | 2,400 | 1,624 | 12 |

The fixed count remains 12 because the fixed boundary is the left Y–Z face:

```text
(cells_y + 1)(cells_z + 1) = (3 + 1)(2 + 1) = 12
```

Increasing `cells_x` changes mesh resolution, not material density or physical dimensions. A useful intuition is cutting the same sheet into more numerical pieces while leaving the sheet's external dimensions and material definition unchanged.

## Quantities and acceptance criteria

The free tip is the particle set at the maximum initial X coordinate. Its measured vertical quantity is the mean particle height, not a target value:

```text
mean_tip_z = mean(z of all free-tip particles)
tip_deflection = initial_mean_tip_z - mean_tip_z
```

The initial mean tip height is `0.51 m`. The equilibrium comparison uses the mean tip height over the final one-second window.

The preselected near-settled criterion was:

```text
max(tip_z over final 1 s) - min(tip_z over final 1 s) < 0.004 m
```

The 4 mm threshold is 1% of the strip length. Additional checks required finite particle coordinates and zero displacement of the fixed particles. Longitudinal mesh convergence was accepted when the change between the two finest deflections was below 5%, using the finer result as the denominator:

```text
relative change = abs(deflection_fine - deflection_coarse) / abs(deflection_fine)
```

## Results

![Newton FEM rubber strip deforming and oscillating under gravity](experiments/newton-soft-strip/results/soft_strip_oscillation.gif)

**Qualitative visual evidence.** The approximately 2× playback shows the `20 × 3 × 2` mesh during an eight-second Newton simulation with `1,000 Pa·s` damping, `dt = 1/600 s`, and 10 solver iterations. The left face remains fixed while the free end deforms and oscillates. The GIF demonstrates the modeled behavior over time; it does not establish accuracy or replace the numerical checks.

| X cells | Mean downward tip deflection | Final 1 s tip range | Result |
|---:|---:|---:|---|
| 20 | `0.38028 m` | `3.995 mm` | near-settled |
| 40 | `0.4028276 m` | `3.922 mm` | near-settled |
| 80, minimum 5 s observation | `0.4100183 m` | `0.091 mm` | passed, then cross-checked |
| 80, minimum 20 s observation | `0.4080032 m` | `0.135 mm` | passed |

All reported runs had finite particle state and `0.0 m` maximum fixed-end error.

The deflection changed by `5.60%` from 20 to 40 cells, which exceeded the 5% tolerance. It changed by only `1.27%` from 40 to the validated 80-cell result, so the equilibrium deflection passed the defined **X-direction mesh-convergence** check. Repeating the 80-cell case with a forced 20 s minimum changed its deflection by only `0.49%` relative to the first 80-cell run.

The 80-cell cross-check simulated 20.0 s in 212.07 s of wall time, a real-time factor of `0.0943`. On this CPU configuration, one simulated second therefore required about 10.60 wall-clock seconds.

## Why the 80-cell result was repeated

The first 80-cell batch stopped as soon as the default minimum observation time of 5 s was reached, whereas coarser cases remained above the motion threshold for much longer. That trend was surprising. Instead of accepting the Boolean `settled` field, the run was repeated with `--min-duration 20`.

The second run preserved the equilibrium deflection and produced an even smaller final-window range. This supports the equilibrium result. It does **not** validate a physical settling time. In the current program, `settling_time_s` is the time when the stop condition is detected after the configured minimum duration; when the condition is already true at that boundary, it is not an independently identified material settling time.

This distinction is central to the result:

- the equilibrium deflection has passed the stated longitudinal refinement check;
- transient damping and settling time have not been shown to converge;
- the refinement changed only X resolution, so full three-dimensional mesh convergence has not been established.

The deflection is also comparable to the 0.40 m strip length, so small-deflection beam theory is not an appropriate validation model for this configuration.

## Rigid-segment approximation

The second representation approximates the same `0.40 × 0.05 × 0.02 m`, `0.44 kg` strip as a chain of rigid boxes connected by compliant revolute joints. It is comparable to an articulated ruler: every link is rigid, while the hinges provide bending compliance.

For a rectangular cross-section,

```text
I = b h^3 / 12 = 3.333333e-8 m^4
EI = 0.033333 N m^2
segment length = L / N
joint stiffness = EI / segment_length
```

The hinge torque follows the intended spring-damper interpretation:

```text
torque = -joint_stiffness * angle - joint_damping * angular_velocity
```

The first segment is kinematic, which exactly imposes the cantilever boundary. A fixed-joint root was tested first, but its root drift decreased only from `0.418 mm` at 10 solver iterations to `0.214 mm` at 40 iterations while computation became slower. The kinematic root produced `0.0 m` root-position error and made the boundary condition explicit.

Joint damping was scaled with segment count from the 20-segment reference value:

```text
c_joint = 0.02 * segments / 20  N m s/rad
```

This preserves a consistent distributed-damping rule during refinement; it is a numerical modeling choice, not a calibrated material coefficient.

Self-collision was disabled for this bending comparison. With 40 segments and shape collision enabled, the folded chain generated more contacts than the configured 2,000-contact buffer and emitted repeated contact-buffer overflow warnings. Disabling segment collisions removed that invalid comparison factor and matched the FEM experiment, which also did not claim self-contact validation. Contact must be restored later with suitable collision filtering and capacity for robot grasp tests.

## Segmented-model results

The topology checks passed: the model contained `N` bodies and shapes, `N - 1` revolute joints, equal segment masses totaling `0.44 kg`, and the intended initial geometry. All runs below had finite state and exactly zero kinematic-root position error.

| Segments | Joint stiffness | Joint damping | Mean downward tip deflection | Final 1 s range | RTF |
|---:|---:|---:|---:|---:|---:|
| 20 | `1.6667 N m/rad` | `0.020 N m s/rad` | `0.352523 m` | `3.176 mm` | `0.827` |
| 40 | `3.3333 N m/rad` | `0.040 N m s/rad` | `0.381506 m` | `3.945 mm` | `0.825` |
| 80 | `6.6667 N m/rad` | `0.080 N m s/rad` | `0.401202 m` | `3.980 mm` | `0.762` |

The 20-to-40 change was `7.60%`, so 20 segments were insufficient under the predefined 5% criterion. The 40-to-80 change was `4.91%`, so the equilibrium deflection narrowly passed the longitudinal refinement criterion. The 80-segment final-window range was `3.980 mm`, only `0.020 mm` below the 4 mm stopping threshold; this is a boundary pass and does not establish robust transient convergence.

The 80-segment result differed from the validated 80-cell FEM deflection (`0.408003 m`) by `1.67%`. Agreement between two discretizations is useful cross-evidence, but it is not experimental material validation because both models use assumptions derived from the same nominal geometry and Young's modulus.

At 80 longitudinal divisions, the segmented model achieved an RTF of `0.762`, versus `0.0943` for the validated FEM case on the same CPU environment. The segmented approximation was therefore about `8.1×` faster by this measurement. Its advantage is speed and direct joint control; its cost is that three-dimensional continuum deformation and local contact deformation are absent.

## Reproduction

The prototype is stored under `docs/experiments/newton-soft-strip/prototype/`.

```bash
cd docs/experiments/newton-soft-strip/prototype

# Verify topology and dimensions.
python soft_strip_topology.py --cells-x 20

# Inspect the undeformed mesh in Viser.
python soft_strip_viewer.py --cells-x 20 --port 8084

# Observe the gravity response.
python soft_strip_simulation.py \
  --cells-x 20 --duration 8 --start-delay 5 \
  --damping 1000 --iterations 10 --port 8084

# Run the headless convergence measurement.
python soft_strip_batch.py \
  --cells-x 80 --damping 1000 --iterations 10 \
  --min-duration 20 --settle-threshold 0.004 --max-duration 60

# Verify the rigid-segment topology and run its finest comparison.
python segmented_strip_topology.py --segments 80
python segmented_strip_batch.py \
  --segments 80 --min-duration 5 \
  --settle-threshold 0.004 --max-duration 60
```

Use the validated Newton virtual environment in the migrated VM when reproducing these commands. The visual program streams live frames and does not replay early motion for a browser that connects late; `--start-delay` provides time to open the viewer.

## Implementation map

| File | Responsibility |
|---|---|
| `soft_strip_topology.py` | Define geometry and material, build the tetrahedral grid, and verify topology and dimensions |
| `soft_strip_viewer.py` | Display the undeformed mesh without advancing physics |
| `soft_strip_simulation.py` | Run and display the gravity-loaded VBD simulation while measuring the tip and fixed boundary |
| `soft_strip_batch.py` | Run the same model headlessly and stop on a measurable final-window motion threshold |
| `segmented_strip_topology.py` | Build the rigid-link chain, derive `EI`-based hinge stiffness, and verify mass, topology, and initial geometry |
| `segmented_strip_batch.py` | Measure the segmented chain's response, automatic stopping, root error, finite state, and runtime |

## Engineering conclusion

The experiment establishes two reproducible Newton baselines with verified topology, boundary preservation, finite state, and longitudinal equilibrium-deflection convergence at the stated 5% tolerance. At the finest tested resolution, the segmented approximation was within `1.67%` of the FEM deflection and ran about `8.1×` faster. Its settling and convergence passes were narrow, and neither model has been calibrated against a physical specimen.

The evidence supports using the segmented model for the next fast robot-motion integration experiment and retaining FEM as the higher-fidelity reference. It does not establish calibrated rubber behavior, transient convergence, full 3D convergence, self-contact validity, robot contact, or grasping. MoveIt integration should preserve this distinction between commanded robot motion and Newton-measured object response.
