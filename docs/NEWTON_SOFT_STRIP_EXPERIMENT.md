# Newton FEM Soft-Strip Experiment

Recorded on 2026-09-11. This experiment establishes a measurable deformable-body baseline in Newton 1.5.1 before coupling a soft object to the UR5/Robotiq and MoveIt workflow.

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
```

Use the validated Newton virtual environment in the migrated VM when reproducing these commands. The visual program streams live frames and does not replay early motion for a browser that connects late; `--start-delay` provides time to open the viewer.

## Implementation map

| File | Responsibility |
|---|---|
| `soft_strip_topology.py` | Define geometry and material, build the tetrahedral grid, and verify topology and dimensions |
| `soft_strip_viewer.py` | Display the undeformed mesh without advancing physics |
| `soft_strip_simulation.py` | Run and display the gravity-loaded VBD simulation while measuring the tip and fixed boundary |
| `soft_strip_batch.py` | Run the same model headlessly and stop on a measurable final-window motion threshold |

## Engineering conclusion

The experiment establishes a reproducible Newton FEM baseline with verified topology, boundary preservation, finite state, and longitudinal equilibrium-deflection convergence at the stated 5% tolerance. It does not yet establish calibrated rubber behavior, transient convergence, full 3D mesh convergence, self-contact validity, robot contact, or grasping.

The next modeling comparison will represent a flexible strip as rigid segments connected by compliant joints. Comparing the FEM and segmented models under the same geometry, loading, and measurement definition will show how model choice affects deformation, computation cost, and suitability for robot interaction. MoveIt integration should follow after the deformable-object model and its acceptance measures are explicit.
