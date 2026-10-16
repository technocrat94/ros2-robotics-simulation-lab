# cuRobo moving-cylinder dodge demo

This is an offline CUDA/Viser demonstration. It starts neither ROS nor Newton,
does not command robot hardware, and leaves the accepted
`moving_wall_replan.py` unchanged.

The UR5 continuously shuttles between its recorded start pose (tool above
`y = 0.116 m`) and cable pre-grasp (`y = 0.410 m`). A vertical red cylinder has
radius `0.05 m` and height `0.25 m`; the old static Step C wall is not loaded.
cuRobo V2 exposes a `Cylinder` scene type, but its generic collision backend
has no analytic cylinder kernel. Viser renders the true cylinder while the
CUDA planner uses the closest fast primitive: its conservative
`0.10 x 0.10 x 0.25 m` cuboid bounding volume. This slightly overestimates the
corners and preserves the requested 0.2-second replanning cadence.

Every control cycle the program updates the obstacle, replans from the current
simulated joint state, advances eight 25 ms trajectory samples (0.2 seconds of
motion), measures signed clearance through cuRobo's collision checker, and
updates the cylinder, robot, and cyan path in Viser. Positive clearance means
separation; negative clearance means penetration.

The suggested `z = 0.30-0.45 m` range intersects this repository's fitted
UR5 + Robotiq collision spheres at both shuttle endpoints. The calibrated
runtime/drag limits are therefore x `0.44-0.53 m`, y `0.15-0.38 m`, and z
`0.60-0.68 m`. This keeps the tall cylinder near the arm route while allowing
positive clearance. This is a collision-model constraint, not a physics or
grasp-parameter change.

## Random mode

Random mode creates new seeded waypoints at runtime and joins them with a
smoothstep curve. Segment duration accounts for smoothstep's peak derivative,
so commanded speed never exceeds `0.15 m/s`. The seed and obstacle xyz are
logged every cycle.

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_curobo_dodge_demo.sh --mode random --seed 11
```

## Interactive mode

Interactive mode adds a translation gizmo to Viser. Open the printed URL
(normally <http://127.0.0.1:8082/>) and drag `/controls/cylinder`; its live
position becomes the next cuRobo collision-world pose.

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_curobo_dodge_demo.sh --mode interactive
```

Stop either live mode with `Ctrl-C`. The launcher verifies both `nvidia-smi`
and PyTorch CUDA and refuses CPU fallback. No video or screenshot is required.

## Three-seed evidence

On 2026-10-16, three 12-second RTX 3080 runs completed 180/180 replans with
nine total goal switches. All clearances were positive; the overall minimum was
`0.035993 m`. Weighted mean replanning time was `87.668 ms`, the largest
per-run p95 was `94.910 ms`, and maximum was `97.044 ms`. Maximum observed
obstacle speed was `0.149395 m/s`, below the `0.15 m/s` limit.

All three obstacle-path hashes differ and all three arm-path hashes differ,
showing that neither motion path is a fixed playback. Each JSON contains both
hashes, path bounds, timing, clearance, and switch count; each log contains
every obstacle pose and cycle result.

- [Three-seed summary](results/dodge/three_seed_summary.json)
- Seed 11: [JSON](results/dodge/seed_11.json), [log](results/dodge/seed_11.log)
- Seed 22: [JSON](results/dodge/seed_22.json), [log](results/dodge/seed_22.log)
- Seed 33: [JSON](results/dodge/seed_33.json), [log](results/dodge/seed_33.log)
