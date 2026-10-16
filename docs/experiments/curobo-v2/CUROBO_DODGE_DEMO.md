# cuRobo moving-cube dodge demo

This is an offline CUDA/Viser demonstration. It starts neither ROS nor Newton,
does not command robot hardware, and leaves the accepted
`moving_wall_replan.py` unchanged.

The UR5 continuously shuttles between its recorded start pose (tool above
`y = 0.116 m`) and the cable pre-grasp (`y = 0.410 m`). A red 10 cm cube at
route height moves smoothly between `y = 0.15 m` and `y = 0.38 m` with a
six-second period. The old static Step C wall is not loaded. Its tested center
height is `z = 0.52 m`: the requested approximate `z = 0.40 m` causes this
UR5 + Robotiq collision-sphere model to penetrate the cube at both shuttle
ends, so no valid replan can begin there. `0.52 m` is the nearest tested height
that keeps both endpoints separated throughout the cube's full y sweep.

Every 0.2 seconds the program:

1. updates the cube pose in cuRobo's collision world;
2. replans from the simulated current joint state to the active goal;
3. advances eight 25 ms trajectory samples, which is 0.2 seconds of motion;
4. queries cuRobo's sphere collision checker for the minimum signed clearance;
5. updates the cube, robot, cyan planned path, and statistics in Viser.

Positive clearance means separation; negative clearance means penetration.
The checker internally returns a smoothed activation cost. The demo uses unit
weight and one obstacle and reverses cuRobo's documented activation function
to report the result in metres.

Run continuously and open the printed URL (normally
<http://127.0.0.1:8082/>):

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_curobo_dodge_demo.sh
```

Stop it with `Ctrl-C`. The launcher verifies both `nvidia-smi` and PyTorch CUDA
and refuses CPU fallback. For a finite evidence run:

```bash
./scripts/run_curobo_dodge_demo.sh \
  --headless --duration 30 \
  --result-json docs/experiments/curobo-v2/results/dodge/result.json \
  | tee docs/experiments/curobo-v2/results/dodge/run.log
```

The final `CUROBO_DODGE_RESULT` reports mean/p95/maximum replanning time,
success rate, overall minimum clearance, and the number of completed goal
switches. No video or screenshot is required for this experiment.

## Recorded result

The 2026-10-16 RTX 3080 evidence run completed 150/150 replans in 30.000 s
with seven goal switches. Mean / p95 / maximum replanning time was
89.674 / 97.029 / 99.598 ms. Overall minimum signed clearance stayed positive
at 0.021935 m.

- [Result JSON](results/dodge/result.json)
- [Full cycle log](results/dodge/run.log)
