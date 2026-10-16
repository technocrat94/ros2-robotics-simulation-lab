# cuRobo moving-wall replanning demo

This is an offline CUDA demonstration. It starts neither ROS nor Newton and
does not command robot hardware. The accepted `ur5_viser_preflight.py` remains
unchanged.

The scene uses `step_c_cable_wall`. Its wall center follows

```text
x(t) = 0.487 + 0.10 sin(2 pi t / 8) metres
```

while the cable target, wall dimensions, robot start state, and pre-grasp pose
stay identical to Step C. Every 0.2 seconds the program updates cuRobo's
collision scene, replans from the current simulated joint state, advances one
waypoint along the newest successful plan, and updates the wall, robot, and
cyan tool path in Viser.

Run the 30-second demo:

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_curobo_moving_wall_demo.sh
```

Open the printed URL, normally <http://127.0.0.1:8081/>. The runner verifies
both `nvidia-smi` and PyTorch CUDA and refuses CPU fallback. Each attempt prints
`MOVING_WALL_REPLAN`; the final `CUROBO_MOVING_WALL_RESULT` contains mean, p95,
maximum replanning time, and success rate.

Useful validation options:

```bash
# Short viewer smoke test
./scripts/run_curobo_moving_wall_demo.sh --duration 5

# CUDA planning without the Viser server
./scripts/run_curobo_moving_wall_demo.sh --headless --duration 5

# Save the machine-readable summary
./scripts/run_curobo_moving_wall_demo.sh \
  --result-json ~/ur5_ws/run_logs/curobo_moving_wall.result.json
```

## Recorded result

The 2026-10-16 RTX 3080 recording ran for 30.000 seconds on `cuda:0` and
completed 150/150 replans. Mean / p95 / maximum replanning time was
75.845 / 78.745 / 82.526 ms, with a 100% success rate. The one-time lazy CUDA
prime is logged separately and excluded from these periodic timing statistics.

- [1280x720 MP4](media/moving_wall_replan.mp4)
- [GIF preview](media/moving_wall_replan.gif)
- [Recorded-run JSON](results/moving-wall/recording_result.json)
- [Recorded-run log](results/moving-wall/recording_run.log)

The recording shows the robot advancing from its current simulated state along
the newest cyan tool path while the red wall moves through its commanded
sinusoid. The orange sphere is the unchanged Step C cable pre-grasp target.
