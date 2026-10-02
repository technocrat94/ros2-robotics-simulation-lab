# Verified Humble FEM Ground Pick

This record freezes the accepted UTM reference run in which MoveIt used
Newton's measured object position, executed a direct collision-aware top-down
path, closed the Robotiq gripper using the width calibration, lifted the FEM
strip, and reopened the gripper. The operator confirmed all three physical
acceptance conditions in the Newton viewer: no initial arm detour, strip lift,
and release only after the gripper opened.

This is the **ROS 2 Humble reference**. The ROS 2 Jazzy school-computer port
must reproduce the same observable behavior before it is accepted.

## What changed

- The pre-grasp route is a four-stage Cartesian path: raise, orient, translate
  above the object, and descend to the pre-grasp pose.
- `use_named_start=false` prevents the earlier unnecessary trip through the
  named `test_configuration` pose.
- Every arm joint's cumulative travel and maximum single step are checked.
- The vertical lift is explicitly time-parameterized at a separate low speed.
- The Newton endpoint uses VBD for the FEM strip, analytic fingertip collision
  proxies, full-surface rigid-soft contact, and interpolated ROS commands.
- The exact physics and grasp parameters are versioned in two runnable scripts.

## Reproduce the accepted Humble run

Build after cloning:

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select newton_ros_bridge ur5_moveit_demo --symlink-install
source install/setup.bash
```

Use four terminals:

1. Bringup and RViz:

   ```bash
   cd ~/ur5_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
   ```

2. ROS adapter:

   ```bash
   cd ~/ur5_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run newton_ros_bridge ros_adapter
   ```

3. Newton FEM endpoint:

   ```bash
   ~/ur5_ws/src/ur5_moveit_demo/scripts/start_humble_fem_endpoint.sh
   ```

4. Synchronize, guard, execute, and save a log:

   ```bash
   ~/ur5_ws/src/ur5_moveit_demo/scripts/run_humble_fem_pick.sh
   ```

The Newton viewer is served on port `8086`. A remote client must forward that
port before opening `http://127.0.0.1:8086/`.

## Final combined acceptance

The final run changed one variable relative to the physically successful grasp:
`use_named_start` was changed from `true` to `false`. The FEM physics, object
position, width calibration, closure compensation, descent, and lift settings
were preserved. This controlled test established that the named-start detour
was unnecessary and could be removed without losing the grasp.

The terminal log proves command and trajectory completion. Physical lift and
release remain operator-observed results because automated object-rise and
release assertions are not implemented yet.

## Accepted evidence

- Measured object center: `(0.4869, 0.1093, 0.0110) m`
- Width calibration: `50 mm` object, `2 mm` total compression
- Robotiq leader command: `0.375145 rad`
- Closure compensation: `10.578 mm`
- Compensated descent: `0.094422 m`
- Lift: `0.120 m` at velocity scale `0.030`
- Safe pre-grasp, descent, and lift paths: `100%`
- Timed lift trajectory: `3.517 s`
- No `Moving UR5 to named target` event in the accepted log
- Logged MoveIt sequence elapsed time: approximately `12.765 s`
- Final log: `ABSOLUTE POSITION PICK SUCCEEDED`
- Visual acceptance: no large detour; the strip lifted and fell only after
  opening

The machine-readable record is
[`humble_fem_pick_success_2026-10-01.json`](experiments/newton-ros2-bridge/results/humble_fem_pick_success_2026-10-01.json).
The accepted terminal output is preserved as
[`humble_fem_pick_success_2026-10-01.log`](experiments/newton-ros2-bridge/results/humble_fem_pick_success_2026-10-01.log).

## Runtime interpretation

At the audit sample, the endpoint had advanced `1611.82 s` of simulation
during `19180 s` of wall time. The approximate real-time factor was therefore
`0.084`: one simulated second required about `11.9` wall-clock seconds on
the UTM CPU. This is a process-lifetime estimate, not an exact per-run timing
measurement. Future runs should record Newton simulation time immediately
before and after the grasp.

## Engineering acceptance boundary

A MoveIt success line alone proves planning and controller completion. This
milestone is accepted because the same run also had direct operator observation
of FEM-strip lift and release. The next improvement is an automated per-run
assertion for object rise, bounded penetration, retained contact during lift,
and release after opening.
