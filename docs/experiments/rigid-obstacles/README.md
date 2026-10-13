# Rigid-obstacle milestone: Steps A and B

This directory records the isolated checks requested before integrating an
obstacle into the complete Newton/ROS grasp loop.  Both checks used commit
`c15ace47daf7433b757d49fd5e25a2d4ab2424e4` on the home ARM64 Jazzy machine.

The shared definition is [`config/obstacles.yaml`](../../../config/obstacles.yaml).
It deliberately has `enabled: false`, so the verified FEM grasp remains the
default.  Each experiment enables the box explicitly.

## Step A — pure Newton physics

Command:

```bash
./scripts/run_newton_rigid_obstacle_drop.sh
```

This test does not start ROS.  It drops a sphere onto the configured static box
using a 1/600 s Newton step and checks that the sphere settles, remains finite,
and penetrates by less than 1 mm.

Result: [`step_a_newton_cpu_arm64.json`](results/step_a_newton_cpu_arm64.json)

- maximum penetration: `0.0 m`
- last-window vertical range: `0.0 m`
- simulated/wall time: `4.017 / 6.067 s`
- real-time factor: `0.662`
- all checks: pass

## Step B — pure MoveIt planning

Prerequisite: run the existing Jazzy robot/MoveIt bringup.  Then run:

```bash
./scripts/run_moveit_rigid_obstacle_preview_jazzy.sh
```

The preview target is intentionally moved to `x=0.20 m`, on the opposite side
of the box from the initial tool pose.  The script first asks OMPL for a
collision-free plan, then verifies the explicit raised route.  Its three
spatial segments are raise, translate over the box, and descend; orientation is
reported separately.  Both runs are plan-only and send no controller command.

Results:

- [`step_b_moveit_jazzy_arm64.json`](results/step_b_moveit_jazzy_arm64.json)
- [`step_b_moveit_jazzy_arm64.log`](results/step_b_moveit_jazzy_arm64.log)

OMPL succeeded, all three spatial stages and the orientation stage reported
100%, the complete route was accepted, and the existing joint-travel limits
were not relaxed.

## Scope boundary

Step C has not started.  The Newton grasp endpoint does not yet create this
obstacle, and no complete grasp-with-obstacle claim is made here.
