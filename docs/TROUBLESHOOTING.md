# Troubleshooting Record

This record preserves the symptom, evidence, resolution, and lesson from each important failure. It documents the engineering process rather than only the final successful code.

| Symptom | Diagnostic evidence | Resolution | Engineering lesson |
|---|---|---|---|
| Xacro reported `Undefined substitution argument name` | Running `xacro` directly reproduced the error | Added the required argument/default and inspected the generated URDF before launch | Validate model generation independently before launching the full stack |
| A launch file existed in source but `ros2 launch` could not find it | The file was absent from the installed package share directory | Installed `launch`, `urdf`, and `config` resources in CMake, then rebuilt and sourced | ROS runs installed package resources, not arbitrary source paths |
| Planning to the named state aborted with error `-26` | The action server was reachable, but the planning request aborted | Loaded the combined project SRDF, kinematics, and robot description | URDF, SRDF, and MoveIt groups must represent the same combined robot |
| Planning succeeded but execution aborted | A trajectory was generated, then rejected immediately during execution | Aligned MoveIt's controller mapping with the active trajectory controller | Planning and controller execution are separate integration layers |
| The robot disappeared in RViz | Global Status reported `Frame [map] does not exist` | Changed the fixed frame to the available `world` frame | Visualization depends on a valid TF frame |
| Orange and grey robots appeared together | Grey followed `/joint_states`; orange represented a planned/goal state | Kept both when useful for comparison | A goal-state model is not a second physical robot |
| A small endpoint change caused a large rotation | Cartesian displacement was small but joint travel was excessive | Used a Cartesian path, held orientation, and added a 1 rad joint-travel guard | Equivalent IK endpoints can produce very different joint motions |
| The `up` state was reachable but its approach path completed 0% | Named-state execution succeeded, straight-line interpolation did not | Returned to the verified `test_configuration` work start | A reachable pose does not guarantee a feasible Cartesian departure |
| Larger approaches completed only 12.5% or 93.4% | `computeCartesianPath()` reported an incomplete fraction | Refused execution and restored the verified 3 cm approach | Refusing an incomplete path is a successful safety behavior |
| `/joint_states` occasionally reported a lost message | A complete fresh state was still available and updates continued | Monitored ongoing state/QoS instead of treating one warning as a controller failure | Diagnose sustained system behavior, not one isolated line |
| The ROS CLI reported an `rclpy.ok()` daemon error | Bringup could remain active while CLI discovery failed | Restarted the ROS 2 daemon and repeated the query | CLI discovery and running robot nodes are different processes |

## Recommended diagnostic order

Work from the lowest layer upward:

```text
1. Are all controllers active?
2. Is /joint_states updating?
3. Is the gripper action server available?
4. Do the TF frame and RViz fixed frame exist?
5. Did MoveIt load the intended robot description and SRDF?
6. Did planning succeed?
7. Did execution succeed?
8. Did Cartesian completion and joint-travel guards pass?
```

This is similar to diagnosing a lamp: check the power supply before replacing the switch, socket, or bulb.

## Verified recovery baseline

Return to these values when an experiment fails:

```text
Named target:       test_configuration
Approach:           (+0.03, 0.00, 0.00) m
Lift and transport: (-0.03, 0.00,+0.05) m
Automatic return:  ( 0.00, 0.00,-0.05) m
Velocity scaling:   0.2
Acceleration scale: 0.2
Cartesian minimum:  99%
Joint-travel guard: 1 rad
```

A successful baseline run reports 100.0% for all three Cartesian stages and ends with `PICK AND PLACE DEMO SUCCEEDED`.
