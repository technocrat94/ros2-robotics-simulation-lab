# Home ARM64 FEM cable pick success — 2026-10-10

## Result

The complete ROS 2 Jazzy, MoveIt 2, ROS–Newton bridge, and Newton FEM cable
workflow was rerun from a freshly restarted ROBO virtual machine and visually
verified by the operator.

The successful revision was `7c61465` on branch `experiment/fem-cable`.

The operator confirmed all three physical acceptance criteria in the Newton
viewer:

1. the UR5 approached the cable and the Robotiq gripper closed around it;
2. the deformable cable remained with the gripper during the lift;
3. the cable was released when the gripper reopened.

This visual confirmation matters because a MoveIt `SUCCEEDED` result only proves
that the robot trajectory and controller sequence completed. It does not, by
itself, prove that the simulated object was physically grasped.

## Failure before success and the two corrections

The first home-machine attempt did not produce a valid grasp. Instead of starting
the investigation again from friction tuning, the earlier ground-strip failures
were used as diagnostic evidence. Those experiments had already shown that high
friction cannot compensate for missing or unsuitable rigid–soft contact geometry.

Only two main model corrections were needed:

1. **Restore the validated deformable representation.** The simplified rigid
   `rod_cable` chain was retained only as a comparison model. The grasp candidate
   was changed back to a volumetric tetrahedral FEM cable so that compression and
   distributed deformation could occur.
2. **Reuse stable analytic finger contact proxies and calibrated geometry.** The
   detailed Robotiq meshes remain visible, while hidden box proxies provide stable
   finger–FEM contact. Their gap was aligned with the 6 mm cable width, 2 mm
   nominal compression, and the calibrated `0.7664 rad` closure command.

After these two corrections the next complete test succeeded. This is evidence
that the previous experiments created reusable engineering knowledge: the team
could identify the contact-model failure class and avoid repeating the long
friction-only search.

A separate display issue was also isolated: an old Viser page can remain connected
to a terminated server and appear frozen. Reopening the page and applying the
camera through the viewer API restored a useful view, but this display repair was
not counted as a physical grasp-model correction.

## Verified environment

- Host: Apple-silicon Mac mini
- Runtime: ARM64 Ubuntu virtual machine in UTM
- ROS distribution: ROS 2 Jazzy
- Newton device: CPU
- FEM resolution: `40 x 2 x 2` cells
- Object dimensions: `0.40 x 0.006 x 0.006 m`
- Robot/object friction setting: `10`
- Viewer: Viser on `http://127.0.0.1:30000/`

The object is a volumetric tetrahedral FEM model. The current executable creates
Newton's `SolverXPBD`; therefore this result must not be described as proof that
the same scene ran with `SolverVBD`.

## Reproduction

Run one command in each terminal and keep Terminals 1–3 open.

### Terminal 1 — MoveIt and controllers

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

RViz may be closed after startup on the home CPU machine to reduce load while
leaving the launch process running.

### Terminal 2 — ROS–Newton adapter

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

### Terminal 3 — Newton FEM cable endpoint

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_fem_cable_grasp_home_cpu.sh
```

Open `http://127.0.0.1:30000/` after the endpoint reports ready.

### Terminal 4 — one pick cycle

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_fem_cable_pick.sh
```

## Recorded control evidence

The saved MoveIt log records:

- measured Newton object target: `(0.4869, 0.1092, 0.0040) m`;
- safe pre-grasp Cartesian path: `100%`;
- vertical grasp approach path: `100%`;
- calibrated closed command reached: `0.7664 rad`;
- vertical lift Cartesian path: `100%`;
- reopened command reached: `0.0000 rad`;
- final MoveIt result: `ABSOLUTE POSITION PICK SUCCEEDED`.

The raw control log is stored at
[`results/home_arm64_cpu_fem_cable_pick_2026-10-10.log`](results/home_arm64_cpu_fem_cable_pick_2026-10-10.log).

## Engineering conclusion

The successful result depends on the complete chain rather than one parameter:
MoveIt plans the robot motion, the bridge forwards interpolated robot states,
analytic finger proxies provide stable collision geometry, and the tetrahedral
FEM model supplies cable deformation. Both control evidence and direct visual
observation are required before declaring the grasp successful.
