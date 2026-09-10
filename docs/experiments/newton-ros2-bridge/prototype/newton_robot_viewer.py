#!/usr/bin/env python3
"""Visual inspection of the ROS-expanded UR5 + Robotiq URDF in Newton."""
import functools
import os
from pathlib import Path
import time

import newton
import viser
import warp as wp
from newton.viewer import ViewerViser

URDF = os.environ.get(
    "NEWTON_ROBOT_URDF",
    str(Path.home() / "newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf"),
)

newton.use_coord_layout_targets = True
wp.init()
wp.set_device("cpu")

builder = newton.ModelBuilder()
builder.add_urdf(URDF, floating=False, enable_self_collisions=False)
model = builder.finalize()
state = model.state()
newton.eval_fk(model, model.joint_q, model.joint_qd, state)

server_class = viser.ViserServer
viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
try:
    viewer = ViewerViser(port=8083, label="UR5 + Robotiq URDF Check", share=False)
finally:
    viser.ViserServer = server_class

viewer.set_model(model)
server = viewer._server
server.scene.set_up_direction("+z")
server.initial_camera.position = (2.2, -2.8, 1.8)
server.initial_camera.look_at = (0.0, 0.0, 0.6)
server.initial_camera.up = (0.0, 0.0, 1.0)
server.scene.add_grid(
    "/reference/z_zero", width=4.0, height=4.0, plane="xy",
    cell_color=(150, 170, 190), section_color=(70, 100, 140),
)
server.gui.add_markdown(
    "## UR5 + Robotiq import check\n"
    "**Newton 1.5.1 · CPU · ROS-expanded URDF**\n\n"
    f"- Bodies: **{model.body_count}**\n"
    f"- Joints: **{model.joint_count}**\n"
    f"- Shapes: **{model.shape_count}**\n\n"
    "This stage verifies geometry and assembly only. ROS commands, mimic motion, "
    "contact behavior, and dynamics are not yet validated."
)

print(
    f"ROBOT_VIEWER_READY bodies={model.body_count} joints={model.joint_count} "
    f"shapes={model.shape_count} viewer=8083",
    flush=True,
)
try:
    while viewer.is_running():
        viewer.begin_frame(0.0)
        viewer.log_state(state)
        viewer.end_frame()
        time.sleep(0.1)
finally:
    viewer.close()
