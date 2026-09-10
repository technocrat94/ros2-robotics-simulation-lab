#!/usr/bin/env python3
"""ROS-commanded UR5 + Robotiq kinematic demonstration in Newton 1.5.1."""
import functools
import json
import math
import os
from pathlib import Path
import signal
import socket
import time

import newton
import numpy as np
import viser
import warp as wp
from newton.viewer import ViewerViser

STATE_ADDRESS = ("127.0.0.1", 15100)
COMMAND_ADDRESS = ("127.0.0.1", 15101)
PROTOCOL = 1
VIEWER_PORT = int(os.environ.get("NEWTON_VIEWER_PORT", "8083"))
URDF = os.environ.get(
    "NEWTON_ROBOT_URDF",
    str(Path.home() / "newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf"),
)

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint",
]
JOINT_Q_INDICES = [0, 1, 2, 3, 4, 5, 6, 8, 10, 11, 7, 9]

newton.use_coord_layout_targets = True
wp.init()
wp.set_device("cpu")


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class RobotDemo:
    def __init__(self):
        builder = newton.ModelBuilder()
        builder.add_urdf(URDF, floating=False, enable_self_collisions=False)
        self.model = builder.finalize()
        self.state = self.model.state()
        self.q = np.zeros(self.model.joint_q.shape[0], dtype=np.float32)
        self.sim_time = 0.0
        self.phase = "READY"
        self.update_pose()

    def reset(self):
        self.sim_time = 0.0
        self.phase = "READY"
        self.update_pose()

    def step(self, dt):
        self.sim_time = min(8.0, self.sim_time + dt)
        self.update_pose()

    def update_pose(self):
        t = self.sim_time
        base = np.array([0.0, -math.pi / 2, math.pi / 2,
                         -math.pi / 2, -math.pi / 2, 0.0], dtype=np.float32)
        arm = base.copy()
        grip = 0.0
        if t <= 0.0:
            self.phase = "READY"
        elif t < 2.0:
            self.phase = "ARM_APPROACH"
            s = smoothstep(t / 2.0)
            arm[0] += 0.55 * s
            arm[2] -= 0.35 * s
        elif t < 4.0:
            self.phase = "GRIPPER_CLOSE"
            arm[0] += 0.55
            arm[2] -= 0.35
            grip = 0.70 * smoothstep((t - 2.0) / 2.0)
        elif t < 6.0:
            self.phase = "WRIST_MOTION"
            arm[0] += 0.55
            arm[2] -= 0.35
            arm[5] += 0.65 * math.sin(math.pi * (t - 4.0) / 2.0)
            grip = 0.70
        elif t < 8.0:
            self.phase = "RETURN_AND_OPEN"
            s = smoothstep((t - 6.0) / 2.0)
            arm[0] += 0.55 * (1.0 - s)
            arm[2] -= 0.35 * (1.0 - s)
            grip = 0.70 * (1.0 - s)
        else:
            self.phase = "COMPLETE"

        self.q[:] = 0.0
        self.q[:6] = arm
        self.q[6] = grip       # leader
        self.q[8] = -grip      # right knuckle
        self.q[10] = grip      # left inner knuckle
        self.q[11] = -grip     # right inner knuckle
        self.q[7] = -grip      # left finger tip
        self.q[9] = grip       # right finger tip
        self.model.joint_q.assign(self.q)
        newton.eval_fk(
            self.model, self.model.joint_q, self.model.joint_qd, self.state
        )

    def positions(self):
        return [float(self.q[index]) for index in JOINT_Q_INDICES]

    def mimic_error(self):
        leader = float(self.q[6])
        expected = [-leader, leader, -leader, -leader, leader]
        actual = [float(self.q[index]) for index in (8, 10, 11, 7, 9)]
        return max(abs(a - e) for a, e in zip(actual, expected))


alive = True


def stop(_signum, _frame):
    global alive
    alive = False


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
command_socket.bind(COMMAND_ADDRESS)
command_socket.setblocking(False)
state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

demo = RobotDemo()
running = False
sequence = 0
last_command = "none"
last_command_id = ""
last_tick = time.monotonic()
next_state = last_tick

server_class = viser.ViserServer
viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
try:
    viewer = ViewerViser(port=VIEWER_PORT, label="ROS 2 → Newton Robot Demo", share=False)
finally:
    viser.ViserServer = server_class
viewer.set_model(demo.model)
server = viewer._server
server.scene.set_up_direction("+z")
server.initial_camera.position = (1.4, -1.8, 1.2)
server.initial_camera.look_at = (0.0, 0.0, 0.6)
server.initial_camera.up = (0.0, 0.0, 1.0)
server.scene.add_grid("/reference/z_zero", width=2.5, height=2.5, plane="xy")
server.gui.add_markdown(
    "## ROS 2 → Newton robot demo\n"
    "Start, pause, and reset come from ROS services. Motion is prescribed "
    "kinematics for interface and mimic validation; contact dynamics are not claimed."
)
status = server.gui.add_markdown("Preparing...")

print(
    f"ROBOT_ENDPOINT_READY bodies={demo.model.body_count} joints={demo.model.joint_count} "
    f"shapes={demo.model.shape_count} command=15101 state=15100 viewer={VIEWER_PORT}",
    flush=True,
)

try:
    while alive and viewer.is_running():
        now = time.monotonic()
        try:
            payload, sender = command_socket.recvfrom(4096)
            command = json.loads(payload.decode("utf-8"))
            if command.get("protocol") != PROTOCOL:
                raise ValueError("protocol mismatch")
            kind = command.get("command")
            if kind == "set_running":
                running = bool(command["value"])
                last_command = f"set_running({str(running).lower()})"
            elif kind == "reset":
                demo.reset()
                running = False
                last_command = "reset"
            else:
                raise ValueError("unknown command")
            last_command_id = str(command.get("id", ""))
            reply = {"type": "ack", "protocol": PROTOCOL,
                     "id": last_command_id, "command": kind, "ok": True}
            command_socket.sendto(json.dumps(reply).encode("utf-8"), sender)
        except BlockingIOError:
            pass
        except Exception as error:
            reply = {"type": "ack", "protocol": PROTOCOL, "id": "",
                     "ok": False, "error": str(error)}
            command_socket.sendto(json.dumps(reply).encode("utf-8"), sender)

        dt = min(0.05, now - last_tick)
        last_tick = now
        if running:
            demo.step(dt)
            if demo.sim_time >= 8.0:
                running = False

        if now >= next_state:
            message = {
                "type": "state", "protocol": PROTOCOL, "sequence": sequence,
                "wall_time_ns": time.time_ns(), "sim_time_s": demo.sim_time,
                "running": running, "frame_id": "world", "object": "robot_demo",
                "joint_names": JOINT_NAMES, "joint_positions": demo.positions(),
                "demo_phase": demo.phase, "mimic_max_error_rad": demo.mimic_error(),
                "last_command_id": last_command_id,
            }
            state_socket.sendto(json.dumps(message).encode("utf-8"), STATE_ADDRESS)
            viewer.begin_frame(demo.sim_time)
            viewer.log_state(demo.state)
            viewer.end_frame()
            status.content = (
                f"### {demo.phase} · {'RUNNING' if running else 'PAUSED'}\n"
                f"- Simulation time: **{demo.sim_time:.2f} s**\n"
                f"- Last ROS command: **{last_command}**\n"
                f"- Gripper leader: **{demo.q[6]:.3f} rad**\n"
                f"- Maximum mimic error: **{demo.mimic_error():.6f} rad**\n"
                f"- Returned-state sequence: **{sequence}**"
            )
            sequence += 1
            next_state = now + 0.05
        time.sleep(0.002)
finally:
    viewer.close()
    command_socket.close()
    state_socket.close()
