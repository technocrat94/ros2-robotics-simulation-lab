#!/usr/bin/env python3
"""Newton contact endpoint driven by MoveIt shadow joint positions."""
import functools
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import time

import newton
import numpy as np
import viser
from newton.viewer import ViewerViser

LESSON_DIR = Path.home() / "newton_ws/lessons/segmented_strip"
sys.path.insert(0, str(LESSON_DIR))
from robot_segmented_grasp_batch import (  # noqa: E402
    CONTACT_FRICTION,
    DT,
    FRAME_DT,
    SUBSTEPS,
    build_scene,
    grasp_region_links,
)

STATE_PORT = int(os.environ.get("NEWTON_STATE_PORT", "15100"))
COMMAND_PORT = int(os.environ.get("NEWTON_COMMAND_PORT", "15101"))
STATE_ADDRESS = ("127.0.0.1", STATE_PORT)
COMMAND_ADDRESS = ("127.0.0.1", COMMAND_PORT)
PROTOCOL = 1
VIEWER_PORT = int(os.environ.get("NEWTON_VIEWER_PORT", "8086"))
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


def robot_coordinates(value, previous):
    names = [str(name) for name in value["joint_names"]]
    positions = [float(position) for position in value["joint_positions"]]
    expected = JOINT_NAMES[:6]
    if len(names) != 6 or len(positions) != 6 or set(names) != set(expected):
        raise ValueError("robot command must contain the six UR5 arm joints")
    if not all(math.isfinite(position) for position in positions):
        raise ValueError("robot command contains a non-finite arm position")
    by_name = dict(zip(names, positions))
    q = previous.copy()
    q[:6] = [by_name[name] for name in expected]
    grip = float(value.get("gripper_position", q[6]))
    if not math.isfinite(grip):
        raise ValueError("robot command contains a non-finite gripper position")
    q[6] = grip
    q[8] = -grip
    q[10] = grip
    q[11] = -grip
    q[7] = -grip
    q[9] = grip
    return q


def mimic_error(q):
    leader = float(q[6])
    expected = [-leader, leader, -leader, -leader, leader]
    actual = [float(q[index]) for index in (8, 10, 11, 7, 9)]
    return max(abs(a - e) for a, e in zip(actual, expected))


newton.use_coord_layout_targets = True
(
    model,
    strip_links,
    robot_bodies,
    robot_shapes,
    strip_shapes,
    active_robot_colliders,
) = build_scene()
state_0, state_1 = model.state(), model.state()
control = model.control()
solver = newton.solvers.SolverXPBD(model, iterations=30)
collision = newton.CollisionPipeline(model, rigid_contact_max=10000)
contacts = collision.contacts()
q = model.joint_q.numpy()
qd = model.joint_qd.numpy()
robot_q = np.zeros(12, dtype=np.float32)
previous_robot_q = robot_q.copy()
q[:12] = robot_q
model.joint_q.assign(q)
newton.eval_fk(model, model.joint_q, model.joint_qd, state_0)
grip_links = grasp_region_links(strip_links)
initial_center_z = float(state_0.body_q.numpy()[strip_links, 2].mean())

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

server_class = viser.ViserServer
viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
try:
    viewer = ViewerViser(
        port=VIEWER_PORT,
        label="MoveIt → Newton Ground-Strip Grasp",
        share=False,
    )
finally:
    viser.ViserServer = server_class
viewer.set_model(model)
server = viewer._server
server.scene.set_up_direction("+z")
server.initial_camera.position = (1.15, -1.35, 0.90)
server.initial_camera.look_at = (0.35, 0.12, 0.22)
server.initial_camera.up = (0.0, 0.0, 1.0)
server.gui.add_markdown(
    "## MoveIt absolute-position grasp\n"
    "The robot receives MoveIt controller references through ROS 2. "
    "The segmented strip remains dynamic on Newton's ground plane; "
    "there is no attachment constraint."
)
status = server.gui.add_markdown("Preparing...")

phase = "WAITING_FOR_ROS"
last_command = "none"
last_command_id = ""
sequence = 0
sim_time = 0.0
maximum_center_z = initial_center_z
maximum_contacts = 0
latest_robot_q = robot_q.copy()
next_frame = time.monotonic()
next_state = next_frame

print(
    "MOVEIT_GRASP_ENDPOINT_READY "
    f"robot_bodies={robot_bodies} robot_shapes={robot_shapes} "
    f"strip_bodies={len(strip_links)} strip_shapes={len(strip_shapes)} "
    f"active_robot_colliders={active_robot_colliders} friction={CONTACT_FRICTION} "
    f"object_center=(0.4869,0.10915,{initial_center_z:.5f}) "
    f"command={COMMAND_PORT} state={STATE_PORT} viewer={VIEWER_PORT}",
    flush=True,
)

try:
    while alive and viewer.is_running():
        now = time.monotonic()
        while True:
            try:
                payload, sender = command_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                command = json.loads(payload.decode("utf-8"))
                if command.get("protocol") != PROTOCOL:
                    raise ValueError("protocol mismatch")
                kind = command.get("command")
                if kind not in {"sync_robot_state", "set_robot_positions"}:
                    raise ValueError("contact endpoint accepts only robot-state commands")
                latest_robot_q = robot_coordinates(command["value"], latest_robot_q)
                # Synchronization establishes the initial state; it is not motion.
                # Matching the previous state prevents a false velocity impulse.
                if kind == "sync_robot_state":
                    previous_robot_q = latest_robot_q.copy()
                phase = "ROS_SYNCHRONIZED" if kind == "sync_robot_state" else "MOVEIT_CONTACT"
                last_command = kind
                last_command_id = str(command.get("id", ""))
                reply = {
                    "type": "ack", "protocol": PROTOCOL,
                    "id": last_command_id, "command": kind, "ok": True,
                }
            except Exception as error:
                reply = {
                    "type": "ack", "protocol": PROTOCOL,
                    "id": "", "ok": False, "error": str(error),
                }
            command_socket.sendto(json.dumps(reply).encode("utf-8"), sender)

        if now >= next_frame:
            velocity_dt = max(FRAME_DT, now - (next_frame - FRAME_DT))
            robot_qd = (latest_robot_q - previous_robot_q) / velocity_dt
            previous_robot_q = latest_robot_q.copy()
            q[:12] = latest_robot_q
            qd[:12] = robot_qd
            model.joint_q.assign(q)
            model.joint_qd.assign(qd)

            for _ in range(SUBSTEPS):
                newton.eval_fk(
                    model,
                    model.joint_q,
                    model.joint_qd,
                    state_0,
                    body_flag_filter=newton.BodyFlags.KINEMATIC,
                )
                state_0.clear_forces()
                collision.collide(state_0, contacts)
                count = int(contacts.rigid_contact_count.numpy()[0])
                maximum_contacts = max(maximum_contacts, count)
                solver.step(state_0, state_1, control, contacts, DT)
                state_0, state_1 = state_1, state_0
                sim_time += DT
            next_frame = max(next_frame + FRAME_DT, now)

        if now >= next_state:
            poses = state_0.body_q.numpy()
            center = poses[strip_links, :3].mean(axis=0)
            grasp_z = float(poses[grip_links, 2].mean())
            maximum_center_z = max(maximum_center_z, float(center[2]))
            count = int(contacts.rigid_contact_count.numpy()[0])
            message = {
                "type": "state", "protocol": PROTOCOL, "sequence": sequence,
                "wall_time_ns": time.time_ns(), "sim_time_s": sim_time,
                "running": phase == "MOVEIT_CONTACT", "frame_id": "world",
                "object": "segmented_strip", "position_m": [float(v) for v in center],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "joint_names": JOINT_NAMES,
                "joint_positions": [float(latest_robot_q[i]) for i in JOINT_Q_INDICES],
                "demo_phase": phase,
                "mimic_max_error_rad": mimic_error(latest_robot_q),
                "last_command_id": last_command_id,
            }
            state_socket.sendto(json.dumps(message).encode("utf-8"), STATE_ADDRESS)
            viewer.begin_frame(sim_time)
            viewer.log_state(state_0)
            viewer.end_frame()
            status.content = (
                f"### {phase}\n"
                f"- Object center: **({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}) m**\n"
                f"- Grasp-region z: **{grasp_z:.3f} m**\n"
                f"- Maximum object rise: **{maximum_center_z-initial_center_z:.3f} m**\n"
                f"- Gripper leader: **{latest_robot_q[6]:.3f} rad**\n"
                f"- Contact candidates: **{count}** (maximum {maximum_contacts})\n"
                f"- Last ROS command: **{last_command}**\n"
                f"- Attachment constraint: **none**"
            )
            sequence += 1
            next_state = now + 0.05
        time.sleep(0.001)
finally:
    viewer.close()
    command_socket.close()
    state_socket.close()
