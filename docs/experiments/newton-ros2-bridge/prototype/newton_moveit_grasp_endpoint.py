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
    BASE_ARM,
    CONTACT_FRICTION,
    DT,
    FRAME_DT,
    SUBSTEPS,
    THICKNESS,
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
model.request_contact_attributes("force")
solver = newton.solvers.SolverXPBD(model, iterations=30)
collision = newton.CollisionPipeline(model, rigid_contact_max=10000)
contacts = collision.contacts()
q = model.joint_q.numpy()
qd = model.joint_qd.numpy()
shape_bodies = model.shape_body.numpy()
strip_shape_set = {int(index) for index in strip_shapes}
left_finger_shape_set = set()
right_finger_shape_set = set()
for shape_index in range(robot_shapes):
    body_index = int(shape_bodies[shape_index])
    if body_index < 0:
        continue
    label = str(model.body_label[body_index])
    if "finger" not in label:
        continue
    if "left" in label:
        left_finger_shape_set.add(shape_index)
    elif "right" in label:
        right_finger_shape_set.add(shape_index)


def local_point_to_world(body_index, point, body_poses):
    if body_index < 0:
        return np.asarray(point, dtype=np.float64)
    pose = body_poses[body_index]
    position = np.asarray(pose[:3], dtype=np.float64)
    quaternion_xyz = np.asarray(pose[3:6], dtype=np.float64)
    quaternion_w = float(pose[6])
    vector = np.asarray(point, dtype=np.float64)
    twice_cross = 2.0 * np.cross(quaternion_xyz, vector)
    return position + vector + quaternion_w * twice_cross + np.cross(
        quaternion_xyz, twice_cross
    )


def finger_strip_contacts(contact_buffer, count, body_poses):
    shape0 = contact_buffer.rigid_contact_shape0.numpy()[:count]
    shape1 = contact_buffer.rigid_contact_shape1.numpy()[:count]
    point0 = contact_buffer.rigid_contact_point0.numpy()[:count]
    point1 = contact_buffer.rigid_contact_point1.numpy()[:count]
    # XPBD writes spatial contact forces only after update_contacts().
    # The first three components are the world-frame linear force in newtons.
    forces = contact_buffer.force.numpy()[:count, :3]
    left_candidates = 0
    right_candidates = 0
    left_loaded = 0
    right_loaded = 0
    left_force = 0.0
    right_force = 0.0
    loaded_contact_z = []
    for first, second, first_point, second_point, force in zip(
        shape0, shape1, point0, point1, forces
    ):
        pair = {int(first), int(second)}
        if not pair.intersection(strip_shape_set):
            continue
        force_magnitude = float(np.linalg.norm(force))
        if force_magnitude > 1.0e-6 and pair.intersection(
            left_finger_shape_set | right_finger_shape_set
        ):
            if int(first) in strip_shape_set:
                strip_shape = int(first)
                strip_point = first_point
            else:
                strip_shape = int(second)
                strip_point = second_point
            strip_body = int(shape_bodies[strip_shape])
            loaded_contact_z.append(
                float(local_point_to_world(strip_body, strip_point, body_poses)[2])
            )
        if pair.intersection(left_finger_shape_set):
            left_candidates += 1
            left_force += force_magnitude
            left_loaded += int(force_magnitude > 1.0e-6)
        if pair.intersection(right_finger_shape_set):
            right_candidates += 1
            right_force += force_magnitude
            right_loaded += int(force_magnitude > 1.0e-6)
    return (
        left_candidates, right_candidates,
        left_loaded, right_loaded,
        left_force, right_force,
        min(loaded_contact_z) if loaded_contact_z else None,
        max(loaded_contact_z) if loaded_contact_z else None,
    )


robot_q = np.zeros(12, dtype=np.float32)
# Start from the same validated upright configuration as the standalone
# Newton grasp experiment.  Starting all UR joints at zero lays the arm across
# the ground and can corrupt the strip before the first ROS synchronization.
robot_q[:6] = BASE_ARM
previous_robot_q = robot_q.copy()
q[:12] = robot_q
model.joint_q.assign(q)
newton.eval_fk(model, model.joint_q, model.joint_qd, state_0)
grip_links = grasp_region_links(strip_links)
initial_poses = state_0.body_q.numpy()
initial_center_z = float(initial_poses[strip_links, 2].mean())
initial_grasp_z = float(initial_poses[grip_links, 2].mean())

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
previous_sample_arm_q = robot_q[:6].copy()
robot_state_initialized = False
previous_grasp_z = initial_grasp_z
closed_seen = False
lift_started = False
release_seen = False
prelift_grasp_z = None
maximum_closed_grasp_z = initial_grasp_z
release_start_grasp_z = None
minimum_post_release_grasp_z = None
minimum_strip_bottom_z = float(initial_poses[strip_links, 2].min()) - THICKNESS / 2.0
maximum_mimic_error = 0.0
finite_state = True
minimum_closed_contact_z = math.inf
maximum_closed_contact_z = -math.inf
wall_start = time.monotonic()
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
                if not robot_state_initialized or kind == "sync_robot_state":
                    previous_robot_q = latest_robot_q.copy()
                    previous_sample_arm_q = latest_robot_q[:6].copy()
                    robot_state_initialized = True
                    phase = "ROS_SYNCHRONIZED"
                    last_command = (
                        "sync_robot_state" if kind == "sync_robot_state"
                        else "auto_sync_first_robot_state"
                    )
                else:
                    phase = "MOVEIT_CONTACT"
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
            frame_start_q = previous_robot_q.copy()
            frame_target_q = latest_robot_q.copy()
            robot_qd = (frame_target_q - frame_start_q) / velocity_dt

            # ROS supplies sampled joint positions.  Move the kinematic robot
            # through every intermediate pose so a dynamic object cannot lose
            # contact merely because the latest sample arrived as a jump.
            for substep in range(SUBSTEPS):
                alpha = float(substep + 1) / float(SUBSTEPS)
                q[:12] = frame_start_q + alpha * (frame_target_q - frame_start_q)
                qd[:12] = robot_qd
                model.joint_q.assign(q)
                model.joint_qd.assign(qd)
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
            previous_robot_q = frame_target_q
            solver.update_contacts(contacts)
            next_frame = max(next_frame + FRAME_DT, now)

        if now >= next_state:
            poses = state_0.body_q.numpy()
            center = poses[strip_links, :3].mean(axis=0)
            grasp_z = float(poses[grip_links, 2].mean())
            finite_state = finite_state and bool(np.isfinite(poses[strip_links]).all())
            minimum_strip_bottom_z = min(
                minimum_strip_bottom_z,
                float(poses[strip_links, 2].min()) - THICKNESS / 2.0,
            )
            maximum_center_z = max(maximum_center_z, float(center[2]))
            count = int(contacts.rigid_contact_count.numpy()[0])
            (
                left_strip_contacts, right_strip_contacts,
                left_loaded_contacts, right_loaded_contacts,
                left_contact_force, right_contact_force,
                current_contact_z_min, current_contact_z_max,
            ) = finger_strip_contacts(contacts, count, poses)
            grip = float(latest_robot_q[6])
            arm_step = float(np.max(np.abs(latest_robot_q[:6] - previous_sample_arm_q)))
            current_mimic_error = mimic_error(latest_robot_q)
            maximum_mimic_error = max(maximum_mimic_error, current_mimic_error)

            if grip >= 0.20 and not release_seen:
                closed_seen = True
                if current_contact_z_min is not None:
                    minimum_closed_contact_z = min(
                        minimum_closed_contact_z, current_contact_z_min
                    )
                    maximum_closed_contact_z = max(
                        maximum_closed_contact_z, current_contact_z_max
                    )
                maximum_closed_grasp_z = max(maximum_closed_grasp_z, grasp_z)
                if not lift_started and arm_step > 1.0e-4:
                    lift_started = True
                    prelift_grasp_z = previous_grasp_z
            elif closed_seen and grip <= 0.05:
                if not release_seen:
                    release_seen = True
                    release_start_grasp_z = previous_grasp_z
                    minimum_post_release_grasp_z = grasp_z
                else:
                    minimum_post_release_grasp_z = min(
                        minimum_post_release_grasp_z, grasp_z
                    )

            measured_lift = (
                maximum_closed_grasp_z - prelift_grasp_z
                if prelift_grasp_z is not None else 0.0
            )
            release_drop = (
                release_start_grasp_z - minimum_post_release_grasp_z
                if release_start_grasp_z is not None
                and minimum_post_release_grasp_z is not None else 0.0
            )
            message = {
                "type": "state", "protocol": PROTOCOL, "sequence": sequence,
                "wall_time_ns": time.time_ns(), "sim_time_s": sim_time,
                "running": phase == "MOVEIT_CONTACT", "frame_id": "world",
                "object": "segmented_strip", "position_m": [float(v) for v in center],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "joint_names": JOINT_NAMES,
                "joint_positions": [float(latest_robot_q[i]) for i in JOINT_Q_INDICES],
                "demo_phase": phase,
                "mimic_max_error_rad": current_mimic_error,
                "grasp_region_z_m": grasp_z,
                "grasp_region_lift_m": measured_lift,
                "release_drop_m": release_drop,
                "minimum_strip_bottom_z_m": minimum_strip_bottom_z,
                "left_finger_strip_contacts": left_strip_contacts,
                "right_finger_strip_contacts": right_strip_contacts,
                "left_loaded_strip_contacts": left_loaded_contacts,
                "right_loaded_strip_contacts": right_loaded_contacts,
                "left_strip_contact_force": left_contact_force,
                "right_strip_contact_force": right_contact_force,
                "closed_seen": closed_seen,
                "release_seen": release_seen,
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
                f"- Measured closed-grip lift: **{measured_lift:.3f} m**\n"
                f"- Drop after reopening: **{release_drop:.3f} m**\n"
                f"- Minimum strip bottom: **{minimum_strip_bottom_z:.4f} m**\n"
                f"- Maximum object rise: **{maximum_center_z-initial_center_z:.3f} m**\n"
                f"- Gripper leader: **{latest_robot_q[6]:.3f} rad**\n"
                f"- Left finger-strip contacts: **{left_strip_contacts}**\n"
                f"- Right finger-strip contacts: **{right_strip_contacts}**\n"
                f"- Loaded contacts (left/right): "
                f"**{left_loaded_contacts}/{right_loaded_contacts}**\n"
                f"- Contact-force magnitude sum (left/right): "
                f"**{left_contact_force:.3f}/{right_contact_force:.3f}**\n"
                f"- Closed loaded-contact z range: "
                f"**{minimum_closed_contact_z if math.isfinite(minimum_closed_contact_z) else float('nan'):.4f}/"
                f"{maximum_closed_contact_z if math.isfinite(maximum_closed_contact_z) else float('nan'):.4f} m**\n"
                f"- Contact candidates: **{count}** (maximum {maximum_contacts})\n"
                f"- Last ROS command: **{last_command}**\n"
                f"- Attachment constraint: **none**"
            )
            sequence += 1
            previous_sample_arm_q = latest_robot_q[:6].copy()
            previous_grasp_z = grasp_z
            next_state = now + 0.05
        time.sleep(0.001)
finally:
    measured_lift = (
        maximum_closed_grasp_z - prelift_grasp_z
        if prelift_grasp_z is not None else 0.0
    )
    release_drop = (
        release_start_grasp_z - minimum_post_release_grasp_z
        if release_start_grasp_z is not None
        and minimum_post_release_grasp_z is not None else 0.0
    )
    result = {
        "object": "segmented_strip",
        "closed_seen": closed_seen,
        "lift_started": lift_started,
        "release_seen": release_seen,
        "initial_grasp_region_z_m": initial_grasp_z,
        "prelift_grasp_region_z_m": prelift_grasp_z,
        "maximum_closed_grasp_region_z_m": maximum_closed_grasp_z,
        "grasp_region_lift_m": measured_lift,
        "release_start_grasp_region_z_m": release_start_grasp_z,
        "minimum_post_release_grasp_region_z_m": minimum_post_release_grasp_z,
        "release_drop_m": release_drop,
        "minimum_strip_bottom_z_m": minimum_strip_bottom_z,
        "maximum_contact_count": maximum_contacts,
        "maximum_mimic_error_rad": maximum_mimic_error,
        "finite_state": finite_state,
        "simulated_time_s": sim_time,
        "wall_elapsed_s": time.monotonic() - wall_start,
        "candidate_contact_grasp_pass": bool(
            closed_seen
            and lift_started
            and release_seen
            and measured_lift >= 0.08
            and release_drop >= 0.05
            and minimum_strip_bottom_z >= -0.005
            and maximum_mimic_error <= 1.0e-5
            and finite_state
        ),
    }
    print("MOVEIT_GRASP_RESULT", json.dumps(result), flush=True)
    viewer.close()
    command_socket.close()
    state_socket.close()
