#!/usr/bin/env python3
"""Newton contact endpoint driven by MoveIt shadow joint positions."""
import functools
from collections import deque
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
    DENSITY,
    DEVICE,
    GROUND_FRICTION,
    OBJECT_HALF_THICKNESS,
    OBJECT_MODEL,
    DT,
    FRAME_DT,
    SUBSTEPS,
    STRIP_FRICTION,
    SOFT_CONTACT_KE,
    SOFT_CONTACT_KD,
    YOUNGS_MODULUS,
    build_scene,
    grasp_region_links,
)

STATE_PORT = int(os.environ.get("NEWTON_STATE_PORT", "15100"))
COMMAND_PORT = int(os.environ.get("NEWTON_COMMAND_PORT", "15101"))
STATE_ADDRESS = ("127.0.0.1", STATE_PORT)
COMMAND_ADDRESS = ("127.0.0.1", COMMAND_PORT)
PROTOCOL = 1
VIEWER_PORT = int(os.environ.get("NEWTON_VIEWER_PORT", "8086"))
SOFT_CONTACT_MARGIN = float(os.environ.get("GRASP_SOFT_CONTACT_MARGIN", "0.005"))
if SOFT_CONTACT_MARGIN < 0.0:
    raise ValueError("GRASP_SOFT_CONTACT_MARGIN must be nonnegative")
COMMAND_DT = float(os.environ.get("NEWTON_COMMAND_DT", "0.02"))
if COMMAND_DT <= 0.0:
    raise ValueError("NEWTON_COMMAND_DT must be greater than zero")
MAX_HOLD_SECONDS = float(os.environ.get("NEWTON_MAX_HOLD_SECONDS", "0.5"))
if MAX_HOLD_SECONDS < COMMAND_DT:
    raise ValueError("NEWTON_MAX_HOLD_SECONDS must be at least NEWTON_COMMAND_DT")
MAX_IDENTICAL_SAMPLES = max(1, int(round(MAX_HOLD_SECONDS / COMMAND_DT)))
MAX_COMMAND_QUEUE = int(os.environ.get("NEWTON_MAX_COMMAND_QUEUE", "4096"))
if MAX_COMMAND_QUEUE <= 0:
    raise ValueError("NEWTON_MAX_COMMAND_QUEUE must be greater than zero")
MAX_ALLOWED_PENETRATION = float(
    os.environ.get("GRASP_MAX_ALLOWED_PENETRATION", "0.005")
)
if MAX_ALLOWED_PENETRATION < 0.0:
    raise ValueError("GRASP_MAX_ALLOWED_PENETRATION must be nonnegative")
GRIPPER_CLOSED_MIN = 0.05
GRIPPER_CLOSED_MAX = 0.81
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
FEM_MODE = OBJECT_MODEL == "fem_strip"
ROD_MODE = OBJECT_MODEL == "rod_cable"
if ROD_MODE and not str(model.device).startswith("cuda"):
    print(
        "ROD_CABLE_CPU_MODE "
        f"requested={DEVICE!r} model_device={model.device}; "
        "physics is supported but will run slower than the school CUDA system"
    )
state_0, state_1 = model.state(), model.state()
control = model.control()
model.request_contact_attributes("force")
solver = (
    newton.solvers.SolverVBD(
        model=model,
        iterations=10,
        particle_enable_self_contact=False,
        particle_enable_tile_solve=False,
    )
    if FEM_MODE
    else newton.solvers.SolverXPBD(model, iterations=10 if ROD_MODE else 30)
)
collision = newton.CollisionPipeline(
    model,
    rigid_contact_max=10000,
    soft_contact_margin=SOFT_CONTACT_MARGIN,
    enable_rigid_soft_full_surface_contact=FEM_MODE,
)
contacts = collision.contacts()
q = model.joint_q.numpy()
qd = model.joint_qd.numpy()
shape_bodies = model.shape_body.numpy()
shape_flags = model.shape_flags.numpy()
shape_margins = model.shape_margin.numpy()
particle_radii = model.particle_radius.numpy()
robot_particle_colliders = sum(
    bool(int(shape_flags[index]) & int(newton.ShapeFlags.COLLIDE_PARTICLES))
    for index in range(robot_shapes)
)
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

left_tip_body = next(
    index for index, label in enumerate(model.body_label[:robot_bodies])
    if "robotiq_85_left_finger_tip_link" in str(label)
)
right_tip_body = next(
    index for index, label in enumerate(model.body_label[:robot_bodies])
    if "robotiq_85_right_finger_tip_link" in str(label)
)


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


def fem_soft_contact_metrics(contact_buffer, count, particles, body_poses):
    available = min(count, contact_buffer.soft_contact_shape.shape[0])
    shapes = contact_buffer.soft_contact_shape.numpy()[:available]
    indices = contact_buffer.soft_contact_indices.numpy()[:available]
    barycentric = contact_buffer.soft_contact_barycentric.numpy()[:available]
    body_points = contact_buffer.soft_contact_body_pos.numpy()[:available]
    normals = contact_buffer.soft_contact_normal.numpy()[:available]
    result = {
        "left_candidates": 0, "right_candidates": 0,
        "left_loaded": 0, "right_loaded": 0,
        "maximum_activation_depth_m": 0.0,
        "maximum_particle_surface_penetration_m": 0.0,
    }
    for shape, corners, weights, body_point, normal in zip(
        shapes, indices, barycentric, body_points, normals
    ):
        shape = int(shape)
        is_left = shape in left_finger_shape_set
        is_right = shape in right_finger_shape_set
        if not is_left and not is_right:
            continue
        slots = [slot for slot, particle in enumerate(corners) if int(particle) >= 0]
        if not slots:
            continue
        particle_indices = [int(corners[slot]) for slot in slots]
        soft_point = np.zeros(3, dtype=np.float64)
        for slot, particle_index in zip(slots, particle_indices):
            soft_point += float(weights[slot]) * particles[particle_index]
        body_point_world = local_point_to_world(
            int(shape_bodies[shape]), body_point, body_poses
        )
        separation = float(np.dot(
            np.asarray(normal, dtype=np.float64), soft_point - body_point_world
        ))
        radius = float(np.max(particle_radii[particle_indices]))
        activation_depth = max(
            0.0, radius + float(shape_margins[shape]) - separation
        )
        surface_penetration = max(0.0, radius - separation)
        result["maximum_activation_depth_m"] = max(
            result["maximum_activation_depth_m"], activation_depth
        )
        result["maximum_particle_surface_penetration_m"] = max(
            result["maximum_particle_surface_penetration_m"], surface_penetration
        )
        if is_left:
            result["left_candidates"] += 1
            result["left_loaded"] += int(activation_depth > 0.0)
        if is_right:
            result["right_candidates"] += 1
            result["right_loaded"] += int(activation_depth > 0.0)
    return result


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
    left_force_abs_xyz = np.zeros(3, dtype=np.float64)
    right_force_abs_xyz = np.zeros(3, dtype=np.float64)
    left_force_on_strip_xyz = np.zeros(3, dtype=np.float64)
    right_force_on_strip_xyz = np.zeros(3, dtype=np.float64)
    loaded_contact_z = []
    for first, second, first_point, second_point, force in zip(
        shape0, shape1, point0, point1, forces
    ):
        pair = {int(first), int(second)}
        if not pair.intersection(strip_shape_set):
            continue
        # Newton reports the force on shape0 from shape1.  Convert every
        # contact to the world-frame force acting on the strip so that the
        # sign of Fz has a direct physical meaning: +Fz lifts the strip and
        # -Fz pushes it toward the floor.
        force_on_strip = (
            np.asarray(force, dtype=np.float64)
            if int(first) in strip_shape_set
            else -np.asarray(force, dtype=np.float64)
        )
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
            left_force_abs_xyz += np.abs(force)
            left_force_on_strip_xyz += force_on_strip
            left_loaded += int(force_magnitude > 1.0e-6)
        if pair.intersection(right_finger_shape_set):
            right_candidates += 1
            right_force += force_magnitude
            right_force_abs_xyz += np.abs(force)
            right_force_on_strip_xyz += force_on_strip
            right_loaded += int(force_magnitude > 1.0e-6)
    return (
        left_candidates, right_candidates,
        left_loaded, right_loaded,
        left_force, right_force,
        left_force_abs_xyz, right_force_abs_xyz,
        left_force_on_strip_xyz, right_force_on_strip_xyz,
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
initial_poses = state_0.body_q.numpy()
if FEM_MODE:
    initial_particles = state_0.particle_q.numpy()
    grasp_particle_indices = np.flatnonzero(
        np.isclose(initial_particles[:, 0], 0.4869, atol=1.0e-5)
    )
    if len(grasp_particle_indices) == 0:
        raise RuntimeError("FEM grasp-region particles were not found")
    initial_center_z = float(initial_particles[:, 2].mean())
    initial_grasp_z = float(
        initial_particles[grasp_particle_indices, 2].mean()
    )
    grip_links = []
else:
    grip_links = grasp_region_links(strip_links)
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
maximum_soft_contacts = 0
maximum_left_finger_soft_contacts = 0
maximum_right_finger_soft_contacts = 0
maximum_closed_left_loaded_soft_contacts = 0
maximum_closed_right_loaded_soft_contacts = 0
maximum_lift_left_loaded_soft_contacts = 0
maximum_lift_right_loaded_soft_contacts = 0
bilateral_closed_soft_contact_samples = 0
bilateral_lift_soft_contact_samples = 0
maximum_closed_left_loaded_rigid_contacts = 0
maximum_closed_right_loaded_rigid_contacts = 0
maximum_lift_left_loaded_rigid_contacts = 0
maximum_lift_right_loaded_rigid_contacts = 0
bilateral_closed_rigid_contact_samples = 0
bilateral_lift_rigid_contact_samples = 0
maximum_soft_contact_activation_depth = 0.0
maximum_particle_surface_penetration = 0.0
latest_robot_q = robot_q.copy()
received_robot_q = robot_q.copy()
pending_robot_q = deque(maxlen=MAX_COMMAND_QUEUE)
command_queue_overflow_count = 0
command_segment_active = False
command_segment_start_q = robot_q.copy()
command_segment_target_q = robot_q.copy()
command_segment_elapsed = 0.0
maximum_command_backlog = 0
queue_drained_reported = True
consecutive_identical_samples = 1
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
minimum_strip_bottom_z = (
    float(initial_particles[:, 2].min() - np.max(particle_radii))
    if FEM_MODE
    else float(initial_poses[strip_links, 2].min()) - OBJECT_HALF_THICKNESS
)
maximum_mimic_error = 0.0
maximum_left_force_abs_xyz = np.zeros(3, dtype=np.float64)
maximum_right_force_abs_xyz = np.zeros(3, dtype=np.float64)
lift_left_force_on_strip_sum_xyz = np.zeros(3, dtype=np.float64)
lift_right_force_on_strip_sum_xyz = np.zeros(3, dtype=np.float64)
lift_force_sample_count = 0
minimum_lift_combined_force_z = math.inf
maximum_lift_combined_force_z = -math.inf
finite_state = True
minimum_closed_contact_z = math.inf
maximum_closed_contact_z = -math.inf
wall_start = time.monotonic()
next_frame = time.monotonic()
next_state = next_frame

print(
    "MOVEIT_GRASP_ENDPOINT_READY "
    f"device={model.device} "
    f"robot_bodies={robot_bodies} robot_shapes={robot_shapes} "
    f"strip_bodies={len(strip_links)} strip_shapes={len(strip_shapes)} "
    f"particles={model.particle_count} tetrahedra={model.tet_count} "
    f"particle_radius={float(model.particle_radius.numpy()[0]) if model.particle_count else 0.0:.4f} "
    f"robot_particle_colliders={robot_particle_colliders} "
    f"active_robot_colliders={active_robot_colliders} "
    f"robot_friction={CONTACT_FRICTION} strip_friction={STRIP_FRICTION} "
    f"ground_friction={GROUND_FRICTION} "
    f"density={DENSITY:.1f} youngs_modulus={YOUNGS_MODULUS:.1f} "
    f"soft_contact_ke={SOFT_CONTACT_KE:.1f} "
    f"soft_contact_kd={SOFT_CONTACT_KD:.1f} "
    f"soft_contact_margin={SOFT_CONTACT_MARGIN:.4f} substeps={SUBSTEPS} "
    f"command_dt={COMMAND_DT:.3f} max_hold={MAX_HOLD_SECONDS:.3f} "
    f"command_queue_capacity={MAX_COMMAND_QUEUE} "
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
                command_robot_q = robot_coordinates(
                    command["value"], received_robot_q
                )
                same_as_previous_received = np.allclose(
                    command_robot_q, received_robot_q, rtol=0.0, atol=1.0e-7
                )
                received_robot_q = command_robot_q.copy()
                # Synchronization establishes the initial state; it is not motion.
                # Matching the previous state prevents a false velocity impulse.
                if not robot_state_initialized or kind == "sync_robot_state":
                    pending_robot_q.clear()
                    command_segment_active = False
                    command_segment_start_q = command_robot_q.copy()
                    command_segment_target_q = command_robot_q.copy()
                    command_segment_elapsed = 0.0
                    latest_robot_q = command_robot_q.copy()
                    previous_robot_q = command_robot_q.copy()
                    previous_sample_arm_q = command_robot_q[:6].copy()
                    robot_state_initialized = True
                    queue_drained_reported = True
                    consecutive_identical_samples = 1
                    maximum_command_backlog = 0
                    phase = "ROS_SYNCHRONIZED"
                    last_command = (
                        "sync_robot_state" if kind == "sync_robot_state"
                        else "auto_sync_first_robot_state"
                    )
                else:
                    if same_as_previous_received:
                        consecutive_identical_samples += 1
                    else:
                        consecutive_identical_samples = 1
                    # Keep a bounded hold interval for contact settling, but
                    # do not let an idle ROS publisher grow the queue forever.
                    if consecutive_identical_samples <= MAX_IDENTICAL_SAMPLES:
                        if len(pending_robot_q) >= MAX_COMMAND_QUEUE:
                            command_queue_overflow_count += 1
                            raise BufferError(
                                f"robot command queue reached capacity {MAX_COMMAND_QUEUE}"
                            )
                        pending_robot_q.append(command_robot_q.copy())
                        maximum_command_backlog = max(
                            maximum_command_backlog, len(pending_robot_q)
                        )
                        queue_drained_reported = False
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
            frame_start_q = previous_robot_q.copy()
            frame_target_q = previous_robot_q.copy()
            remaining_frame_time = FRAME_DT
            while remaining_frame_time > 1.0e-12:
                if not command_segment_active:
                    if not pending_robot_q:
                        break
                    command_segment_start_q = frame_target_q.copy()
                    command_segment_target_q = pending_robot_q.popleft()
                    command_segment_elapsed = 0.0
                    command_segment_active = True
                segment_remaining = COMMAND_DT - command_segment_elapsed
                advance = min(remaining_frame_time, segment_remaining)
                command_segment_elapsed += advance
                ratio = min(1.0, command_segment_elapsed / COMMAND_DT)
                frame_target_q = (
                    command_segment_start_q
                    + ratio * (command_segment_target_q - command_segment_start_q)
                )
                remaining_frame_time -= advance
                if command_segment_elapsed + 1.0e-12 >= COMMAND_DT:
                    frame_target_q = command_segment_target_q.copy()
                    command_segment_active = False
            latest_robot_q = frame_target_q.copy()
            robot_qd = (frame_target_q - frame_start_q) / FRAME_DT

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
                soft_count = int(contacts.soft_contact_count.numpy()[0])
                maximum_soft_contacts = max(maximum_soft_contacts, soft_count)
                solver.step(state_0, state_1, control, contacts, DT)
                state_0, state_1 = state_1, state_0
                sim_time += DT
            previous_robot_q = frame_target_q
            if not FEM_MODE:
                solver.update_contacts(contacts)
            if (
                not pending_robot_q
                and not command_segment_active
                and not queue_drained_reported
            ):
                print(
                    "MOVEIT_COMMAND_QUEUE_DRAINED",
                    json.dumps({
                        "sim_time_s": sim_time,
                        "gripper_rad": float(latest_robot_q[6]),
                        "maximum_backlog": maximum_command_backlog,
                    }),
                    flush=True,
                )
                queue_drained_reported = True
            next_frame = max(next_frame + FRAME_DT, now)

        if now >= next_state:
            poses = state_0.body_q.numpy()
            if FEM_MODE:
                particles = state_0.particle_q.numpy()
                center = particles.mean(axis=0)
                grasp_z = float(
                    particles[grasp_particle_indices, 2].mean()
                )
                finite_state = finite_state and bool(
                    np.isfinite(particles).all()
                )
                minimum_strip_bottom_z = min(
                    minimum_strip_bottom_z,
                    float(particles[:, 2].min() - np.max(particle_radii)),
                )
            else:
                center = poses[strip_links, :3].mean(axis=0)
                grasp_z = float(poses[grip_links, 2].mean())
                finite_state = finite_state and bool(
                    np.isfinite(poses[strip_links]).all()
                )
                minimum_strip_bottom_z = min(
                    minimum_strip_bottom_z,
                    float(poses[strip_links, 2].min()) - OBJECT_HALF_THICKNESS,
                )
            finger_midpoint = 0.5 * (
                poses[left_tip_body, :3] + poses[right_tip_body, :3]
            )
            finger_separation = float(np.linalg.norm(
                poses[left_tip_body, :3] - poses[right_tip_body, :3]
            ))
            object_from_fingers = center - finger_midpoint
            maximum_center_z = max(maximum_center_z, float(center[2]))
            count = int(contacts.rigid_contact_count.numpy()[0])
            if FEM_MODE:
                soft_count = int(contacts.soft_contact_count.numpy()[0])
                soft_metrics = fem_soft_contact_metrics(
                    contacts, soft_count, particles, poses
                )
                left_soft_contacts = soft_metrics["left_candidates"]
                right_soft_contacts = soft_metrics["right_candidates"]
                left_loaded_soft_contacts = soft_metrics["left_loaded"]
                right_loaded_soft_contacts = soft_metrics["right_loaded"]
                current_soft_activation_depth = soft_metrics[
                    "maximum_activation_depth_m"
                ]
                current_particle_surface_penetration = soft_metrics[
                    "maximum_particle_surface_penetration_m"
                ]
                maximum_left_finger_soft_contacts = max(
                    maximum_left_finger_soft_contacts, left_soft_contacts
                )
                maximum_right_finger_soft_contacts = max(
                    maximum_right_finger_soft_contacts, right_soft_contacts
                )
                left_strip_contacts = right_strip_contacts = 0
                left_loaded_contacts = right_loaded_contacts = 0
                left_contact_force = right_contact_force = 0.0
                left_force_abs_xyz = np.zeros(3, dtype=np.float64)
                right_force_abs_xyz = np.zeros(3, dtype=np.float64)
                left_force_on_strip_xyz = np.zeros(3, dtype=np.float64)
                right_force_on_strip_xyz = np.zeros(3, dtype=np.float64)
                current_contact_z_min = current_contact_z_max = None
            else:
                soft_count = left_soft_contacts = right_soft_contacts = 0
                left_loaded_soft_contacts = right_loaded_soft_contacts = 0
                current_soft_activation_depth = 0.0
                current_particle_surface_penetration = 0.0
                (
                    left_strip_contacts, right_strip_contacts,
                    left_loaded_contacts, right_loaded_contacts,
                    left_contact_force, right_contact_force,
                    left_force_abs_xyz, right_force_abs_xyz,
                    left_force_on_strip_xyz, right_force_on_strip_xyz,
                    current_contact_z_min, current_contact_z_max,
                ) = finger_strip_contacts(contacts, count, poses)
            maximum_soft_contact_activation_depth = max(
                maximum_soft_contact_activation_depth, current_soft_activation_depth
            )
            maximum_particle_surface_penetration = max(
                maximum_particle_surface_penetration,
                current_particle_surface_penetration,
            )
            grip = float(latest_robot_q[6])
            arm_step = float(np.max(np.abs(latest_robot_q[:6] - previous_sample_arm_q)))
            current_mimic_error = mimic_error(latest_robot_q)
            maximum_mimic_error = max(maximum_mimic_error, current_mimic_error)

            if GRIPPER_CLOSED_MIN < grip <= GRIPPER_CLOSED_MAX and not release_seen:
                closed_seen = True
                if FEM_MODE and not lift_started:
                    maximum_closed_left_loaded_soft_contacts = max(
                        maximum_closed_left_loaded_soft_contacts,
                        left_loaded_soft_contacts,
                    )
                    maximum_closed_right_loaded_soft_contacts = max(
                        maximum_closed_right_loaded_soft_contacts,
                        right_loaded_soft_contacts,
                    )
                    bilateral_closed_soft_contact_samples += int(
                        left_loaded_soft_contacts > 0
                        and right_loaded_soft_contacts > 0
                    )
                if not FEM_MODE and not lift_started:
                    maximum_closed_left_loaded_rigid_contacts = max(
                        maximum_closed_left_loaded_rigid_contacts,
                        left_loaded_contacts,
                    )
                    maximum_closed_right_loaded_rigid_contacts = max(
                        maximum_closed_right_loaded_rigid_contacts,
                        right_loaded_contacts,
                    )
                    bilateral_closed_rigid_contact_samples += int(
                        left_loaded_contacts > 0 and right_loaded_contacts > 0
                    )
                maximum_left_force_abs_xyz = np.maximum(
                    maximum_left_force_abs_xyz, left_force_abs_xyz
                )
                maximum_right_force_abs_xyz = np.maximum(
                    maximum_right_force_abs_xyz, right_force_abs_xyz
                )
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
                if FEM_MODE and lift_started:
                    maximum_lift_left_loaded_soft_contacts = max(
                        maximum_lift_left_loaded_soft_contacts,
                        left_loaded_soft_contacts,
                    )
                    maximum_lift_right_loaded_soft_contacts = max(
                        maximum_lift_right_loaded_soft_contacts,
                        right_loaded_soft_contacts,
                    )
                    bilateral_lift_soft_contact_samples += int(
                        left_loaded_soft_contacts > 0
                        and right_loaded_soft_contacts > 0
                    )
                if not FEM_MODE and lift_started:
                    maximum_lift_left_loaded_rigid_contacts = max(
                        maximum_lift_left_loaded_rigid_contacts,
                        left_loaded_contacts,
                    )
                    maximum_lift_right_loaded_rigid_contacts = max(
                        maximum_lift_right_loaded_rigid_contacts,
                        right_loaded_contacts,
                    )
                    bilateral_lift_rigid_contact_samples += int(
                        left_loaded_contacts > 0 and right_loaded_contacts > 0
                    )
                if (lift_started or arm_step > 1.0e-4) and (
                    left_loaded_contacts > 0 and right_loaded_contacts > 0
                ):
                    lift_left_force_on_strip_sum_xyz += left_force_on_strip_xyz
                    lift_right_force_on_strip_sum_xyz += right_force_on_strip_xyz
                    lift_force_sample_count += 1
                    combined_force_z = float(
                        left_force_on_strip_xyz[2] + right_force_on_strip_xyz[2]
                    )
                    minimum_lift_combined_force_z = min(
                        minimum_lift_combined_force_z, combined_force_z
                    )
                    maximum_lift_combined_force_z = max(
                        maximum_lift_combined_force_z, combined_force_z
                    )
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
                "object": OBJECT_MODEL, "position_m": [float(v) for v in center],
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
                "left_contact_force_abs_xyz_n": [
                    float(value) for value in left_force_abs_xyz
                ],
                "right_contact_force_abs_xyz_n": [
                    float(value) for value in right_force_abs_xyz
                ],
                "left_force_on_strip_xyz_n": [
                    float(value) for value in left_force_on_strip_xyz
                ],
                "right_force_on_strip_xyz_n": [
                    float(value) for value in right_force_on_strip_xyz
                ],
                "closed_seen": closed_seen,
                "release_seen": release_seen,
                "last_command_id": last_command_id,
                "pending_robot_commands": (
                    len(pending_robot_q) + int(command_segment_active)
                ),
            }
            state_socket.sendto(json.dumps(message).encode("utf-8"), STATE_ADDRESS)
            if grip >= 0.20 and phase == "MOVEIT_CONTACT":
                print(
                    "MOVEIT_CONTACT_SAMPLE",
                    json.dumps({
                        "sim_time_s": sim_time,
                        "gripper_rad": grip,
                        "arm_step_rad": arm_step,
                        "object_center_z_m": float(center[2]),
                        "grasp_region_z_m": grasp_z,
                        "finger_midpoint_xyz_m": [
                            float(value) for value in finger_midpoint
                        ],
                        "finger_separation_m": finger_separation,
                        "object_from_fingers_xyz_m": [
                            float(value) for value in object_from_fingers
                        ],
                        "arm_joint_positions_rad": [
                            float(value) for value in latest_robot_q[:6]
                        ],
                        "left_loaded_contacts": left_loaded_contacts,
                        "right_loaded_contacts": right_loaded_contacts,
                        "soft_contacts_total": soft_count,
                        "left_finger_soft_contacts": left_soft_contacts,
                        "right_finger_soft_contacts": right_soft_contacts,
                        "left_force_abs_xyz_n": [
                            float(value) for value in left_force_abs_xyz
                        ],
                        "right_force_abs_xyz_n": [
                            float(value) for value in right_force_abs_xyz
                        ],
                        "left_force_on_strip_xyz_n": [
                            float(value) for value in left_force_on_strip_xyz
                        ],
                        "right_force_on_strip_xyz_n": [
                            float(value) for value in right_force_on_strip_xyz
                        ],
                        "contact_z_min_m": current_contact_z_min,
                        "contact_z_max_m": current_contact_z_max,
                        "pending_robot_commands": (
                            len(pending_robot_q) + int(command_segment_active)
                        ),
                    }),
                    flush=True,
                )
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
                f"- FEM soft contacts (total/left/right): "
                f"**{soft_count}/{left_soft_contacts}/{right_soft_contacts}**\n"
                f"- Contact-force magnitude sum (left/right): "
                f"**{left_contact_force:.3f}/{right_contact_force:.3f}**\n"
                f"- Contact |Fx|/|Fy|/|Fz| left: "
                f"**{left_force_abs_xyz[0]:.3f}/{left_force_abs_xyz[1]:.3f}/"
                f"{left_force_abs_xyz[2]:.3f} N**\n"
                f"- Contact |Fx|/|Fy|/|Fz| right: "
                f"**{right_force_abs_xyz[0]:.3f}/{right_force_abs_xyz[1]:.3f}/"
                f"{right_force_abs_xyz[2]:.3f} N**\n"
                f"- Closed loaded-contact z range: "
                f"**{minimum_closed_contact_z if math.isfinite(minimum_closed_contact_z) else float('nan'):.4f}/"
                f"{maximum_closed_contact_z if math.isfinite(maximum_closed_contact_z) else float('nan'):.4f} m**\n"
                f"- Contact candidates: **{count}** (maximum {maximum_contacts})\n"
                f"- Pending robot commands: "
                f"**{len(pending_robot_q) + int(command_segment_active)}** "
                f"(maximum {maximum_command_backlog})\n"
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
        "object": OBJECT_MODEL,
        "device": str(model.device),
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
        "maximum_soft_contact_count": maximum_soft_contacts,
        "maximum_left_finger_soft_contacts": maximum_left_finger_soft_contacts,
        "maximum_right_finger_soft_contacts": maximum_right_finger_soft_contacts,
        "maximum_closed_left_loaded_soft_contacts": maximum_closed_left_loaded_soft_contacts,
        "maximum_closed_right_loaded_soft_contacts": maximum_closed_right_loaded_soft_contacts,
        "maximum_lift_left_loaded_soft_contacts": maximum_lift_left_loaded_soft_contacts,
        "maximum_lift_right_loaded_soft_contacts": maximum_lift_right_loaded_soft_contacts,
        "bilateral_closed_soft_contact_samples": bilateral_closed_soft_contact_samples,
        "bilateral_lift_soft_contact_samples": bilateral_lift_soft_contact_samples,
        "maximum_closed_left_loaded_rigid_contacts": maximum_closed_left_loaded_rigid_contacts,
        "maximum_closed_right_loaded_rigid_contacts": maximum_closed_right_loaded_rigid_contacts,
        "maximum_lift_left_loaded_rigid_contacts": maximum_lift_left_loaded_rigid_contacts,
        "maximum_lift_right_loaded_rigid_contacts": maximum_lift_right_loaded_rigid_contacts,
        "bilateral_closed_rigid_contact_samples": bilateral_closed_rigid_contact_samples,
        "bilateral_lift_rigid_contact_samples": bilateral_lift_rigid_contact_samples,
        "maximum_soft_contact_activation_depth_m": maximum_soft_contact_activation_depth,
        "maximum_particle_surface_penetration_m": maximum_particle_surface_penetration,
        "maximum_floor_penetration_m": max(0.0, -minimum_strip_bottom_z),
        "maximum_allowed_penetration_m": MAX_ALLOWED_PENETRATION,
        "command_queue_capacity": MAX_COMMAND_QUEUE,
        "command_queue_overflow_count": command_queue_overflow_count,
        "maximum_command_backlog": maximum_command_backlog,
        "pending_robot_commands_at_shutdown": (
            len(pending_robot_q) + int(command_segment_active)
        ),
        "maximum_left_force_abs_xyz_n": [
            float(value) for value in maximum_left_force_abs_xyz
        ],
        "maximum_right_force_abs_xyz_n": [
            float(value) for value in maximum_right_force_abs_xyz
        ],
        "lift_bilateral_force_sample_count": lift_force_sample_count,
        "mean_lift_left_force_on_strip_xyz_n": [
            float(value / lift_force_sample_count)
            if lift_force_sample_count else 0.0
            for value in lift_left_force_on_strip_sum_xyz
        ],
        "mean_lift_right_force_on_strip_xyz_n": [
            float(value / lift_force_sample_count)
            if lift_force_sample_count else 0.0
            for value in lift_right_force_on_strip_sum_xyz
        ],
        "mean_lift_combined_force_on_strip_xyz_n": [
            float(value / lift_force_sample_count)
            if lift_force_sample_count else 0.0
            for value in (
                lift_left_force_on_strip_sum_xyz
                + lift_right_force_on_strip_sum_xyz
            )
        ],
        "minimum_lift_combined_force_z_n": (
            minimum_lift_combined_force_z
            if math.isfinite(minimum_lift_combined_force_z) else None
        ),
        "maximum_lift_combined_force_z_n": (
            maximum_lift_combined_force_z
            if math.isfinite(maximum_lift_combined_force_z) else None
        ),
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
            and minimum_strip_bottom_z >= -MAX_ALLOWED_PENETRATION
            and maximum_particle_surface_penetration <= MAX_ALLOWED_PENETRATION
            and (
                (
                    FEM_MODE
                    and maximum_closed_left_loaded_soft_contacts > 0
                    and maximum_closed_right_loaded_soft_contacts > 0
                    and maximum_lift_left_loaded_soft_contacts > 0
                    and maximum_lift_right_loaded_soft_contacts > 0
                    and bilateral_lift_soft_contact_samples > 0
                )
                or (
                    ROD_MODE
                    and maximum_closed_left_loaded_rigid_contacts > 0
                    and maximum_closed_right_loaded_rigid_contacts > 0
                    and maximum_lift_left_loaded_rigid_contacts > 0
                    and maximum_lift_right_loaded_rigid_contacts > 0
                    and bilateral_lift_rigid_contact_samples > 0
                )
                or (
                    not FEM_MODE
                    and not ROD_MODE
                )
            )
            and command_queue_overflow_count == 0
            and maximum_mimic_error <= 1.0e-5
            and finite_state
        ),
    }
    print("MOVEIT_GRASP_RESULT", json.dumps(result), flush=True)
    viewer.close()
    command_socket.close()
    state_socket.close()
