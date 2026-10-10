#!/usr/bin/env python3
"""Headless contact proof: kinematic UR5/Robotiq lifts a dynamic segmented strip."""

import json
import math
import os
from pathlib import Path
import time

import newton
import numpy as np
import warp as wp

from rod_cable_topology import (
    CONTACT_FRICTION as CABLE_CONTACT_FRICTION,
    CONTACT_GAP_M as CABLE_CONTACT_GAP,
    CONTACT_KD_N_S_M as CABLE_CONTACT_KD,
    CONTACT_KE_N_M as CABLE_CONTACT_KE,
    LENGTH_M as CABLE_LENGTH,
    RADIUS_M as CABLE_RADIUS,
    add_rod_cable,
)


URDF = os.environ.get(
    "NEWTON_ROBOT_URDF",
    str(Path.home() / "newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf"),
)
DEVICE = os.environ.get("GRASP_DEVICE", "cpu")
OBJECT_MODEL = os.environ.get("GRASP_OBJECT_MODEL", "segmented_strip")
if OBJECT_MODEL not in {
    "segmented_strip", "rigid_block", "fem_strip", "fem_cable", "rod_cable"
}:
    raise ValueError(
        "GRASP_OBJECT_MODEL must be 'segmented_strip', 'rigid_block', "
        "'fem_strip', 'fem_cable', or 'rod_cable'"
    )
FEM_MODE = OBJECT_MODEL in {"fem_strip", "fem_cable"}
LENGTH = float(os.environ.get("GRASP_OBJECT_LENGTH", "0.40"))
WIDTH = float(
    os.environ.get("GRASP_OBJECT_WIDTH", "0.006" if OBJECT_MODEL == "fem_cable" else "0.05")
)
THICKNESS = float(
    os.environ.get(
        "GRASP_OBJECT_THICKNESS", "0.006" if OBJECT_MODEL == "fem_cable" else "0.02"
    )
)
if LENGTH <= 0.0 or WIDTH <= 0.0 or THICKNESS <= 0.0:
    raise ValueError("FEM object dimensions must be greater than zero")
DENSITY = float(os.environ.get("GRASP_DENSITY", "1100.0"))
if DENSITY <= 0.0:
    raise ValueError("GRASP_DENSITY must be greater than zero")
YOUNGS_MODULUS = float(os.environ.get("GRASP_YOUNGS_MODULUS", "1000000.0"))
if YOUNGS_MODULUS <= 0.0:
    raise ValueError("GRASP_YOUNGS_MODULUS must be greater than zero")
SEGMENTS = 20
FRAME_DT = 1.0 / 60.0
SUBSTEPS = int(os.environ.get("GRASP_SUBSTEPS", "10"))
if SUBSTEPS <= 0:
    raise ValueError("GRASP_SUBSTEPS must be greater than zero")
DT = FRAME_DT / SUBSTEPS
BASE_ARM = np.array(
    [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
)
LIFT_ARM = np.array(
    [0.0, -1.52621116, 1.21416760, -1.25875297, -1.57079627, 0.0]
)
CLOSED_GRIP = float(os.environ.get("GRASP_CLOSED_GRIP", "0.78"))
CONTACT_FRICTION = float(os.environ.get("GRASP_FRICTION", "1.5"))
STRIP_FRICTION = float(
    os.environ.get("GRASP_STRIP_FRICTION", str(CONTACT_FRICTION))
)
GROUND_FRICTION = float(
    os.environ.get("GRASP_GROUND_FRICTION", str(CONTACT_FRICTION))
)
SHOW_COLLIDERS = os.environ.get("GRASP_SHOW_COLLIDERS", "0") == "1"
COLLISION_SCOPE = os.environ.get("GRASP_COLLISION_SCOPE", "all")
STRIP_CENTER_Z = float(os.environ.get("GRASP_STRIP_CENTER_Z", "0.34"))
GRASP_POSITION = os.environ.get("GRASP_POSITION", "near_end")
OBJECT_HALF_THICKNESS = CABLE_RADIUS if OBJECT_MODEL == "rod_cable" else THICKNESS / 2.0
POISSON_RATIO = float(os.environ.get("GRASP_POISSON_RATIO", "0.45"))
if not 0.0 <= POISSON_RATIO < 0.5:
    raise ValueError("GRASP_POISSON_RATIO must be in [0, 0.5)")
SOFT_DAMPING = float(os.environ.get("GRASP_SOFT_DAMPING", "1000.0"))
if SOFT_DAMPING < 0.0:
    raise ValueError("GRASP_SOFT_DAMPING must be zero or greater")
SOFT_CONTACT_KE = float(os.environ.get("GRASP_SOFT_CONTACT_KE", "1000.0"))
SOFT_CONTACT_KD = float(os.environ.get("GRASP_SOFT_CONTACT_KD", "10.0"))
FEM_PROXY_INSET = float(os.environ.get("GRASP_FEM_PROXY_INSET", "0.001"))
SHOW_FEM_PROXIES = os.environ.get("GRASP_SHOW_FEM_PROXIES", "0") == "1"
CONTACT_PROXY_INSET = float(
    os.environ.get(
        "GRASP_CONTACT_PROXY_INSET",
        "0.0" if OBJECT_MODEL == "rod_cable" else str(FEM_PROXY_INSET),
    )
)
SHOW_CONTACT_PROXIES = (
    os.environ.get(
        "GRASP_SHOW_CONTACT_PROXIES",
        os.environ.get("GRASP_SHOW_FEM_PROXIES", "0"),
    )
    == "1"
)
if SOFT_CONTACT_KE <= 0.0 or SOFT_CONTACT_KD < 0.0:
    raise ValueError("FEM contact stiffness must be positive and damping nonnegative")
if FEM_PROXY_INSET < 0.0:
    raise ValueError("GRASP_FEM_PROXY_INSET must be nonnegative")
if CONTACT_PROXY_INSET < 0.0:
    raise ValueError("GRASP_CONTACT_PROXY_INSET must be nonnegative")
COMMAND_LIFT_Z = 0.12
CONTACT_HOLD_DURATION = 1.0
LIFT_DURATION = float(os.environ.get("GRASP_LIFT_DURATION", "2.0"))
if LIFT_DURATION <= 0.0:
    raise ValueError("GRASP_LIFT_DURATION must be greater than zero")
LIFT_HOLD_DURATION = float(os.environ.get("GRASP_LIFT_HOLD_DURATION", "1.0"))
if LIFT_HOLD_DURATION < 0.0:
    raise ValueError("GRASP_LIFT_HOLD_DURATION must be zero or greater")
DANCE_MODE = os.environ.get("GRASP_MOTION_MODE", "lift_only")
DANCE_DURATION = float(os.environ.get("GRASP_DANCE_DURATION", "4.0"))
DANCE_CYCLES = float(os.environ.get("GRASP_DANCE_CYCLES", "2.0"))
DANCE_SHOULDER_AMPLITUDE = float(
    os.environ.get("GRASP_DANCE_SHOULDER_AMPLITUDE", "0.18")
)
DANCE_WRIST_AMPLITUDE = float(
    os.environ.get("GRASP_DANCE_WRIST_AMPLITUDE", "0.35")
)
if DANCE_MODE not in {"lift_only", "dance"}:
    raise ValueError("GRASP_MOTION_MODE must be 'lift_only' or 'dance'")
if DANCE_DURATION <= 0.0:
    raise ValueError("GRASP_DANCE_DURATION must be greater than zero")
OPEN_DURATION = 1.0
RELEASE_DURATION = 1.0
CONTACT_END = CONTACT_HOLD_DURATION
LIFT_END = CONTACT_END + LIFT_DURATION
LIFT_HOLD_END = LIFT_END + LIFT_HOLD_DURATION
DANCE_END = LIFT_HOLD_END + (DANCE_DURATION if DANCE_MODE == "dance" else 0.0)
OPEN_END = DANCE_END + OPEN_DURATION
TEST_DURATION = OPEN_END + RELEASE_DURATION


def smoothstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3.0 - 2.0 * x)


def grasp_region_links(strip_links):
    if GRASP_POSITION == "center":
        middle = len(strip_links) // 2
        return strip_links[middle - 1 : middle + 1]
    return strip_links[:1]


def robot_coordinates(t):
    if t < CONTACT_END:
        arm = BASE_ARM
        grip = CLOSED_GRIP
        phase = "CONTACT_HOLD"
    elif t < LIFT_END:
        s = smoothstep((t - CONTACT_END) / LIFT_DURATION)
        arm = BASE_ARM + s * (LIFT_ARM - BASE_ARM)
        grip = CLOSED_GRIP
        phase = "LIFT"
    elif t < LIFT_HOLD_END:
        arm = LIFT_ARM
        grip = CLOSED_GRIP
        phase = "LIFT_HOLD"
    elif DANCE_MODE == "dance" and t < DANCE_END:
        u = (t - LIFT_HOLD_END) / DANCE_DURATION
        envelope = math.sin(math.pi * u) ** 2
        arm = LIFT_ARM.copy()
        arm[0] += (
            DANCE_SHOULDER_AMPLITUDE
            * envelope
            * math.sin(2.0 * math.pi * DANCE_CYCLES * u)
        )
        arm[5] += (
            DANCE_WRIST_AMPLITUDE
            * envelope
            * math.sin(2.0 * math.pi * DANCE_CYCLES * u + math.pi / 2.0)
        )
        grip = CLOSED_GRIP
        phase = "DANCE"
    elif t < OPEN_END:
        arm = LIFT_ARM
        grip = CLOSED_GRIP * (
            1.0 - smoothstep((t - DANCE_END) / OPEN_DURATION)
        )
        phase = "OPEN"
    else:
        arm = LIFT_ARM
        grip = 0.0
        phase = "RELEASE"
    q = np.zeros(12, dtype=np.float32)
    q[:6] = arm
    q[6] = grip
    q[8] = -grip
    q[10] = grip
    q[11] = -grip
    q[7] = -grip
    q[9] = grip
    return q, phase


def build_scene():
    newton.use_coord_layout_targets = True
    wp.init()
    wp.set_device(DEVICE)
    builder = newton.ModelBuilder()
    builder.add_urdf(
        URDF,
        floating=False,
        enable_self_collisions=False,
        hide_visuals=SHOW_COLLIDERS,
        force_show_colliders=SHOW_COLLIDERS,
    )
    robot_bodies = builder.body_count
    robot_shapes = builder.shape_count
    for index in range(robot_bodies):
        builder.body_flags[index] = newton.BodyFlags.KINEMATIC
    finger_bodies = {
        index
        for index, label in enumerate(builder.body_label[:robot_bodies])
        if "finger_link" in label or "finger_tip_link" in label
    }
    active_robot_colliders = []
    fem_contact_proxies = []
    rod_contact_proxies = []
    for shape_index in range(robot_shapes):
        body_index = builder.shape_body[shape_index]
        is_collider = bool(
            builder.shape_flags[shape_index] & newton.ShapeFlags.COLLIDE_SHAPES
        )
        enabled = is_collider and (
            COLLISION_SCOPE == "all" or body_index in finger_bodies
        )
        if enabled:
            builder.shape_material_mu[shape_index] = CONTACT_FRICTION
            use_contact_proxy = (
                body_index in finger_bodies
                and (FEM_MODE or OBJECT_MODEL == "rod_cable")
            )
            if use_contact_proxy:
                if FEM_MODE:
                    # Keep the imported shape for rigid contacts, but route
                    # VBD particle contact through the analytic proxy.
                    active_robot_colliders.append(shape_index)
                    builder.shape_flags[shape_index] &= ~(
                        newton.ShapeFlags.COLLIDE_PARTICLES
                    )
                else:
                    # Capsule-chain cable contact is rigid-rigid.  Disable the
                    # detailed finger mesh and use a convex analytic box so
                    # both distance and contact normals remain well defined.
                    builder.shape_flags[shape_index] &= ~(
                        newton.ShapeFlags.COLLIDE_SHAPES
                    )

                vertices = np.asarray(
                    builder.shape_source[shape_index].vertices,
                    dtype=np.float32,
                ) * np.asarray(builder.shape_scale[shape_index], dtype=np.float32)
                lower = vertices.min(axis=0)
                upper = vertices.max(axis=0)
                center = 0.5 * (lower + upper)
                half = np.maximum(
                    0.5 * (upper - lower) - CONTACT_PROXY_INSET, 0.001
                )
                source_xform = builder.shape_transform[shape_index]
                proxy_center = wp.transform_point(
                    source_xform, wp.vec3(*center.tolist())
                )
                proxy_rotation = wp.transform_get_rotation(source_xform)
                proxy_cfg_kwargs = dict(
                    density=0.0,
                    mu=CONTACT_FRICTION,
                    restitution=0.0,
                    has_shape_collision=OBJECT_MODEL == "rod_cable",
                    has_particle_collision=FEM_MODE,
                    is_visible=(
                        SHOW_FEM_PROXIES
                        if FEM_MODE
                        else SHOW_CONTACT_PROXIES
                    ),
                )
                if OBJECT_MODEL == "rod_cable":
                    proxy_cfg_kwargs.update(
                        ke=CABLE_CONTACT_KE,
                        kd=CABLE_CONTACT_KD,
                        margin=0.0,
                        gap=CABLE_CONTACT_GAP,
                    )
                proxy_cfg = builder.ShapeConfig(**proxy_cfg_kwargs)
                proxy = builder.add_shape_box(
                    body_index,
                    xform=wp.transform(proxy_center, proxy_rotation),
                    hx=float(half[0]),
                    hy=float(half[1]),
                    hz=float(half[2]),
                    cfg=proxy_cfg,
                    label=(
                        f"fem_contact_proxy_{shape_index}"
                        if FEM_MODE
                        else f"rod_cable_contact_proxy_{shape_index}"
                    ),
                )
                if FEM_MODE:
                    fem_contact_proxies.append(proxy)
                else:
                    rod_contact_proxies.append(proxy)
                    active_robot_colliders.append(proxy)
            else:
                active_robot_colliders.append(shape_index)
                if FEM_MODE:
                    builder.shape_flags[shape_index] &= ~(
                        newton.ShapeFlags.COLLIDE_PARTICLES
                    )
        elif is_collider:
            builder.shape_flags[shape_index] &= ~(
                newton.ShapeFlags.COLLIDE_SHAPES
                | newton.ShapeFlags.COLLIDE_PARTICLES
            )

    # The cable must collide with the proxies, but the proxies must not collide
    # with their own robot or with one another.  Cable shapes are created later,
    # so they intentionally are not part of these filter pairs.
    for proxy in rod_contact_proxies:
        for robot_shape in range(robot_shapes):
            builder.add_shape_collision_filter_pair(proxy, robot_shape)
    for proxy_index, proxy_a in enumerate(rod_contact_proxies):
        for proxy_b in rod_contact_proxies[proxy_index + 1 :]:
            builder.add_shape_collision_filter_pair(proxy_a, proxy_b)

    # Include hidden contact proxies in the robot-shape range returned to
    # diagnostics.  Modes without proxies retain the imported shape count.
    robot_shapes = builder.shape_count

    grasp_point = np.array([0.4869, 0.10915, STRIP_CENTER_Z])
    if GRASP_POSITION == "center":
        start_x = grasp_point[0] - LENGTH / 2.0
    elif GRASP_POSITION == "near_end":
        start_x = grasp_point[0]
    else:
        raise ValueError("GRASP_POSITION must be 'center' or 'near_end'")
    shape_cfg = builder.ShapeConfig(
        density=DENSITY,
        ke=5.0e4,
        kd=500.0,
        mu=STRIP_FRICTION,
        restitution=0.0,
        collision_filter_parent=True,
        has_shape_collision=True,
    )
    links = []
    shapes = []
    if FEM_MODE:
        default_cells = (40, 2, 2) if OBJECT_MODEL == "fem_cable" else (20, 3, 2)
        cells_x = int(os.environ.get("GRASP_FEM_CELLS_X", str(default_cells[0])))
        cells_y = int(os.environ.get("GRASP_FEM_CELLS_Y", str(default_cells[1])))
        cells_z = int(os.environ.get("GRASP_FEM_CELLS_Z", str(default_cells[2])))
        if cells_x <= 0 or cells_y <= 0 or cells_z <= 0:
            raise ValueError("FEM cell counts must be positive")
        particle_count = (cells_x + 1) * (cells_y + 1) * (cells_z + 1)
        cell_count = cells_x * cells_y * cells_z
        # add_soft_grid assigns one cell-volume mass to every grid node.  Scale
        # the input density so the nodal mass sum still equals rho * volume.
        grid_density = DENSITY * cell_count / particle_count
        k_mu = YOUNGS_MODULUS / (2.0 * (1.0 + POISSON_RATIO))
        k_lambda = (
            YOUNGS_MODULUS
            * POISSON_RATIO
            / ((1.0 + POISSON_RATIO) * (1.0 - 2.0 * POISSON_RATIO))
        )
        builder.add_soft_grid(
            pos=wp.vec3(
                grasp_point[0] - LENGTH / 2.0,
                grasp_point[1] - WIDTH / 2.0,
                STRIP_CENTER_Z - THICKNESS / 2.0,
            ),
            rot=wp.quat_identity(),
            vel=wp.vec3(0.0, 0.0, 0.0),
            dim_x=cells_x,
            dim_y=cells_y,
            dim_z=cells_z,
            cell_x=LENGTH / cells_x,
            cell_y=WIDTH / cells_y,
            cell_z=THICKNESS / cells_z,
            density=grid_density,
            k_mu=k_mu,
            k_lambda=k_lambda,
            k_damp=SOFT_DAMPING,
            fix_left=False,
            particle_radius=0.001,
            label=(
                "grasp_fem_rubber_cable"
                if OBJECT_MODEL == "fem_cable"
                else "grasp_fem_rubber_strip"
            ),
        )
        links = []
        shapes = []
    elif OBJECT_MODEL == "rod_cable":
        shape_start = builder.shape_count
        links, _cable_joints = add_rod_cable(
            builder,
            wp.vec3(
                grasp_point[0] - CABLE_LENGTH / 2.0,
                grasp_point[1],
                grasp_point[2],
            ),
            damping_scale=1.0,
            fix_root=False,
            label="grasp_rod_cable",
        )
        shapes = list(range(shape_start, builder.shape_count))
    elif OBJECT_MODEL == "rigid_block":
        link = builder.add_link(
            xform=wp.transform(
                p=wp.vec3(grasp_point[0], grasp_point[1], grasp_point[2]),
                q=wp.quat_identity(),
            ),
            label="grasp_rigid_block",
        )
        shape = builder.add_shape_box(
            link,
            hx=LENGTH / 2.0,
            hy=WIDTH / 2.0,
            hz=THICKNESS / 2.0,
            cfg=shape_cfg,
            color=wp.vec3(0.15, 0.55, 0.85),
            label="grasp_rigid_block_shape",
        )
        links = [link]
        shapes = [shape]
        joints = [builder.add_joint_free(child=link, label="free_rigid_block")]
        builder.add_articulation(joints, label="free_rigid_block")
    else:
        segment_length = LENGTH / SEGMENTS
        for index in range(SEGMENTS):
            link = builder.add_link(
                xform=wp.transform(
                    p=wp.vec3(
                        start_x + (index + 0.5) * segment_length,
                        grasp_point[1],
                        grasp_point[2],
                    ),
                    q=wp.quat_identity(),
                ),
                label=f"grasp_strip_segment_{index:02d}",
            )
            shape = builder.add_shape_box(
                link,
                hx=segment_length / 2.0,
                hy=WIDTH / 2.0,
                hz=THICKNESS / 2.0,
                cfg=shape_cfg,
                color=wp.vec3(0.15 + 0.03 * (index % 4), 0.55, 0.85),
                label=f"grasp_strip_shape_{index:02d}",
            )
            links.append(link)
            shapes.append(shape)

        joints = [builder.add_joint_free(child=links[0], label="free_strip_root")]
        inertia = WIDTH * THICKNESS**3 / 12.0
        stiffness = YOUNGS_MODULUS * inertia / segment_length
        damping = 0.02
        for index in range(1, SEGMENTS):
            joints.append(
                builder.add_joint_revolute(
                    parent=links[index - 1],
                    child=links[index],
                    axis=wp.vec3(0.0, 1.0, 0.0),
                    parent_xform=wp.transform(
                        p=wp.vec3(segment_length / 2.0, 0.0, 0.0),
                        q=wp.quat_identity(),
                    ),
                    child_xform=wp.transform(
                        p=wp.vec3(-segment_length / 2.0, 0.0, 0.0),
                        q=wp.quat_identity(),
                    ),
                    target_pos=0.0,
                    target_vel=0.0,
                    target_ke=stiffness,
                    target_kd=damping,
                    limit_lower=-math.pi,
                    limit_upper=math.pi,
                    actuator_mode=newton.JointTargetMode.POSITION,
                    label=f"grasp_strip_hinge_{index:02d}",
                )
            )
        builder.add_articulation(joints, label="free_segmented_strip")

    if OBJECT_MODEL != "rod_cable":
        for a, shape_a in enumerate(shapes):
            for shape_b in shapes[a + 1 :]:
                builder.add_shape_collision_filter_pair(shape_a, shape_b)

    if OBJECT_MODEL == "rod_cable":
        ground_cfg = builder.ShapeConfig(
            density=0.0,
            ke=CABLE_CONTACT_KE,
            kd=CABLE_CONTACT_KD,
            mu=CABLE_CONTACT_FRICTION,
            restitution=0.0,
            margin=0.0,
            gap=CABLE_CONTACT_GAP,
        )
        ground_shape = builder.add_ground_plane(cfg=ground_cfg)
    else:
        ground_shape = builder.add_ground_plane()
        builder.shape_material_mu[ground_shape] = GROUND_FRICTION
    for robot_shape in active_robot_colliders:
        builder.add_shape_collision_filter_pair(robot_shape, ground_shape)

    if FEM_MODE:
        builder.color()
    model = builder.finalize()
    if FEM_MODE:
        model.soft_contact_ke = SOFT_CONTACT_KE
        model.soft_contact_kd = SOFT_CONTACT_KD
        model.soft_contact_mu = STRIP_FRICTION
    return (
        model,
        links,
        robot_bodies,
        robot_shapes,
        shapes,
        len(active_robot_colliders),
    )


def main():
    (
        model,
        strip_links,
        robot_bodies,
        robot_shapes,
        strip_shapes,
        active_robot_colliders,
    ) = build_scene()
    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    solver = newton.solvers.SolverXPBD(model, iterations=30)
    collision = newton.CollisionPipeline(model, rigid_contact_max=10000)
    contacts = collision.contacts()

    q = model.joint_q.numpy()
    qd = model.joint_qd.numpy()
    robot_q, _ = robot_coordinates(0.0)
    q[:12] = robot_q
    model.joint_q.assign(q)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_0)
    initial_center_z = float(state_0.body_q.numpy()[strip_links, 2].mean())
    grip_links = grasp_region_links(strip_links)
    initial_grasp_region_z = float(state_0.body_q.numpy()[grip_links, 2].mean())

    maximum_center_z = initial_center_z
    maximum_contacts = 0
    contact_frames = 0
    phase_max_z = {}
    phase_end_z = {}
    phase_end_grasp_region_z = {}
    contacted_robot_bodies = set()
    shape_bodies = model.shape_body.numpy()
    previous_robot_q = robot_q.copy()
    wall_start = time.monotonic()
    sim_time = 0.0
    while sim_time < TEST_DURATION:
        for _ in range(SUBSTEPS):
            next_time = sim_time + DT
            robot_q, phase = robot_coordinates(next_time)
            q[:12] = robot_q
            qd[:12] = (robot_q - previous_robot_q) / DT
            previous_robot_q = robot_q.copy()
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
            contact_frames += int(count > 0)
            solver.step(state_0, state_1, control, contacts, DT)
            state_0, state_1 = state_1, state_0
            sim_time = next_time
        center_z = float(state_0.body_q.numpy()[strip_links, 2].mean())
        grasp_region_z = float(state_0.body_q.numpy()[grip_links, 2].mean())
        maximum_center_z = max(maximum_center_z, center_z)
        phase_max_z[phase] = max(phase_max_z.get(phase, -1e9), center_z)
        phase_end_z[phase] = center_z
        phase_end_grasp_region_z[phase] = grasp_region_z
        count = int(contacts.rigid_contact_count.numpy()[0])
        shape_0 = contacts.rigid_contact_shape0.numpy()[:count]
        shape_1 = contacts.rigid_contact_shape1.numpy()[:count]
        for first, second in zip(shape_0, shape_1):
            body_0 = int(shape_bodies[int(first)])
            body_1 = int(shape_bodies[int(second)])
            if 0 <= body_0 < robot_bodies and body_1 in strip_links:
                contacted_robot_bodies.add(model.body_label[body_0])
            if 0 <= body_1 < robot_bodies and body_0 in strip_links:
                contacted_robot_bodies.add(model.body_label[body_1])

    final_poses = state_0.body_q.numpy()
    final_center_z = float(final_poses[strip_links, 2].mean())
    final_grasp_region_z = float(final_poses[grip_links, 2].mean())
    result = {
        "robot_bodies": robot_bodies,
        "robot_shapes": robot_shapes,
        "active_robot_colliders": active_robot_colliders,
        "contact_friction": CONTACT_FRICTION,
        "motion_mode": DANCE_MODE,
        "lift_duration_s": LIFT_DURATION,
        "lift_hold_duration_s": LIFT_HOLD_DURATION,
        "estimated_peak_lift_speed_m_s": 1.5 * COMMAND_LIFT_Z / LIFT_DURATION,
        "estimated_peak_lift_acceleration_m_s2": 6.0 * COMMAND_LIFT_Z / LIFT_DURATION**2,
        "closed_grip_rad": CLOSED_GRIP,
        "dance_duration_s": DANCE_DURATION if DANCE_MODE == "dance" else 0.0,
        "dance_cycles": DANCE_CYCLES if DANCE_MODE == "dance" else 0.0,
        "dance_shoulder_amplitude_rad": (
            DANCE_SHOULDER_AMPLITUDE if DANCE_MODE == "dance" else 0.0
        ),
        "dance_wrist_amplitude_rad": (
            DANCE_WRIST_AMPLITUDE if DANCE_MODE == "dance" else 0.0
        ),
        "collision_scope": COLLISION_SCOPE,
        "strip_initial_center_z_setting_m": STRIP_CENTER_Z,
        "grasp_location": GRASP_POSITION,
        "strip_bodies": len(strip_links),
        "strip_shapes": len(strip_shapes),
        "initial_center_z_m": initial_center_z,
        "initial_grasp_region_z_m": initial_grasp_region_z,
        "maximum_center_z_m": maximum_center_z,
        "lift_m": maximum_center_z - initial_center_z,
        "final_center_z_m": final_center_z,
        "drop_after_peak_m": maximum_center_z - final_center_z,
        "lift_hold_end_z_m": phase_end_z.get(
            "LIFT_HOLD", phase_end_z.get("LIFT")
        ),
        "grasp_region_contact_hold_end_z_m": phase_end_grasp_region_z.get("CONTACT_HOLD"),
        "grasp_region_lift_hold_end_z_m": phase_end_grasp_region_z.get(
            "LIFT_HOLD", phase_end_grasp_region_z.get("LIFT")
        ),
        "final_grasp_region_z_m": final_grasp_region_z,
        "maximum_contact_count": maximum_contacts,
        "contact_substeps": contact_frames,
        "contacted_robot_bodies": sorted(contacted_robot_bodies),
        "finite_state": bool(np.isfinite(final_poses).all()),
        "wall_elapsed_s": time.monotonic() - wall_start,
    }
    result["prelift_grasp_region_settling_m"] = (
        initial_grasp_region_z - result["grasp_region_contact_hold_end_z_m"]
    )
    result["grasp_region_lift_rise_m"] = (
        result["grasp_region_lift_hold_end_z_m"]
        - result["grasp_region_contact_hold_end_z_m"]
    )
    result["commanded_lift_m"] = COMMAND_LIFT_Z
    result["lift_tracking_error_m"] = abs(
        result["grasp_region_lift_rise_m"] - COMMAND_LIFT_Z
    )
    result["release_drop_from_lift_hold_m"] = (
        result["grasp_region_lift_hold_end_z_m"] - final_grasp_region_z
    )
    post_motion_z = phase_end_grasp_region_z.get(
        "DANCE", result["grasp_region_lift_hold_end_z_m"]
    )
    result["grasp_region_post_motion_z_m"] = post_motion_z
    result["motion_retention_error_m"] = abs(
        post_motion_z - result["grasp_region_lift_hold_end_z_m"]
    )
    result["motion_retention_pass"] = bool(result["motion_retention_error_m"] < 0.02)
    result["contact_lift_release_pass"] = bool(
        result["lift_tracking_error_m"] < 0.02
        and result["motion_retention_pass"]
        and result["release_drop_from_lift_hold_m"] > 0.05
        and result["finite_state"]
    )
    print("PHASE_MAX_Z", json.dumps(phase_max_z))
    print("PHASE_END_Z", json.dumps(phase_end_z))
    print("PHASE_END_GRASP_REGION_Z", json.dumps(phase_end_grasp_region_z))
    print("RESULT", json.dumps(result))


if __name__ == "__main__":
    main()
