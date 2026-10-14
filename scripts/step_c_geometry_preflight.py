#!/usr/bin/env python3
"""Measure the proposed Step C wall against the actual Newton collision model."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import numpy as np


JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def quaternion_rotate(quaternion, vector):
    xyz = quaternion[:3]
    return vector + 2.0 * np.cross(
        xyz, np.cross(xyz, vector) + quaternion[3] * vector
    )


def compose_transform(parent, child):
    return np.concatenate((
        parent[:3] + quaternion_rotate(parent[3:], child[:3]),
        quaternion_multiply(parent[3:], child[3:]),
    ))


def transform_points(transform, points):
    return np.asarray([
        transform[:3] + quaternion_rotate(transform[3:], point)
        for point in points
    ])


def box_corners(half_extents):
    return np.asarray([
        [x, y, z]
        for x in (-half_extents[0], half_extents[0])
        for y in (-half_extents[1], half_extents[1])
        for z in (-half_extents[2], half_extents[2])
    ])


def parse_initial_positions(path):
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, value = line.split(":", 1)
        values[name.strip()] = float(value.strip())
    return np.asarray([values[name] for name in JOINT_ORDER], dtype=np.float32)


def aabb_separation(lower_a, upper_a, lower_b, upper_b):
    return np.maximum(
        0.0, np.maximum(lower_b - upper_a, lower_a - upper_b)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--profile", default="step_c_cable_wall")
    parser.add_argument("--padding", type=float, default=0.02)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--tested-commit", required=True)
    args = parser.parse_args()

    config = json.loads(
        (args.repo / "config/obstacles.yaml").read_text(encoding="utf-8")
    )
    profile = config["scene_profiles"][args.profile]
    wall = next(box for box in config["boxes"] if box["id"] == profile["box_id"])
    cable_center = np.asarray(profile["object_center_m"], dtype=np.float64)
    wall_center = np.asarray(wall["center_m"], dtype=np.float64)
    wall_size = np.asarray(wall["size_m"], dtype=np.float64)
    wall_lower = wall_center - wall_size / 2.0
    wall_upper = wall_center + wall_size / 2.0

    os.environ.update({
        "GRASP_DEVICE": os.environ.get("GRASP_DEVICE", "cpu"),
        "GRASP_OBJECT_MODEL": "fem_cable",
        "GRASP_OBJECT_LENGTH": "0.40",
        "GRASP_OBJECT_WIDTH": "0.006",
        "GRASP_OBJECT_THICKNESS": "0.006",
        "GRASP_STRIP_CENTER_Z": "0.004",
        "GRASP_OBJECT_CENTER_X": str(cable_center[0]),
        "GRASP_OBJECT_CENTER_Y": str(cable_center[1]),
        "GRASP_OBJECT_CENTER_Z": str(cable_center[2]),
        "GRASP_POSITION": "center",
        "GRASP_FEM_CELLS_X": "40",
        "GRASP_FEM_CELLS_Y": "2",
        "GRASP_FEM_CELLS_Z": "2",
    })
    lesson_dir = Path.home() / "newton_ws/lessons/segmented_strip"
    sys.path.insert(0, str(lesson_dir))
    import newton  # pylint: disable=import-outside-toplevel
    import robot_segmented_grasp_batch as scene  # pylint: disable=import-outside-toplevel

    model, *_ = scene.build_scene(obstacles=[])
    q = model.joint_q.numpy()
    q[:6] = parse_initial_positions(
        args.repo / "config/jazzy_fem_initial_positions.yaml"
    )
    q[6:] = 0.0
    jazzy_initial_q = q[:6].copy()
    model.joint_q.assign(q)
    state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)

    body_transforms = state.body_q.numpy()
    shape_transforms = model.shape_transform.numpy()
    shape_scales = model.shape_scale.numpy()
    shape_bodies = model.shape_body.numpy()
    shape_flags = model.shape_flags.numpy()
    collide_shapes = int(newton.ShapeFlags.COLLIDE_SHAPES)

    def world_vertices(shape_index, transforms):
        source = model.shape_source[shape_index]
        scale = shape_scales[shape_index]
        if source is not None and hasattr(source, "vertices"):
            points = np.asarray(source.vertices, dtype=np.float64) * scale
        else:
            points = box_corners(scale)
        body_index = int(shape_bodies[shape_index])
        transform = shape_transforms[shape_index]
        if body_index >= 0:
            transform = compose_transform(
                transforms[body_index], transform
            )
        return transform_points(transform, points)

    def robot_wall_measurements(transforms):
        measurements = []
        for shape_index in range(model.shape_count):
            body_index = int(shape_bodies[shape_index])
            if body_index < 0 or body_index >= 24:
                continue
            if not (int(shape_flags[shape_index]) & collide_shapes):
                continue
            vertices = world_vertices(shape_index, transforms)
            separation = aabb_separation(
                vertices.min(axis=0), vertices.max(axis=0),
                wall_lower, wall_upper,
            )
            measurements.append({
                "distance_m": float(np.linalg.norm(separation)),
                "separation_xyz_m": separation.astype(float).tolist(),
                "link": model.body_label[body_index],
                "shape_index": shape_index,
            })
        return measurements

    robot_vertices = []
    finger_vertices = []
    fingertip_vertices = []
    for index in range(model.shape_count):
        body_index = int(shape_bodies[index])
        if body_index < 0 or body_index >= 24:
            continue
        if not (int(shape_flags[index]) & collide_shapes):
            continue
        vertices = world_vertices(index, body_transforms)
        robot_vertices.append(vertices)
        body_label = model.body_label[body_index]
        if "finger_link" in body_label or "finger_tip_link" in body_label:
            finger_vertices.append(vertices)
        if "finger_tip_link" in body_label:
            fingertip_vertices.append(vertices)

    robot_vertices = np.concatenate(robot_vertices)
    finger_vertices = np.concatenate(finger_vertices)
    fingertip_vertices = np.concatenate(fingertip_vertices)
    tool_index = next(
        index for index, label in enumerate(model.body_label)
        if label.endswith("/tool0") or label == "tool0"
    )
    tool_position = body_transforms[tool_index, :3].astype(np.float64)
    finger_lower = finger_vertices.min(axis=0)
    finger_upper = finger_vertices.max(axis=0)
    finger_outer_half_width = float(np.max(np.abs(
        finger_vertices[:, 1] - tool_position[1]
    )))
    start_measurements = robot_wall_measurements(body_transforms)
    closest_start_shape = min(
        start_measurements, key=lambda measurement: measurement["distance_m"]
    )
    minimum_start_distance = closest_start_shape["distance_m"]
    padded_start_clearance = minimum_start_distance - args.padding

    base_q = model.joint_q.numpy()
    base_q[:6] = scene.BASE_ARM
    base_q[6:] = 0.0
    model.joint_q.assign(base_q)
    base_state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, base_state)
    base_measurements = robot_wall_measurements(base_state.body_q.numpy())
    closest_base_shape = min(
        base_measurements, key=lambda measurement: measurement["distance_m"]
    )
    minimum_base_arm_distance = closest_base_shape["distance_m"]

    axis_names = ("x", "y", "z")
    closest_start_separation_axes = [
        axis_names[index]
        for index, value in enumerate(closest_start_shape["separation_xyz_m"])
        if value > 1.0e-12
    ]
    closest_start_separation_direction = "/".join(
        closest_start_separation_axes
    ) or "overlap"

    cable_radius = 0.003
    # Measure toward the wall face that faces the cable, on either side.
    if cable_center[1] < wall_lower[1]:
        cable_to_face_y = float(wall_lower[1] - cable_center[1])
    else:
        cable_to_face_y = float(cable_center[1] - wall_upper[1])
    cable_surface_gap = cable_to_face_y - cable_radius
    open_finger_clearance = cable_to_face_y - finger_outer_half_width

    # Sweep the actual open-finger collision AABB from the unchanged start pose
    # to the proposed cable center.  An overlap proves a straight transfer is
    # invalid; MoveIt reachability is checked separately by plan-only.
    horizontal_target = np.asarray([
        cable_center[0], cable_center[1], tool_position[2]
    ])
    translation = horizontal_target - tool_position
    sweep_lower = np.minimum(finger_lower, finger_lower + translation)
    sweep_upper = np.maximum(finger_upper, finger_upper + translation)
    padded_wall_lower = wall_lower - args.padding
    padded_wall_upper = wall_upper + args.padding
    straight_sweep_intersects_wall = bool(np.all(
        sweep_upper >= padded_wall_lower
    ) and np.all(sweep_lower <= padded_wall_upper))

    checks = {
        "start_pose_clearance_exceeds_padding": minimum_start_distance > args.padding,
        "straight_transfer_would_hit_wall": straight_sweep_intersects_wall,
        "open_finger_clearance_at_least_3cm": open_finger_clearance >= 0.03,
        "newton_initial_base_arm_collision_free": minimum_base_arm_distance > 0.0,
        "finite_geometry": bool(np.isfinite(robot_vertices).all()),
    }
    result = {
        "step": "C_geometry_preflight_collision_model",
        "tested_commit": args.tested_commit,
        "scene_profile": args.profile,
        "robot_padding_m": args.padding,
        "jazzy_initial_positions_rad": jazzy_initial_q.astype(float).tolist(),
        "newton_base_arm_rad": scene.BASE_ARM.astype(float).tolist(),
        "tool_start_xyz_m": tool_position.tolist(),
        "finger_collision_aabb_min_m": finger_lower.tolist(),
        "finger_collision_aabb_max_m": finger_upper.tolist(),
        "finger_outer_half_width_m": finger_outer_half_width,
        "fingertip_lowest_z_m": float(fingertip_vertices[:, 2].min()),
        "minimum_start_robot_wall_distance_m": minimum_start_distance,
        "closest_start_robot_shape_link": closest_start_shape["link"],
        "closest_start_robot_shape_index": closest_start_shape["shape_index"],
        "closest_start_robot_wall_separation_xyz_m": (
            closest_start_shape["separation_xyz_m"]
        ),
        "closest_start_robot_wall_separation_axes": closest_start_separation_axes,
        "closest_start_robot_wall_separation_direction": (
            closest_start_separation_direction
        ),
        "padded_start_clearance_m": padded_start_clearance,
        "minimum_newton_base_arm_wall_distance_m": minimum_base_arm_distance,
        "closest_newton_base_arm_shape_link": closest_base_shape["link"],
        "robot_obstacle_contact_counting_policy": (
            "count_from_simulation_start_base_arm_verified_collision_free"
            if minimum_base_arm_distance > 0.0
            else "invalid_base_arm_requires_post_sync_gating"
        ),
        "cable_center_m": cable_center.tolist(),
        "cable_axis": "x",
        "wall_center_m": wall_center.tolist(),
        "wall_size_m": wall_size.tolist(),
        "cable_surface_to_wall_face_gap_m": cable_surface_gap,
        "open_finger_to_wall_face_clearance_m": open_finger_clearance,
        "padded_open_finger_to_wall_clearance_m": open_finger_clearance - args.padding,
        "straight_sweep_aabb_min_m": sweep_lower.tolist(),
        "straight_sweep_aabb_max_m": sweep_upper.tolist(),
        "checks": checks,
        "geometry_pass": all(checks.values()),
    }
    print("STEP_C_GEOMETRY_RESULT", json.dumps(result, sort_keys=True))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return 0 if result["geometry_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
