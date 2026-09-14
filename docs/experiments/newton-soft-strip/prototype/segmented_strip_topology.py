#!/usr/bin/env python3
"""Build and verify a rigid-segment approximation of the FEM strip."""

import argparse
import json
import math

import newton
import numpy as np
import warp as wp


LENGTH_M = 0.40
WIDTH_M = 0.05
THICKNESS_M = 0.02
YOUNG_MODULUS_PA = 1_000_000.0
DENSITY_KG_M3 = 1100.0
DEFAULT_SEGMENTS = 20


def calculated_properties(segments):
    second_moment_m4 = WIDTH_M * THICKNESS_M**3 / 12.0
    flexural_rigidity_nm2 = YOUNG_MODULUS_PA * second_moment_m4
    segment_length_m = LENGTH_M / segments
    joint_stiffness_nm_rad = flexural_rigidity_nm2 / segment_length_m
    total_mass_kg = DENSITY_KG_M3 * LENGTH_M * WIDTH_M * THICKNESS_M
    return {
        "second_moment_m4": second_moment_m4,
        "flexural_rigidity_nm2": flexural_rigidity_nm2,
        "segment_length_m": segment_length_m,
        "joint_stiffness_nm_rad": joint_stiffness_nm_rad,
        "expected_total_mass_kg": total_mass_kg,
        "expected_segment_mass_kg": total_mass_kg / segments,
    }


def build_model(
    segments,
    joint_damping_nm_s_rad=0.0,
    kinematic_root=True,
    enable_shape_collisions=False,
    add_ground=False,
):
    properties = calculated_properties(segments)
    segment_length = properties["segment_length_m"]
    builder = newton.ModelBuilder()
    shape_cfg = builder.ShapeConfig(
        density=DENSITY_KG_M3,
        collision_filter_parent=True,
        has_shape_collision=enable_shape_collisions,
    )

    links = []
    for index in range(segments):
        initial_xform = None
        if index == 0 and kinematic_root:
            initial_xform = wp.transform(
                p=wp.vec3(
                    segment_length / 2.0,
                    WIDTH_M / 2.0,
                    0.50 + THICKNESS_M / 2.0,
                ),
                q=wp.quat_identity(),
            )
        link = builder.add_link(
            xform=initial_xform,
            is_kinematic=(index == 0 and kinematic_root),
            label=f"strip_segment_{index:02d}",
        )
        builder.add_shape_box(
            link,
            hx=segment_length / 2.0,
            hy=WIDTH_M / 2.0,
            hz=THICKNESS_M / 2.0,
            cfg=shape_cfg,
            label=f"strip_shape_{index:02d}",
        )
        links.append(link)

    joints = []
    if not kinematic_root:
        joints.append(builder.add_joint_fixed(
            parent=-1,
            child=links[0],
            parent_xform=wp.transform(
                p=wp.vec3(0.0, WIDTH_M / 2.0, 0.50 + THICKNESS_M / 2.0),
                q=wp.quat_identity(),
            ),
            child_xform=wp.transform(
                p=wp.vec3(-segment_length / 2.0, 0.0, 0.0),
                q=wp.quat_identity(),
            ),
            label="fixed_left_end",
        ))

    for index in range(1, segments):
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
                target_ke=properties["joint_stiffness_nm_rad"],
                target_kd=joint_damping_nm_s_rad,
                damping=0.0,
                limit_lower=-math.pi,
                limit_upper=math.pi,
                actuator_mode=newton.JointTargetMode.POSITION,
                label=f"bending_hinge_{index:02d}",
            )
        )

    builder.add_articulation(joints, label="segmented_rubber_strip")
    if add_ground:
        builder.add_ground_plane()
    model = builder.finalize()
    state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    return builder, model, state, properties, links, joints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--segments", type=int, default=DEFAULT_SEGMENTS)
    parser.add_argument("--fixed-joint-root", action="store_true")
    args = parser.parse_args()
    if args.segments < 2:
        parser.error("--segments must be at least 2")

    kinematic_root = not args.fixed_joint_root
    builder, model, state, properties, links, joints = build_model(
        args.segments,
        kinematic_root=kinematic_root,
    )
    masses = model.body_mass.numpy()
    centers = state.body_q.numpy()[:, :3]
    length = properties["segment_length_m"]
    expected_centers = np.array(
        [
            [(index + 0.5) * length, WIDTH_M / 2.0, 0.50 + THICKNESS_M / 2.0]
            for index in range(args.segments)
        ]
    )

    actual = {
        "root_mode": "kinematic" if kinematic_root else "fixed_joint",
        "bodies": int(model.body_count),
        "shapes": int(model.shape_count),
        "joints_total": int(model.joint_count),
        "revolute_joints": args.segments - 1,
        "total_mass_kg": float(masses.sum()),
        "min_segment_mass_kg": float(masses.min()),
        "max_segment_mass_kg": float(masses.max()),
        "maximum_initial_center_error_m": float(
            np.linalg.norm(centers - expected_centers, axis=1).max()
        ),
    }
    checks = {
        "body_count": actual["bodies"] == args.segments,
        "shape_count": actual["shapes"] == args.segments,
        "joint_count": actual["joints_total"]
        == (args.segments - 1 if kinematic_root else args.segments),
        "revolute_joint_count": actual["revolute_joints"] == args.segments - 1,
        "total_mass": abs(
            actual["total_mass_kg"] - properties["expected_total_mass_kg"]
        ) < 1.0e-6,
        "equal_segment_masses": (
            actual["max_segment_mass_kg"] - actual["min_segment_mass_kg"]
        ) < 1.0e-9,
        "initial_geometry": actual["maximum_initial_center_error_m"] < 1.0e-6,
    }

    print("CONFIG", json.dumps({"segments": args.segments, **properties}))
    print("ACTUAL", json.dumps(actual))
    print("CHECKS", json.dumps(checks))


if __name__ == "__main__":
    main()
