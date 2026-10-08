#!/usr/bin/env python3
"""Build and inspect a nearly inextensible Newton rod cable."""

import json
import math

import newton
import warp as wp


LENGTH_M = 0.40
DIAMETER_M = 0.006
RADIUS_M = DIAMETER_M / 2.0
SEGMENTS = 40
SEGMENT_LENGTH_M = LENGTH_M / SEGMENTS
MATERIAL_DENSITY_KG_M3 = 1100.0

# Provisional cable-joint parameters.  They deliberately decouple axial and
# transverse rigidity from bending and twist; they are not measured material
# properties yet.
STRETCH_STIFFNESS_N_M = 1.0e6
STRETCH_DAMPING_N_S_M = 30.0
SHEAR_STIFFNESS_N_M = 1.0e6
SHEAR_DAMPING_N_S_M = 30.0
BEND_STIFFNESS_N_M_RAD = 0.01
BEND_DAMPING_N_M_S_RAD = 0.04
TWIST_STIFFNESS_N_M_RAD = 0.005
TWIST_DAMPING_N_M_S_RAD = 0.02

CONTACT_KE_N_M = 5.0e4
CONTACT_KD_N_S_M = 500.0
CONTACT_FRICTION = 1.0
CONTACT_GAP_M = 0.001


def cylinder_volume():
    """Physical cable volume represented by its centerline and radius."""
    return math.pi * RADIUS_M**2 * LENGTH_M


def capsule_chain_volume():
    """Sum of overlapping capsule volumes used by add_rod for mass creation."""
    cylinder_per_segment = math.pi * RADIUS_M**2 * SEGMENT_LENGTH_M
    spherical_caps_per_segment = 4.0 / 3.0 * math.pi * RADIUS_M**3
    return SEGMENTS * (cylinder_per_segment + spherical_caps_per_segment)


def corrected_capsule_density():
    """Preserve rho*pi*r^2*L despite overlapping hemispherical end caps."""
    return (
        MATERIAL_DENSITY_KG_M3
        * cylinder_volume()
        / capsule_chain_volume()
    )


def expected_total_mass():
    return MATERIAL_DENSITY_KG_M3 * cylinder_volume()


def add_rod_cable(
    builder,
    start,
    direction=wp.vec3(1.0, 0.0, 0.0),
    *,
    damping_scale=1.0,
    fix_root=False,
    label="rod_cable",
):
    """Add the validated rod cable to an existing Newton scene builder."""
    if damping_scale <= 0.0:
        raise ValueError("damping_scale must be greater than zero")
    points = newton.utils.create_straight_cable_points(
        start=start,
        direction=direction,
        length=LENGTH_M,
        num_segments=SEGMENTS,
    )
    quaternions = newton.utils.create_parallel_transport_cable_quaternions(points)
    cable_cfg = newton.ModelBuilder.ShapeConfig(
        density=corrected_capsule_density(),
        ke=CONTACT_KE_N_M,
        kd=CONTACT_KD_N_S_M,
        mu=CONTACT_FRICTION,
        restitution=0.0,
        margin=0.0,
        gap=CONTACT_GAP_M,
    )
    bodies, joints = builder.add_rod(
        positions=points,
        quaternions=quaternions,
        radius=RADIUS_M,
        cfg=cable_cfg,
        stretch_stiffness=STRETCH_STIFFNESS_N_M,
        stretch_damping=STRETCH_DAMPING_N_S_M * damping_scale,
        shear_stiffness=SHEAR_STIFFNESS_N_M,
        shear_damping=SHEAR_DAMPING_N_S_M * damping_scale,
        bend_stiffness=BEND_STIFFNESS_N_M_RAD,
        bend_damping=BEND_DAMPING_N_M_S_RAD * damping_scale,
        twist_stiffness=TWIST_STIFFNESS_N_M_RAD,
        twist_damping=TWIST_DAMPING_N_M_S_RAD * damping_scale,
        label=label,
        body_frame_origin="com",
    )

    if fix_root:
        # A kinematic root fixes only the first rigid segment.  The remaining
        # joints retain independent bend and twist degrees of freedom.
        builder.body_flags[bodies[0]] = int(newton.BodyFlags.KINEMATIC)

    return list(map(int, bodies)), list(map(int, joints))


def build_rod_builder(include_ground=False, damping_scale=1.0):
    builder = newton.ModelBuilder()
    bodies, joints = add_rod_cable(
        builder,
        wp.vec3(0.0, 0.0, 0.50),
        damping_scale=damping_scale,
        fix_root=True,
        label="cantilever_rod_cable",
    )

    ground_shape = None
    if include_ground:
        ground_cfg = newton.ModelBuilder.ShapeConfig(
            density=0.0,
            ke=CONTACT_KE_N_M,
            kd=CONTACT_KD_N_S_M,
            mu=CONTACT_FRICTION,
            restitution=0.0,
            margin=0.0,
            gap=CONTACT_GAP_M,
        )
        ground_shape = builder.add_ground_plane(
            cfg=ground_cfg, label="rod_cable_ground"
        )

    return builder, bodies, joints, ground_shape


def main():
    builder, bodies, joints, _ground_shape = build_rod_builder()
    actual_total_mass = sum(float(builder.body_mass[index]) for index in bodies)
    actual_dynamic_mass = sum(float(builder.body_mass[index]) for index in bodies[1:])
    target_total_mass = expected_total_mass()
    target_dynamic_mass = target_total_mass * (SEGMENTS - 1) / SEGMENTS

    config = {
        "length_m": LENGTH_M,
        "diameter_m": DIAMETER_M,
        "segments": SEGMENTS,
        "segment_length_m": SEGMENT_LENGTH_M,
        "material_density_kg_m3": MATERIAL_DENSITY_KG_M3,
        "newton_capsule_density_kg_m3": corrected_capsule_density(),
        "stretch_stiffness_n_m": STRETCH_STIFFNESS_N_M,
        "stretch_damping_n_s_m": STRETCH_DAMPING_N_S_M,
        "shear_stiffness_n_m": SHEAR_STIFFNESS_N_M,
        "shear_damping_n_s_m": SHEAR_DAMPING_N_S_M,
        "bend_stiffness_n_m_rad": BEND_STIFFNESS_N_M_RAD,
        "bend_damping_n_m_s_rad": BEND_DAMPING_N_M_S_RAD,
        "twist_stiffness_n_m_rad": TWIST_STIFFNESS_N_M_RAD,
        "twist_damping_n_m_s_rad": TWIST_DAMPING_N_M_S_RAD,
        "contact_gap_m": CONTACT_GAP_M,
    }
    expected = {
        "bodies": SEGMENTS,
        "joints": SEGMENTS - 1,
        "shapes": SEGMENTS,
        "total_mass_kg": target_total_mass,
        "dynamic_mass_kg": target_dynamic_mass,
    }
    actual = {
        "bodies": len(bodies),
        "joints": len(joints),
        "shapes": builder.shape_count,
        "total_mass_kg": actual_total_mass,
        "dynamic_mass_kg": actual_dynamic_mass,
    }
    checks = {
        key: (
            abs(actual[key] - expected[key]) < 1.0e-8
            if key.endswith("_kg")
            else actual[key] == expected[key]
        )
        for key in expected
    }

    print("ROD_CABLE_CONFIG", json.dumps(config))
    print("ROD_CABLE_EXPECTED", json.dumps(expected))
    print("ROD_CABLE_ACTUAL", json.dumps(actual))
    print("ROD_CABLE_CHECKS", json.dumps(checks))


if __name__ == "__main__":
    main()
