#!/usr/bin/env python3
"""Simulate a damped, nearly inextensible cantilever rod cable on CUDA."""

import argparse
import functools
import json
import time

import newton
import numpy as np
import viser
import warp as wp
from newton.viewer import ViewerViser

from rod_cable_topology import (
    LENGTH_M,
    RADIUS_M,
    SEGMENTS,
    SEGMENT_LENGTH_M,
    build_rod_builder,
    expected_total_mass,
)


FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
SIM_DT = FRAME_DT / SUBSTEPS
SETTLED_RANGE_M = 0.01 * LENGTH_M
MAXIMUM_ACCEPTED_EXTENSION_RATIO = 0.01


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a damped, nearly inextensible rod cable on CUDA."
    )
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument(
        "--damping-scale",
        type=float,
        default=1.0,
        help="multiply all rod damping terms without changing stiffness",
    )
    parser.add_argument("--port", type=int, default=8085)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    if (
        args.duration <= 0.0
        or args.start_delay < 0.0
        or args.iterations < 1
        or args.damping_scale <= 0.0
    ):
        parser.error("duration and iterations must be positive")
    return args


def select_cuda_device(device_name):
    wp.init()
    device = wp.get_device(device_name)
    if not device.is_cuda:
        raise RuntimeError(
            f"Refusing to run the rod cable solver on non-CUDA device: {device}"
        )
    wp.set_device(device)
    print(f"ROD_CABLE_DEVICE device={device} is_cuda={device.is_cuda}", flush=True)
    return device


def rotate_vector(quaternion_xyzw, vector):
    q_xyz = np.asarray(quaternion_xyzw[:3], dtype=np.float64)
    q_w = float(quaternion_xyzw[3])
    vector = np.asarray(vector, dtype=np.float64)
    twice_cross = 2.0 * np.cross(q_xyz, vector)
    return vector + q_w * twice_cross + np.cross(q_xyz, twice_cross)


def segment_endpoints(body_poses, bodies):
    starts = []
    ends = []
    local_half_segment = np.array(
        [0.0, 0.0, 0.5 * SEGMENT_LENGTH_M], dtype=np.float64
    )
    for body in bodies:
        pose = body_poses[body]
        center = np.asarray(pose[:3], dtype=np.float64)
        half_segment = rotate_vector(pose[3:7], local_half_segment)
        starts.append(center - half_segment)
        ends.append(center + half_segment)
    return np.asarray(starts), np.asarray(ends)


def cable_metrics(body_poses, bodies):
    starts, ends = segment_endpoints(body_poses, bodies)
    joint_gaps = np.linalg.norm(starts[1:] - ends[:-1], axis=1)
    effective_length = SEGMENTS * SEGMENT_LENGTH_M + float(joint_gaps.sum())
    extension_ratio = effective_length / LENGTH_M - 1.0
    minimum_surface_z = float(min(starts[:, 2].min(), ends[:, 2].min()) - RADIUS_M)
    return {
        "tip_z": float(ends[-1, 2]),
        "effective_length": effective_length,
        "extension_ratio": extension_ratio,
        "maximum_joint_gap": float(joint_gaps.max(initial=0.0)),
        "minimum_surface_z": minimum_surface_z,
    }


def main():
    args = parse_args()
    device = select_cuda_device(args.device)
    builder, bodies, _joints, ground_shape = build_rod_builder(
        include_ground=True, damping_scale=args.damping_scale
    )
    if ground_shape is None:
        raise RuntimeError("Rod cable ground shape was not created")
    builder.color()
    model = builder.finalize(device=device)
    if not model.device.is_cuda:
        raise RuntimeError(f"Newton model was not allocated on CUDA: {model.device}")

    solver = newton.solvers.SolverVBD(model=model, iterations=args.iterations)
    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    collision_pipeline = newton.CollisionPipeline(
        model, rigid_contact_max=10000
    )
    contacts = collision_pipeline.contacts()

    initial_poses = state_0.body_q.numpy()
    initial_metrics = cable_metrics(initial_poses, bodies)
    initial_root_pose = initial_poses[bodies[0]].copy()
    body_masses = model.body_mass.numpy()
    actual_total_mass = float(body_masses[bodies].sum())
    actual_dynamic_mass = float(body_masses[bodies[1:]].sum())
    target_total_mass = expected_total_mass()

    server_class = viser.ViserServer
    viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
    try:
        viewer = ViewerViser(
            port=args.port,
            label="Newton Rod Cable — CUDA Cantilever Test",
            share=False,
        )
    finally:
        viser.ViserServer = server_class
    viewer.set_model(model)
    server = viewer._server
    server.scene.set_up_direction("+z")
    server.initial_camera.position = (0.55, -0.55, 0.78)
    server.initial_camera.look_at = (0.20, 0.0, 0.38)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.gui.add_markdown(
        "## Nearly inextensible rod cable (CUDA)\n"
        "Capsule segments use stiff stretch/shear joints and independent "
        "soft bend/twist joints. The first segment is kinematic."
    )
    status = server.gui.add_markdown("Preparing...")

    sim_time = 0.0
    wall_start = time.monotonic()
    physics_wall_start = None
    next_wall_frame = time.monotonic()
    tip_history = []
    extension_history = []
    last_recorded_time = -1.0
    maximum_extension_ratio = 0.0
    maximum_joint_gap = 0.0
    minimum_surface_z = float("inf")
    maximum_rigid_contacts = 0
    maximum_ground_contacts = 0
    maximum_self_contacts = 0
    result_printed = False

    print(
        "ROD_CABLE_SIM_READY "
        f"device={model.device} segments={SEGMENTS} bodies={len(bodies)} "
        f"expected_mass={target_total_mass:.8f}kg "
        f"actual_mass={actual_total_mass:.8f}kg "
        f"dynamic_mass={actual_dynamic_mass:.8f}kg "
        f"damping_scale={args.damping_scale:.3f} "
        f"dt={SIM_DT:.9f} substeps={SUBSTEPS} iterations={args.iterations}",
        flush=True,
    )

    try:
        while viewer.is_running():
            remaining_delay = max(
                0.0, args.start_delay - (time.monotonic() - wall_start)
            )
            if remaining_delay == 0.0 and physics_wall_start is None:
                physics_wall_start = time.monotonic()
            if remaining_delay == 0.0 and sim_time < args.duration:
                for _ in range(SUBSTEPS):
                    state_0.clear_forces()
                    collision_pipeline.collide(state_0, contacts)
                    solver.step(state_0, state_1, control, contacts, SIM_DT)
                    state_0, state_1 = state_1, state_0
                sim_time = min(args.duration, sim_time + FRAME_DT)

            poses = state_0.body_q.numpy()
            metrics = cable_metrics(poses, bodies)
            finite = bool(np.isfinite(poses[bodies]).all())
            root_position_error = float(
                np.linalg.norm(poses[bodies[0], :3] - initial_root_pose[:3])
            )
            root_orientation_error = float(
                min(
                    np.linalg.norm(poses[bodies[0], 3:7] - initial_root_pose[3:7]),
                    np.linalg.norm(poses[bodies[0], 3:7] + initial_root_pose[3:7]),
                )
            )
            rigid_contacts = int(contacts.rigid_contact_count.numpy()[0])
            shape_0 = contacts.rigid_contact_shape0.numpy()[:rigid_contacts]
            shape_1 = contacts.rigid_contact_shape1.numpy()[:rigid_contacts]
            ground_contacts = int(
                np.count_nonzero((shape_0 == ground_shape) | (shape_1 == ground_shape))
            )
            self_contacts = rigid_contacts - ground_contacts
            maximum_extension_ratio = max(
                maximum_extension_ratio, metrics["extension_ratio"]
            )
            maximum_joint_gap = max(
                maximum_joint_gap, metrics["maximum_joint_gap"]
            )
            minimum_surface_z = min(
                minimum_surface_z, metrics["minimum_surface_z"]
            )
            maximum_rigid_contacts = max(maximum_rigid_contacts, rigid_contacts)
            maximum_ground_contacts = max(maximum_ground_contacts, ground_contacts)
            maximum_self_contacts = max(maximum_self_contacts, self_contacts)
            if sim_time > last_recorded_time:
                tip_history.append((sim_time, metrics["tip_z"]))
                extension_history.append((sim_time, metrics["extension_ratio"]))
                last_recorded_time = sim_time

            if remaining_delay > 0.0:
                phase = f"STARTING IN {remaining_delay:.1f} s"
            elif sim_time < args.duration:
                phase = "RUNNING"
            else:
                phase = "COMPLETE"
            status.content = (
                f"### {phase}\n"
                f"- Device: **{model.device}**\n"
                f"- Simulation time: **{sim_time:.2f} s**\n"
                f"- Tip z: **{metrics['tip_z']:.5f} m**\n"
                f"- Effective length: **{metrics['effective_length']:.6f} m**\n"
                f"- Extension: **{100.0 * metrics['extension_ratio']:.4f}%**\n"
                f"- Maximum joint gap: **{metrics['maximum_joint_gap']:.3e} m**\n"
                f"- Ground/self contacts: **{ground_contacts}/{self_contacts}**\n"
                f"- Finite state: **{finite}**"
            )

            viewer.begin_frame(sim_time)
            viewer.log_state(state_0)
            viewer.log_contacts(contacts, state_0)
            viewer.end_frame()

            if phase == "COMPLETE" and not result_printed:
                tail = [
                    z
                    for sample_time, z in tip_history
                    if sample_time >= args.duration - 1.0
                ]
                tail_extensions = [
                    extension
                    for sample_time, extension in extension_history
                    if sample_time >= args.duration - 1.0
                ]
                tail_range = max(tail) - min(tail)
                last_1s_max_extension = max(tail_extensions)
                one_second_tip_ranges = []
                for window_start in range(int(np.ceil(args.duration))):
                    window_end = min(float(window_start + 1), args.duration)
                    values = [
                        z
                        for sample_time, z in tip_history
                        if window_start <= sample_time <= window_end
                    ]
                    if values:
                        one_second_tip_ranges.append(max(values) - min(values))
                wall_elapsed = time.monotonic() - physics_wall_start
                accepted = bool(
                    finite
                    and root_position_error < 1.0e-8
                    and root_orientation_error < 1.0e-8
                    and last_1s_max_extension < MAXIMUM_ACCEPTED_EXTENSION_RATIO
                    and maximum_ground_contacts == 0
                    and tail_range < SETTLED_RANGE_M
                )
                result = {
                    "device": str(model.device),
                    "segments": SEGMENTS,
                    "expected_total_mass_kg": target_total_mass,
                    "actual_total_mass_kg": actual_total_mass,
                    "actual_dynamic_mass_kg": actual_dynamic_mass,
                    "duration_s": sim_time,
                    "wall_elapsed_s": wall_elapsed,
                    "real_time_factor": sim_time / wall_elapsed,
                    "dt_s": SIM_DT,
                    "iterations": args.iterations,
                    "damping_scale": args.damping_scale,
                    "initial_tip_z_m": initial_metrics["tip_z"],
                    "final_tip_z_m": metrics["tip_z"],
                    "tip_deflection_m": initial_metrics["tip_z"] - metrics["tip_z"],
                    "last_1s_tip_z_range_m": tail_range,
                    "tip_range_by_second_m": one_second_tip_ranges,
                    "settled_range_threshold_m": SETTLED_RANGE_M,
                    "settled": tail_range < SETTLED_RANGE_M,
                    "final_effective_length_m": metrics["effective_length"],
                    "final_extension_ratio": metrics["extension_ratio"],
                    "maximum_extension_ratio": maximum_extension_ratio,
                    "last_1s_max_extension_ratio": last_1s_max_extension,
                    "extension_threshold_ratio": MAXIMUM_ACCEPTED_EXTENSION_RATIO,
                    "maximum_joint_gap_m": maximum_joint_gap,
                    "minimum_surface_z_m": minimum_surface_z,
                    "maximum_rigid_contacts": maximum_rigid_contacts,
                    "maximum_ground_contacts": maximum_ground_contacts,
                    "maximum_self_contacts": maximum_self_contacts,
                    "root_position_error_m": root_position_error,
                    "root_orientation_error": root_orientation_error,
                    "finite_state": finite,
                    "candidate_rod_cable_pass": accepted,
                }
                print("ROD_CABLE_RESULT", json.dumps(result), flush=True)
                result_printed = True

            next_wall_frame += FRAME_DT
            time.sleep(max(0.0, next_wall_frame - time.monotonic()))
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
