#!/usr/bin/env python3
"""Run and measure a gravity-loaded cantilever FEM cable on CUDA."""

import argparse
import functools
import json
import time

import newton
import numpy as np
import viser
import warp as wp
from newton.viewer import ViewerViser

from fem_cable_topology import (
    DEFAULT_CELLS_X,
    LENGTH_M,
    PARTICLE_RADIUS_M,
    build_builder,
    expected_physical_mass,
)


FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
SIM_DT = FRAME_DT / SUBSTEPS
SETTLED_RANGE_M = 0.01 * LENGTH_M


def cable_centerline_length(positions, layer_indices):
    """Approximate arc length through the mean point of every X layer."""
    centers = np.array(
        [positions[indices].mean(axis=0) for indices in layer_indices],
        dtype=np.float64,
    )
    return float(np.linalg.norm(np.diff(centers, axis=0), axis=1).sum())


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a gravity-loaded FEM cable on a CUDA device."
    )
    parser.add_argument("--cells-x", type=int, default=DEFAULT_CELLS_X)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--damping", type=float, default=100.0)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--port", type=int, default=8084)
    parser.add_argument(
        "--device",
        default="cuda:0",
        help="Warp CUDA device index, for example cuda:0 (default: cuda:0)",
    )
    args = parser.parse_args()
    if (
        args.cells_x < 1
        or args.duration <= 0.0
        or args.start_delay < 0.0
        or args.iterations < 1
    ):
        parser.error("cells, duration, and iterations must be positive")
    return args


def select_cuda_device(device_name):
    """Select CUDA explicitly so the expensive FEM solve cannot fall back to CPU."""
    wp.init()
    device = wp.get_device(device_name)
    if not device.is_cuda:
        raise RuntimeError(
            f"Refusing to run the FEM cable solver on non-CUDA device: {device}"
        )
    wp.set_device(device)
    print(f"CABLE_DEVICE device={device} is_cuda={device.is_cuda}", flush=True)
    return device


def main():
    args = parse_args()
    device = select_cuda_device(args.device)

    builder = build_builder(args.cells_x, damping_pa_s=args.damping)
    initial = np.array(
        [[float(p[axis]) for axis in range(3)] for p in builder.particle_q],
        dtype=np.float64,
    )
    masses = np.array([float(mass) for mass in builder.particle_mass])
    tip_indices = np.flatnonzero(np.isclose(initial[:, 0], initial[:, 0].max()))
    fixed_indices = np.flatnonzero(masses == 0.0)
    initial_tip_z = float(initial[tip_indices, 2].mean())
    initial_x_layers = np.unique(initial[:, 0])
    layer_indices = [
        np.flatnonzero(np.isclose(initial[:, 0], x))
        for x in initial_x_layers
    ]
    initial_centerline_length = cable_centerline_length(initial, layer_indices)

    builder.add_ground_plane()
    builder.color()
    # Passing the device here is the definitive placement of model arrays and
    # solver work on the selected GPU; wp.set_device alone is only a default.
    model = builder.finalize(device=device)
    if not model.device.is_cuda:
        raise RuntimeError(f"Newton model was not allocated on CUDA: {model.device}")
    actual_mass = float(model.particle_mass.numpy().sum())
    expected_mass = expected_physical_mass()
    model.soft_contact_ke = 1.0e2
    model.soft_contact_kd = 0.0
    model.soft_contact_mu = 1.0

    solver = newton.solvers.SolverVBD(
        model=model,
        iterations=args.iterations,
        particle_enable_self_contact=False,
        particle_enable_tile_solve=False,
    )
    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    collision_pipeline = newton.CollisionPipeline(model)
    contacts = collision_pipeline.contacts()

    server_class = viser.ViserServer
    viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
    try:
        viewer = ViewerViser(
            port=args.port,
            label="Newton FEM Cable — CUDA Gravity Test",
            share=False,
        )
    finally:
        viser.ViserServer = server_class
    viewer.set_model(model)
    server = viewer._server
    server.scene.set_up_direction("+z")
    server.initial_camera.position = (0.55, -0.55, 0.78)
    server.initial_camera.look_at = (0.20, 0.003, 0.42)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.gui.add_markdown(
        "## Gravity-loaded FEM cable (CUDA)\n"
        "The left face is fixed. Python and the viewer use the CPU; "
        "Newton FEM arrays and solver kernels use the selected GPU."
    )
    status = server.gui.add_markdown("Preparing...")

    sim_time = 0.0
    result_printed = False
    tip_history = []
    last_recorded_time = -1.0
    physics_wall_start = None
    maximum_centerline_length = initial_centerline_length
    minimum_particle_surface_z = float("inf")
    maximum_soft_contacts = 0
    wall_start = time.monotonic()
    next_wall_frame = time.monotonic()
    print(
        f"FEM_CABLE_SIM_READY device={model.device} cells_x={args.cells_x} "
        f"particles={model.particle_count} tetrahedra={model.tet_count} "
        f"mass={actual_mass:.8f}kg expected_mass={expected_mass:.8f}kg "
        f"initial_centerline={initial_centerline_length:.6f}m "
        f"dt={SIM_DT:.9f} substeps={SUBSTEPS} "
        f"iterations={args.iterations} start_delay={args.start_delay}",
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

            positions = state_0.particle_q.numpy()
            tip_z = float(positions[tip_indices, 2].mean())
            tip_deflection = initial_tip_z - tip_z
            centerline_length = cable_centerline_length(positions, layer_indices)
            maximum_centerline_length = max(
                maximum_centerline_length, centerline_length
            )
            centerline_strain = (
                centerline_length / initial_centerline_length - 1.0
            )
            minimum_surface_z = float(positions[:, 2].min() - PARTICLE_RADIUS_M)
            minimum_particle_surface_z = min(
                minimum_particle_surface_z, minimum_surface_z
            )
            soft_contacts = int(contacts.soft_contact_count.numpy()[0])
            maximum_soft_contacts = max(maximum_soft_contacts, soft_contacts)
            if sim_time > last_recorded_time:
                tip_history.append((sim_time, tip_z))
                last_recorded_time = sim_time
            fixed_error = float(
                np.linalg.norm(
                    positions[fixed_indices] - initial[fixed_indices], axis=1
                ).max()
            )
            finite = bool(np.isfinite(positions).all())
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
                f"- Mean free-tip z: **{tip_z:.5f} m**\n"
                f"- Downward tip deflection: **{tip_deflection:.5f} m**\n"
                f"- Centerline length: **{centerline_length:.5f} m**\n"
                f"- Centerline strain: **{100.0 * centerline_strain:.2f}%**\n"
                f"- Ground soft contacts: **{soft_contacts}**\n"
                f"- Maximum fixed-end error: **{fixed_error:.3e} m**\n"
                f"- Finite particle state: **{finite}**"
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
                tail_range = max(tail) - min(tail)
                wall_elapsed = time.monotonic() - physics_wall_start
                result = {
                    "device": str(model.device),
                    "cells_x": args.cells_x,
                    "particles": model.particle_count,
                    "tetrahedra": model.tet_count,
                    "expected_mass_kg": expected_mass,
                    "actual_dynamic_mass_kg": actual_mass,
                    "mass_relative_error": abs(actual_mass - expected_mass) / expected_mass,
                    "duration_s": sim_time,
                    "wall_elapsed_s": wall_elapsed,
                    "real_time_factor": sim_time / wall_elapsed,
                    "dt_s": SIM_DT,
                    "iterations": args.iterations,
                    "damping_pa_s": args.damping,
                    "mean_tip_z_m": tip_z,
                    "downward_tip_deflection_m": tip_deflection,
                    "last_1s_tip_z_range_m": tail_range,
                    "settled_range_threshold_m": SETTLED_RANGE_M,
                    "settled": tail_range < SETTLED_RANGE_M,
                    "initial_centerline_length_m": initial_centerline_length,
                    "final_centerline_length_m": centerline_length,
                    "final_centerline_strain": centerline_strain,
                    "maximum_centerline_length_m": maximum_centerline_length,
                    "maximum_centerline_strain": (
                        maximum_centerline_length / initial_centerline_length - 1.0
                    ),
                    "minimum_particle_surface_z_m": minimum_particle_surface_z,
                    "maximum_ground_soft_contacts": maximum_soft_contacts,
                    "max_fixed_end_error_m": fixed_error,
                    "finite_state": finite,
                }
                print("FEM_CABLE_RESULT", json.dumps(result), flush=True)
                result_printed = True

            next_wall_frame += FRAME_DT
            time.sleep(max(0.0, next_wall_frame - time.monotonic()))
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
