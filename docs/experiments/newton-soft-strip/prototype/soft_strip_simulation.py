#!/usr/bin/env python3
"""Run and measure a gravity-loaded cantilever rubber strip in Newton."""

import argparse
import functools
import json
import time

import newton
import numpy as np
import viser
import warp as wp
from newton.viewer import ViewerViser

from soft_strip_topology import build_builder


FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
SIM_DT = FRAME_DT / SUBSTEPS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cells-x", type=int, default=20)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--damping", type=float, default=100.0)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--port", type=int, default=8084)
    args = parser.parse_args()
    if (
        args.cells_x < 1
        or args.duration <= 0.0
        or args.start_delay < 0.0
        or args.iterations < 1
    ):
        parser.error("cells, duration, and iterations must be positive")

    builder = build_builder(args.cells_x, damping_pa_s=args.damping)
    initial = np.array(
        [[float(p[axis]) for axis in range(3)] for p in builder.particle_q],
        dtype=np.float64,
    )
    masses = np.array([float(mass) for mass in builder.particle_mass])
    tip_indices = np.flatnonzero(np.isclose(initial[:, 0], initial[:, 0].max()))
    fixed_indices = np.flatnonzero(masses == 0.0)
    initial_tip_z = float(initial[tip_indices, 2].mean())

    builder.add_ground_plane()
    builder.color()
    model = builder.finalize()
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
            label="Newton FEM Rubber Strip — Gravity Test",
            share=False,
        )
    finally:
        viser.ViserServer = server_class
    viewer.set_model(model)
    server = viewer._server
    server.scene.set_up_direction("+z")
    server.initial_camera.position = (0.55, -0.55, 0.78)
    server.initial_camera.look_at = (0.20, 0.025, 0.42)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.gui.add_markdown(
        "## Gravity-loaded FEM rubber strip\n"
        "The left face is fixed. The displayed value is the mean vertical "
        "displacement of the free-end particles."
    )
    status = server.gui.add_markdown("Preparing...")

    sim_time = 0.0
    result_printed = False
    tip_history = []
    last_recorded_time = -1.0
    physics_wall_start = None
    wall_start = time.monotonic()
    next_wall_frame = time.monotonic()
    print(
        f"SOFT_STRIP_SIM_READY cells_x={args.cells_x} dt={SIM_DT:.9f} "
        f"substeps={SUBSTEPS} iterations={args.iterations} "
        f"start_delay={args.start_delay}",
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
                f"- Simulation time: **{sim_time:.2f} s**\n"
                f"- Mean free-tip z: **{tip_z:.5f} m**\n"
                f"- Downward tip deflection: **{tip_deflection:.5f} m**\n"
                f"- Maximum fixed-end error: **{fixed_error:.3e} m**\n"
                f"- Finite particle state: **{finite}**"
            )

            viewer.begin_frame(sim_time)
            viewer.log_state(state_0)
            viewer.log_contacts(contacts, state_0)
            viewer.end_frame()

            if phase == "COMPLETE" and not result_printed:
                tail = [
                    z for sample_time, z in tip_history
                    if sample_time >= args.duration - 1.0
                ]
                tail_range = max(tail) - min(tail)
                wall_elapsed = time.monotonic() - physics_wall_start
                result = {
                    "cells_x": args.cells_x,
                    "duration_s": sim_time,
                    "wall_elapsed_s": wall_elapsed,
                    "real_time_factor": sim_time / wall_elapsed,
                    "dt_s": SIM_DT,
                    "iterations": args.iterations,
                    "damping_pa_s": args.damping,
                    "mean_tip_z_m": tip_z,
                    "downward_tip_deflection_m": tip_deflection,
                    "last_1s_tip_z_range_m": tail_range,
                    "max_fixed_end_error_m": fixed_error,
                    "finite_state": finite,
                }
                print("RESULT", json.dumps(result), flush=True)
                result_printed = True

            next_wall_frame += FRAME_DT
            time.sleep(max(0.0, next_wall_frame - time.monotonic()))
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
