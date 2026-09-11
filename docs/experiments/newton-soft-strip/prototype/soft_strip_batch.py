#!/usr/bin/env python3
"""Headless settling test for the Newton FEM cantilever strip."""

import argparse
import json
import time

import newton
import numpy as np

from soft_strip_topology import build_builder


FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
SIM_DT = FRAME_DT / SUBSTEPS
WINDOW_S = 1.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cells-x", type=int, default=20)
    parser.add_argument("--damping", type=float, default=1000.0)
    parser.add_argument("--iterations", type=int, default=10)
    parser.add_argument("--min-duration", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=60.0)
    parser.add_argument("--settle-threshold", type=float, default=0.004)
    args = parser.parse_args()
    if (
        args.cells_x < 1
        or args.iterations < 1
        or args.min_duration < WINDOW_S
        or args.max_duration < args.min_duration
        or args.settle_threshold <= 0.0
    ):
        parser.error("invalid cells, iterations, duration, or threshold")

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

    print(
        "BATCH_READY "
        + json.dumps(
            {
                "cells_x": args.cells_x,
                "damping_pa_s": args.damping,
                "dt_s": SIM_DT,
                "iterations": args.iterations,
                "settle_threshold_m": args.settle_threshold,
                "max_duration_s": args.max_duration,
            }
        ),
        flush=True,
    )

    sim_time = 0.0
    history = []
    settled = False
    finite = True
    tail = []
    wall_start = time.monotonic()

    while sim_time < args.max_duration:
        for _ in range(SUBSTEPS):
            state_0.clear_forces()
            collision_pipeline.collide(state_0, contacts)
            solver.step(state_0, state_1, control, contacts, SIM_DT)
            state_0, state_1 = state_1, state_0

        sim_time = min(args.max_duration, sim_time + FRAME_DT)
        positions = state_0.particle_q.numpy()
        finite = bool(np.isfinite(positions).all())
        if not finite:
            break

        tip_z = float(positions[tip_indices, 2].mean())
        history.append((sim_time, tip_z))
        window_start = sim_time - WINDOW_S
        tail = [z for sample_time, z in history if sample_time >= window_start]
        full_window = len(tail) >= int(0.9 * WINDOW_S / FRAME_DT)
        if (
            sim_time >= args.min_duration
            and full_window
            and max(tail) - min(tail) < args.settle_threshold
        ):
            settled = True
            break

    wall_elapsed = time.monotonic() - wall_start
    positions = state_0.particle_q.numpy()
    tip_z = float(positions[tip_indices, 2].mean())
    fixed_error = float(
        np.linalg.norm(positions[fixed_indices] - initial[fixed_indices], axis=1).max()
    )
    tail_range = max(tail) - min(tail) if tail else float("nan")
    tail_mean_z = float(np.mean(tail)) if tail else float("nan")
    result = {
        "cells_x": args.cells_x,
        "settled": settled,
        "settling_time_s": sim_time if settled else None,
        "simulated_time_s": sim_time,
        "wall_elapsed_s": wall_elapsed,
        "real_time_factor": sim_time / wall_elapsed,
        "damping_pa_s": args.damping,
        "mean_tip_z_last_1s_m": tail_mean_z,
        "mean_downward_deflection_last_1s_m": initial_tip_z - tail_mean_z,
        "last_1s_tip_z_range_m": tail_range,
        "max_fixed_end_error_m": fixed_error,
        "finite_state": finite,
    }
    print("RESULT " + json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
