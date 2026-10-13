#!/usr/bin/env python3
"""Step A1: drop the production FEM-strip material model onto a static box."""

import argparse
import json
import math
import time
from pathlib import Path

import newton
import numpy as np
import warp as wp

newton.use_coord_layout_targets = True

FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
DT = FRAME_DT / SUBSTEPS


def box_sdf(points: np.ndarray, center: np.ndarray, half: np.ndarray) -> np.ndarray:
    relative = np.abs(points - center) - half
    outside = np.linalg.norm(np.maximum(relative, 0.0), axis=1)
    inside = np.minimum(np.max(relative, axis=1), 0.0)
    return outside + inside


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--enable-obstacles", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--tested-commit", required=True)
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1 or len(config.get("boxes", [])) != 1:
        raise ValueError("Step A1 requires obstacles schema 1 with exactly one box")
    if not (config["enabled"] or args.enable_obstacles):
        raise ValueError("obstacle is disabled; pass --enable-obstacles for Step A1")
    obstacle = config["boxes"][0]
    settings = config["step_a1_fem_strip"]
    center = np.asarray(obstacle["center_m"], dtype=np.float64)
    size = np.asarray(obstacle["size_m"], dtype=np.float64)
    half = size / 2.0
    contact = obstacle["contact"]

    length, width, thickness = (float(v) for v in settings["size_m"])
    cells_x, cells_y, cells_z = (int(v) for v in settings["cells"])
    density = float(settings["density_kg_m3"])
    young = float(settings["youngs_modulus_pa"])
    poisson = float(settings["poisson_ratio"])
    damping = float(settings["damping_pa_s"])
    particles = (cells_x + 1) * (cells_y + 1) * (cells_z + 1)
    cells = cells_x * cells_y * cells_z
    grid_density = density * cells / particles
    k_mu = young / (2.0 * (1.0 + poisson))
    k_lambda = young * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson))
    obstacle_top = float(center[2] + half[2])
    initial_bottom = obstacle_top + float(settings["initial_clearance_m"])

    with wp.ScopedDevice("cpu"):
        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
        box_cfg = builder.ShapeConfig(
            density=0.0,
            ke=float(contact["ke_n_m"]),
            kd=float(contact["kd_n_s_m"]),
            mu=float(contact["mu"]),
            restitution=float(contact["restitution"]),
            gap=float(contact["gap_m"]),
            has_shape_collision=True,
            has_particle_collision=True,
        )
        obstacle_shape = builder.add_shape_box(
            -1,
            xform=wp.transform(wp.vec3(*center), wp.quat_identity()),
            hx=float(half[0]), hy=float(half[1]), hz=float(half[2]),
            cfg=box_cfg, label=obstacle["id"],
        )
        builder.add_soft_grid(
            pos=wp.vec3(
                float(center[0] - length / 2.0),
                float(center[1] - width / 2.0),
                float(initial_bottom),
            ),
            rot=wp.quat_identity(), vel=wp.vec3(0.0, 0.0, 0.0),
            dim_x=cells_x, dim_y=cells_y, dim_z=cells_z,
            cell_x=length / cells_x, cell_y=width / cells_y,
            cell_z=thickness / cells_z, density=grid_density,
            k_mu=k_mu, k_lambda=k_lambda, k_damp=damping,
            fix_left=False, particle_radius=float(settings["particle_radius_m"]),
            label="step_a1_fem_strip",
        )
        builder.color()
        model = builder.finalize()
        # FEM contact uses model-level parameters; the box's rigid ke/kd are not
        # the material/contact law used by VBD particles.
        model.soft_contact_ke = float(settings["soft_contact_ke_n_m"])
        model.soft_contact_kd = float(settings["soft_contact_kd_n_s_m"])
        model.soft_contact_mu = float(settings["soft_contact_mu"])
        solver = newton.solvers.SolverVBD(
            model=model, iterations=int(settings["iterations"]),
            particle_enable_self_contact=False, particle_enable_tile_solve=False,
        )
        state_0, state_1 = model.state(), model.state()
        control = model.control()
        pipeline = newton.CollisionPipeline(
            model, soft_contact_margin=float(settings["soft_contact_margin_m"]),
            enable_rigid_soft_full_surface_contact=True,
        )
        contacts = pipeline.contacts()
        radii = model.particle_radius.numpy().copy()
        samples = []
        settled_during_run = False
        maximum_penetration = 0.0
        maximum_soft_contacts = 0
        sim_time = 0.0
        wall_start = time.monotonic()
        duration = float(settings["duration_s"])
        while sim_time < duration:
            for _ in range(SUBSTEPS):
                state_0.clear_forces()
                pipeline.collide(state_0, contacts)
                maximum_soft_contacts = max(
                    maximum_soft_contacts,
                    int(contacts.soft_contact_count.numpy()[0]),
                )
                positions = state_0.particle_q.numpy()
                penetration = np.maximum(0.0, radii - box_sdf(positions, center, half))
                maximum_penetration = max(maximum_penetration, float(penetration.max()))
                solver.step(state_0, state_1, control, contacts, DT)
                state_0, state_1 = state_1, state_0
                sim_time += DT
            positions = state_0.particle_q.numpy()
            samples.append((sim_time, float(positions[:, 2].mean())))
            window_start = sim_time - float(settings["settling_window_s"])
            window = [value for stamp, value in samples if stamp >= window_start]
            if (
                sim_time >= float(settings["minimum_run_time_s"])
                and maximum_soft_contacts > 0
                and len(window) > 2
                and float(np.ptp(window))
                < float(settings["settling_range_limit_m"])
            ):
                settled_during_run = True
                break
        wall_elapsed = time.monotonic() - wall_start

    values = np.asarray(samples, dtype=np.float64)
    tail = values[values[:, 0] >= sim_time - float(settings["settling_window_s"]), 1]
    tail_range = float(np.ptp(tail))
    finite = bool(np.isfinite(state_0.particle_q.numpy()).all())
    checks = {
        "used_fem_strip": True,
        "soft_contact_candidates_observed": maximum_soft_contacts > 0,
        "maximum_particle_penetration_under_1mm": maximum_penetration < 0.001,
        "settled": settled_during_run,
        "finite_state": finite,
    }
    result = {
        "step": "A1_fem_strip_obstacle_drop",
        "object_model": "fem_strip",
        "tested_commit": args.tested_commit,
        "device": "cpu",
        "obstacle_id": obstacle["id"],
        "obstacle_shape_index": int(obstacle_shape),
        "particles": int(model.particle_count),
        "tetrahedra": int(model.tet_count),
        "model_soft_contact_ke_n_m": float(model.soft_contact_ke),
        "model_soft_contact_kd_n_s_m": float(model.soft_contact_kd),
        "model_soft_contact_mu": float(model.soft_contact_mu),
        "maximum_soft_contact_candidates": maximum_soft_contacts,
        "maximum_particle_penetration_m": maximum_penetration,
        "last_window_center_z_range_m": tail_range,
        "simulated_time_s": sim_time,
        "wall_elapsed_s": wall_elapsed,
        "real_time_factor": sim_time / wall_elapsed,
        "checks": checks,
        "pass": all(checks.values()),
    }
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"FEM_OBSTACLE_RESULT_JSON {args.output}")
    print("FEM_OBSTACLE_RESULT", json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
