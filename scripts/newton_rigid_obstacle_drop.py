#!/usr/bin/env python3
"""Step A: headless Newton drop test for the configured rigid obstacle."""

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
SIM_DT = FRAME_DT / SUBSTEPS


def load_config(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or len(data.get("boxes", [])) != 1:
        raise ValueError("Step A requires obstacles schema 1 with exactly one box")
    return data


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--enable-obstacles", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--tested-commit", required=True)
    args = parser.parse_args()

    config = load_config(args.config)
    effective_enabled = bool(config["enabled"] or args.enable_obstacles)
    if not effective_enabled:
        print("ERROR: obstacle is disabled; pass --enable-obstacles for Step A")
        return 2

    obstacle = config["boxes"][0]
    validation = config["step_a"]
    center = np.asarray(obstacle["center_m"], dtype=np.float64)
    size = np.asarray(obstacle["size_m"], dtype=np.float64)
    if not np.isfinite(center).all() or not np.isfinite(size).all() or np.any(size <= 0.0):
        raise ValueError("obstacle center and positive size must be finite")

    radius = float(validation["sphere_radius_m"])
    obstacle_top = float(center[2] + size[2] / 2.0)
    initial_center_z = obstacle_top + radius + float(validation["initial_clearance_m"])
    contact = obstacle["contact"]
    shape_config = newton.ModelBuilder.ShapeConfig(
        density=0.0,
        ke=float(contact["ke_n_m"]),
        kd=float(contact["kd_n_s_m"]),
        mu=float(contact["mu"]),
        restitution=float(contact["restitution"]),
        gap=float(contact["gap_m"]),
    )
    sphere_config = newton.ModelBuilder.ShapeConfig(
        density=float(validation["sphere_density_kg_m3"]),
        ke=float(contact["ke_n_m"]),
        kd=float(contact["kd_n_s_m"]),
        mu=float(contact["mu"]),
        restitution=float(contact["restitution"]),
        gap=float(contact["gap_m"]),
    )

    with wp.ScopedDevice("cpu"):
        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
        obstacle_shape = builder.add_shape_box(
            -1,
            xform=wp.transform(wp.vec3(*center), wp.quat_identity()),
            hx=float(size[0] / 2.0),
            hy=float(size[1] / 2.0),
            hz=float(size[2] / 2.0),
            cfg=shape_config,
            label=obstacle["id"],
        )
        body = builder.add_body(
            xform=wp.transform(
                wp.vec3(float(center[0]), float(center[1]), initial_center_z),
                wp.quat_identity(),
            ),
            label="step_a_drop_sphere",
        )
        builder.add_shape_sphere(body, radius=radius, cfg=sphere_config)
        model = builder.finalize()
        solver = newton.solvers.SolverXPBD(model, iterations=20)
        state_0, state_1 = model.state(), model.state()
        control = model.control()
        pipeline = newton.CollisionPipeline(model)
        contacts = pipeline.contacts()

        duration = float(validation["duration_s"])
        settling_window = float(validation["settling_window_s"])
        samples = []
        sim_time = 0.0
        wall_start = time.monotonic()
        while sim_time < duration:
            for _ in range(SUBSTEPS):
                state_0.clear_forces()
                pipeline.collide(state_0, contacts)
                solver.step(state_0, state_1, control, contacts, SIM_DT)
                state_0, state_1 = state_1, state_0
            sim_time += FRAME_DT
            samples.append((sim_time, float(state_0.body_q.numpy()[body, 2])))
        wall_elapsed = time.monotonic() - wall_start

    values = np.asarray(samples, dtype=np.float64)
    bottom = values[:, 1] - radius
    tail = values[values[:, 0] >= sim_time - settling_window, 1]
    final_bottom = float(bottom[-1])
    maximum_penetration = max(0.0, obstacle_top - float(bottom.min()))
    final_penetration = max(0.0, obstacle_top - final_bottom)
    tail_range = float(np.ptp(tail))
    finite_state = bool(np.isfinite(values).all())
    checks = {
        "finite_state": finite_state,
        "settled_on_obstacle": abs(final_bottom - obstacle_top) < 0.001,
        "penetration_under_1mm": maximum_penetration
        < float(validation["maximum_penetration_m"]),
        "tail_motion_under_limit": tail_range
        < float(validation["settling_range_limit_m"]),
    }
    result = {
        "step": "A_pure_newton_physics",
        "tested_commit": args.tested_commit,
        "config_enabled": bool(config["enabled"]),
        "effective_obstacle_enabled": effective_enabled,
        "obstacle_id": obstacle["id"],
        "obstacle_shape_index": obstacle_shape,
        "obstacle_top_z_m": obstacle_top,
        "sphere_radius_m": radius,
        "initial_center_z_m": initial_center_z,
        "final_center_z_m": float(values[-1, 1]),
        "final_bottom_z_m": final_bottom,
        "maximum_penetration_m": maximum_penetration,
        "final_penetration_m": final_penetration,
        "last_window_center_z_range_m": tail_range,
        "dt_s": SIM_DT,
        "simulated_time_s": sim_time,
        "wall_elapsed_s": wall_elapsed,
        "real_time_factor": sim_time / wall_elapsed,
        "checks": checks,
    }
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"RIGID_OBSTACLE_RESULT_JSON {args.output}")
    print("RIGID_OBSTACLE_RESULT", json.dumps(result, sort_keys=True), flush=True)
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
