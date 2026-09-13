#!/usr/bin/env python3
"""Run a short headless gravity test of the segmented strip."""

import argparse
import json
import time

import newton
import numpy as np

from segmented_strip_topology import build_model


FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
SIM_DT = FRAME_DT / SUBSTEPS
WINDOW_S = 1.0
REFERENCE_SEGMENTS = 20
REFERENCE_JOINT_DAMPING = 0.02


def rotated_x_endpoint(pose, half_length):
    position = pose[:3]
    quaternion_xyz = pose[3:6]
    quaternion_w = pose[6]
    local = np.array([half_length, 0.0, 0.0])
    cross = 2.0 * np.cross(quaternion_xyz, local)
    rotated = local + quaternion_w * cross + np.cross(quaternion_xyz, cross)
    return position + rotated


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--segments", type=int, default=20)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--min-duration", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=60.0)
    parser.add_argument("--settle-threshold", type=float, default=0.004)
    parser.add_argument("--joint-damping", type=float)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--fixed-joint-root", action="store_true")
    args = parser.parse_args()
    if (
        args.segments < 2
        or (args.duration is not None and args.duration <= 0.0)
        or args.min_duration < WINDOW_S
        or args.max_duration < args.min_duration
        or args.settle_threshold <= 0.0
        or args.iterations < 1
    ):
        parser.error("invalid segments, duration, or iterations")

    kinematic_root = not args.fixed_joint_root
    joint_damping = (
        args.joint_damping
        if args.joint_damping is not None
        else REFERENCE_JOINT_DAMPING * args.segments / REFERENCE_SEGMENTS
    )
    _, model, state_0, properties, links, _ = build_model(
        args.segments,
        joint_damping_nm_s_rad=joint_damping,
        kinematic_root=kinematic_root,
        enable_shape_collisions=False,
    )
    state_1 = model.state()
    control = model.control()
    solver = newton.solvers.SolverXPBD(model, iterations=args.iterations)
    collision_pipeline = newton.CollisionPipeline(model)
    contacts = collision_pipeline.contacts()

    initial_poses = state_0.body_q.numpy().copy()
    initial_tip = rotated_x_endpoint(
        initial_poses[links[-1]], properties["segment_length_m"] / 2.0
    )
    sim_time = 0.0
    tip_history = []
    automatic_stop = args.duration is None
    end_time = args.max_duration if automatic_stop else args.duration
    settled = False
    tail = []
    wall_start = time.monotonic()

    while sim_time < end_time:
        for _ in range(SUBSTEPS):
            state_0.clear_forces()
            collision_pipeline.collide(state_0, contacts)
            solver.step(state_0, state_1, control, contacts, SIM_DT)
            state_0, state_1 = state_1, state_0
        sim_time = min(end_time, sim_time + FRAME_DT)
        current_poses = state_0.body_q.numpy()
        current_tip = rotated_x_endpoint(
            current_poses[links[-1]], properties["segment_length_m"] / 2.0
        )
        tip_history.append((sim_time, float(current_tip[2])))
        window_start = sim_time - WINDOW_S
        tail = [
            tip_z
            for sample_time, tip_z in tip_history
            if sample_time >= window_start
        ]
        full_window = len(tail) >= int(0.9 * WINDOW_S / FRAME_DT)
        if (
            automatic_stop
            and sim_time >= args.min_duration
            and full_window
            and max(tail) - min(tail) < args.settle_threshold
        ):
            settled = True
            break

    wall_elapsed = time.monotonic() - wall_start
    poses = state_0.body_q.numpy()
    tip = rotated_x_endpoint(
        poses[links[-1]], properties["segment_length_m"] / 2.0
    )
    root_position_error = float(
        np.linalg.norm(poses[links[0], :3] - initial_poses[links[0], :3])
    )
    tail_mean_z = float(np.mean(tail))
    tail_range = float(max(tail) - min(tail))
    if not automatic_stop:
        settled = tail_range < args.settle_threshold
    result = {
        "segments": args.segments,
        "root_mode": "kinematic" if kinematic_root else "fixed_joint",
        "shape_collisions_enabled": False,
        "run_mode": "automatic_settling" if automatic_stop else "fixed_duration",
        "settled": settled,
        "settlement_detected_at_s": sim_time if automatic_stop and settled else None,
        "simulated_time_s": sim_time,
        "dt_s": SIM_DT,
        "iterations": args.iterations,
        "joint_stiffness_nm_rad": properties["joint_stiffness_nm_rad"],
        "joint_damping_nm_s_rad": joint_damping,
        "settle_threshold_m": args.settle_threshold,
        "initial_tip_z_m": float(initial_tip[2]),
        "final_tip_z_m": float(tip[2]),
        "downward_tip_deflection_m": float(initial_tip[2] - tip[2]),
        "mean_tip_z_last_1s_m": tail_mean_z,
        "mean_downward_deflection_last_1s_m": float(initial_tip[2] - tail_mean_z),
        "last_1s_tip_z_range_m": tail_range,
        "maximum_root_position_error_m": root_position_error,
        "finite_state": bool(np.isfinite(poses).all()),
        "wall_elapsed_s": wall_elapsed,
        "real_time_factor": sim_time / wall_elapsed,
    }
    print("RESULT", json.dumps(result))


if __name__ == "__main__":
    main()
