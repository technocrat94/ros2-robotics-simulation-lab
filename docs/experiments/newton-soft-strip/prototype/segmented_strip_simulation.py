#!/usr/bin/env python3
"""Display the gravity response of the compliant-joint strip in Viser."""

import argparse
import functools
import time

import newton
import numpy as np
import viser
from newton.viewer import ViewerViser

from segmented_strip_batch import rotated_x_endpoint
from segmented_strip_topology import build_model


FRAME_DT = 1.0 / 60.0
SUBSTEPS = 10
SIM_DT = FRAME_DT / SUBSTEPS
REFERENCE_SEGMENTS = 20
REFERENCE_JOINT_DAMPING = 0.02


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--segments", type=int, default=20)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--port", type=int, default=8084)
    args = parser.parse_args()
    if args.segments < 2 or args.duration <= 0 or args.start_delay < 0:
        parser.error("invalid segments, duration, or delay")

    joint_damping = REFERENCE_JOINT_DAMPING * args.segments / REFERENCE_SEGMENTS
    _, model, state_0, properties, links, _ = build_model(
        args.segments,
        joint_damping_nm_s_rad=joint_damping,
        kinematic_root=True,
        enable_shape_collisions=False,
        add_ground=True,
    )
    state_1 = model.state()
    control = model.control()
    solver = newton.solvers.SolverXPBD(model, iterations=args.iterations)
    collision_pipeline = newton.CollisionPipeline(model)
    contacts = collision_pipeline.contacts()

    initial_poses = state_0.body_q.numpy().copy()
    initial_tip_z = float(
        rotated_x_endpoint(
            initial_poses[links[-1]], properties["segment_length_m"] / 2.0
        )[2]
    )

    server_class = viser.ViserServer
    viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
    try:
        viewer = ViewerViser(
            port=args.port,
            label="Newton Segmented Soft Strip",
            share=False,
        )
    finally:
        viser.ViserServer = server_class
    viewer.set_model(model)
    server = viewer._server
    server.scene.set_up_direction("+z")
    server.initial_camera.position = (0.58, -0.55, 0.68)
    server.initial_camera.look_at = (0.20, 0.025, 0.31)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.gui.add_markdown(
        "## Compliant-joint strip\n"
        "Rigid segments are linked by torsional springs and rotational damping. "
        "The first segment is kinematic; segment self-collision is disabled."
    )
    status = server.gui.add_markdown("Preparing...")

    sim_time = 0.0
    wall_start = time.monotonic()
    print(
        f"SEGMENTED_STRIP_SIM_READY segments={args.segments} "
        f"duration={args.duration} start_delay={args.start_delay} port={args.port}",
        flush=True,
    )

    try:
        while viewer.is_running():
            remaining = max(0.0, args.start_delay - (time.monotonic() - wall_start))
            if remaining == 0.0 and sim_time < args.duration:
                for _ in range(SUBSTEPS):
                    state_0.clear_forces()
                    collision_pipeline.collide(state_0, contacts)
                    solver.step(state_0, state_1, control, contacts, SIM_DT)
                    state_0, state_1 = state_1, state_0
                sim_time = min(args.duration, sim_time + FRAME_DT)

            poses = state_0.body_q.numpy()
            tip_z = float(
                rotated_x_endpoint(
                    poses[links[-1]], properties["segment_length_m"] / 2.0
                )[2]
            )
            root_error = float(
                np.linalg.norm(poses[links[0], :3] - initial_poses[links[0], :3])
            )
            if remaining > 0.0:
                phase = f"STARTING IN {remaining:.1f} s"
            elif sim_time < args.duration:
                phase = "RUNNING"
            else:
                phase = "COMPLETE"
            status.content = (
                f"### {phase}\n"
                f"- Segments: **{args.segments}**\n"
                f"- Simulation time: **{sim_time:.2f} s**\n"
                f"- Tip deflection: **{initial_tip_z - tip_z:.4f} m**\n"
                f"- Root error: **{root_error:.3e} m**\n"
                f"- Joint stiffness: **{properties['joint_stiffness_nm_rad']:.4f} N m/rad**\n"
                f"- Joint damping: **{joint_damping:.4f} N m s/rad**"
            )

            viewer.begin_frame(sim_time)
            viewer.log_state(state_0)
            viewer.end_frame()
            time.sleep(0.001)
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
