#!/usr/bin/env python3
"""Display the verified cantilever soft-body mesh without advancing physics."""

import argparse
import functools
import time

import viser
from newton.viewer import ViewerViser

from soft_strip_topology import (
    CELLS_Y,
    CELLS_Z,
    LENGTH_M,
    THICKNESS_M,
    WIDTH_M,
    build_builder,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cells-x", type=int, default=20)
    parser.add_argument("--port", type=int, default=8084)
    args = parser.parse_args()
    if args.cells_x < 1:
        parser.error("--cells-x must be at least 1")

    builder = build_builder(args.cells_x)
    builder.color()
    model = builder.finalize()
    state = model.state()

    server_class = viser.ViserServer
    viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
    try:
        viewer = ViewerViser(
            port=args.port,
            label="Newton FEM Rubber Strip — Static Inspection",
            share=False,
        )
    finally:
        viser.ViserServer = server_class

    viewer.set_model(model)
    server = viewer._server
    server.scene.set_up_direction("+z")
    server.initial_camera.position = (0.55, -0.55, 0.78)
    server.initial_camera.look_at = (0.20, 0.025, 0.51)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    server.gui.add_markdown(
        "## Static topology inspection\n"
        "The strip exists in Newton, but physics time is not advancing yet.\n\n"
        f"- Dimensions: **{LENGTH_M:.2f} × {WIDTH_M:.2f} × {THICKNESS_M:.2f} m**\n"
        f"- Cells: **{args.cells_x} × {CELLS_Y} × {CELLS_Z}**\n"
        f"- Particles: **{builder.particle_count}**\n"
        f"- Tetrahedra: **{builder.tet_count}**\n"
        f"- Fixed left particles: **{sum(float(m) == 0.0 for m in builder.particle_mass)}**"
    )
    print(
        f"SOFT_STRIP_VIEWER_READY port={args.port} particles={builder.particle_count} "
        f"tetrahedra={builder.tet_count}",
        flush=True,
    )

    try:
        while viewer.is_running():
            viewer.begin_frame(0.0)
            viewer.log_state(state)
            viewer.end_frame()
            time.sleep(0.05)
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
