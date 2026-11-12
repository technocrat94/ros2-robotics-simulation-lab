#!/usr/bin/env python3
"""Persistent Newton renderer for a kinematically moved camera-visible rod."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import warp as wp

import newton
from newton.sensors import SensorTiledCamera

import newton_sim_depth_cloud_viser as camera_common
from newton_camera_static_capture import (
    HEIGHT,
    START_ARM,
    VERTICAL_FOV_RAD,
    WIDTH,
    StaticObstacleCamera,
    voxel_downsample,
)


ROD_RADIUS_M = 0.025
ROD_TOP_M = 0.35
ROD_Y_M = 0.263
ROD_CENTER_Z_M = 0.5 * ROD_TOP_M
DEPTH_PREVIEW_STRIDE = 4


@wp.kernel(enable_backward=False)
def compose_camera(
    body_q: wp.array(dtype=wp.transform),
    parent: wp.int32,
    local: wp.array(dtype=wp.transform),
    output: wp.array2d(dtype=wp.transform),
):
    output[0, 0] = wp.transform_multiply(body_q[parent], local[0])


class MovingRodCamera:
    """One Newton model whose kinematic rod body is repositioned per capture."""

    def __init__(
        self,
        urdf: Path,
        mount: camera_common.MountConfig,
        device_name: str,
    ):
        newton.use_coord_layout_targets = True
        self.device = wp.get_device(device_name)
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        builder.add_urdf(urdf, floating=False)
        self.robot_shape_count = builder.shape_count
        self.parent = builder.body_label.index(mount.parent_link)
        self.rod_body = builder.add_body(
            xform=wp.transform(
                (0.30, ROD_Y_M, ROD_CENTER_Z_M), wp.quat_identity()
            ),
            mass=0.0,
            is_kinematic=True,
            label="moving_rod",
        )
        self.rod_shape = builder.add_shape_cylinder(
            self.rod_body,
            xform=wp.transform_identity(),
            radius=ROD_RADIUS_M,
            half_height=0.5 * ROD_TOP_M,
            color=(0.9, 0.25, 0.15),
            label="moving_rod_shape",
        )
        builder.add_ground_plane()
        self.model = builder.finalize(device=self.device)
        self.state = self.model.state()
        self.q_template = self.model.joint_q.numpy().copy()
        self._assign_pose(START_ARM, 0.30)
        self.axes = StaticObstacleCamera._derive_axes(self)
        local = StaticObstacleCamera._mount_transform(self, mount).astype(np.float32)
        self.camera_local = wp.array(
            local.reshape(1, 7), dtype=wp.transform, device=self.device
        )
        self.camera_transforms = wp.zeros(
            (1, self.model.world_count), dtype=wp.transform, device=self.device
        )
        self.sensor = SensorTiledCamera(model=self.model, load_textures=False)
        self.rays = self.sensor.utils.compute_camera_rays_pinhole(
            WIDTH, HEIGHT, camera_fovs=VERTICAL_FOV_RAD
        )
        self.depth = self.sensor.utils.create_forward_depth_image_output(WIDTH, HEIGHT)
        self.shape_index = self.sensor.utils.create_shape_index_image_output(WIDTH, HEIGHT)
        pixel_x = (np.arange(WIDTH, dtype=np.float64) + 0.5) / WIDTH - 0.5
        pixel_y = (np.arange(HEIGHT, dtype=np.float64) + 0.5) / HEIGHT - 0.5
        half_height = math.tan(0.5 * VERTICAL_FOV_RAD)
        self.x_factor = np.broadcast_to(
            pixel_x[None, :] * 2.0 * half_height * WIDTH / HEIGHT,
            (HEIGHT, WIDTH),
        )
        self.y_factor = np.broadcast_to(
            -pixel_y[:, None] * 2.0 * half_height,
            (HEIGHT, WIDTH),
        )
        self.capture_count = 0

    def _assign_pose(self, arm: np.ndarray, rod_x_m: float) -> None:
        # Preserve every non-arm joint from the imported model. In particular,
        # do not use the old q[6:] = 0.0 shortcut: the rod pose is independent.
        q = self.q_template.copy()
        q[:6] = arm
        self.model.joint_q.assign(q)
        newton.eval_fk(
            self.model, self.model.joint_q, self.model.joint_qd, self.state
        )
        body_q = self.state.body_q.numpy()
        body_q[self.rod_body] = np.array(
            (rod_x_m, ROD_Y_M, ROD_CENTER_Z_M, 0.0, 0.0, 0.0, 1.0),
            dtype=np.float32,
        )
        self.state.body_q.assign(body_q)
        self.model.bvh_refit_shapes(self.state)

    def capture(self, arm: np.ndarray, rod_x_m: float) -> dict[str, object]:
        self._assign_pose(arm, rod_x_m)
        wp.launch(
            compose_camera,
            dim=1,
            inputs=[self.state.body_q, self.parent, self.camera_local],
            outputs=[self.camera_transforms],
            device=self.device,
        )
        self.sensor.update(
            self.state,
            self.camera_transforms,
            self.rays,
            forward_depth_image=self.depth,
            shape_index_image=self.shape_index,
        )
        wp.synchronize_device(self.device)
        depth = self.depth.numpy()[0, 0].copy()
        shape_index = self.shape_index.numpy()[0, 0].copy()
        camera_transform = self.camera_transforms.numpy()[0, 0].copy()
        body_q = self.state.body_q.numpy()
        labels = self.model.body_label

        conversion_started = time.perf_counter()
        valid = np.isfinite(depth) & (depth > 0.0)
        camera_points = np.column_stack(
            (
                depth[valid] * self.x_factor[valid],
                depth[valid] * self.y_factor[valid],
                -depth[valid],
            )
        )
        rotation = camera_common.quaternion_xyzw_to_matrix(camera_transform[3:7])
        world = camera_points @ rotation.T + camera_transform[:3]
        shape_ids = shape_index[valid]
        robot = shape_ids < self.robot_shape_count
        environment = (~robot) & (world[:, 2] > 0.012)
        valid_rows, valid_cols = np.nonzero(valid)
        environment_rows = valid_rows[environment]
        environment_cols = valid_cols[environment]
        border_pixel_counts = {
            "top": int(np.count_nonzero(environment_rows == 0)),
            "bottom": int(np.count_nonzero(environment_rows == HEIGHT - 1)),
            "left": int(np.count_nonzero(environment_cols == 0)),
            "right": int(np.count_nonzero(environment_cols == WIDTH - 1)),
        }
        perceived = voxel_downsample(world[environment])
        conversion_ms = (time.perf_counter() - conversion_started) * 1000.0
        self.capture_count += 1
        return {
            "points": perceived,
            "camera_transform": camera_transform,
            "depth_preview": camera_common.depth_to_rgb(depth)[
                ::DEPTH_PREVIEW_STRIDE, ::DEPTH_PREVIEW_STRIDE
            ],
            "conversion_ms": conversion_ms,
            "rod_pixels": int(np.count_nonzero(shape_ids == self.rod_shape)),
            "border_touches": {
                name: count > 0 for name, count in border_pixel_counts.items()
            },
            "border_pixel_counts": border_pixel_counts,
            "robot_percent": 100.0
            * float(np.count_nonzero(robot))
            / max(int(np.count_nonzero(valid)), 1),
            "flange_m": body_q[labels.index("ur/flange"), :3].tolist(),
            "rod_body_m": body_q[self.rod_body, :3].tolist(),
            "capture_count": self.capture_count,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path.home()
        / "newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf",
    )
    parser.add_argument(
        "--mount-config",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "config/sim_camera_mount.yaml",
    )
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    mount, _ = camera_common.read_mount_config(args.mount_config)
    scene = MovingRodCamera(args.urdf, mount, args.device)
    print(
        "NEWTON_MOVING_ROD_WORKER_READY "
        + json.dumps(
            {
                "newton": newton.__version__,
                "mount": mount.__dict__,
                "scene_builds": 1,
            }
        ),
        flush=True,
    )
    for line in sys.stdin:
        request = json.loads(line)
        if request.get("command") == "shutdown":
            print("NEWTON_MOVING_ROD_WORKER_STOPPED", flush=True)
            return
        arm = np.asarray(request["arm_q"], dtype=np.float32)
        rod_x_m = float(request["rod_x_m"])
        started = time.perf_counter()
        capture = scene.capture(arm, rod_x_m)
        capture_ms = (time.perf_counter() - started) * 1000.0
        output = Path(request["output"])
        np.savez_compressed(
            output,
            points=capture["points"],
            camera_transform=capture["camera_transform"],
            depth_preview=capture["depth_preview"],
        )
        response = {
            "id": request["id"],
            "output": str(output),
            "points": int(len(capture["points"])),
            "capture_ms": capture_ms,
            "conversion_ms": capture["conversion_ms"],
            "rod_pixels": capture["rod_pixels"],
            "border_touches": capture["border_touches"],
            "border_pixel_counts": capture["border_pixel_counts"],
            "robot_percent": capture["robot_percent"],
            "flange_m": capture["flange_m"],
            "rod_body_m": capture["rod_body_m"],
            "capture_count": capture["capture_count"],
            "scene_builds": 1,
        }
        print(
            "NEWTON_MOVING_ROD_WORKER_RESPONSE " + json.dumps(response),
            flush=True,
        )


if __name__ == "__main__":
    main()
