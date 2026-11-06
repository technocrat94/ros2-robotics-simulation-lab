#!/usr/bin/env python3
"""Capture static obstacles through the Newton link camera for cuRobo exchange."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any

import numpy as np
import warp as wp

import newton
from newton.sensors import SensorTiledCamera

import newton_sim_depth_cloud_viser as camera_common


WIDTH = 848
HEIGHT = 480
VERTICAL_FOV_RAD = math.radians(58.0)
START_ARM = np.array(
    [-2.6888591245, -2.0617183994, -1.2870012279, 1.7779233188, 4.7123889501, -5.8304517710],
    dtype=np.float32,
)
SCAN_ARMS = (
    START_ARM,
    START_ARM + np.array((0.25, 0.10, 0.0, 0.0, 0.0, 0.0), dtype=np.float32),
    START_ARM + np.array((-0.25, 0.18, 0.0, 0.0, 0.0, 0.0), dtype=np.float32),
    START_ARM + np.array((0.0, -0.12, 0.15, 0.0, 0.0, 0.0), dtype=np.float32),
)
MAIN_CENTER = np.array((0.487, 0.263, 0.160), dtype=np.float64)
MAIN_DIMS = np.array((0.200, 0.060, 0.300), dtype=np.float64)
BLIND_CENTER = np.array((0.487, 0.390, 0.160), dtype=np.float64)
BLIND_DIMS = np.array((0.160, 0.050, 0.280), dtype=np.float64)
VOXEL_SIZE_M = 0.008


@wp.kernel(enable_backward=False)
def compose_camera(
    body_q: wp.array(dtype=wp.transform),
    parent: wp.int32,
    local: wp.array(dtype=wp.transform),
    output: wp.array2d(dtype=wp.transform),
):
    output[0, 0] = wp.transform_multiply(body_q[parent], local[0])


def voxel_downsample(points: np.ndarray, size_m: float = VOXEL_SIZE_M) -> np.ndarray:
    if len(points) == 0:
        return points.astype(np.float32)
    keys = np.floor(points / size_m).astype(np.int32)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)].astype(np.float32)


def cuboid_surface_errors(points: np.ndarray, center: np.ndarray, dims: np.ndarray) -> dict[str, float]:
    relative = np.abs(points - center) - 0.5 * dims
    outside = np.linalg.norm(np.maximum(relative, 0.0), axis=1)
    inside = np.minimum(np.max(relative, axis=1), 0.0)
    errors_mm = np.abs(outside + inside) * 1000.0
    return {
        "median_mm": float(np.median(errors_mm)),
        "p95_mm": float(np.percentile(errors_mm, 95.0)),
        "max_mm": float(np.max(errors_mm)),
    }


class StaticObstacleCamera:
    def __init__(
        self,
        urdf: Path,
        mount: camera_common.MountConfig,
        center: np.ndarray,
        dims: np.ndarray,
        device_name: str,
    ):
        newton.use_coord_layout_targets = True
        self.device = wp.get_device(device_name)
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        builder.add_urdf(urdf, floating=False)
        self.robot_shape_count = builder.shape_count
        self.parent = builder.body_label.index(mount.parent_link)
        self.obstacle_shape = builder.add_shape_box(
            -1,
            xform=wp.transform(tuple(center), wp.quat_identity()),
            hx=float(0.5 * dims[0]),
            hy=float(0.5 * dims[1]),
            hz=float(0.5 * dims[2]),
        )
        builder.add_ground_plane()
        self.model = builder.finalize(device=self.device)
        self.state = self.model.state()
        self.q_template = self.model.joint_q.numpy()
        self._assign_pose(START_ARM)
        self.axes = self._derive_axes()
        local = self._mount_transform(mount).astype(np.float32)
        self.camera_local = wp.array(local.reshape(1, 7), dtype=wp.transform, device=self.device)
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

    def _assign_pose(self, arm: np.ndarray) -> None:
        q = self.q_template.copy()
        q[:6] = arm
        q[6:] = 0.0
        self.model.joint_q.assign(q)
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state)
        self.model.bvh_refit_shapes(self.state)

    def _derive_axes(self) -> dict[str, np.ndarray]:
        wp.synchronize_device(self.device)
        body_q = self.state.body_q.numpy()
        labels = self.model.body_label
        parent = body_q[self.parent]
        rotation = camera_common.quaternion_xyzw_to_matrix(parent[3:7])
        flange = body_q[labels.index("ur/flange"), :3]
        left_tip = body_q[labels.index("ur/robotiq_85_left_finger_tip_link"), :3]
        right_tip = body_q[labels.index("ur/robotiq_85_right_finger_tip_link"), :3]
        forward_world = camera_common.normalized(0.5 * (left_tip + right_tip) - flange)
        left_world = left_tip - right_tip
        left_world = camera_common.normalized(
            left_world - np.dot(left_world, forward_world) * forward_world
        )
        up_world = camera_common.normalized(np.cross(forward_world, left_world))
        return {
            "forward": camera_common.normalized(rotation.T @ forward_world),
            "left": camera_common.normalized(rotation.T @ left_world),
            "up": camera_common.normalized(rotation.T @ up_world),
        }

    def _mount_transform(self, mount: camera_common.MountConfig) -> np.ndarray:
        forward, left, up = self.axes["forward"], self.axes["left"], self.axes["up"]
        position = mount.forward_m * forward + mount.left_m * left + mount.up_m * up
        transverse = mount.left_m * left + mount.up_m * up
        toward_center = -up if np.linalg.norm(transverse) < 1.0e-9 else camera_common.normalized(-transverse)
        tilt = math.radians(mount.tilt_deg)
        optical = camera_common.normalized(
            math.cos(tilt) * forward + math.sin(tilt) * toward_center
        )
        image_up = camera_common.normalized(up - np.dot(up, optical) * optical)
        rotation = np.column_stack(
            (camera_common.normalized(np.cross(image_up, -optical)), image_up, -optical)
        )
        quaternion = camera_common.matrix_to_quaternion_xyzw(rotation)
        return np.concatenate((position, quaternion))

    def capture(self, arm: np.ndarray) -> dict[str, np.ndarray | float | int]:
        self._assign_pose(arm)
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

        started = time.perf_counter()
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
        # The floor is intentionally not a planner obstacle. Remove it geometrically
        # after robot pixels, so it cannot become a floor-sized blocking cuboid.
        perceived = world[(~robot) & (world[:, 2] > 0.012)]
        perceived = voxel_downsample(perceived)
        conversion_ms = (time.perf_counter() - started) * 1000.0
        return {
            "points": perceived,
            "camera_transform": camera_transform,
            "conversion_ms": conversion_ms,
            "valid_points": int(valid.sum()),
            "robot_percent": 100.0 * float(robot.sum()) / max(int(valid.sum()), 1),
            "obstacle_pixels": int(np.sum(shape_ids == self.obstacle_shape)),
            "flange_m": body_q[labels.index("ur/flange"), :3].tolist(),
            "wrist_1_center_m": body_q[labels.index("ur/wrist_1_link"), :3].tolist(),
        }


def capture_scenario(
    urdf: Path,
    mount: camera_common.MountConfig,
    center: np.ndarray,
    dims: np.ndarray,
    arms: tuple[np.ndarray, ...],
    device: str,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    camera = StaticObstacleCamera(urdf, mount, center, dims, device)
    captures = [camera.capture(arm) for arm in arms]
    merged = voxel_downsample(
        np.concatenate([capture["points"] for capture in captures], axis=0)
    )
    metadata = {
        "center_m": center.tolist(),
        "dims_m": dims.tolist(),
        "captures": [
            {
                "points": int(len(capture["points"])),
                "valid_points": capture["valid_points"],
                "robot_percent": capture["robot_percent"],
                "obstacle_pixels": capture["obstacle_pixels"],
                "conversion_ms": capture["conversion_ms"],
            }
            for capture in captures
        ],
        "merged_points": int(len(merged)),
        "surface_error": cuboid_surface_errors(merged, center, dims) if len(merged) else None,
    }
    return captures, {"merged": merged, "metadata": metadata}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path.home() / "newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf",
    )
    parser.add_argument(
        "--mount-config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config/sim_camera_mount.yaml",
    )
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    mount, _ = camera_common.read_mount_config(args.mount_config)
    main_captures, main_result = capture_scenario(
        args.urdf, mount, MAIN_CENTER, MAIN_DIMS, SCAN_ARMS, args.device
    )
    blind_captures, blind_result = capture_scenario(
        args.urdf, mount, BLIND_CENTER, BLIND_DIMS, (START_ARM,), args.device
    )
    conversion_samples = [
        float(capture["conversion_ms"])
        for capture in (*main_captures, *blind_captures)
    ]
    metadata = {
        "newton_version": newton.__version__,
        "device": args.device,
        "mount": mount.__dict__,
        "voxel_downsample_m": VOXEL_SIZE_M,
        "ground_handling": "discard non-robot points at z <= 0.012 m",
        "main": main_result["metadata"],
        "blind": blind_result["metadata"],
        "conversion_median_ms": float(statistics.median(conversion_samples)),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        main_start_points=main_captures[0]["points"],
        main_scan_points=main_result["merged"],
        main_camera_transforms=np.stack(
            [capture["camera_transform"] for capture in main_captures]
        ),
        main_scan_arms=np.stack(SCAN_ARMS),
        main_center=MAIN_CENTER,
        main_dims=MAIN_DIMS,
        blind_start_points=blind_captures[0]["points"],
        blind_camera_transform=blind_captures[0]["camera_transform"],
        blind_center=BLIND_CENTER,
        blind_dims=BLIND_DIMS,
        start_arm=START_ARM,
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    print("NEWTON_CAMERA_CUROBO_CAPTURE " + json.dumps(metadata, sort_keys=True))


if __name__ == "__main__":
    main()
