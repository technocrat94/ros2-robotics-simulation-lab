#!/usr/bin/env python3
"""Render Newton depth, back-project a world cloud, and inspect it in Viser."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import functools
import json
import math
from pathlib import Path
import re
import statistics
import threading
import time
from typing import Any, Callable

import numpy as np
import viser
import warp as wp

import newton
from newton.sensors import SensorTiledCamera
from newton.viewer import ViewerViser


WIDTH = 848
HEIGHT = 480
VERTICAL_FOV_DEG = 58.0
VERTICAL_FOV_RAD = math.radians(VERTICAL_FOV_DEG)

BOX_CENTER = np.array((0.487, 0.109, 0.06), dtype=np.float64)
BOX_HALF = np.array((0.12, 0.10, 0.06), dtype=np.float64)
SPHERE_CENTER = np.array((0.68, 0.05, 0.08), dtype=np.float64)
SPHERE_RADIUS = 0.08
CYLINDER_CENTER = np.array((0.32, 0.18, 0.10), dtype=np.float64)
CYLINDER_RADIUS = 0.055
CYLINDER_HALF_HEIGHT = 0.10

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
BASE_ARM = np.array(
    [0.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0, -math.pi / 2.0, 0.0],
    dtype=np.float32,
)
ACCURACY_POSES = (
    ("baseline", BASE_ARM),
    ("shoulder_lift_plus_0_15", BASE_ARM + np.array((0.0, 0.15, 0.0, 0.0, 0.0, 0.0), dtype=np.float32)),
    (
        "pan_plus_0_25_elbow_minus_0_15",
        BASE_ARM + np.array((0.25, 0.0, -0.15, 0.0, 0.0, 0.0), dtype=np.float32),
    ),
)
SURFACE_NAMES = ("box", "sphere", "cylinder", "ground")


@dataclass(frozen=True)
class MountConfig:
    parent_link: str
    left_m: float
    up_m: float
    forward_m: float
    tilt_deg: float
    status: str


MOUNT_KEYS = (
    "parent_link",
    "left_m",
    "up_m",
    "forward_m",
    "tilt_deg",
    "status",
)


def read_mount_config(path: Path) -> tuple[MountConfig, str]:
    """Read the deliberately small YAML schema without adding a dependency."""
    text = path.read_text(encoding="utf-8")
    values: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        values[key.strip()] = value.strip()
    missing = [key for key in MOUNT_KEYS if key not in values]
    if missing:
        raise ValueError(f"Missing mount config keys: {missing}")
    config = MountConfig(
        parent_link=values["parent_link"],
        left_m=float(values["left_m"]),
        up_m=float(values["up_m"]),
        forward_m=float(values["forward_m"]),
        tilt_deg=float(values["tilt_deg"]),
        status=values["status"],
    )
    return config, text


def save_mount_config(path: Path, template: str, config: MountConfig) -> str:
    """Replace only schema values, preserving YAML key order and comments."""
    replacements = {
        "parent_link": config.parent_link,
        "left_m": f"{config.left_m:.6f}",
        "up_m": f"{config.up_m:.6f}",
        "forward_m": f"{config.forward_m:.6f}",
        "tilt_deg": f"{config.tilt_deg:.3f}",
        "status": config.status,
    }
    updated = template
    for key in MOUNT_KEYS:
        pattern = rf"(?m)^(\s*{re.escape(key)}\s*:\s*).*$"
        updated, count = re.subn(pattern, rf"\g<1>{replacements[key]}", updated, count=1)
        if count != 1:
            raise ValueError(f"Could not preserve mount YAML field {key!r}")
    path.write_text(updated, encoding="utf-8")
    return updated


@wp.kernel(enable_backward=False)
def update_link_camera_transform(
    body_q: wp.array(dtype=wp.transform),
    camera_parent: wp.int32,
    camera_local_transform: wp.array(dtype=wp.transform),
    camera_transforms: wp.array2d(dtype=wp.transform),
):
    """Compose the fixed camera-site transform with its live parent-link pose."""
    camera_transforms[0, 0] = wp.transform_multiply(
        body_q[camera_parent], camera_local_transform[0]
    )


def quaternion_xyzw_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Return a local-to-parent rotation matrix for an XYZW quaternion."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("Zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized XYZW quaternion."""
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            )
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                (
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                )
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                (
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                )
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                (
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                )
            )
    return quaternion / np.linalg.norm(quaternion)


def normalized(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-9:
        raise ValueError("Cannot normalize a zero-length vector")
    return np.asarray(vector, dtype=np.float64) / norm


def quaternion_multiply_xyzw(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product for XYZW quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        dtype=np.float64,
    )


def box_sdf(points: np.ndarray) -> np.ndarray:
    relative = np.abs(points - BOX_CENTER) - BOX_HALF
    outside = np.linalg.norm(np.maximum(relative, 0.0), axis=1)
    inside = np.minimum(np.max(relative, axis=1), 0.0)
    return outside + inside


def sphere_sdf(points: np.ndarray) -> np.ndarray:
    return np.linalg.norm(points - SPHERE_CENTER, axis=1) - SPHERE_RADIUS


def cylinder_sdf(points: np.ndarray) -> np.ndarray:
    relative = points - CYLINDER_CENTER
    radial = np.linalg.norm(relative[:, :2], axis=1) - CYLINDER_RADIUS
    axial = np.abs(relative[:, 2]) - CYLINDER_HALF_HEIGHT
    pair = np.column_stack((radial, axial))
    outside = np.linalg.norm(np.maximum(pair, 0.0), axis=1)
    inside = np.minimum(np.max(pair, axis=1), 0.0)
    return outside + inside


def surface_signed_distances(points: np.ndarray) -> np.ndarray:
    """Signed distances to box, sphere, Z-cylinder, and z=0 ground."""
    return np.column_stack(
        (box_sdf(points), sphere_sdf(points), cylinder_sdf(points), points[:, 2])
    )


def error_metrics(values_m: np.ndarray) -> dict[str, float | int]:
    values_m = np.asarray(values_m, dtype=np.float64)
    absolute_mm = np.abs(values_m) * 1000.0
    return {
        "count": int(values_m.size),
        "signed_median_mm": float(np.median(values_m) * 1000.0),
        "absolute_median_mm": float(np.median(absolute_mm)),
        "absolute_p95_mm": float(np.percentile(absolute_mm, 95.0)),
        "absolute_max_mm": float(absolute_mm.max()),
    }


def summarize_samples(samples_ms: list[float]) -> dict[str, float | int]:
    return {
        "frames": len(samples_ms),
        "median_ms": float(statistics.median(samples_ms)),
        "p95_ms": float(np.percentile(samples_ms, 95.0)),
        "min_ms": float(min(samples_ms)),
        "max_ms": float(max(samples_ms)),
    }


def depth_to_rgb(depth: np.ndarray) -> np.ndarray:
    """Create a dependency-free blue-to-yellow depth visualization."""
    valid = np.isfinite(depth) & (depth > 0.0)
    rgb = np.zeros((*depth.shape, 3), dtype=np.uint8)
    if not valid.any():
        return rgb
    low, high = np.percentile(depth[valid], (2.0, 98.0))
    span = max(float(high - low), 1.0e-6)
    t = np.clip((depth - low) / span, 0.0, 1.0)
    rgb[..., 0] = np.asarray(255.0 * t, dtype=np.uint8)
    rgb[..., 1] = np.asarray(255.0 * (1.0 - np.abs(2.0 * t - 1.0)), dtype=np.uint8)
    rgb[..., 2] = np.asarray(255.0 * (1.0 - t), dtype=np.uint8)
    rgb[~valid] = 0
    return rgb


def height_colors(points: np.ndarray) -> np.ndarray:
    """Color world points blue-to-yellow by height."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.uint8)
    t = np.clip(points[:, 2] / 0.70, 0.0, 1.0)
    return np.column_stack(
        (
            np.asarray(255.0 * t, dtype=np.uint8),
            np.asarray(180.0 + 75.0 * t, dtype=np.uint8),
            np.asarray(255.0 * (1.0 - t), dtype=np.uint8),
        )
    )


class DepthCloudScene:
    """Newton scene, link-mounted camera, and same-frame host conversion."""

    def __init__(self, urdf: Path, device_name: str, mount_config: MountConfig):
        newton.use_coord_layout_targets = True
        self.urdf = urdf
        self.device = wp.get_device(device_name)

        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        builder.add_urdf(urdf, floating=False)
        self.robot_body_count = builder.body_count
        self.robot_shape_count = builder.shape_count
        self.camera_parent = builder.body_label.index(mount_config.parent_link)
        self.camera_site = builder.add_site(
            self.camera_parent,
            xform=wp.transform_identity(),
            label="adjustable_depth_camera_mount",
        )

        self.box_shape = builder.add_shape_box(
            -1,
            xform=wp.transform(tuple(BOX_CENTER), wp.quat_identity()),
            hx=BOX_HALF[0],
            hy=BOX_HALF[1],
            hz=BOX_HALF[2],
            color=(0.2, 0.7, 0.9),
        )
        self.sphere_shape = builder.add_shape_sphere(
            -1,
            xform=wp.transform(tuple(SPHERE_CENTER), wp.quat_identity()),
            radius=SPHERE_RADIUS,
            color=(0.9, 0.4, 0.2),
        )
        self.cylinder_shape = builder.add_shape_cylinder(
            -1,
            xform=wp.transform(tuple(CYLINDER_CENTER), wp.quat_identity()),
            radius=CYLINDER_RADIUS,
            half_height=CYLINDER_HALF_HEIGHT,
            color=(0.3, 0.9, 0.4),
        )
        self.ground_shape = builder.add_ground_plane(color=(0.55, 0.55, 0.55))

        self.model = builder.finalize(device=self.device)
        self.state = self.model.state()
        self.q_template = self.model.joint_q.numpy()
        self._assign_pose(BASE_ARM)
        self.mount_axes = self._derive_mount_axes()
        self.camera_local_transform = wp.array(
            [wp.transform_identity()], dtype=wp.transform, device=self.device
        )
        self.mount_config = mount_config
        self.set_mount(mount_config)
        self.sensor = SensorTiledCamera(model=self.model, load_textures=False)

        # Newton documents camera_fovs as the vertical FOV in radians. Its
        # pinhole kernel multiplies tan(vfov/2) by width/height for horizontal.
        self.camera_rays = self.sensor.utils.compute_camera_rays_pinhole(
            WIDTH, HEIGHT, camera_fovs=VERTICAL_FOV_RAD
        )
        self.forward_depth_image = self.sensor.utils.create_forward_depth_image_output(WIDTH, HEIGHT)
        self.ray_depth_image = self.sensor.utils.create_depth_image_output(WIDTH, HEIGHT)
        self.shape_index_image = self.sensor.utils.create_shape_index_image_output(WIDTH, HEIGHT)
        self.camera_transforms = wp.zeros(
            (1, self.model.world_count), dtype=wp.transform, device=self.device
        )

        pixel_x = (np.arange(WIDTH, dtype=np.float64) + 0.5) / WIDTH - 0.5
        pixel_y = (np.arange(HEIGHT, dtype=np.float64) + 0.5) / HEIGHT - 0.5
        half_height = math.tan(VERTICAL_FOV_RAD / 2.0)
        self.x_factor = np.broadcast_to(
            pixel_x[None, :] * 2.0 * half_height * (WIDTH / HEIGHT),
            (HEIGHT, WIDTH),
        )
        self.y_factor = np.broadcast_to(
            -pixel_y[:, None] * 2.0 * half_height,
            (HEIGHT, WIDTH),
        )

        self.set_pose(BASE_ARM)
        self.render()
        wp.synchronize_device(self.device)

        body_q = self.state.body_q.numpy()
        shoulder_index = self.model.body_label.index("ur/shoulder_link")
        self.shoulder_height_m = float(body_q[shoulder_index, 2])

    def _assign_pose(self, arm_q: np.ndarray) -> None:
        q = self.q_template.copy()
        q[:6] = np.asarray(arm_q, dtype=np.float32)
        q[6:] = 0.0
        self.model.joint_q.assign(q)
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state)
        self.model.bvh_refit_shapes(self.state)

    def _derive_mount_axes(self) -> dict[str, np.ndarray | float | str]:
        """Measure forward and finger-opening directions in the parent frame."""
        wp.synchronize_device(self.device)
        body_q = self.state.body_q.numpy()
        labels = self.model.body_label
        parent = body_q[self.camera_parent]
        parent_rotation = quaternion_xyzw_to_matrix(parent[3:7])
        flange_position = body_q[labels.index("ur/flange"), :3]
        left_tip = body_q[labels.index("ur/robotiq_85_left_finger_tip_link"), :3]
        right_tip = body_q[labels.index("ur/robotiq_85_right_finger_tip_link"), :3]
        fingertip_center = 0.5 * (left_tip + right_tip)

        forward_world = normalized(fingertip_center - flange_position)
        left_world = left_tip - right_tip
        left_world = normalized(left_world - np.dot(left_world, forward_world) * forward_world)
        up_world = normalized(np.cross(forward_world, left_world))
        forward_local = normalized(parent_rotation.T @ forward_world)
        left_local = normalized(parent_rotation.T @ left_world)
        up_local = normalized(parent_rotation.T @ up_world)
        return {
            "forward_local": forward_local,
            "left_local": left_local,
            "up_local": up_local,
            "forward_tool0_alignment": float(np.dot(forward_local, np.array((0.0, 0.0, 1.0)))),
            "left_tool0_alignment": float(np.dot(left_local, np.array((1.0, 0.0, 0.0)))),
            "up_tool0_alignment": float(np.dot(up_local, np.array((0.0, 1.0, 0.0)))),
            "definition": (
                "forward=flange to fingertip midpoint; left=right fingertip to left fingertip; "
                "up=forward cross left"
            ),
        }

    def mount_transform(self, config: MountConfig) -> np.ndarray:
        forward = self.mount_axes["forward_local"]
        left = self.mount_axes["left_local"]
        up = self.mount_axes["up_local"]
        assert isinstance(forward, np.ndarray)
        assert isinstance(left, np.ndarray)
        assert isinstance(up, np.ndarray)
        position = config.forward_m * forward + config.left_m * left + config.up_m * up

        transverse = config.left_m * left + config.up_m * up
        if np.linalg.norm(transverse) <= 1.0e-9:
            toward_center = -up
        else:
            toward_center = normalized(-transverse)
        tilt = math.radians(config.tilt_deg)
        optical_forward = normalized(math.cos(tilt) * forward + math.sin(tilt) * toward_center)
        image_up = normalized(up - np.dot(up, optical_forward) * optical_forward)
        camera_z = -optical_forward
        camera_x = normalized(np.cross(image_up, camera_z))
        rotation = np.column_stack((camera_x, image_up, camera_z))
        quaternion = matrix_to_quaternion_xyzw(rotation)
        return np.concatenate((position, quaternion))

    def set_mount(self, config: MountConfig) -> None:
        if config.parent_link != self.model.body_label[self.camera_parent]:
            raise ValueError(
                f"Mount parent {config.parent_link!r} does not match loaded parent "
                f"{self.model.body_label[self.camera_parent]!r}"
            )
        transform = self.mount_transform(config).astype(np.float32)
        self.camera_local_transform.assign(transform.reshape(1, 7))
        self.mount_config = config

    def mount_axis_world(self) -> dict[str, np.ndarray]:
        wp.synchronize_device(self.device)
        body_q = self.state.body_q.numpy()
        parent = body_q[self.camera_parent]
        rotation = quaternion_xyzw_to_matrix(parent[3:7])
        flange = body_q[self.model.body_label.index("ur/flange"), :3].astype(np.float64)
        result = {"origin": flange}
        for name in ("forward", "left", "up"):
            local = self.mount_axes[f"{name}_local"]
            assert isinstance(local, np.ndarray)
            result[name] = normalized(rotation @ local)
        return result

    def set_pose(self, arm_q: np.ndarray) -> None:
        self._assign_pose(arm_q)
        self.update_camera_transform()

    def update_camera_transform(self) -> None:
        wp.launch(
            update_link_camera_transform,
            dim=1,
            inputs=[
                self.state.body_q,
                self.camera_parent,
                self.camera_local_transform,
            ],
            outputs=[self.camera_transforms],
            device=self.device,
        )

    def render(self) -> None:
        self.update_camera_transform()
        self.sensor.update(
            self.state,
            self.camera_transforms,
            self.camera_rays,
            forward_depth_image=self.forward_depth_image,
            depth_image=self.ray_depth_image,
            shape_index_image=self.shape_index_image,
        )

    def copy_same_frame(self) -> dict[str, np.ndarray]:
        """Synchronize once, then copy image, segmentation, and pose unchanged."""
        wp.synchronize_device(self.device)
        return {
            "forward_depth": self.forward_depth_image.numpy()[0, 0].copy(),
            "ray_depth": self.ray_depth_image.numpy()[0, 0].copy(),
            "shape_index": self.shape_index_image.numpy()[0, 0].copy(),
            "camera_transform": self.camera_transforms.numpy()[0, 0].copy(),
        }

    def capture(self, arm_q: np.ndarray) -> dict[str, np.ndarray]:
        self.set_pose(arm_q)
        self.render()
        return self.copy_same_frame()

    def backproject(
        self, depth: np.ndarray, camera_transform: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Back-project forward depth and transform camera points to world."""
        valid = np.isfinite(depth) & (depth > 0.0)
        camera_points = np.column_stack(
            (
                depth[valid] * self.x_factor[valid],
                depth[valid] * self.y_factor[valid],
                -depth[valid],
            )
        )
        rotation = quaternion_xyzw_to_matrix(camera_transform[3:7])
        world_points = camera_points @ rotation.T + camera_transform[:3]
        return world_points.astype(np.float32), valid

    def describe_minimum_ray_hit(self, frame: dict[str, np.ndarray]) -> dict[str, Any]:
        depth = frame["ray_depth"]
        valid = depth > 0.0
        flat_valid = np.flatnonzero(valid.ravel())
        flat_index = int(flat_valid[np.argmin(depth.ravel()[flat_valid])])
        shape_id = int(frame["shape_index"].ravel()[flat_index])
        body_id = int(self.model.shape_body.numpy()[shape_id])
        return {
            "depth_m": float(depth.ravel()[flat_index]),
            "pixel_yx": [flat_index // WIDTH, flat_index % WIDTH],
            "shape_index": shape_id,
            "shape_label": self.model.shape_label[shape_id],
            "body_index": body_id,
            "body_label": self.model.body_label[body_id] if body_id >= 0 else "world",
            "is_robot": bool(0 <= shape_id < self.robot_shape_count),
        }


def analyze_cloud(
    scene: DepthCloudScene,
    frame: dict[str, np.ndarray],
    depth_key: str,
) -> dict[str, Any]:
    points, valid = scene.backproject(frame[depth_key], frame["camera_transform"])
    shape_ids = frame["shape_index"][valid]
    robot = shape_ids < scene.robot_shape_count
    environment_points = points[~robot]
    signed = surface_signed_distances(environment_points)
    nearest = np.argmin(np.abs(signed), axis=1)
    nearest_signed = signed[np.arange(len(environment_points)), nearest]

    per_surface: dict[str, dict[str, float | int]] = {}
    for surface_index, surface_name in enumerate(SURFACE_NAMES):
        values = nearest_signed[nearest == surface_index]
        if len(values) > 0:
            per_surface[surface_name] = error_metrics(values)
        else:
            per_surface[surface_name] = {"count": 0}

    return {
        "valid_points": int(len(points)),
        "robot_points": int(robot.sum()),
        "environment_points": int((~robot).sum()),
        "robot_fraction": float(robot.mean()) if len(robot) else 0.0,
        "overall": error_metrics(nearest_signed),
        "per_surface": per_surface,
    }


def run_accuracy_at_mount(
    scene: DepthCloudScene, config: MountConfig
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    scene.set_mount(config)
    correct: dict[str, Any] = {}
    negative: dict[str, Any] = {}
    baseline_frame: dict[str, np.ndarray] | None = None
    for pose_name, pose in ACCURACY_POSES:
        frame = scene.capture(pose)
        if baseline_frame is None:
            baseline_frame = frame
        correct[pose_name] = analyze_cloud(scene, frame, "forward_depth")
        negative[pose_name] = analyze_cloud(scene, frame, "ray_depth")

    assert baseline_frame is not None
    correct_p95 = [correct[name]["overall"]["absolute_p95_mm"] for name, _ in ACCURACY_POSES]
    negative_p95 = [negative[name]["overall"]["absolute_p95_mm"] for name, _ in ACCURACY_POSES]
    negative_control_detects_mistake = bool(
        statistics.median(negative_p95) > statistics.median(correct_p95) + 5.0
        and statistics.median(negative_p95) > 5.0 * max(statistics.median(correct_p95), 1.0e-6)
    )
    result = {
        "correct_forward_depth": correct,
        "negative_ray_depth_as_forward": negative,
        "target": {"absolute_median_lt_mm": 1.0, "absolute_p95_lt_mm": 5.0},
        "all_poses_meet_target": all(
            correct[name]["overall"]["absolute_median_mm"] < 1.0
            and correct[name]["overall"]["absolute_p95_mm"] < 5.0
            for name, _ in ACCURACY_POSES
        ),
        "negative_control_detects_mistake": negative_control_detects_mistake,
        "median_correct_p95_mm": float(statistics.median(correct_p95)),
        "median_negative_p95_mm": float(statistics.median(negative_p95)),
        "mount": config.__dict__,
        "robot_percent_by_pose": {
            name: 100.0 * correct[name]["robot_fraction"] for name, _ in ACCURACY_POSES
        },
        "nearest_hit": scene.describe_minimum_ray_hit(baseline_frame),
    }
    return result, baseline_frame


def run_accuracy(
    scene: DepthCloudScene, initial: MountConfig
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    other = replace(
        initial,
        left_m=initial.left_m + 0.03,
        up_m=initial.up_m - 0.02,
        forward_m=initial.forward_m + 0.03,
        tilt_deg=initial.tilt_deg + 10.0,
        status="accuracy_test_mount",
    )
    initial_result, baseline_frame = run_accuracy_at_mount(scene, initial)
    other_result, _ = run_accuracy_at_mount(scene, other)
    scene.set_mount(initial)
    scene.set_pose(BASE_ARM)
    baseline_frame = scene.capture(BASE_ARM)
    return {"initial_guess": initial_result, "other_mount": other_result}, baseline_frame


def make_display_cloud(
    scene: DepthCloudScene,
    frame: dict[str, np.ndarray],
    stride: int,
) -> dict[str, np.ndarray | float | int]:
    points, valid = scene.backproject(frame["forward_depth"], frame["camera_transform"])
    valid_flat = np.flatnonzero(valid.ravel())
    rows = valid_flat // WIDTH
    cols = valid_flat % WIDTH
    sampled = (rows % stride == 0) & (cols % stride == 0)
    sampled_points = points[sampled]
    sampled_shape_ids = frame["shape_index"].ravel()[valid_flat[sampled]]
    robot = sampled_shape_ids < scene.robot_shape_count

    environment_points = sampled_points[~robot]
    signed = surface_signed_distances(environment_points)
    nearest = np.argmin(np.abs(signed), axis=1)
    nearest_signed = signed[np.arange(len(environment_points)), nearest]
    return {
        "environment_points": environment_points,
        "environment_colors": height_colors(environment_points),
        "robot_points": sampled_points[robot],
        "robot_colors": np.tile(np.array((255, 35, 35), dtype=np.uint8), (int(robot.sum()), 1)),
        "display_point_count": int(len(sampled_points)),
        "full_valid_point_count": int(len(points)),
        "robot_point_count": int((frame["shape_index"][valid] < scene.robot_shape_count).sum()),
        "sampled_robot_point_count": int(robot.sum()),
        "median_error_mm": float(np.median(np.abs(nearest_signed)) * 1000.0),
        "depth_rgb": depth_to_rgb(frame["forward_depth"]),
    }


def build_robot_display_model(urdf: Path) -> tuple[newton.Model, newton.State]:
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    builder.add_urdf(urdf, floating=False)
    model = builder.finalize(device="cpu")
    return model, model.state()


def test_old_free_body_zeroing(urdf: Path, device_name: str) -> dict[str, Any]:
    """Reproduce whether the old q[6:]=0 assignment moves free bodies."""
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    builder.add_urdf(urdf, floating=False)
    bodies = (
        builder.add_body(xform=wp.transform(tuple(BOX_CENTER), wp.quat_identity())),
        builder.add_body(xform=wp.transform(tuple(SPHERE_CENTER), wp.quat_identity())),
        builder.add_body(xform=wp.transform(tuple(CYLINDER_CENTER), wp.quat_identity())),
    )
    expected = np.vstack((BOX_CENTER, SPHERE_CENTER, CYLINDER_CENTER))
    model = builder.finalize(device=wp.get_device(device_name))
    state = model.state()
    template = model.joint_q.numpy()

    def positions(q: np.ndarray) -> np.ndarray:
        model.joint_q.assign(q)
        newton.eval_fk(model, model.joint_q, model.joint_qd, state)
        wp.synchronize_device(model.device)
        return state.body_q.numpy()[list(bodies), :3].copy()

    preserved_q = template.copy()
    preserved_q[:6] = BASE_ARM
    preserved = positions(preserved_q)
    zeroed_q = preserved_q.copy()
    zeroed_q[6:] = 0.0
    zeroed = positions(zeroed_q)
    preserved_error = np.linalg.norm(preserved - expected, axis=1)
    zeroed_error = np.linalg.norm(zeroed - expected, axis=1)
    return {
        "test": "old unjointed bodies with q[6:] preserved versus set to zero",
        "joint_q_length": int(len(template)),
        "expected_positions_m": expected.tolist(),
        "preserved_positions_m": preserved.tolist(),
        "zeroed_positions_m": zeroed.tolist(),
        "preserved_max_error_m": float(preserved_error.max()),
        "zeroed_min_error_m": float(zeroed_error.min()),
        "q_suffix_zeroing_is_cause": bool(
            preserved_error.max() < 1.0e-6 and zeroed_error.min() > 0.05
        ),
    }


class LegacyCloudViewer:
    """Newton robot display plus custom depth/cloud Viser controls."""

    def __init__(
        self,
        scene: DepthCloudScene,
        baseline_frame: dict[str, np.ndarray],
        port: int,
        display_stride: int,
    ):
        self.scene = scene
        self.display_stride = display_stride
        self.robot_model, self.robot_state = build_robot_display_model(scene.urdf)
        self.robot_q_template = self.robot_model.joint_q.numpy()

        server_class = viser.ViserServer
        viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
        try:
            self.viewer = ViewerViser(
                port=port,
                label="Newton link depth cloud",
                verbose=False,
                share=False,
            )
        finally:
            viser.ViserServer = server_class
        self.viewer.set_model(self.robot_model)
        self.server = self.viewer._server
        self.server.scene.set_up_direction("+z")
        self.viewer.set_camera(wp.vec3(1.25, -1.35, 0.95), pitch=-24.0, yaw=126.0)

        self.object_handles = [
            self.server.scene.add_box(
                "/truth/box",
                dimensions=tuple(2.0 * BOX_HALF),
                position=tuple(BOX_CENTER),
                color=(51, 179, 230),
            ),
            self.server.scene.add_icosphere(
                "/truth/sphere",
                radius=SPHERE_RADIUS,
                position=tuple(SPHERE_CENTER),
                color=(230, 102, 51),
            ),
            self.server.scene.add_cylinder(
                "/truth/cylinder",
                radius=CYLINDER_RADIUS,
                height=2.0 * CYLINDER_HALF_HEIGHT,
                position=tuple(CYLINDER_CENTER),
                color=(77, 230, 102),
            ),
            self.server.scene.add_box(
                "/truth/ground",
                dimensions=(1.8, 1.8, 0.01),
                position=(0.45, 0.10, -0.005),
                color=(140, 140, 140),
            ),
        ]

        display = make_display_cloud(scene, baseline_frame, display_stride)
        camera_transform = baseline_frame["camera_transform"]
        frustum_wxyz = self._frustum_wxyz(camera_transform)
        self.environment_cloud = self.server.scene.add_point_cloud(
            "/depth/environment",
            points=display["environment_points"],
            colors=display["environment_colors"],
            point_size=0.006,
            point_shape="circle",
            precision="float32",
        )
        self.robot_cloud = self.server.scene.add_point_cloud(
            "/depth/robot",
            points=display["robot_points"],
            colors=display["robot_colors"],
            point_size=0.007,
            point_shape="circle",
            precision="float32",
        )
        self.frustum = self.server.scene.add_camera_frustum(
            "/camera/frustum",
            fov=VERTICAL_FOV_RAD,
            aspect=WIDTH / HEIGHT,
            scale=0.14,
            color=(255, 215, 0),
            image=display["depth_rgb"],
            format="jpeg",
            jpeg_quality=80,
            position=tuple(camera_transform[:3]),
            wxyz=tuple(frustum_wxyz),
        )
        self.camera_frame = self.server.scene.add_frame(
            "/camera/frame",
            axes_length=0.08,
            axes_radius=0.003,
            position=tuple(camera_transform[:3]),
            wxyz=tuple(camera_transform[[6, 3, 4, 5]]),
        )

        self.server.gui.add_markdown(
            "## Newton 1.5.1 simulated depth cloud\n"
            "848x480 forward depth. Match the unmeasured mount to the real camera by eye, then save it. "
            "A slider change captures one frame; the GPU stays idle otherwise."
        )
        self.depth_panel = self.server.gui.add_image(
            display["depth_rgb"], label="Live forward depth", format="jpeg", jpeg_quality=80
        )
        self.show_objects = self.server.gui.add_checkbox("Show true objects", initial_value=True)
        self.show_cloud = self.server.gui.add_checkbox("Show point cloud", initial_value=True)
        self.show_robot_points = self.server.gui.add_checkbox("Show robot points", initial_value=True)
        self.point_readout = self.server.gui.add_text(
            "Current point count", str(display["full_valid_point_count"]), disabled=True
        )
        self.error_readout = self.server.gui.add_text(
            "Median environment error", f"{display['median_error_mm']:.6f} mm", disabled=True
        )

        lower = self.scene.model.joint_limit_lower.numpy()[:6]
        upper = self.scene.model.joint_limit_upper.numpy()[:6]
        self.sliders = []
        for index, name in enumerate(JOINT_NAMES):
            self.sliders.append(
                self.server.gui.add_slider(
                    name,
                    min=float(lower[index]),
                    max=float(upper[index]),
                    step=0.01,
                    initial_value=float(BASE_ARM[index]),
                )
            )

        self.dirty = threading.Event()
        for slider in self.sliders:
            slider.on_update(lambda _event: self.dirty.set())
        self.show_objects.on_update(self._update_visibility)
        self.show_cloud.on_update(self._update_visibility)
        self.show_robot_points.on_update(self._update_visibility)
        self.push_robot_state(BASE_ARM)
        self._update_visibility(None)
        self.url = f"http://127.0.0.1:{port}"
        print(f"NEWTON_SIM_DEPTH_CLOUD_VISER_URL {self.url}", flush=True)

    @staticmethod
    def _frustum_wxyz(camera_transform: np.ndarray) -> np.ndarray:
        # Newton rays use +X right, +Y up, -Z forward. Viser frustums use the
        # OpenCV convention +X right, +Y down, +Z forward, a pi-X rotation.
        camera_xyzw = np.asarray(camera_transform[3:7], dtype=np.float64)
        viser_xyzw = quaternion_multiply_xyzw(camera_xyzw, np.array((1.0, 0.0, 0.0, 0.0)))
        return viser_xyzw[[3, 0, 1, 2]]

    def current_q(self) -> np.ndarray:
        return np.asarray([slider.value for slider in self.sliders], dtype=np.float32)

    def push_robot_state(self, arm_q: np.ndarray) -> None:
        q = self.robot_q_template.copy()
        q[:6] = arm_q
        q[6:] = 0.0
        self.robot_model.joint_q.assign(q)
        newton.eval_fk(
            self.robot_model,
            self.robot_model.joint_q,
            self.robot_model.joint_qd,
            self.robot_state,
        )
        self.viewer.begin_frame(0.0)
        self.viewer.log_state(self.robot_state)
        self.viewer.end_frame()

    def push_frame(
        self,
        frame: dict[str, np.ndarray],
        arm_q: np.ndarray,
        display: dict[str, np.ndarray | float | int] | None = None,
    ) -> None:
        if display is None:
            display = make_display_cloud(self.scene, frame, self.display_stride)
        camera_transform = frame["camera_transform"]
        self.environment_cloud.points = display["environment_points"]
        self.environment_cloud.colors = display["environment_colors"]
        self.robot_cloud.points = display["robot_points"]
        self.robot_cloud.colors = display["robot_colors"]
        self.frustum.position = tuple(camera_transform[:3])
        self.frustum.wxyz = tuple(self._frustum_wxyz(camera_transform))
        self.frustum.image = display["depth_rgb"]
        self.camera_frame.position = tuple(camera_transform[:3])
        self.camera_frame.wxyz = tuple(camera_transform[[6, 3, 4, 5]])
        self.depth_panel.image = display["depth_rgb"]
        self.point_readout.value = str(display["full_valid_point_count"])
        self.error_readout.value = f"{display['median_error_mm']:.6f} mm"
        self.push_robot_state(arm_q)
        self._update_visibility(None)

    def _update_visibility(self, _event: Any) -> None:
        for handle in self.object_handles:
            handle.visible = self.show_objects.value
        self.environment_cloud.visible = self.show_cloud.value
        self.robot_cloud.visible = self.show_cloud.value and self.show_robot_points.value

    def update_from_sliders(self) -> None:
        arm_q = self.current_q()
        frame = self.scene.capture(arm_q)
        self.push_frame(frame, arm_q)

    def close(self) -> None:
        self.viewer.close()


def arrow_segments(origin: np.ndarray, direction: np.ndarray, length: float = 0.13) -> np.ndarray:
    """Make a shaft and two arrow-head strokes for a labelled model axis."""
    direction = normalized(direction)
    end = origin + length * direction
    reference = np.array((0.0, 0.0, 1.0))
    if abs(float(np.dot(reference, direction))) > 0.9:
        reference = np.array((0.0, 1.0, 0.0))
    side = normalized(np.cross(direction, reference))
    head = end - 0.025 * direction
    return np.asarray(
        ((origin, end), (end, head + 0.010 * side), (end, head - 0.010 * side))
    )


class CloudViewer(LegacyCloudViewer):
    """Legacy point-cloud viewer extended with a calibrated mount panel."""

    def __init__(
        self,
        scene: DepthCloudScene,
        baseline_frame: dict[str, np.ndarray],
        port: int,
        display_stride: int,
        mount_path: Path,
        mount_template: str,
        initial_mount: MountConfig,
    ):
        self.mount_path = mount_path
        self.mount_template = mount_template
        self.initial_mount = initial_mount
        self.mount_slider_callback_count = 0
        self.save_callback_count = 0
        super().__init__(scene, baseline_frame, port, display_stride)

        display = make_display_cloud(scene, baseline_frame, display_stride)
        initial_percent = 100.0 * float(display["robot_point_count"]) / max(
            int(display["full_valid_point_count"]), 1
        )
        self.robot_percent_readout = self.server.gui.add_text(
            "Robot points", f"{initial_percent:.4f}%", disabled=True
        )
        with self.server.gui.add_folder("Camera mount"):
            self.mount_sliders = {
                "left_m": self.server.gui.add_slider(
                    "left_m", min=-0.20, max=0.20, step=0.001,
                    initial_value=initial_mount.left_m,
                ),
                "up_m": self.server.gui.add_slider(
                    "up_m", min=-0.20, max=0.20, step=0.001,
                    initial_value=initial_mount.up_m,
                ),
                "forward_m": self.server.gui.add_slider(
                    "forward_m", min=-0.10, max=0.20, step=0.001,
                    initial_value=initial_mount.forward_m,
                ),
                "tilt_deg": self.server.gui.add_slider(
                    "tilt_deg", min=-60.0, max=60.0, step=1.0,
                    initial_value=initial_mount.tilt_deg,
                ),
            }
            self.mount_readout = self.server.gui.add_text(
                "Current mount", self.mount_text(initial_mount), disabled=True
            )
            self.save_button = self.server.gui.add_button("Save mount")
            self.save_status = self.server.gui.add_text(
                "Save status", initial_mount.status, disabled=True
            )
            self.show_mount_axes = self.server.gui.add_checkbox(
                "Show LEFT / UP / FORWARD axes", initial_value=True
            )

        axes = scene.mount_axis_world()
        colors = {
            "left": (255, 70, 70),
            "up": (70, 255, 70),
            "forward": (70, 140, 255),
        }
        self.axis_arrows: dict[str, Any] = {}
        self.axis_labels: dict[str, Any] = {}
        for name in ("left", "up", "forward"):
            self.axis_arrows[name] = self.server.scene.add_line_segments(
                f"/mount_axes/{name}",
                points=arrow_segments(axes["origin"], axes[name]),
                colors=colors[name],
                line_width=3.0,
            )
            self.axis_labels[name] = self.server.scene.add_label(
                f"/mount_axes/{name}_label",
                name.upper(),
                position=tuple(axes["origin"] + 0.145 * axes[name]),
            )

        for slider in self.mount_sliders.values():
            slider.on_update(self._on_mount_slider)
        self.save_button.on_click(self._save_mount)
        self.show_mount_axes.on_update(self._update_visibility)
        self._update_visibility(None)

    @staticmethod
    def mount_text(config: MountConfig) -> str:
        return (
            f"left={config.left_m:+.3f} m, up={config.up_m:+.3f} m, "
            f"forward={config.forward_m:+.3f} m, tilt={config.tilt_deg:+.1f} deg"
        )

    def current_mount(self, status: str | None = None) -> MountConfig:
        return MountConfig(
            parent_link=self.initial_mount.parent_link,
            left_m=float(self.mount_sliders["left_m"].value),
            up_m=float(self.mount_sliders["up_m"].value),
            forward_m=float(self.mount_sliders["forward_m"].value),
            tilt_deg=float(self.mount_sliders["tilt_deg"].value),
            status=status or self.scene.mount_config.status,
        )

    def _on_mount_slider(self, _event: Any) -> None:
        self.mount_slider_callback_count += 1
        self.dirty.set()

    def _save_mount(self, _event: Any) -> None:
        saved = self.current_mount(status="matched_by_eye")
        self.mount_template = save_mount_config(
            self.mount_path, self.mount_template, saved
        )
        self.scene.mount_config = saved
        self.save_status.value = "matched_by_eye (saved)"
        self.save_callback_count += 1

    def push_frame(
        self,
        frame: dict[str, np.ndarray],
        arm_q: np.ndarray,
        display: dict[str, np.ndarray | float | int] | None = None,
    ) -> None:
        if display is None:
            display = make_display_cloud(self.scene, frame, self.display_stride)
        super().push_frame(frame, arm_q, display)
        robot_percent = 100.0 * float(display["robot_point_count"]) / max(
            int(display["full_valid_point_count"]), 1
        )
        self.robot_percent_readout.value = f"{robot_percent:.4f}%"
        self.mount_readout.value = self.mount_text(self.scene.mount_config)
        axes = self.scene.mount_axis_world()
        for name in ("left", "up", "forward"):
            self.axis_arrows[name].points = arrow_segments(axes["origin"], axes[name])
            self.axis_labels[name].position = tuple(
                axes["origin"] + 0.145 * axes[name]
            )

    def _update_visibility(self, event: Any) -> None:
        super()._update_visibility(event)
        if hasattr(self, "axis_arrows"):
            for handle in (*self.axis_arrows.values(), *self.axis_labels.values()):
                handle.visible = self.show_mount_axes.value

    def update_from_controls(self) -> None:
        arm_q = self.current_q()
        self.scene.set_mount(self.current_mount())
        self.push_frame(self.scene.capture(arm_q), arm_q)

    def exercise_mount_callbacks(self) -> dict[str, Any]:
        """Exercise real Viser callbacks, then restore the YAML byte-for-byte."""
        original_text = self.mount_path.read_text(encoding="utf-8")
        slider_before = self.mount_slider_callback_count
        save_before = self.save_callback_count
        recaptured = False
        saved_status = "not_observed"
        slider_delta = 0
        save_delta = 0
        try:
            increments = {
                "left_m": 0.001,
                "up_m": -0.001,
                "forward_m": 0.001,
                "tilt_deg": 1.0,
            }
            for name, increment in increments.items():
                self.mount_sliders[name].value = (
                    float(self.mount_sliders[name].value) + increment
                )
            if not self.dirty.is_set():
                raise RuntimeError("Mount slider callbacks did not mark the viewer dirty")
            self.dirty.clear()
            self.update_from_controls()
            recaptured = True
            self.save_button.value = True
            saved_status = read_mount_config(self.mount_path)[0].status
            slider_delta = self.mount_slider_callback_count - slider_before
            save_delta = self.save_callback_count - save_before
            if saved_status != "matched_by_eye" or save_delta != 1:
                raise RuntimeError("Save mount button callback did not persist matched_by_eye")
        finally:
            self.mount_path.write_text(original_text, encoding="utf-8")
            self.mount_template = original_text
            for name in self.mount_sliders:
                self.mount_sliders[name].value = float(
                    getattr(self.initial_mount, name)
                )
            self.scene.set_mount(self.initial_mount)
            self.push_frame(self.scene.capture(BASE_ARM), BASE_ARM)
            self.dirty.clear()
        return {
            "slider_callbacks": slider_delta,
            "save_callbacks": save_delta,
            "recaptured_after_slider_callbacks": recaptured,
            "saved_status_observed": saved_status,
            "yaml_restored_exact": (
                self.mount_path.read_text(encoding="utf-8") == original_text
            ),
        }


def measure_call(call: Callable[[], None], frames: int) -> dict[str, Any]:
    samples: list[float] = []
    for _ in range(frames):
        started = time.perf_counter()
        call()
        samples.append((time.perf_counter() - started) * 1000.0)
    return summarize_samples(samples)


def run_timings(
    scene: DepthCloudScene,
    viewer: CloudViewer,
    baseline_frame: dict[str, np.ndarray],
    frames: int,
) -> dict[str, Any]:
    scene.set_pose(BASE_ARM)
    for _ in range(10):
        scene.render()
    wp.synchronize_device(scene.device)

    def render_once() -> None:
        scene.render()
        wp.synchronize_device(scene.device)

    render_timing = measure_call(render_once, frames)
    scene.render()
    wp.synchronize_device(scene.device)

    copied: dict[str, np.ndarray] = {}

    def copy_once() -> None:
        nonlocal copied
        copied = scene.copy_same_frame()

    copy_timing = measure_call(copy_once, frames)
    if not copied:
        copied = baseline_frame

    converted: tuple[np.ndarray, np.ndarray] | None = None

    def convert_once() -> None:
        nonlocal converted
        converted = scene.backproject(copied["forward_depth"], copied["camera_transform"])

    conversion_timing = measure_call(convert_once, frames)
    if converted is None:
        raise RuntimeError("Point cloud timing produced no result")

    display = make_display_cloud(scene, copied, viewer.display_stride)

    def viser_once() -> None:
        viewer.push_frame(copied, BASE_ARM, display)

    viser_timing = measure_call(viser_once, frames)
    return {
        "frames_each": frames,
        "depth_render_forward_raw_and_shape_index": render_timing,
        "copy_forward_raw_shape_and_pose_to_host": copy_timing,
        "point_cloud_conversion_full_resolution": conversion_timing,
        "viser_update_stride": viewer.display_stride,
        "viser_display_points": int(display["display_point_count"]),
        "viser_full_update": viser_timing,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
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
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8083)
    parser.add_argument("--timing-frames", type=int, default=120)
    parser.add_argument("--display-stride", type=int, default=3)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()

    if args.host != "127.0.0.1":
        raise ValueError("This safety-scoped viewer must bind to 127.0.0.1")
    if not args.urdf.is_file():
        raise FileNotFoundError(args.urdf)
    if not args.mount_config.is_file():
        raise FileNotFoundError(args.mount_config)
    if args.timing_frames < 120:
        raise ValueError("--timing-frames must be at least 120")
    if args.display_stride <= 0:
        raise ValueError("--display-stride must be positive")

    initial_mount, mount_template = read_mount_config(args.mount_config)
    scene = DepthCloudScene(args.urdf, args.device, initial_mount)
    accuracy, baseline_frame = run_accuracy(scene, initial_mount)
    old_free_body_test = test_old_free_body_zeroing(args.urdf, args.device)
    scene.set_pose(BASE_ARM)
    viewer = CloudViewer(
        scene,
        baseline_frame,
        args.port,
        args.display_stride,
        args.mount_config,
        mount_template,
        initial_mount,
    )
    try:
        callback_check = viewer.exercise_mount_callbacks() if args.check_only else None
        timings = run_timings(scene, viewer, baseline_frame, args.timing_frames)
        scene.set_mount(initial_mount)
        scene.set_pose(BASE_ARM)
        baseline_frame = scene.capture(BASE_ARM)
        viewer.push_frame(baseline_frame, BASE_ARM)

        ur_variant = "UR5" if abs(scene.shoulder_height_m - 0.089159) < 0.01 else "not_UR5"
        result = {
            "newton_version": newton.__version__,
            "warp_version": wp.__version__,
            "device": str(scene.device),
            "camera": {
                "resolution": [WIDTH, HEIGHT],
                "camera_fovs_argument": "vertical field of view in radians",
                "vertical_fov_deg": VERTICAL_FOV_DEG,
                "horizontal_fov_deg": math.degrees(
                    2.0 * math.atan(math.tan(VERTICAL_FOV_RAD / 2.0) * WIDTH / HEIGHT)
                ),
                "depth_channel": "forward_depth_image",
                "mount_config": initial_mount.__dict__,
                "mount_axes": {
                    key: value.tolist() if isinstance(value, np.ndarray) else value
                    for key, value in scene.mount_axes.items()
                },
            },
            "robot": {
                "urdf": str(args.urdf),
                "shoulder_height_m": scene.shoulder_height_m,
                "identified_variant": ur_variant,
                "ur5_reference_m": 0.0892,
                "ur5e_reference_m": 0.1625,
                "robot_bodies": scene.robot_body_count,
                "robot_shapes": scene.robot_shape_count,
            },
            "accuracy": accuracy,
            "old_free_body_zeroing_test": old_free_body_test,
            "callback_check": callback_check,
            "timing": timings,
            "viser": {
                "url": viewer.url,
                "bind": args.host,
                "port": args.port,
                "idle_policy": "event-driven; capture only after a slider update",
            },
        }
        print("NEWTON_SIM_DEPTH_CLOUD_RESULT " + json.dumps(result, sort_keys=True), flush=True)

        if args.check_only:
            return

        print("Move a joint slider in the browser; press Ctrl-C here to stop.", flush=True)
        while viewer.viewer.is_running():
            if viewer.dirty.wait(timeout=0.25):
                viewer.dirty.clear()
                viewer.update_from_controls()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
