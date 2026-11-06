#!/usr/bin/env python3
"""Render Newton depth, back-project a world cloud, and inspect it in Viser."""

from __future__ import annotations

import argparse
import functools
import json
import math
from pathlib import Path
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
CAMERA_LOCAL_TRANSLATION = (0.18, 0.0, -0.08)

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


@wp.kernel(enable_backward=False)
def update_link_camera_transform(
    body_q: wp.array(dtype=wp.transform),
    shape_body: wp.array(dtype=wp.int32),
    shape_transform: wp.array(dtype=wp.transform),
    camera_site: wp.int32,
    camera_transforms: wp.array2d(dtype=wp.transform),
):
    """Compose the fixed camera-site transform with its live parent-link pose."""
    parent = shape_body[camera_site]
    camera_transforms[0, 0] = wp.transform_multiply(
        body_q[parent], shape_transform[camera_site]
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

    def __init__(self, urdf: Path, device_name: str):
        newton.use_coord_layout_targets = True
        self.urdf = urdf
        self.device = wp.get_device(device_name)

        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        builder.add_urdf(urdf, floating=False)
        self.robot_body_count = builder.body_count
        self.robot_shape_count = builder.shape_count
        self.camera_parent = builder.body_label.index("ur/tool0")
        self.camera_site = builder.add_site(
            self.camera_parent,
            xform=wp.transform(
                wp.vec3(*CAMERA_LOCAL_TRANSLATION),
                wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), math.pi),
            ),
            label="ur/tool0_depth_camera_placeholder",
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

    def set_pose(self, arm_q: np.ndarray) -> None:
        q = self.q_template.copy()
        q[:6] = np.asarray(arm_q, dtype=np.float32)
        q[6:] = 0.0
        self.model.joint_q.assign(q)
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state)
        self.model.bvh_refit_shapes(self.state)
        self.update_camera_transform()

    def update_camera_transform(self) -> None:
        wp.launch(
            update_link_camera_transform,
            dim=1,
            inputs=[
                self.state.body_q,
                self.model.shape_body,
                self.model.shape_transform,
                self.camera_site,
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


def run_accuracy(scene: DepthCloudScene) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
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
        "previous_minimum_ray_hit": scene.describe_minimum_ray_hit(baseline_frame),
    }
    return result, baseline_frame


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
        "robot_point_count": int(robot.sum()),
        "median_error_mm": float(np.median(np.abs(nearest_signed)) * 1000.0),
        "depth_rgb": depth_to_rgb(frame["forward_depth"]),
    }


def build_robot_display_model(urdf: Path) -> tuple[newton.Model, newton.State]:
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    builder.add_urdf(urdf, floating=False)
    model = builder.finalize(device="cpu")
    return model, model.state()


class CloudViewer:
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
            "848x480 forward depth; the tool0 camera mount is a **placeholder**, not the real mount. "
            "Move a joint slider to capture one new frame. The GPU stays idle otherwise."
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
    if args.timing_frames < 120:
        raise ValueError("--timing-frames must be at least 120")
    if args.display_stride <= 0:
        raise ValueError("--display-stride must be positive")

    scene = DepthCloudScene(args.urdf, args.device)
    accuracy, baseline_frame = run_accuracy(scene)
    scene.set_pose(BASE_ARM)
    viewer = CloudViewer(scene, baseline_frame, args.port, args.display_stride)
    try:
        timings = run_timings(scene, viewer, baseline_frame, args.timing_frames)
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
                "mount": "tool0 placeholder; not the real camera mount",
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
                viewer.update_from_sliders()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
