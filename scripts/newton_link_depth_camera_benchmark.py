#!/usr/bin/env python3
"""Benchmark a Newton depth camera rigidly attached to a robot link."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np
import warp as wp

import newton
from newton.sensors import SensorTiledCamera


BASE_ARM = np.array(
    [0.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0, -math.pi / 2.0, 0.0],
    dtype=np.float32,
)


@wp.kernel(enable_backward=False)
def update_link_camera_transform(
    body_q: wp.array(dtype=wp.transform),
    shape_body: wp.array(dtype=wp.int32),
    shape_transform: wp.array(dtype=wp.transform),
    camera_site: wp.int32,
    camera_transforms: wp.array2d(dtype=wp.transform),
):
    """Compose the camera site's fixed local transform with its link pose."""
    parent = shape_body[camera_site]
    camera_transforms[0, 0] = wp.transform_multiply(
        body_q[parent], shape_transform[camera_site]
    )


def depth_stats(depth: np.ndarray) -> dict[str, float | int | list[int]]:
    valid = depth > 0.0
    values = depth[valid]
    if values.size == 0:
        raise RuntimeError("Depth render contained no valid pixels")
    return {
        "shape": list(depth.shape),
        "valid_pixels": int(values.size),
        "valid_fraction": float(values.size / depth.size),
        "min_m": float(values.min()),
        "median_m": float(np.median(values)),
        "max_m": float(values.max()),
    }


def summarize_timings(samples_ms: list[float]) -> dict[str, float | list[float]]:
    return {
        "samples_ms_per_frame": samples_ms,
        "median_ms_per_frame": float(statistics.median(samples_ms)),
        "min_ms_per_frame": float(min(samples_ms)),
        "max_ms_per_frame": float(max(samples_ms)),
        "median_fps": float(1000.0 / statistics.median(samples_ms)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()

    if min(args.width, args.height, args.frames, args.repeats) <= 0 or args.warmup < 0:
        raise ValueError("Image dimensions, frames, and repeats must be positive; warmup must be non-negative")
    if not args.urdf.is_file():
        raise FileNotFoundError(args.urdf)

    newton.use_coord_layout_targets = True
    device = wp.get_device(args.device)

    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    builder.add_urdf(args.urdf, floating=False)
    camera_parent = builder.body_label.index("ur/tool0")

    # The camera is mounted beside and behind tool0 so the gripper does not
    # occlude its entire view. Rotating 180 degrees about local X makes the
    # camera's -Z optical axis look down in the verified BASE_ARM pose.
    camera_local_transform = wp.transform(
        wp.vec3(0.18, 0.0, -0.08),
        wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), math.pi),
    )
    camera_site = builder.add_site(
        camera_parent,
        xform=camera_local_transform,
        label="ur/tool0_depth_camera",
    )

    # Static objects below the eye-in-hand camera provide depth structure.
    box = builder.add_body(
        xform=wp.transform((0.487, 0.109, 0.06), wp.quat_identity())
    )
    builder.add_shape_box(box, hx=0.12, hy=0.10, hz=0.06, color=(0.2, 0.7, 0.9))
    sphere = builder.add_body(
        xform=wp.transform((0.68, 0.05, 0.08), wp.quat_identity())
    )
    builder.add_shape_sphere(sphere, radius=0.08, color=(0.9, 0.4, 0.2))
    cylinder = builder.add_body(
        xform=wp.transform((0.32, 0.18, 0.10), wp.quat_identity())
    )
    builder.add_shape_cylinder(
        cylinder, radius=0.055, half_height=0.10, color=(0.3, 0.9, 0.4)
    )
    builder.add_ground_plane(color=(0.55, 0.55, 0.55))

    model = builder.finalize(device=device)
    state = model.state()
    q = model.joint_q.numpy()
    q[:6] = BASE_ARM
    q[6:] = 0.0
    model.joint_q.assign(q)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    model.bvh_refit_shapes(state)

    parent_from_model = int(model.shape_body.numpy()[camera_site])
    site_flag = bool(
        int(model.shape_flags.numpy()[camera_site]) & int(newton.ShapeFlags.SITE)
    )
    if parent_from_model != camera_parent or not site_flag:
        raise RuntimeError("Camera site is not attached to the requested robot link")

    sensor = SensorTiledCamera(model=model, load_textures=False)
    camera_rays = sensor.utils.compute_camera_rays_pinhole(
        args.width,
        args.height,
        camera_fovs=math.radians(58.0),
    )
    depth_image = sensor.utils.create_depth_image_output(args.width, args.height)
    camera_transforms = wp.zeros(
        (1, model.world_count), dtype=wp.transform, device=device
    )

    def update_camera() -> None:
        wp.launch(
            update_link_camera_transform,
            dim=1,
            inputs=[
                state.body_q,
                model.shape_body,
                model.shape_transform,
                camera_site,
            ],
            outputs=[camera_transforms],
            device=device,
        )

    def render_depth() -> None:
        sensor.update(
            state,
            camera_transforms,
            camera_rays,
            depth_image=depth_image,
        )

    # First render includes any process-local lazy initialization and cached
    # Warp module loads. It is reported separately from steady state.
    cold_start = time.perf_counter()
    update_camera()
    render_depth()
    wp.synchronize_device(device)
    first_frame_ms = (time.perf_counter() - cold_start) * 1000.0

    camera_a = camera_transforms.numpy()[0, 0].copy()
    depth_a = depth_image.numpy()[0, 0].copy()

    # Change an upstream joint, then prove that the camera world pose and its
    # rendered depth both change while its site-to-link transform stays fixed.
    q_pose_b = q.copy()
    q_pose_b[1] += 0.25
    model.joint_q.assign(q_pose_b)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    model.bvh_refit_shapes(state)
    update_camera()
    render_depth()
    wp.synchronize_device(device)
    camera_b = camera_transforms.numpy()[0, 0].copy()
    depth_b = depth_image.numpy()[0, 0].copy()

    camera_translation_delta = float(np.linalg.norm(camera_b[:3] - camera_a[:3]))
    depth_changed_fraction = float(np.mean(np.abs(depth_b - depth_a) > 1.0e-4))
    if camera_translation_delta <= 1.0e-3 or depth_changed_fraction <= 1.0e-3:
        raise RuntimeError(
            "The attached camera or depth image did not respond to robot motion: "
            f"camera_translation_delta_m={camera_translation_delta}, "
            f"depth_pixels_changed_fraction={depth_changed_fraction}, "
            f"pose_a={camera_a.tolist()}, pose_b={camera_b.tolist()}, "
            f"depth_a={depth_stats(depth_a)}, depth_b={depth_stats(depth_b)}"
        )

    # Restore the baseline pose for stable timing runs.
    model.joint_q.assign(q)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    model.bvh_refit_shapes(state)
    update_camera()

    for _ in range(args.warmup):
        update_camera()
        render_depth()
    wp.synchronize_device(device)

    render_samples_ms: list[float] = []
    for _ in range(args.repeats):
        started = time.perf_counter()
        for _ in range(args.frames):
            update_camera()
            render_depth()
        wp.synchronize_device(device)
        render_samples_ms.append(
            (time.perf_counter() - started) * 1000.0 / args.frames
        )

    pipeline_samples_ms: list[float] = []
    for _ in range(args.repeats):
        started = time.perf_counter()
        for _ in range(args.frames):
            newton.eval_fk(model, model.joint_q, model.joint_qd, state)
            model.bvh_refit_shapes(state)
            update_camera()
            render_depth()
        wp.synchronize_device(device)
        pipeline_samples_ms.append(
            (time.perf_counter() - started) * 1000.0 / args.frames
        )

    final_depth = depth_image.numpy()[0, 0]
    result = {
        "newton_version": newton.__version__,
        "warp_version": wp.__version__,
        "device": str(device),
        "device_name": getattr(device, "name", str(device)),
        "urdf": str(args.urdf),
        "model": {
            "bodies": model.body_count,
            "joints": model.joint_count,
            "shapes": model.shape_count,
        },
        "camera": {
            "parent_link": model.body_label[camera_parent],
            "parent_body_index": camera_parent,
            "site_shape_index": camera_site,
            "site_flag": site_flag,
            "local_translation_m": [0.18, 0.0, -0.08],
            "vertical_fov_deg": 58.0,
            "resolution": [args.width, args.height],
            "output": "ray-hit depth in metres, float32",
        },
        "attachment_validation": {
            "pose_a_camera_transform_xyzw": camera_a.astype(float).tolist(),
            "pose_b_camera_transform_xyzw": camera_b.astype(float).tolist(),
            "camera_translation_delta_m": camera_translation_delta,
            "depth_pixels_changed_fraction": depth_changed_fraction,
            "pose_a_depth": depth_stats(depth_a),
            "pose_b_depth": depth_stats(depth_b),
        },
        "timing": {
            "first_frame_this_process_ms": first_frame_ms,
            "warmup_frames": args.warmup,
            "frames_per_repeat": args.frames,
            "repeats": args.repeats,
            "render_depth_only": summarize_timings(render_samples_ms),
            "robot_frame_pipeline": summarize_timings(pipeline_samples_ms),
        },
        "final_depth": depth_stats(final_depth),
    }
    print("NEWTON_LINK_DEPTH_CAMERA_RESULT " + json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
