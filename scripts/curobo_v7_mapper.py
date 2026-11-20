#!/usr/bin/env python3
"""Fixed/wrist Newton RGB-D input to the cuRobo live volumetric mapping MPC pipeline.

Pipeline reference: NVlabs/cuRobo, curobo/examples/reference/live_volumetric_mapping_mpc.py.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
FIXED_CONFIG = ROOT/'config/sim_fixed_camera.yaml'
NEWTON_URDF = Path.home()/'newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf'
BASE_LINK = 'base_link_inertia'
DEPTH_MIN, DEPTH_MAX = .15, 2.
TSDF_SIZE, ESDF_SIZE = .005, .03
FRAME_RATE, CONTROL_DT = 30, .025
DURATION = 10.
BAR_TIP_INNER_X, BAR_STROKE_M, BAR_SPEED_MPS = .50, .125, .10
BAR_LANE_Y, BAR_HEIGHT_M = .36, .23
RESULTS_DIR = ROOT/'docs/experiments/sim-camera-curobo/results'
BAR_LENGTH_M, BAR_RADIUS_M = .30, .02
OBSTACLE_ACTIVATION_M = .07
FLOOR_ACTIVATION_M = .01
DIAGNOSIS_S = 10.
ESDF_NO_SITE_M = 1e4
DIAGNOSIS_FILE = ROOT/'docs/experiments/sim-camera-curobo/results/v7_diagnosis.json'
FORENSICS_FILE = DIAGNOSIS_FILE.with_name('v7_bar_forensics.json')
DECAY_COMPARISON_FILE = DIAGNOSIS_FILE.with_name('v7_decay_comparison.json')
DECAY_CONFIGS = {'A': (.3, 1.), 'B': (1., 1.), 'C': (1., .3)}
DEFAULT_DECAY = DECAY_CONFIGS['A']
ROBOT_MASK_DISTANCE = .015  # Moving-link maximum rounded up to the next mm.
CALIBRATION_FILE = DIAGNOSIS_FILE.with_name('v7_segmenter_calibration.json')
FAIR_SCENE_FILE = DIAGNOSIS_FILE.with_name('v7_reciprocating_checkpoint.json')
SEGMENTATION_CHECKPOINT_FILE = DIAGNOSIS_FILE.with_name('v7_segmentation_checkpoint.json')
CAMERA_LABELS = ('固定相機', '手上相機')
PARK = np.array([2., .28, .20])


def camera_basis(config):
    origin = np.asarray(config['position_m'], dtype=float)
    forward = np.asarray(config['look_at_m'])-origin
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, config['up'])
    right /= np.linalg.norm(right)
    return origin, np.column_stack((right, np.cross(forward, right), forward))


def render_worker(mode):
    """Render RGB-D, with a separate ground-truth count for forensics only."""
    import newton_camera_moving_rod_worker as source

    mount, _ = source.camera_common.read_mount_config(ROOT/'config/sim_camera_mount.yaml')
    scene = source.MovingRodCamera(
        NEWTON_URDF, mount,
        'cuda:0', horizontal_bar=True, bar_length=BAR_LENGTH_M,
    )
    width, height = source.WIDTH, source.HEIGHT
    color = scene.sensor.utils.create_color_image_output(width, height)
    origin, fixed = camera_basis(yaml.safe_load(FIXED_CONFIG.read_text()))
    convention = np.diag([1., -1., -1.])
    fixed_transform = np.array(
        [*origin, *source.camera_common.matrix_to_quaternion_xyzw(fixed @ convention)],
        dtype=np.float32).reshape(1, 1, 7)
    focal = height/(2*np.tan(source.VERTICAL_FOV_RAD/2))
    shape_bodies = scene.model.shape_body.numpy()[:scene.robot_shape_count]
    links = [scene.model.body_label[i] if i >= 0 else 'world' for i in shape_bodies]
    print('V7_RENDERER '+json.dumps(dict(width=width, height=height, focal=focal, robot_shape_links=links)),
          flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        if request.get('shutdown'):
            return
        scene._assign_pose(np.asarray(request['q'], dtype=np.float32), *request['bar'], request['rod'])
        depths, rgbs, poses, bar_pixels, robot_shapes = [], [], [], [], []
        calibrate = request.get('robot_shape_ids')
        count_bar = request.get('count_bar_pixels')
        shape_image = scene.shape_index if calibrate or count_bar else None
        scene.camera_transforms.assign(fixed_transform)
        for i in range(1 if mode == 'fixed' else 2):
            if i:
                source.wp.launch(source.compose_camera, dim=1,
                                 inputs=[scene.state.body_q, scene.parent, scene.camera_local],
                                 outputs=[scene.camera_transforms], device=scene.device)
            scene.sensor.update(scene.state, scene.camera_transforms, scene.rays,
                                forward_depth_image=scene.depth, color_image=color, shape_index_image=shape_image)
            source.wp.synchronize_device(scene.device)
            if count_bar or calibrate:
                ids = shape_image.numpy()[0, 0]
                if count_bar:
                    bar_pixels.append(ids == scene.rod_shape)
                if calibrate:
                    robot_shapes.append(np.where(ids < len(links), ids.astype(np.int32), -1))
            depths.append(scene.depth.numpy()[0, 0].copy())
            rgba = color.numpy()[0, 0]
            rgbs.append(np.stack([(rgba >> shift) & 255 for shift in (0, 8, 16)], axis=-1).astype(np.uint8))
            pose = scene.camera_transforms.numpy()[0, 0]
            rotation = source.camera_common.quaternion_xyzw_to_matrix(pose[3:]) @ convention
            orientation = source.camera_common.matrix_to_quaternion_xyzw(rotation)
            poses.append([*pose[:3], orientation[3], *orientation[:3]])
        arrays = dict(depth=depths, rgb=rgbs, poses=poses)
        if count_bar:
            arrays['bar_pixels'] = bar_pixels
        if calibrate:
            arrays['robot_shapes'] = robot_shapes
        np.savez(request['output'], **arrays)
        print('V7_FRAME', flush=True)


class NewtonStream:
    def __init__(self, args, directory):
        self.output = directory/'rgbd.npz'
        self.count_bar_pixels = args.bar_forensics
        self.bar_pixels = None
        self.process = subprocess.Popen(
            [str(args.newton_python), str(Path(__file__).resolve()), '--renderer', args.cameras],
            cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
        )
        try:
            self.calibration = json.loads(self.read('V7_RENDERER '))
        except BaseException:
            self.close()
            raise

    def read(self, prefix):
        recent = []
        for line in self.process.stdout:
            if line.startswith(prefix):
                return line[len(prefix):].strip()
            recent.append(line)
            recent = recent[-12:]
        raise RuntimeError(f'Newton stream exited while waiting for {prefix}: '+''.join(recent))

    def capture(self, q, bar, rod, *, robot_shape_ids=False):
        self.process.stdin.write(json.dumps(dict(q=q.tolist(), bar=bar.tolist(), rod=rod,
                                                 count_bar_pixels=self.count_bar_pixels, robot_shape_ids=robot_shape_ids,
                                                 output=str(self.output)))+'\n')
        self.process.stdin.flush()
        self.read('V7_FRAME')
        with np.load(self.output) as arrays:
            if robot_shape_ids:
                self.robot_shapes = arrays['robot_shapes']
            if self.count_bar_pixels:
                self.bar_pixels = arrays['bar_pixels']
            return arrays['depth'], arrays['rgb'], arrays['poses']

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.write('{"shutdown":true}\n')
                self.process.stdin.flush()
                self.process.wait(timeout=10.)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                self.process.terminate()
                self.process.wait(timeout=5.)


def fixed_base_volume():
    """Compute world bounds from the fixed base link's URDF collision mesh."""
    import xml.etree.ElementTree as ET

    import trimesh
    from trimesh.transformations import euler_matrix

    robot = ET.parse(NEWTON_URDF).getroot()

    def transform(node):
        origin = node.find('origin')
        if origin is None:
            return np.eye(4)
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        matrix = euler_matrix(*rpy)
        matrix[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        return matrix

    link = robot.find(f"link[@name='{BASE_LINK}']")
    parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    current = BASE_LINK
    fixed_world = np.eye(4)
    while current in parents:
        joint = parents[current]
        if joint.get('type') != 'fixed':
            raise ValueError(f'{BASE_LINK} is not fixed')
        fixed_world = transform(joint) @ fixed_world
        current = joint.find('parent').get('link')
    vertices = []
    for collision in link.findall('collision'):
        mesh = collision.find('geometry/mesh')
        filename = NEWTON_URDF.parent/mesh.get('filename')
        scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        points = trimesh.load_mesh(filename).vertices*scale
        vertices.append(trimesh.transform_points(points, fixed_world @ transform(collision)))
    vertices = np.concatenate(vertices)
    margin = .01
    return dict(link=BASE_LINK, source=NEWTON_URDF.name, geometry='collision mesh world AABB',
                margin_m=margin, min_m=(vertices.min(axis=0)-margin).tolist(),
                max_m=(vertices.max(axis=0)+margin).tolist())


class LiveMapping:
    """Adapt the reference example's FilterDepth, Mapper, and camera batching."""
    def __init__(self, calibration, count, robot, *, depth_max=DEPTH_MAX, decay=DEFAULT_DECAY):
        import torch
        from curobo.kinematics import Kinematics
        from curobo.perception import FilterDepth, Mapper, MapperCfg
        from curobo._src.perception.robot_segmenter import RobotSegmenter

        self.torch = torch
        self.width, self.height = calibration['width'], calibration['height']
        self.count = count
        self.focal = f = calibration['focal']
        self.depth_max = depth_max
        volume = fixed_base_volume()
        self.base_min = torch.tensor(volume['min_m'], dtype=torch.float32, device='cuda')[:, None, None]
        self.base_max = torch.tensor(volume['max_m'], dtype=torch.float32, device='cuda')[:, None, None]
        self.ray_x = (torch.arange(self.width, device='cuda')+.5-self.width/2)/f
        self.ray_y = ((torch.arange(self.height, device='cuda')+.5-self.height/2)/f)[:, None]
        self.intrinsics = torch.tensor([[f, 0., self.width/2], [0., f, self.height/2], [0., 0., 1.]],
                                      dtype=torch.float32, device='cuda').repeat(count, 1, 1)
        # Newton uses pixel centres. The mapper/segmenter project integer pixel indices.
        self.intrinsics[:, 0, 2] -= .5
        self.intrinsics[:, 1, 2] -= .5
        self.filter = FilterDepth(image_shape=(self.height, self.width), depth_minimum_distance=DEPTH_MIN,
                                  depth_maximum_distance=depth_max, flying_pixel_threshold=.5,
                                  bilateral_kernel_size=3)
        # The segmenter captures the single-pose kinematics buffers in a CUDA graph.
        # Scoring batches whole motion segments and reallocates its own buffers.
        self.segmenter = RobotSegmenter(Kinematics(robot.config), distance_threshold=ROBOT_MASK_DISTANCE,
                                        ops_dtype=torch.float32)
        cfg = MapperCfg(
            voxel_size=TSDF_SIZE, esdf_voxel_size=ESDF_SIZE,
            extent_meters_xyz=(2., 2., 2.), extent_esdf_meters_xyz=(2., 2., 2.),
            grid_center=torch.zeros(3, dtype=torch.float32), truncation_distance=4*TSDF_SIZE,
            depth_minimum_distance=DEPTH_MIN, depth_maximum_distance=depth_max,
            minimum_tsdf_weight=1., decay_factor=decay[0], frustum_decay_factor=decay[1],
            roughness=3., num_cameras=count,
            image_height=self.height, image_width=self.width, device='cuda:0', block_size=2,
        )
        self.mapper = Mapper(cfg)
        self.filtered = None
        self.bar_pixel_stages = None
        self.occupied = None
        self.poses = None

    def observe(self, depths, rgb, poses, state, bar_pixels=None):
        from curobo.types import CameraObservation, Pose
        from curobo_camera_second_pass import wxyz_matrix

        torch = self.torch
        camera = CameraObservation(
            depth_image=torch.nan_to_num(torch.as_tensor(depths, device='cuda'), nan=0., posinf=0., neginf=0.),
            rgb_image=torch.as_tensor(rgb, device='cuda'),
            pose=Pose(position=torch.as_tensor(poses[:, :3], device='cuda', dtype=torch.float32),
                      quaternion=torch.as_tensor(poses[:, 3:], device='cuda', dtype=torch.float32)),
            intrinsics=self.intrinsics, depth_to_meter=1.,
        )
        if bar_pixels is not None:
            mask = torch.as_tensor(bar_pixels, device='cuda')
            raw = mask.sum(dim=(1, 2))
            in_range = mask & (camera.depth_image >= DEPTH_MIN) & (camera.depth_image <= self.depth_max)
            within_limits = in_range.sum(dim=(1, 2))
        camera.depth_image, _ = self.filter(camera.depth_image)
        if bar_pixels is not None:
            after_depth = (in_range & (camera.depth_image > 0.)).sum(dim=(1, 2))
        _, camera.depth_image = self.segmenter.get_robot_mask(camera, state)
        if bar_pixels is not None:
            after_segmenter = (in_range & (camera.depth_image > 0.)).sum(dim=(1, 2))
        # The fixed base mesh and analytic floor are masked in world coordinates.
        for i, pose in enumerate(poses):
            rotation = torch.as_tensor(wxyz_matrix(pose[3:]), dtype=torch.float32, device='cuda')
            rays = (rotation[:, 0, None, None]*self.ray_x
                    + rotation[:, 1, None, None]*self.ray_y + rotation[:, 2, None, None])
            world = camera.pose.position[i, :, None, None]+camera.depth_image[i]*rays
            inside_base = ((world >= self.base_min) & (world <= self.base_max)).all(dim=0)
            camera.depth_image[i].masked_fill_(inside_base | (world[2] <= .012), 0.)
        if bar_pixels is not None:
            mapped_depth = (in_range & (camera.depth_image > 0.)).sum(dim=(1, 2))
            counts = torch.stack((raw, within_limits, after_depth, after_segmenter, mapped_depth)).cpu().tolist()
            stages = ('raw', 'within_depth_limits', 'after_depth_filter', 'after_robot_segmenter', 'used_for_mapping')
            self.bar_pixel_stages = {stage: dict(zip(('fixed', 'wrist'), pair, strict=True))
                                     for stage, pair in zip(stages, counts, strict=True)}
        self.filtered, self.poses = camera.depth_image, poses
        torch.cuda.synchronize()
        started = time.perf_counter()
        self.mapper.integrate(camera)
        torch.cuda.synchronize()
        integrate_ms = (time.perf_counter()-started)*1000
        started = time.perf_counter()
        grid = self.mapper.compute_esdf()
        torch.cuda.synchronize()
        esdf_ms = (time.perf_counter()-started)*1000
        self.occupied = self.mapper.extract_occupied_voxels(surface_only=False)
        return grid, integrate_ms, esdf_ms


def build_mpc(settings, robot_cfg, grid):
    from types import SimpleNamespace
    from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg
    from curobo.scene import Scene
    import curobo_moving_rod_v41 as v41

    cfg = ModelPredictiveControlCfg.create(
        robot=robot_cfg, scene_model=Scene(voxel=[grid]),
        optimization_dt=settings.get('optimization_dt', v41.tuned.OPTIMIZATION_DT),
        interpolation_steps=4, use_cuda_graph=True,
        optimizer_collision_activation_distance=OBSTACLE_ACTIVATION_M,
        warm_start_optimization_num_iters=200, cold_start_optimization_num_iters=300,
        squared_l2_regularization_weight=[.01, settings['acceleration_regularization'], 10., 0., 0.],
        non_terminal_tool_pose_weight_factor=settings['non_terminal_tool_pose_weight_factor'],
    )
    mask = v41.nonbase_mask(SimpleNamespace(kinematics_config=robot_cfg.kinematics.kinematics_config))
    checker = v41.FloorMaskedScene.create(cfg.core_cfg.scene_collision_cfg, mask, floor_activation_m=FLOOR_ACTIVATION_M)
    return ModelPredictiveControl(cfg, scene_collision_checker=checker), mask


def bar_position(clock):
    stroke_time = BAR_STROKE_M/BAR_SPEED_MPS
    phase = clock % (2*stroke_time)
    tip = BAR_TIP_INNER_X+BAR_SPEED_MPS*abs(stroke_time-phase)
    return np.array([tip+BAR_LENGTH_M/2, BAR_LANE_Y, BAR_HEIGHT_M])


def update_truth_bar(mpc, bar):
    from curobo_moving_rod_mpc import update_box

    update_box(mpc, dict(center_m=bar, dims_m=(BAR_LENGTH_M, 2*BAR_RADIUS_M, 2*BAR_RADIUS_M)),
               name='truth_bar')


def cylinder_sdf(points, center, axis, length, radius):
    relative = points-center
    radial = np.linalg.norm(np.delete(relative, axis, axis=-1), axis=-1)-radius
    axial = np.abs(relative[..., axis])-length/2
    pair = np.stack((radial, axial), axis=-1)
    return np.linalg.norm(np.maximum(pair, 0.), axis=-1)+np.minimum(pair.max(axis=-1), 0.)


def empty_frame(time_s, mapping, grid, robot, state, flips):
    import torch
    import torch.nn.functional as functional

    spheres = robot.compute_kinematics(state).robot_spheres.reshape(-1, 4)
    spheres = spheres[spheres[:, 3] > 0.]
    points = mapping.occupied.centers
    if len(points):
        # Signed distance to a sphere surface, counting each occupied voxel once.
        gap = torch.cdist(points.float(), spheres[:, :3].float())-spheres[:, 3]
        near = int((gap.amin(dim=1) <= .05).sum().item())
    else:
        near = 0
    # The map uses XYZ axes, half-cell centres and positive free-space distances.
    field = grid.feature_tensor.float()
    half_extent = torch.tensor(field.shape, device=field.device)*(grid.voxel_size/2)
    half_extent -= grid.voxel_size/2
    coords = (spheres[:, :3]-torch.tensor(grid.pose[:3], device=field.device))/half_extent
    inside = torch.all(coords.abs() <= 1., dim=1)
    samples = functional.grid_sample(
        field[None, None], coords[inside][:, [2, 1, 0]].reshape(1, 1, 1, -1, 3),
        align_corners=True,
    ).reshape(-1) if inside.any() else None
    return dict(time_s=time_s, occupied_voxels=len(points), voxels_within_50mm_of_arm=near,
                min_esdf_at_sphere_center_m=None if samples is None else float(samples.min()),
                min_esdf_sphere_clearance_m=None if samples is None else float((samples-spheres[inside, 3]).min()),
                spheres_outside_map=int((~inside).sum().item()),
                direction_reversals_last_half_s=[sum(time_s-.5 < stamp <= time_s for stamp in joint)
                                                 for joint in flips])


def static_alignment(stream, mapping, q, state, bar):
    """Measure a clean map of one bar and the residual points at floor height."""
    from curobo_camera_second_pass import wxyz_matrix

    mapping.mapper.reset()
    for _ in range(10):
        depth, rgb, poses = stream.capture(q, bar, None)
        mapping.observe(depth, rgb, poses, state)
    points = mapping.occupied.centers.cpu().numpy()
    error = np.abs(cylinder_sdf(points, bar, 0, .30, .02))

    def depth_support(image):
        y, x = np.nonzero(np.isfinite(image) & (image > 0.))
        optical = image[y, x, None]*np.column_stack(
            ((x+.5-mapping.width/2)/mapping.focal, (y+.5-mapping.height/2)/mapping.focal, np.ones(len(x))))
        world = optical @ wxyz_matrix(poses[0, 3:]).T+poses[0, :3]
        return int((np.abs(cylinder_sdf(world, bar, 0, .30, .02)) <= .01).sum())

    def distances(values):
        return dict(voxels=len(values), median_m=float(np.median(values)) if len(values) else None,
                    p95_m=float(np.percentile(values, 95)) if len(values) else None)

    z = points[:, 2]
    return dict(bar_center_m=bar.tolist(), stationary_q_rad=q.tolist(), frames=10,
                raw_depth_near_bar_pixels=depth_support(depth[0]),
                filtered_depth_near_bar_pixels=depth_support(mapping.filtered[0].cpu().numpy()),
                occupied_voxels=len(points), bar_within_50mm=distances(error[error <= .05]),
                floor_surface_band_20mm=distances(np.abs(z[np.abs(z) < .02])),
                occupied_below_20mm=distances(np.abs(z[z < .02])),
                far_below_floor_voxels=int((z <= -.02).sum()))


class DecayProbe:
    """Follow one actual occupied TSDF voxel after the true bar leaves it."""
    def __init__(self):
        self.point = None
        self.left_at = None
        self.cleared_at = None

    def observe(self, time_s, bar, centers):
        if self.cleared_at is not None:
            return
        if self.point is None:
            if not .8 <= time_s % (2*BAR_STROKE_M/BAR_SPEED_MPS) <= 1.3 or time_s >= DURATION or not len(centers):
                return
            tip = bar-np.array([.145, 0., 0.])
            offset = centers-bar
            candidates = centers[(np.abs(offset[:, 0]) <= .153) &
                                 (np.linalg.norm(offset[:, 1:], axis=1) <= .024)]
            if len(candidates):
                self.point = candidates[np.argmin(np.linalg.norm(candidates-tip, axis=1))].copy()
            return
        if self.left_at is None:
            offset = self.point-bar
            if abs(offset[0]) > .153 or np.linalg.norm(offset[1:]) > .024:
                self.left_at = time_s
        if self.left_at is not None:
            if not np.any(np.max(np.abs(centers-self.point), axis=1) < TSDF_SIZE/3):
                self.cleared_at = time_s

    def result(self, end):
        return dict(voxel_m=None if self.point is None else self.point.tolist(),
                    left_at_s=self.left_at, cleared_at_s=self.cleared_at,
                    delay_s=None if self.cleared_at is None else self.cleared_at-self.left_at,
                    still_occupied_after_s=None if self.left_at is None or self.cleared_at is not None
                    else end-self.left_at)


class RetractedTip:
    """Follow occupied voxels at the extended tip of the first Dance stroke."""

    def __init__(self):
        self.centers = None

    def observe(self, clock, bar, centers):
        if self.centers is None:
            if clock < BAR_STROKE_M/BAR_SPEED_MPS:
                return
            at_tip = (centers[:, 0] < bar[0]-.12) & (cylinder_sdf(centers, bar, 0, .30, .02) <= TSDF_SIZE)
            self.centers = centers[at_tip].copy()
            self.start = np.full(len(self.centers), np.nan)
            self.end = self.start.copy()
            self.cleared = np.zeros(len(self.centers), dtype=bool)
            self.keys = [tuple(key) for key in np.floor(self.centers/TSDF_SIZE).astype(int)]
            return
        vacated = cylinder_sdf(self.centers, bar, 0, .30, .02) > TSDF_SIZE
        self.start[vacated & np.isnan(self.start)] = clock
        active = np.isfinite(self.start) & np.isnan(self.end)
        if not active.any():
            return
        occupied = {tuple(key) for key in np.floor(centers/TSDF_SIZE).astype(int)}
        for i in np.flatnonzero(active):
            if not vacated[i] or self.keys[i] not in occupied:
                self.end[i] = clock
                self.cleared[i] = vacated[i]

    def result(self, last_frame_s):
        vacated = np.isfinite(self.start)
        durations = np.where(np.isnan(self.end), last_frame_s, self.end)-self.start
        return dict(voxels_at_extended_tip=len(self.centers), vacated_voxels=int(vacated.sum()),
                    cleared_voxels=int(self.cleared.sum()),
                    censored_voxels=int(vacated.sum()-self.cleared.sum()),
                    max_stale_s=float(durations[vacated].max()) if vacated.any() else None)


class BarForensics:
    """Keep the camera frames and motion segments needed for causal contact samples."""

    def __init__(self):
        self.frames = []
        self.motion = []
        self.contacts = []
        self.closest = None
        self.retraction = RetractedTip()

    def observe(self, clock, available_at, bar, mapping, grid):
        points = mapping.occupied.centers.cpu().numpy()
        near = np.count_nonzero(cylinder_sdf(points, bar, 0, .30, .02) <= .03)
        self.retraction.observe(clock, bar, points)
        self.frames.append((clock, mapping.bar_pixel_stages, int(near),
                            grid.feature_tensor.cpu(), grid.pose[:3], grid.voxel_size, available_at))

    def record_motion(self, clock, spheres, bar_path, gaps, new_contact):
        self.motion.append((clock, spheres, bar_path))
        sample_index, sphere_index = np.unravel_index(np.argmin(gaps), gaps.shape)
        clearance = float(gaps[sample_index, sphere_index])
        if self.closest is None or clearance < self.closest[0]:
            self.closest = (clearance, clock, sample_index, int(sphere_index), len(self.motion)-1)
        if new_contact:
            crosses = (gaps[0] >= 0.) & (gaps[-1] < 0.)
            if np.any(gaps[0] < 0.):
                fraction, index = 0., int(np.argmin(gaps[0]))
            else:
                candidates = np.flatnonzero(crosses)
                fractions = gaps[0, candidates] / (gaps[0, candidates]-gaps[-1, candidates])
                first = int(np.argmin(fractions))
                fraction, index = float(fractions[first]), int(candidates[first])
            self.contacts.append((clock+fraction*CONTROL_DT, index))

    @staticmethod
    def esdf_at(point, field, origin, voxel_size):
        import torch
        from torch.nn.functional import grid_sample

        half = (np.asarray(field.shape)-1)*voxel_size/2
        coord = (point-np.asarray(origin))/half
        if np.any(np.abs(coord) > 1.):
            return None
        sample = torch.tensor(coord[::-1].copy(), dtype=torch.float32).reshape(1, 1, 1, 1, 3)
        return float(grid_sample(field[None, None].float(), sample, align_corners=True).item())

    def sample(self, t, sphere_index, frame_idx, motion_idx):
        frame_time, stages, near, field, origin, voxel_size, ready = self.frames[frame_idx]
        clock, spheres, bar_path = self.motion[motion_idx]
        ends = spheres[[0, -1], sphere_index, :3]
        fraction = np.clip((t-clock)/CONTROL_DT, 0., 1.)
        point = ends[0]+fraction*(ends[1]-ends[0])
        bar = bar_path[0]+fraction*(bar_path[-1]-bar_path[0])
        # Estimate speed along the cylinder SDF normal.
        sweep = (ends[1]-ends[0])/2
        speed = -np.diff(cylinder_sdf(np.stack((point-sweep, point+sweep)), bar, 0, .30, .02))[0]/CONTROL_DT
        radius = float(spheres[0, sphere_index, 3])
        esdf = self.esdf_at(point, field, origin, voxel_size)
        return dict(time_s=t, camera_frame_s=frame_time, map_available_s=ready,
                    bar_pixels=stages['raw'], bar_pixel_stages=stages,
                    occupied_voxels_within_30mm=near, sphere_index=sphere_index,
                    esdf_at_sphere_center_m=esdf,
                    esdf_surface_clearance_m=None if esdf is None else esdf-radius,
                    arm_speed_toward_bar_mps=float(speed), sphere_radius_m=radius,
                    true_sphere_clearance_m=float(cylinder_sdf(point, bar, 0, .30, .02)-radius))

    def report(self, result):
        frame_times = [frame[0] for frame in self.frames]
        available_times = [frame[-1] for frame in self.frames]
        motion_times = [entry[0] for entry in self.motion]
        first_seen = next((t for t, pixels, *_ in self.frames if any(pixels['raw'].values())), None)
        contacts = []
        for contact_time, sphere_index in self.contacts:
            rows = []
            for offset in range(10, -1, -1):
                t = contact_time-offset/10
                if t < 0.:
                    continue
                rows.append(self.sample(t, sphere_index, bisect_right(available_times, t+1e-9)-1,
                                        bisect_right(motion_times, t+1e-9)-1))
            frame_idx = bisect_right(available_times, contact_time+1e-9)-1
            first_in_window = bisect_left(frame_times, rows[0]['camera_frame_s'])
            window = self.frames[first_in_window:frame_idx+1]
            first_mapped = next((t for t, _, near, *_ in window if near), None)
            # Include the start of a continuous mapping interval crossing the window boundary.
            if first_mapped == frame_times[first_in_window]:
                while first_in_window and self.frames[first_in_window-1][2]:
                    first_in_window -= 1
                first_mapped = frame_times[first_in_window]
            lead = None if first_mapped is None else contact_time-first_mapped
            if not any(any(pixels['raw'].values()) for _, pixels, *_ in window):
                classification = 'NOT_SEEN'
            elif lead is None:
                classification = 'SEEN_NOT_MAPPED'
            elif lead < .3-1e-9:
                classification = 'MAPPED_LATE'
            else:
                classification = 'MAPPED_BUT_HIT'
            contacts.append(dict(time_s=contact_time, sphere_index=sphere_index,
                                 first_visible_s=first_seen, first_mapped_s=first_mapped,
                                 mapped_lead_s=lead, classification=classification, samples=rows))
        _, clock, sample_index, sphere_index, motion_idx = self.closest
        t = clock+sample_index*CONTROL_DT
        frame_idx = bisect_right(available_times, clock+1e-9)-1
        closest = self.sample(t, sphere_index, frame_idx, motion_idx)
        closest_history = []
        for offset in range(10, 0, -1):
            sample_time = t-offset/10
            if sample_time >= 0.:
                frame = bisect_right(available_times, sample_time+1e-9)-1
                motion = bisect_right(motion_times, sample_time+1e-9)-1
                closest_history.append(self.sample(sample_time, sphere_index, frame, motion))
        closest_history.append(closest)
        return dict(run=result, contact_episodes=contacts, closest_bar_approach=closest,
                    closest_bar_approach_history=closest_history,
                    retracted_tip=self.retraction.result(frame_times[-1]),
                    method=dict(camera_hz=FRAME_RATE, sample_period_s=.1, history_s=1.,
                                nearby_voxel_distance_m=.03, mapped_late_threshold_s=.3,
                                no_site_esdf_value_m=ESDF_NO_SITE_M, prediction=False,
                                alignment='latest camera frame processed by the control loop at or before each sample',
                                visibility='Newton shape-index count, diagnostics only',
                                mapping='first near-bar voxel in the one-second window or earlier if continuous',
                                contact='first crossing of zero clearance within a held control step',
                                speed='positive when the sphere moves toward the bar at its sampled pose'))


class MappingView:
    def __init__(self, args, mapping):
        from curobo.types import ContentPath
        from curobo.viewer import ViserVisualizer
        from curobo_v6_memory import FOOTPRINT_COLOR

        self.viewer = ViserVisualizer(content_path=ContentPath(robot_config_file=str(args.robot_config)),
                                      connect_ip='127.0.0.1', connect_port=args.port,
                                      add_robot_to_scene=True, add_control_frames=False)
        self.server = self.viewer._server
        if self.server._websock_server._port != args.port:
            self.server.stop()
            raise RuntimeError(f'viewer port {args.port} occupied')
        scene, gui = self.server.scene, self.server.gui
        gui.add_markdown('**v7 圖例**\n- 紅色短桿為 Newton 真值\n'
                         '- 彩色點為 cuRobo TSDF 佔據體素，深度圖白色表示已移除\n'
                         '- 青綠色是固定相機，紫色是腕相機。各有視錐、地板投影、RGB 與過濾後深度')
        self.camera_names = '、'.join(CAMERA_LABELS[:mapping.count])
        self.status = gui.add_text('Simulation', self.camera_names+' | starting', disabled=True)
        self.views = []
        fov = 2*np.arctan(mapping.height/(2*mapping.focal))
        for i, name in enumerate(CAMERA_LABELS[:mapping.count]):
            with gui.add_folder(name):
                initial = np.zeros((mapping.height//4, mapping.width//4, 3), dtype=np.uint8)
                rgb = gui.add_image(initial, label=f'{name} RGB')
                depth = gui.add_image(initial, label=f'{name} 深度')
            color = FOOTPRINT_COLOR if i == 0 else (164, 115, 235)
            frustum = scene.add_camera_frustum(f'/v7/camera/{i}', fov=fov,
                                               aspect=mapping.width/mapping.height, scale=.15 if i == 0 else .30, color=color)
            outline = scene.add_line_segments(f'/v7/footprint/{i}/outline', np.zeros((4, 2, 3)),
                                              color, line_width=1.5)
            fill = scene.add_mesh_simple(f'/v7/footprint/{i}/fill', np.zeros((4, 3)),
                                         np.array([[0, 1, 2], [0, 2, 3]]), color=color,
                                         opacity=.13, side='double', cast_shadow=False, receive_shadow=False)
            self.views.append((frustum, rgb, depth, outline, fill))
        angle = np.sqrt(.5)
        self.bar = scene.add_cylinder('/truth/bar', radius=.02, height=.30, color=(235, 55, 45),
                                      wxyz=(angle, 0., angle, 0.))
        scene.add_box('/truth/floor', dimensions=(10., 10., .1), position=(0., 0., -.051),
                      color=(120, 130, 140), opacity=.18)
        self.voxels = scene.add_point_cloud('/reconstruction', np.zeros((0, 3)), (240, 160, 70),
                                            point_size=TSDF_SIZE)

    def update(self, state, mapping, rgb, bar, clock, integrate_ms, esdf_ms):
        from curobo_camera_second_pass import wxyz_matrix
        from curobo_v6_memory import floor_footprint

        self.viewer.set_joint_state(state.squeeze(0))
        self.bar.position = tuple(bar)
        points = mapping.occupied.centers
        step = max(1, (len(points)+99_999)//100_000)
        with self.server.atomic():
            self.voxels.points = points[::step].cpu().numpy()
            self.voxels.colors = mapping.occupied.colors_uint8()[::step].cpu().numpy()
        depth = mapping.filtered.cpu().numpy()
        intrinsics = (mapping.focal, mapping.focal, mapping.width/2, mapping.height/2)
        for i, (frustum, image, panel, outline, fill) in enumerate(self.views):
            pose = mapping.poses[i]
            frustum.position, frustum.wxyz = pose[:3], pose[3:]
            preview_rgb = rgb[i, ::4, ::4]
            white = np.percentile(preview_rgb.max(axis=-1), 99)
            gain = 255./white if white > 0. else 1.
            image.image = np.clip(preview_rgb*gain, 0., 255.).astype(np.uint8)
            t = np.clip(depth[i, ::4, ::4]/mapping.depth_max, 0., 1.)
            preview = np.stack((t, 1.-np.abs(2*t-1), 1.-t), axis=-1)
            preview[t <= 0.] = 1.
            panel.image = (255*preview).astype(np.uint8)
            vertices = floor_footprint(pose[:3], wxyz_matrix(pose[3:]), intrinsics,
                                       mapping.width, mapping.height, mapping.depth_max)
            vertices[:, 2] += .002
            outline.points = np.stack((vertices, np.roll(vertices, -1, axis=0)), axis=1)
            fill.vertices = vertices
        self.status.value = f'{self.camera_names} | {clock:.2f} sim s | integrate {integrate_ms:.1f} ms | ESDF {esdf_ms:.1f} ms'


def run(args, stream):
    import hashlib

    import torch
    from curobo.kinematics import Kinematics
    import curobo_moving_rod_v41 as v41
    from curobo_moving_rod_v4_rules import floor_distances, sample_held_pose, sphere_series
    from curobo_moving_rod_v5_rules import FloorPlane

    v4, v2, tuned = v41.v4, v41.v2, v41.tuned
    preflight = json.loads(v41.PREFLIGHT_FILE.read_text())
    v41.require_preflight(preflight)
    if preflight['robot_config_sha256'] != hashlib.sha256(args.robot_config.read_bytes()).hexdigest():
        raise RuntimeError('robot config differs from the frozen preflight')
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    robot_cfg, _ = v2.sixth.limited_robot_cfg(args.robot_config, v2.sixth.CAMERA_FRONT_SEED)
    robot = Kinematics(robot_cfg.kinematics)
    mapping = None if stream is None else LiveMapping(
        stream.calibration, 1 if args.cameras == 'fixed' else 2, robot, decay=args.decay,
    )
    q = np.asarray(baseline['raised_start']['joints_rad'], dtype=np.float32)
    start_q = q.copy()
    velocity = np.zeros(6)
    state = tuned.joint_state(q, velocity)
    initial = robot.compute_kinematics(state).tool_poses.as_goal().clone()
    goal = v2.sixth.goal_from_scene()
    target = goal.position.detach().cpu().numpy().reshape(-1, 3)[0]
    start = v2.sixth.fourth.tool_position(robot, state.position)
    tool = start.copy()
    has_bar = not (args.diagnosis or args.calibrate_segmenter)
    bar = bar_position(0.) if has_bar else PARK
    truth_obstacle = args.mode == 'B' and has_bar
    forensics = BarForensics() if args.bar_forensics else None
    if mapping is None:
        mask = v41.nonbase_mask(robot)
        mpc, _ = v41.make_mpc(args.robot_config, preflight['frozen_config'],
                              floor_activation_m=FLOOR_ACTIVATION_M,
                              additional_boxes=('truth_bar',) if truth_obstacle else (),
                              obstacle_activation_m=OBSTACLE_ACTIVATION_M)
        if truth_obstacle:
            update_truth_bar(mpc, bar)
        integrate_ms = esdf_ms = 0.
    else:
        depth, rgb, poses = stream.capture(q, bar, None)
        grid, integrate_ms, esdf_ms = mapping.observe(depth, rgb, poses, state, stream.bar_pixels)
        if forensics is not None:
            forensics.observe(0., 0., bar, mapping, grid)
        print(f'V7_MAPPED {args.cameras} decay={args.decay[0]:.1f} frustum={args.decay[1]:.1f} '
              f'robot_mask_m={ROBOT_MASK_DISTANCE:.3f} occupied={len(mapping.occupied)} '
              f'integrate={integrate_ms:.1f} esdf={esdf_ms:.1f}', flush=True)
        mpc, mask = build_mpc(preflight['frozen_config'], robot_cfg, grid)
    view = None
    try:
        mpc.setup(state)
        mpc.update_goal_tool_poses(goal, run_ik=False)
        mpc.optimize_action_sequence(state)
        limits = mpc.kinematics.get_joint_limits()
        position, speed, acceleration = [getattr(limits, key).detach().cpu().numpy()
                                          for key in ('position', 'velocity', 'acceleration')]
        plane = FloorPlane(mpc.kinematics, mask, position, acceleration)
        if args.interactive:
            view = MappingView(args, mapping)
            print(f'V7_INTERACTIVE http://127.0.0.1:{args.port}', flush=True)
            view.update(state, mapping, rgb, bar, 0., integrate_ms, esdf_ms)
        voxel_data = mpc.scene_collision_checker.data.voxels
        next_frame, step = 1, 0
        previous_q, previous_velocity, previous_time = q.copy(), velocity.copy(), 0.
        previous_bar = bar.copy()
        reaches, returns, seeking_goal = 0, 0, True
        best_goal_error = float(np.linalg.norm(tool-target))
        arrival_times = []
        return_times = []
        measurements = []
        touched = set()
        contacts = dict(bar=0, rod=0, floor=0)
        min_clearance = float('inf')
        timings = [(integrate_ms, esdf_ms)]
        probe = DecayProbe()
        flips = [list() for _ in range(6)]
        last_direction = np.zeros(6)
        peak_flips = 0
        max_bar_speed = 0.
        closest_truth = None
        sweep = []
        frames = 0
        duration = DIAGNOSIS_S if args.diagnosis else DURATION
        if args.calibrate_segmenter:
            duration = DIAGNOSIS_S
        calibration_poses = [q.tolist()] if args.calibrate_segmenter else None
        if args.diagnosis:
            measurements.append(empty_frame(0., mapping, grid, robot, state, flips))
        while args.interactive or step < round(duration/CONTROL_DT):
            clock = step*CONTROL_DT
            began = time.monotonic()
            if has_bar:
                bar = bar_position(clock)
            if step:
                max_bar_speed = max(max_bar_speed, float(np.linalg.norm(bar-previous_bar))/CONTROL_DT)
            if mapping is None:
                if truth_obstacle:
                    update_truth_bar(mpc, bar)
                if calibration_poses is not None and next_frame/FRAME_RATE <= clock+1e-9:
                    observed_q = sample_held_pose(previous_q, previous_velocity,
                                                  next_frame/FRAME_RATE-previous_time, position)
                    calibration_poses.append(observed_q.tolist())
                    next_frame += 1
            elif next_frame/FRAME_RATE <= clock+1e-9:
                frame_time = next_frame/FRAME_RATE
                next_frame += 1
                observed_q = sample_held_pose(previous_q, previous_velocity, frame_time-previous_time, position)
                fraction = np.clip((frame_time-previous_time)/CONTROL_DT, 0., 1.)
                observed_bar = previous_bar+(bar-previous_bar)*fraction
                depth, rgb, poses = stream.capture(observed_q, observed_bar, None)
                observed_state = tuned.joint_state(observed_q, np.zeros(6))
                grid, integrate_ms, esdf_ms = mapping.observe(depth, rgb, poses, observed_state, stream.bar_pixels)
                if args.diagnosis:
                    measurements.append(empty_frame(frame_time, mapping, grid, robot, observed_state, flips))
                if forensics is not None:
                    forensics.observe(frame_time, clock, observed_bar, mapping, grid)
                if voxel_data.features.data_ptr() != grid.feature_tensor.data_ptr():
                    voxel_data.update_features(grid.feature_tensor.reshape(-1, 1), grid.name)
                if not args.interactive:
                    timings.append((integrate_ms, esdf_ms))
                    if not args.diagnosis:
                        probe.observe(frame_time, observed_bar, mapping.occupied.centers.cpu().numpy())
                if view is not None and frames % 3 == 0:
                    view.update(state, mapping, rgb, bar, clock, integrate_ms, esdf_ms)
                frames += 1
            result = mpc.optimize_action_sequence(state)
            commands = result.action_sequence.velocity.detach().cpu().numpy().reshape(-1, 6)
            i = min(max(1, int(np.ceil(CONTROL_DT/(mpc.config.optimization_dt/4)))), len(commands)-1)
            if not np.isfinite(commands[i]).all():
                raise RuntimeError('nonfinite MPC command')
            previous_q, previous_velocity, previous_time = q.copy(), velocity.copy(), clock
            previous_bar = bar.copy()
            _, path = tuned.integrate_command(q, velocity, CONTROL_DT, position)
            q = path[-1]
            requested = v41.v3.limited_command(commands[i], velocity, speed, acceleration)
            velocity = plane.filter(q, velocity, requested)
            state = tuned.joint_state(q, velocity)
            tool = v2.sixth.fourth.tool_position(robot, state.position)
            goal_reached = float(np.linalg.norm(tool-(target if seeking_goal else start))) <= v2.sixth.GOAL_TOLERANCE_M
            best_goal_error = min(best_goal_error, float(np.linalg.norm(tool-target)))
            if goal_reached:
                reaches += int(seeking_goal)
                returns += int(not seeking_goal)
                (arrival_times if seeking_goal else return_times).append((step+1)*CONTROL_DT)
                seeking_goal = not seeking_goal
                mpc.update_goal_tool_poses(goal if seeking_goal else initial, run_ik=False)
            spheres = sphere_series(robot, torch.as_tensor(path, device='cuda', dtype=torch.float32))
            if args.save_a_sweep is not None:
                sweep.append(spheres if step == 0 else spheres[1:])
            bar_end = bar_position(clock+CONTROL_DT) if has_bar else bar
            bar_path = np.stack((bar, bar_end))
            bar_gaps = cylinder_sdf(spheres[..., :3], bar_path[:, None, :], 0,
                                    BAR_LENGTH_M, BAR_RADIUS_M)-spheres[..., 3]
            endpoint, sphere = np.unravel_index(np.argmin(bar_gaps), bar_gaps.shape)
            bar_gap = float(bar_gaps[endpoint, sphere])
            if closest_truth is None or bar_gap < closest_truth['clearance_m']:
                closest_truth = dict(time_s=clock+endpoint*CONTROL_DT, clearance_m=bar_gap,
                                     sphere_index=int(sphere), q_rad=path[endpoint].tolist(),
                                     bar_center_m=bar_path[endpoint].tolist(), rod=None)
            clearances = {
                'bar': bar_gap,
                'floor': float(np.min(floor_distances(spheres, mask, 0.))),
            }
            current = {name for name, value in clearances.items() if value < 0.}
            if forensics is not None:
                forensics.record_motion(clock, spheres, bar_path, bar_gaps, 'bar' in current-touched)
            for name in current-touched:
                contacts[name] += 1
            touched = current
            min_clearance = min(min_clearance, *clearances.values())
            if not args.interactive:
                for j, speed_j in enumerate(velocity):
                    if goal_reached and not args.diagnosis:
                        flips[j].clear()
                        last_direction[j] = 0
                    if abs(speed_j) >= .05:
                        direction = np.sign(speed_j)
                        if last_direction[j] and direction != last_direction[j]:
                            flips[j].append(clock)
                        last_direction[j] = direction
                    flips[j] = [t for t in flips[j] if clock-t < .5]
                    peak_flips = max(peak_flips, len(flips[j]))
            step += 1
            if args.interactive:
                time.sleep(max(0., CONTROL_DT-(time.monotonic()-began)))
        if calibration_poses is not None:
            return dict(poses=calibration_poses, position_limits=position.tolist(), reaches=reaches, returns=returns)
        if args.diagnosis:
            return dict(cameras=args.cameras, seconds=step*CONTROL_DT, contacts=contacts,
                        reaches=reaches, returns=returns, arrival_times_s=arrival_times,
                        return_times_s=return_times, min_goal_error_m=best_goal_error,
                        minimum_clearance_m=min_clearance, frames=measurements,
                        max_voxels_near_arm=max(f['voxels_within_50mm_of_arm'] for f in measurements),
                        peak_joint_reversals_in_half_s=peak_flips,
                        alignment=static_alignment(stream, mapping, start_q,
                                                   tuned.joint_state(start_q, np.zeros(6)),
                                                   bar_position(BAR_STROKE_M/BAR_SPEED_MPS))
                        if args.cameras == 'fixed' else None)
        if sweep:
            sampled_spheres = np.concatenate(sweep)
            np.savez_compressed(args.save_a_sweep,
                                time_s=np.arange(len(sampled_spheres))*CONTROL_DT,
                                spheres=sampled_spheres)
        arrays = np.asarray(timings)
        result = dict(mode=args.mode, cameras=None if mapping is None else args.cameras,
                      seconds=step*CONTROL_DT, rgbd_batches=0 if mapping is None else len(timings),
                      max_bar_speed_mps=max_bar_speed, closest_truth=closest_truth,
                    contacts=contacts, reaches=reaches, minimum_clearance_m=min_clearance,
                    integrate_ms=dict(mean=float(arrays[:, 0].mean()), p95=float(np.percentile(arrays[:, 0], 95))),
                    esdf_ms=dict(mean=float(arrays[:, 1].mean()), p95=float(np.percentile(arrays[:, 1], 95))),
                    decay=probe.result(duration), max_direction_reversals_per_joint_in_half_second=peak_flips,
                    thrash_detected=peak_flips >= 3)
        return forensics.report(result) if forensics is not None else result
    finally:
        if view is not None:
            view.server.stop()
        mpc.destroy()


def run_mode(args):
    if args.mode in ('A', 'B'):
        scene = 'bar ignored' if args.mode == 'A' else 'truth cuboid'
        print(f'V7_MODE {args.mode} {scene}; no cameras or mapper', flush=True)
        return run(args, None)
    print('V7_CAMERAS '+json.dumps(CAMERA_LABELS[:1 if args.cameras == 'fixed' else 2],
                                  ensure_ascii=False), flush=True)
    with tempfile.TemporaryDirectory(prefix='v7-newton-') as temp:
        stream = NewtonStream(args, Path(temp))
        try:
            return run(args, stream)
        finally:
            stream.close()


def diagnose(args):
    print('V7_DIAGNOSIS 10 simulated seconds per empty scene; estimated wall time 2–4 minutes', flush=True)
    reports = {}
    for mode in ('fixed', 'both'):
        args.cameras = mode
        reports[mode] = run_mode(args)
    payload = dict(
        criteria=dict(near_arm_m=.05, meaningful_joint_speed_rad_s=.05, reversal_window_s=.5,
                      floor_residual_below_m=.02, bar_sample_distance_m=.05,
                      esdf_sign='positive in free space', esdf_no_site_m=ESDF_NO_SITE_M,
                      camera_hz=FRAME_RATE),
        empty={mode: {k: v for k, v in case.items() if k != 'alignment'} for mode, case in reports.items()},
        fixed_alignment=reports['fixed']['alignment'],
    )
    tmp = DIAGNOSIS_FILE.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, separators=(',', ':'), allow_nan=False)+'\n')
    tmp.replace(DIAGNOSIS_FILE)
    print_diagnosis(payload)


def print_diagnosis(payload):
    for mode, case in payload['empty'].items():
        near = case['max_voxels_near_arm']
        esdf = min((f['min_esdf_sphere_clearance_m'] for f in case['frames']
                    if f['min_esdf_sphere_clearance_m'] is not None), default=None)
        # Interpolation and subtracting the sphere radius can lower the sentinel slightly.
        esdf_label = 'no_site' if esdf is not None and esdf >= ESDF_NO_SITE_M-1. else str(esdf)
        print(f"V7_DIAG {mode}: reaches={case['reaches']} returns={case['returns']} "
              f"near_arm_peak={near} min_esdf_clearance_m={esdf_label} "
              f"peak_reversals={case['peak_joint_reversals_in_half_s']}")
    alignment = payload['fixed_alignment']
    print('V7_DIAG bar '+json.dumps(alignment['bar_within_50mm']))
    print('V7_DIAG floor '+json.dumps(dict(surface=alignment['floor_surface_band_20mm'],
                                          below_20mm=alignment['occupied_below_20mm'])))
    print(f'V7_DIAG saved {DIAGNOSIS_FILE.relative_to(ROOT)}', flush=True)


def compare_decay(args):
    print('V7_DECAY three 10-second reciprocating-bar runs; estimated wall time 2–4 minutes', flush=True)
    cases = {}
    for name, knobs in DECAY_CONFIGS.items():
        print(f'V7_DECAY start {name}: decay={knobs[0]:.1f} frustum={knobs[1]:.1f}', flush=True)
        trial = argparse.Namespace(**(vars(args) | dict(bar_forensics=True, decay=knobs)))
        result = run_mode(trial)
        cases[name] = dict(decay_factor=knobs[0], frustum_decay_factor=knobs[1], **result)
        print('V7_DECAY '+name+' '+json.dumps(dict(
            contacts=result['run']['contacts'], reaches=result['run']['reaches'],
            closest=result['closest_bar_approach'], retracted_tip=result['retracted_tip'])), flush=True)

    def rank(case):
        stale = case['retracted_tip']['max_stale_s']
        return (any(case['run']['contacts'].values()), float('inf') if stale is None else stale)

    selected = min(cases, key=lambda name: rank(cases[name]))
    payload = dict(cases=cases, selected=selected,
                   selection='zero contacts first, then shortest maximum observed tip persistence',
                   stale_time='first stroke tip; max time from geometric departure to clearing, '
                              'a lower bound when the bar revisits or the run ends first')
    tmp = DECAY_COMPARISON_FILE.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
    tmp.replace(DECAY_COMPARISON_FILE)
    print(f'V7_DECAY selected={selected} saved {DECAY_COMPARISON_FILE.relative_to(ROOT)}', flush=True)


def segmentation_checkpoint(args):
    print('V7_SEGMENTATION both cameras: 10 s empty + 10 s reciprocating bar; estimated wall time 2–3 minutes', flush=True)
    empty_args = argparse.Namespace(**(vars(args) | dict(diagnosis=True)))
    empty = run_mode(empty_args)
    near_counts = [frame['voxels_within_50mm_of_arm'] for frame in empty['frames']]
    empty_summary = dict(seconds=empty['seconds'], frames=len(near_counts),
                         reaches=empty['reaches'], returns=empty['returns'], contacts=empty['contacts'],
                         peak_reversals_in_half_s=empty['peak_joint_reversals_in_half_s'],
                         max_voxels_within_50mm=max(near_counts),
                         frames_with_near_arm_voxels=sum(count > 0 for count in near_counts))
    print('V7_SEGMENTATION empty '+json.dumps(empty_summary), flush=True)

    dance_args = argparse.Namespace(**(vars(args) | dict(bar_forensics=True)))
    dance = run_mode(dance_args)
    print('V7_SEGMENTATION Dance '+json.dumps(dict(contacts=dance['run']['contacts'],
          reaches=dance['run']['reaches'], closest=dance['closest_bar_approach'])), flush=True)
    payload = dict(robot_mask_distance_m=ROBOT_MASK_DISTANCE, decay_factor=args.decay[0],
                   frustum_decay_factor=args.decay[1], floor_handling='original floor-pixel rejection',
                   empty=empty_summary, empty_near_arm_counts=near_counts, dance=dance)
    tmp = SEGMENTATION_CHECKPOINT_FILE.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
    tmp.replace(SEGMENTATION_CHECKPOINT_FILE)
    print(f'V7_SEGMENTATION saved {SEGMENTATION_CHECKPOINT_FILE.relative_to(ROOT)}', flush=True)


def project_pixels(depth, pose, focal, mask):
    from curobo_camera_second_pass import wxyz_matrix

    y, x = np.nonzero(mask & np.isfinite(depth) & (depth > 0.))
    h, w = depth.shape
    rays = np.column_stack(((x+.5-w/2)/focal, (y+.5-h/2)/focal, np.ones(len(x))))
    return y, x, (rays*depth[y, x, None]) @ wxyz_matrix(pose[3:]).T+pose[:3]


def signed_pixel_distances(depth, pose, focal, mask, spheres):
    """Project selected Newton depth pixels and evaluate the RobotSegmenter metric."""
    import torch

    y, x, world = project_pixels(depth, pose, focal, mask)
    if not len(x):
        return y, x, np.empty(0, dtype=np.float32)
    points = torch.as_tensor(world, device=spheres.device, dtype=spheres.dtype)
    distances = [(torch.cdist(chunk, spheres[:, :3])-spheres[:, 3]).amin(dim=1)
                 for chunk in points.split(16384)]
    return y, x, torch.cat(distances).cpu().numpy()


def calibration_robot(config):
    from curobo.kinematics import Kinematics
    from curobo_moving_rod_v41 import v2

    robot_cfg, _ = v2.sixth.limited_robot_cfg(config, v2.sixth.CAMERA_FRONT_SEED)
    return Kinematics(robot_cfg.kinematics)


def active_spheres(robot, q):
    from curobo_moving_rod_mpc import joint_state

    spheres = robot.compute_kinematics(joint_state(q, np.zeros(6))).robot_spheres.reshape(-1, 4)
    return spheres[spheres[:, 3] > 0.]


def calibrate_segmenter(args):
    print('V7_CALIBRATION 10 s empty motion + 20 poses; estimated wall time 2–4 minutes', flush=True)
    motion = run(args, None)
    poses = np.asarray(motion['poses'])
    lo, hi = np.asarray(motion['position_limits'])
    # Deterministic perturbations around different points on the empty trajectory.
    rng = np.random.default_rng(7)
    pose_indices = np.linspace(0, len(poses)-1, 20, dtype=int)
    varied = np.clip(poses[pose_indices]+rng.uniform(-.35, .35, (20, 6)), lo, hi)
    robot = calibration_robot(args.robot_config)
    by_link = {}
    with tempfile.TemporaryDirectory(prefix='v7-segmenter-') as directory:
        stream = NewtonStream(args, Path(directory))
        try:
            shape_links = stream.calibration['robot_shape_links']
            labels, link_ids = np.unique(shape_links, return_inverse=True)
            for q in np.concatenate((poses, varied)):
                depth, _, cameras = stream.capture(q, PARK, None, robot_shape_ids=True)
                spheres = active_spheres(robot, q)
                for image, pose, shapes in zip(depth, cameras, stream.robot_shapes, strict=True):
                    y, x, gap = signed_pixel_distances(image, pose, stream.calibration['focal'], shapes >= 0, spheres)
                    links = link_ids[shapes[y, x]]
                    for link in np.unique(links):
                        by_link.setdefault(str(labels[link]), []).append(gap[links == link])
        finally:
            stream.close()

    def stats(samples):
        p99, p999 = np.percentile(samples, (99, 99.9))
        return dict(pixels=len(samples), max_m=float(samples.max()), p99_9_m=float(p999), p99_m=float(p99))

    by_link = {link: np.concatenate(chunks) for link, chunks in by_link.items()}
    overall = stats(np.concatenate(list(by_link.values())))
    threshold = (int(np.floor(overall['max_m']*1000))+1)/1000
    report = dict(overall=overall, by_link={name: stats(distances) for name, distances in by_link.items()},
                  threshold_m=threshold, motion_frames=len(poses), motion_reaches=motion['reaches'],
                  motion_returns=motion['returns'], extra_poses_rad=varied.tolist(),
                  method='signed distance to the nearest enabled sphere; each pixel grouped by Newton body',
                  real_d435_additional_margin_m=.01)
    write_json(CALIBRATION_FILE, report)
    print('V7_CALIBRATION '+json.dumps(dict(overall=overall, threshold_m=threshold)), flush=True)


def write_json(path, payload):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')
    tmp.replace(path)


def closest_bar_pixels(robot, stream, closest):
    q = np.asarray(closest['q_rad'])
    depth, _, poses = stream.capture(q, np.asarray(closest['bar_center_m']), closest['rod'])
    spheres = active_spheres(robot, q)
    counts = {}
    for name, image, pose, mask in zip(('fixed', 'wrist'), depth, poses, stream.bar_pixels, strict=True):
        y, x, gap = signed_pixel_distances(image, pose, stream.calibration['focal'], mask, spheres)
        in_range = (image[y, x] >= DEPTH_MIN) & (image[y, x] <= DEPTH_MAX)
        near = gap < ROBOT_MASK_DISTANCE
        counts[name] = dict(valid_bar_pixels=len(gap), within_threshold=int(near.sum()),
                            in_depth_range=int(in_range.sum()), in_range_within_threshold=int((near & in_range).sum()))
    return counts


def fair_checkpoint(args):
    truth = json.loads(args.fair_checkpoint.read_text())
    if truth['mode'] != 'B' or truth['rgbd_batches'] != 0 or truth['seconds'] != DURATION:
        raise RuntimeError('expected a 10 s camera-free B result')
    if truth['reaches'] < 1 or any(truth['contacts'].values()) or truth['minimum_clearance_m'] < .02:
        raise RuntimeError('B did not pass; stop before the perception tests')
    print('V7_FAIR E empty + E reciprocating bar; estimated wall time 2–3 minutes', flush=True)
    empty = run_mode(argparse.Namespace(**(vars(args) | dict(diagnosis=True))))
    summary = dict(seconds=empty['seconds'], reaches=empty['reaches'], returns=empty['returns'],
                   contacts=empty['contacts'], max_voxels_near_arm=empty['max_voxels_near_arm'],
                   peak_reversals_in_half_s=empty['peak_joint_reversals_in_half_s'],
                   counts_near_arm=[frame['voxels_within_50mm_of_arm'] for frame in empty['frames']])
    print('V7_FAIR empty '+json.dumps({k: v for k, v in summary.items() if k != 'counts_near_arm'}), flush=True)
    dance_args = argparse.Namespace(**(vars(args) | dict(bar_forensics=True)))
    dance = run_mode(dance_args)
    print('V7_FAIR E '+json.dumps(dict(contacts=dance['run']['contacts'], reaches=dance['run']['reaches'],
          closest=dance['closest_bar_approach'],
          classifications=[event['classification'] for event in dance['contact_episodes']])), flush=True)

    # Render only the saved closest poses for B/E diagnostics. The B controller runs camera-free.
    robot = calibration_robot(args.robot_config)
    with tempfile.TemporaryDirectory(prefix='v7-closest-') as directory:
        stream = NewtonStream(dance_args, Path(directory))
        try:
            truth['closest_bar_pixels'] = closest_bar_pixels(robot, stream, truth['closest_truth'])
            dance['closest_bar_pixels'] = closest_bar_pixels(robot, stream, dance['run']['closest_truth'])
        finally:
            stream.close()
    payload = dict(robot_mask_distance_m=ROBOT_MASK_DISTANCE, decay_factor=args.decay[0],
                   frustum_decay_factor=args.decay[1], truth_dance=truth, perception_empty=summary,
                   perception_dance=dance,
                   empty_pass=summary['max_voxels_near_arm'] <= 5 and summary['peak_reversals_in_half_s'] <= 2,
                   truth_pass=True,
                   perception_pass=dance['run']['reaches'] >= 1 and dance['run']['contacts']['bar'] == 0)
    write_json(FAIR_SCENE_FILE, payload)
    print(f'V7_FAIR saved {FAIR_SCENE_FILE.relative_to(ROOT)}', flush=True)


def verify_base_mask(args):
    calibration = json.loads(CALIBRATION_FILE.read_text())
    volume = fixed_base_volume()
    bounds_min, bounds_max = np.asarray(volume['min_m']), np.asarray(volume['max_m'])
    count = outside = 0
    poses = calibration['extra_poses_rad']
    with tempfile.TemporaryDirectory(prefix='v7-base-mask-') as directory:
        stream = NewtonStream(args, Path(directory))
        try:
            base_ids = np.flatnonzero([name.rsplit('/', 1)[-1] == BASE_LINK
                                       for name in stream.calibration['robot_shape_links']])
            if not len(base_ids):
                raise RuntimeError('Newton did not identify the URDF base link')
            for q in poses:
                depths, _, cameras = stream.capture(np.asarray(q), PARK, None, robot_shape_ids=True)
                for depth, pose, shapes in zip(depths, cameras, stream.robot_shapes, strict=True):
                    _, _, points = project_pixels(depth, pose, stream.calibration['focal'],
                                                   np.isin(shapes, base_ids))
                    count += len(points)
                    outside += int(np.any((points < bounds_min) | (points > bounds_max), axis=1).sum())
        finally:
            stream.close()
    calibration['fixed_base_volume'] = volume
    calibration['fixed_base_pixel_verification'] = dict(poses=len(poses), cameras=args.cameras,
                                                         base_pixels=count, base_pixels_outside=outside)
    calibration['moving_link_threshold_m'] = ROBOT_MASK_DISTANCE
    write_json(CALIBRATION_FILE, calibration)
    print('V7_LANE base '+json.dumps(calibration['fixed_base_pixel_verification']), flush=True)
    if count == 0 or outside:
        raise RuntimeError('computed base mask did not cover all observed calibration base pixels')


def lane_checkpoint(args):
    confirmed = json.loads(args.lane_checkpoint.read_text())
    if confirmed['mode'] != 'A' or confirmed['seconds'] != DURATION or confirmed['contacts']['bar'] < 1:
        raise RuntimeError('a passing confirmed A run is required')
    print('V7_LANE base verification, E empty, B and E; estimated wall time 3–5 minutes', flush=True)
    payload = dict(lane_y_m=BAR_LANE_Y, lane_z_m=BAR_HEIGHT_M, confirmed_A=confirmed,
                   robot_mask_distance_m=ROBOT_MASK_DISTANCE, obstacle_activation_m=OBSTACLE_ACTIVATION_M,
                   floor_activation_m=FLOOR_ACTIVATION_M)
    write_json(FAIR_SCENE_FILE, payload)
    verify_base_mask(args)
    empty = run_mode(argparse.Namespace(**(vars(args) | dict(diagnosis=True))))
    summary = {key: empty[key] for key in ('seconds', 'reaches', 'returns', 'contacts', 'max_voxels_near_arm',
                                           'peak_joint_reversals_in_half_s', 'minimum_clearance_m')}
    summary['near_arm_voxels_per_frame'] = [frame['voxels_within_50mm_of_arm'] for frame in empty['frames']]
    summary['passed'] = summary['max_voxels_near_arm'] <= 5 and summary['peak_joint_reversals_in_half_s'] <= 2
    payload['E_empty'] = summary
    print('V7_LANE E_empty '+json.dumps({k: v for k, v in summary.items() if k != 'near_arm_voxels_per_frame'}),
          flush=True)
    write_json(FAIR_SCENE_FILE, payload)

    truth = run_mode(argparse.Namespace(**(vars(args) | dict(mode='B', smoke=True))))
    truth['passed'] = truth['reaches'] >= 1 and not any(truth['contacts'].values()) and truth['minimum_clearance_m'] >= .02
    payload['B'] = truth
    print('V7_LANE B '+json.dumps(truth), flush=True)
    if not truth['passed']:
        payload['stopped_before'] = 'E reciprocating bar'
        write_json(FAIR_SCENE_FILE, payload)
        print('V7_LANE stopped at B gate', flush=True)
        return
    write_json(FAIR_SCENE_FILE, payload)

    perception = run_mode(argparse.Namespace(**(vars(args) | dict(bar_forensics=True))))
    result = perception['run']
    perception['passed'] = result['reaches'] >= 1 and not any(result['contacts'].values())
    payload['E'] = perception
    write_json(FAIR_SCENE_FILE, payload)
    print('V7_LANE E '+json.dumps(dict(run=result, passed=perception['passed'],
          classifications=[event['classification'] for event in perception['contact_episodes']])), flush=True)


def search_bar_lanes(path):
    with np.load(path) as sweep:
        times, spheres = sweep['time_s'], sweep['spheres']
    expected = np.arange(round(DURATION/CONTROL_DT)+1)*CONTROL_DT
    if times.shape != expected.shape or not np.allclose(times, expected) or not np.isfinite(spheres).all():
        raise ValueError('expected a complete finite 10 s A sweep')
    centers = np.stack([bar_position(t) for t in times])
    rows = []
    for z_cm in range(12, 31):
        for y_cm in range(20, 37, 2):
            centers[:, 1:] = y_cm/100, z_cm/100
            gaps = cylinder_sdf(spheres[..., :3], centers[:, None, :], 0, BAR_LENGTH_M, BAR_RADIUS_M)
            gaps -= spheres[..., 3]
            gaps[:, spheres[0, :, 3] <= 0.] = np.inf
            frame, sphere = np.unravel_index(np.argmin(gaps), gaps.shape)
            rows.append(dict(y_m=y_cm/100, z_m=z_cm/100,
                             min_clearance_m=float(gaps[frame, sphere]), time_s=float(times[frame]),
                             sphere_index=int(sphere)))
    chosen = min((row for row in rows if row['min_clearance_m'] <= -.02),
                 key=lambda row: (row['z_m'], round(abs(row['y_m']-.28), 6), row['y_m']),
                 default=None)
    report = dict(sweep=path.name, lanes=rows, selected=chosen, required_clearance_m=-.02)
    write_json(RESULTS_DIR/'v7_bar_lane_search.json', report)
    print('V7_LANE '+json.dumps(chosen), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('A', 'B', 'E'), default='E')
    parser.add_argument('--cameras', choices=('fixed', 'both'), default='both')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--smoke', action='store_true')
    mode.add_argument('--lane-checkpoint', type=Path, metavar='A_RESULT')
    mode.add_argument('--search-lanes', type=Path, metavar='A_SWEEP')
    parser.add_argument('--save-a-sweep', type=Path, metavar='FILE')
    mode.add_argument('--diagnosis', action='store_true', help='Measure fixed and both in empty scenes, then save diagnostics')
    mode.add_argument('--interactive', action='store_true')
    mode.add_argument('--fair-checkpoint', type=Path, metavar='B_RESULT',
                      help='Run E empty and reciprocating bar after a passing B result')
    mode.add_argument('--calibrate-segmenter', action='store_true', help='Measure the segmentation threshold')
    mode.add_argument('--segmentation-checkpoint', action='store_true', help='Run one dual-camera empty scene and reciprocating-bar run')
    mode.add_argument('--decay-comparison', action='store_true', help='Compare three 10 s dual-camera reciprocating-bar runs')
    mode.add_argument('--bar-forensics', action='store_true', help='Record one 10 s dual-camera reciprocating-bar run')
    parser.add_argument('--port', type=int, default=int(os.environ.get('NEWTON_VIEWER_PORT', '30018')))
    parser.add_argument('--robot-config', type=Path,
                        default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    parser.add_argument('--newton-python', type=Path, default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args = parser.parse_args()
    args.decay = DEFAULT_DECAY
    if args.save_a_sweep is not None and (args.mode != 'A' or not args.smoke):
        parser.error('--save-a-sweep requires --mode A --smoke')
    if args.mode != 'E' and not args.smoke:
        parser.error('--mode A and --mode B require --smoke')
    if (args.bar_forensics or args.decay_comparison or args.segmentation_checkpoint or args.calibrate_segmenter or args.fair_checkpoint or args.lane_checkpoint) and args.cameras != 'both':
        parser.error('Dance checkpoints require --cameras both')
    if args.interactive:
        from curobo_moving_rod_v3 import check_viewer_port
        check_viewer_port(args.port)
    try:
        if args.search_lanes is not None:
            search_bar_lanes(args.search_lanes)
        elif args.diagnosis:
            diagnose(args)
        elif args.lane_checkpoint is not None:
            lane_checkpoint(args)
        elif args.fair_checkpoint is not None:
            fair_checkpoint(args)
        elif args.calibrate_segmenter:
            calibrate_segmenter(args)
        elif args.segmentation_checkpoint:
            segmentation_checkpoint(args)
        elif args.decay_comparison:
            compare_decay(args)
        else:
            result = run_mode(args)
            if args.bar_forensics:
                tmp = FORENSICS_FILE.with_suffix('.tmp')
                tmp.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
                tmp.replace(FORENSICS_FILE)
                print(f'V7_FORENSICS saved {FORENSICS_FILE.relative_to(ROOT)}', flush=True)
            elif result is not None:
                print('V7_RESULT '+json.dumps(result, allow_nan=False), flush=True)
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--renderer':
        render_worker(sys.argv[2])
    else:
        main()
