#!/usr/bin/env python3
"""Fixed/wrist Newton RGB-D input to the cuRobo live volumetric mapping MPC pipeline.

Pipeline reference: NVlabs/cuRobo, curobo/examples/reference/live_volumetric_mapping_mpc.py.
"""
from __future__ import annotations

import argparse
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
DEPTH_MIN, DEPTH_MAX = .15, 2.
TSDF_SIZE, ESDF_SIZE = .005, .03
FRAME_RATE, CONTROL_DT = 30, .025
DANCE_END, DURATION = 6., 9.
PARK = np.array([2., .28, .20])
SOFA = (.78, .52, .50)


def camera_basis(config):
    origin = np.asarray(config['position_m'], dtype=float)
    forward = np.asarray(config['look_at_m'])-origin
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, config['up'])
    right /= np.linalg.norm(right)
    return origin, np.column_stack((right, np.cross(forward, right), forward))


def render_worker(mode):
    """Read raw Newton RGB-D without requesting shape IDs."""
    import newton_camera_moving_rod_worker as source

    mount, _ = source.camera_common.read_mount_config(ROOT/'config/sim_camera_mount.yaml')
    scene = source.MovingRodCamera(
        Path.home()/'newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf', mount,
        'cuda:0', horizontal_bar=True, bar_length=.30, vertical_obstacle=True,
    )
    width, height = source.WIDTH, source.HEIGHT
    color = scene.sensor.utils.create_color_image_output(width, height)
    origin, fixed = camera_basis(yaml.safe_load(FIXED_CONFIG.read_text()))
    convention = np.diag([1., -1., -1.])
    fixed_transform = np.array(
        [*origin, *source.camera_common.matrix_to_quaternion_xyzw(fixed @ convention)],
        dtype=np.float32).reshape(1, 1, 7)
    focal = height/(2*np.tan(source.VERTICAL_FOV_RAD/2))
    print('V7_RENDERER '+json.dumps(dict(width=width, height=height, focal=focal)), flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        if request.get('shutdown'):
            return
        scene._assign_pose(np.asarray(request['q'], dtype=np.float32), *request['bar'], request['rod'])
        depths, rgbs, poses = [], [], []
        scene.camera_transforms.assign(fixed_transform)
        for i in range(1 if mode == 'fixed' else 2):
            if i:
                source.wp.launch(source.compose_camera, dim=1,
                                 inputs=[scene.state.body_q, scene.parent, scene.camera_local],
                                 outputs=[scene.camera_transforms], device=scene.device)
            scene.sensor.update(scene.state, scene.camera_transforms, scene.rays,
                                forward_depth_image=scene.depth, color_image=color)
            source.wp.synchronize_device(scene.device)
            depths.append(scene.depth.numpy()[0, 0].copy())
            rgba = color.numpy()[0, 0]
            rgbs.append(np.stack([(rgba >> shift) & 255 for shift in (0, 8, 16)], axis=-1).astype(np.uint8))
            pose = scene.camera_transforms.numpy()[0, 0]
            rotation = source.camera_common.quaternion_xyzw_to_matrix(pose[3:]) @ convention
            orientation = source.camera_common.matrix_to_quaternion_xyzw(rotation)
            poses.append([*pose[:3], orientation[3], *orientation[:3]])
        np.savez(request['output'], depth=depths, rgb=rgbs, poses=poses)
        print('V7_FRAME', flush=True)


class NewtonStream:
    def __init__(self, args, directory):
        self.output = directory/'rgbd.npz'
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

    def capture(self, q, bar, rod):
        self.process.stdin.write(json.dumps(dict(q=q.tolist(), bar=bar.tolist(), rod=rod,
                                                 output=str(self.output)))+'\n')
        self.process.stdin.flush()
        self.read('V7_FRAME')
        with np.load(self.output) as arrays:
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


class LiveMapping:
    """Adapt the reference example's FilterDepth, Mapper, and camera batching."""
    def __init__(self, calibration, count, robot, *, depth_max=DEPTH_MAX):
        import torch
        from curobo.perception import FilterDepth, Mapper, MapperCfg
        from curobo._src.perception.robot_segmenter import RobotSegmenter

        self.torch = torch
        self.width, self.height = calibration['width'], calibration['height']
        self.count = count
        self.focal = f = calibration['focal']
        self.depth_max = depth_max
        self.intrinsics = torch.tensor([[f, 0., self.width/2], [0., f, self.height/2], [0., 0., 1.]],
                                      dtype=torch.float32, device='cuda').repeat(count, 1, 1)
        # Newton uses pixel centres. The mapper/segmenter project integer pixel indices.
        self.intrinsics[:, 0, 2] -= .5
        self.intrinsics[:, 1, 2] -= .5
        self.filter = FilterDepth(image_shape=(self.height, self.width), depth_minimum_distance=DEPTH_MIN,
                                  depth_maximum_distance=depth_max, flying_pixel_threshold=.5,
                                  bilateral_kernel_size=3)
        self.segmenter = RobotSegmenter(robot, ops_dtype=torch.float32)
        cfg = MapperCfg(
            voxel_size=TSDF_SIZE, esdf_voxel_size=ESDF_SIZE,
            extent_meters_xyz=(2., 2., 2.), extent_esdf_meters_xyz=(2., 2., 2.),
            grid_center=torch.zeros(3, dtype=torch.float32), truncation_distance=4*TSDF_SIZE,
            depth_minimum_distance=DEPTH_MIN, depth_maximum_distance=depth_max,
            minimum_tsdf_weight=1., decay_factor=.3, roughness=3., num_cameras=count,
            image_height=self.height, image_width=self.width, device='cuda:0', block_size=2,
        )
        self.mapper = Mapper(cfg)
        self.filtered = None
        self.occupied = None
        self.poses = None

    def observe(self, depths, rgb, poses, state):
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
        camera.depth_image, _ = self.filter(camera.depth_image)
        _, camera.depth_image = self.segmenter.get_robot_mask(camera, state)
        # The exact floor is already represented by a cuboid with the v6 base-link exclusions.
        # Avoid a duplicate noisy floor surface in the voxel obstacle.
        x = (np.arange(self.width)+.5-self.width/2)/self.focal
        y = (np.arange(self.height)+.5-self.height/2)/self.focal
        for i, pose in enumerate(poses):
            rotation = wxyz_matrix(pose[3:])
            rz = rotation[2, 0]*x[None, :]+rotation[2, 1]*y[:, None]+rotation[2, 2]
            z = pose[2]+camera.depth_image[i]*torch.as_tensor(rz, device='cuda', dtype=torch.float32)
            camera.depth_image[i].masked_fill_(z <= .012, 0.)
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
        interpolation_steps=4, use_cuda_graph=True, optimizer_collision_activation_distance=.03,
        warm_start_optimization_num_iters=200, cold_start_optimization_num_iters=300,
        squared_l2_regularization_weight=[.01, settings['acceleration_regularization'], 10., 0., 0.],
        non_terminal_tool_pose_weight_factor=settings['non_terminal_tool_pose_weight_factor'],
    )
    mask = v41.nonbase_mask(SimpleNamespace(kinematics_config=robot_cfg.kinematics.kinematics_config))
    checker = v41.FloorMaskedScene.create(cfg.core_cfg.scene_collision_cfg, mask, floor_activation_m=.01)
    return ModelPredictiveControl(cfg, scene_collision_checker=checker), mask


class Sequence:
    """Reuse the v6 Dance stroke and the accepted stationary Sofa location."""
    def __init__(self, mapping):
        from curobo_v6_memory import DepthFrame

        origin, rotation = camera_basis(yaml.safe_load(FIXED_CONFIG.read_text()))
        shape = (mapping.height, mapping.width)
        self.fixed = DepthFrame(np.empty(shape), np.empty(shape, dtype=bool),
                                (mapping.focal, mapping.focal, mapping.width/2, mapping.height/2),
                                origin, rotation, mapping.depth_max)
        self.period = None
        self.target = None

    def pose(self, clock, tool, goal):
        from curobo_moving_bar_poke_demo import DANCE_PERIOD_S, dance_center
        from curobo_v6_memory import visible_path_target

        elapsed = clock % DURATION
        if elapsed >= DANCE_END-1e-9:
            return PARK.copy(), SOFA
        started = int((clock+1e-9)/DANCE_PERIOD_S)
        if self.period != started:
            self.period = started
            self.target = visible_path_target(tool, goal, self.fixed)
        if self.target is None:
            return PARK.copy(), None
        xy = dance_center(self.target, max(0., clock-started*DANCE_PERIOD_S))[:2]
        return np.array([*xy, self.target[2]]), None


def cylinder_clearance(spheres, center, axis, length, radius):
    relative = spheres[..., :3]-center
    radial = np.linalg.norm(np.delete(relative, axis, axis=-1), axis=-1)-radius
    axial = np.abs(relative[..., axis])-length/2
    pair = np.stack((radial, axial), axis=-1)
    signed = np.linalg.norm(np.maximum(pair, 0.), axis=-1)+np.minimum(pair.max(axis=-1), 0.)
    return float(np.min(signed-spheres[..., 3]))


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
            if not .8 <= time_s % 3. <= 1.3 or time_s >= DANCE_END or not len(centers):
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
        gui.add_markdown('**v7 圖例**\n- 紅色短桿、棕色直桿為 Newton 真值\n'
                         '- 彩色點為 cuRobo TSDF 佔據體素，ESDF 切面顯示距離\n'
                         '- 青綠色是固定相機，紫色是腕相機。各有視錐、地板投影、RGB 與過濾後深度')
        self.status = gui.add_text('Simulation', 'starting', disabled=True)
        self.views = []
        fov = 2*np.arctan(mapping.height/(2*mapping.focal))
        for i, name in enumerate(('Fixed', 'Wrist')[:mapping.count]):
            with gui.add_folder(name):
                initial = np.zeros((mapping.height//4, mapping.width//4, 3), dtype=np.uint8)
                rgb, depth = gui.add_image(initial, label='RGB'), gui.add_image(initial, label='Filtered depth')
            color = FOOTPRINT_COLOR if i == 0 else (164, 115, 235)
            frustum = scene.add_camera_frustum(f'/v7/camera/{i}', fov=fov,
                                               aspect=mapping.width/mapping.height, scale=.15, color=color)
            outline = scene.add_line_segments(f'/v7/footprint/{i}/outline', np.zeros((4, 2, 3)),
                                              color, line_width=1.5)
            fill = scene.add_mesh_simple(f'/v7/footprint/{i}/fill', np.zeros((4, 3)),
                                         np.array([[0, 1, 2], [0, 2, 3]]), color=color,
                                         opacity=.13, side='double', cast_shadow=False, receive_shadow=False)
            self.views.append((frustum, rgb, depth, outline, fill))
        angle = np.sqrt(.5)
        self.bar = scene.add_cylinder('/truth/bar', radius=.02, height=.30, color=(235, 55, 45),
                                      wxyz=(angle, 0., angle, 0.))
        self.rod = scene.add_cylinder('/truth/rod', radius=.025, height=.50, color=(200, 110, 52), visible=False)
        scene.add_box('/truth/floor', dimensions=(10., 10., .1), position=(0., 0., -.051),
                      color=(120, 130, 140), opacity=.18)
        self.voxels = scene.add_point_cloud('/reconstruction', np.zeros((0, 3)), (240, 160, 70),
                                            point_size=TSDF_SIZE)

    def update(self, state, mapping, rgb, grid, bar, rod, clock, integrate_ms, esdf_ms):
        from curobo_camera_second_pass import wxyz_matrix
        from curobo_v6_memory import floor_footprint
        from curobo.examples.reference.live_volumetric_mapping_mpc import _add_static_esdf_slice

        self.viewer.set_joint_state(state.squeeze(0))
        self.bar.position = tuple(bar)
        self.rod.visible = rod is not None
        if rod is not None:
            self.rod.position = (*rod[:2], rod[2]/2)
        points = mapping.occupied.centers
        step = max(1, (len(points)+99_999)//100_000)
        with self.server.atomic():
            self.voxels.points = points[::step].cpu().numpy()
            self.voxels.colors = mapping.occupied.colors_uint8()[::step].cpu().numpy()
        _add_static_esdf_slice(self.viewer, grid, mapping.mapper.config)
        depth = mapping.filtered.cpu().numpy()
        intrinsics = (mapping.focal, mapping.focal, mapping.width/2, mapping.height/2)
        for i, (frustum, image, panel, outline, fill) in enumerate(self.views):
            pose = mapping.poses[i]
            frustum.position, frustum.wxyz = pose[:3], pose[3:]
            image.image = rgb[i, ::4, ::4]
            t = np.clip(depth[i, ::4, ::4]/mapping.depth_max, 0., 1.)
            preview = np.stack((t, 1.-np.abs(2*t-1), 1.-t), axis=-1)*(t[..., None] > 0.)
            panel.image = (255*preview).astype(np.uint8)
            vertices = floor_footprint(pose[:3], wxyz_matrix(pose[3:]), intrinsics,
                                       mapping.width, mapping.height, mapping.depth_max)
            vertices[:, 2] += .002
            outline.points = np.stack((vertices, np.roll(vertices, -1, axis=0)), axis=1)
            fill.vertices = vertices
        self.status.value = f'{clock:.2f} sim s | integrate {integrate_ms:.1f} ms | ESDF {esdf_ms:.1f} ms'


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
    mapping = LiveMapping(stream.calibration, 1 if args.cameras == 'fixed' else 2, robot)
    q = np.asarray(baseline['raised_start']['joints_rad'], dtype=np.float32)
    velocity = np.zeros(6)
    state = tuned.joint_state(q, velocity)
    initial = robot.compute_kinematics(state).tool_poses.as_goal().clone()
    goal = v2.sixth.goal_from_scene()
    target = goal.position.detach().cpu().numpy().reshape(-1, 3)[0]
    start = v2.sixth.fourth.tool_position(robot, state.position)
    tool = start.copy()
    sequence = Sequence(mapping)
    bar, rod = sequence.pose(0., tool, target)
    depth, rgb, poses = stream.capture(q, bar, rod)
    grid, integrate_ms, esdf_ms = mapping.observe(depth, rgb, poses, state)
    print(f'V7_MAPPED {args.cameras} occupied={len(mapping.occupied)} integrate={integrate_ms:.1f} '
          f'esdf={esdf_ms:.1f}', flush=True)
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
            view.update(state, mapping, rgb, grid, bar, rod, 0., integrate_ms, esdf_ms)
        voxel_data = mpc.scene_collision_checker.data.voxels
        next_frame, step = 1, 0
        previous_q, previous_velocity, previous_time = q.copy(), velocity.copy(), 0.
        previous_bar = bar.copy()
        reaches, seeking_goal = 0, True
        touched = set()
        contacts = dict(bar=0, rod=0, floor=0)
        min_clearance = float('inf')
        timings = [(integrate_ms, esdf_ms)]
        probe = DecayProbe()
        flips = [list() for _ in range(6)]
        last_direction = np.zeros(6)
        peak_flips = 0
        frames = 0
        while args.interactive or step < round(DURATION/CONTROL_DT):
            clock = step*CONTROL_DT
            began = time.monotonic()
            bar, rod = sequence.pose(clock, tool, target)
            if next_frame/FRAME_RATE <= clock+1e-9:
                frame_time = next_frame/FRAME_RATE
                next_frame += 1
                observed_q = sample_held_pose(previous_q, previous_velocity, frame_time-previous_time, position)
                fraction = np.clip((frame_time-previous_time)/CONTROL_DT, 0., 1.)
                observed_bar = previous_bar+(bar-previous_bar)*fraction
                depth, rgb, poses = stream.capture(observed_q, observed_bar, rod)
                grid, integrate_ms, esdf_ms = mapping.observe(
                    depth, rgb, poses, tuned.joint_state(observed_q, np.zeros(6)))
                if voxel_data.features.data_ptr() != grid.feature_tensor.data_ptr():
                    voxel_data.update_features(grid.feature_tensor.reshape(-1, 1), grid.name)
                if not args.interactive:
                    timings.append((integrate_ms, esdf_ms))
                    probe.observe(frame_time, observed_bar, mapping.occupied.centers.cpu().numpy())
                if view is not None and frames % 3 == 0:
                    view.update(state, mapping, rgb, grid, bar, rod, clock, integrate_ms, esdf_ms)
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
            if goal_reached:
                reaches += int(seeking_goal)
                seeking_goal = not seeking_goal
                mpc.update_goal_tool_poses(goal if seeking_goal else initial, run_ik=False)
            spheres = sphere_series(robot, torch.as_tensor(path, device='cuda', dtype=torch.float32))
            clearances = {
                'bar': cylinder_clearance(spheres, bar, 0, .30, .02),
                'floor': float(np.min(floor_distances(spheres, mask, 0.))),
            }
            if rod is not None:
                clearances['rod'] = cylinder_clearance(spheres, (*rod[:2], rod[2]/2), 2, rod[2], .025)
            current = {name for name, value in clearances.items() if value < 0.}
            for name in current-touched:
                contacts[name] += 1
            touched = current
            min_clearance = min(min_clearance, *clearances.values())
            if not args.interactive:
                for j, speed_j in enumerate(velocity):
                    if goal_reached:
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
        arrays = np.asarray(timings)
        return dict(cameras=args.cameras, seconds=step*CONTROL_DT, rgbd_batches=len(timings),
                    contacts=contacts, reaches=reaches, minimum_clearance_m=min_clearance,
                    integrate_ms=dict(mean=float(arrays[:, 0].mean()), p95=float(np.percentile(arrays[:, 0], 95))),
                    esdf_ms=dict(mean=float(arrays[:, 1].mean()), p95=float(np.percentile(arrays[:, 1], 95))),
                    decay=probe.result(DURATION), max_direction_reversals_per_joint_in_half_second=peak_flips,
                    thrash_detected=peak_flips >= 3)
    finally:
        if view is not None:
            view.server.stop()
        mpc.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cameras', choices=('fixed', 'both'), default='both')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--smoke', action='store_true')
    mode.add_argument('--interactive', action='store_true')
    parser.add_argument('--port', type=int, default=int(os.environ.get('NEWTON_VIEWER_PORT', '30018')))
    parser.add_argument('--robot-config', type=Path,
                        default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    parser.add_argument('--newton-python', type=Path, default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args = parser.parse_args()
    if args.interactive:
        from curobo_moving_rod_v3 import check_viewer_port
        check_viewer_port(args.port)
    with tempfile.TemporaryDirectory(prefix='v7-newton-') as temp:
        stream = NewtonStream(args, Path(temp))
        try:
            result = run(args, stream)
        except KeyboardInterrupt:
            return
        finally:
            stream.close()
    if result is not None:
        print('V7_RESULT '+json.dumps(result, allow_nan=False), flush=True)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--renderer':
        render_worker(sys.argv[2])
    else:
        main()
