#!/usr/bin/env python3
"""Camera-observed static and moving obstacle memory and Newton checkpoint."""
from __future__ import annotations

import argparse
import base64
from collections import deque
from dataclasses import dataclass
from itertools import product
import json
from pathlib import Path
import sys
import zlib

import numpy as np

WINDOW_S = .5
STATIC_DISTANCE_M = .02
EMPTY_MARGIN_M = .03
REACH_M = 1.
# A moving object's velocity is measured over WINDOW_S; predicting it further than that is guessing.
MOVING_MEMORY_S = WINDOW_S
COLORS = {'STATIC': (65, 135, 245), 'MOVING': (255, 145, 35)}
SAMPLE_OFFSETS = np.array([*product((-.5, .5), repeat=3), (0., 0., 0.)])
# Carving lattice: 5 x 5 x 3 samples over the box, each cleared only by depth seen past it.
CARVE_OFFSETS = np.array(list(product(np.linspace(-.5, .5, 5), np.linspace(-.5, .5, 5), (-.5, 0., .5))))
ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT/'docs/experiments/sim-camera-curobo/results/v6_memory_checkpoint.json.gz'
CONTROL_DT = .025
SOFA_ROD_XY = (.78, .52)
REPLAY_SIZE_LIMIT = 20_000_000


@dataclass
class DepthFrame:
    """Optical depth (positive forward), pose from optical coordinates to world."""
    depth: np.ndarray
    robot_mask: np.ndarray
    intrinsics: tuple[float, float, float, float]
    origin: np.ndarray
    rotation: np.ndarray

    def free(self, xyz):
        """Per point: True only if the camera saw past it (in image, not robot, depth beyond)."""
        optical = (np.asarray(xyz, dtype=float)-self.origin) @ self.rotation
        z = optical[:, 2]
        result = np.zeros(len(optical), dtype=bool)
        ok = np.isfinite(optical).all(axis=1) & (z > 0.)
        if not ok.any():
            return result
        fx, fy, cx, cy = self.intrinsics
        uv = np.full((len(optical), 2), -1.)
        uv[ok] = optical[ok, :2]/z[ok, None]*[fx, fy]+[cx, cy]
        height, width = self.depth.shape
        ok &= np.all(uv >= 0., axis=1) & (uv[:, 0] < width) & (uv[:, 1] < height)
        x, y = np.floor(uv[ok]).astype(int).T
        depth = self.depth[y, x]
        result[ok] = ~self.robot_mask[y, x] & np.isfinite(depth) & (depth > z[ok]+EMPTY_MARGIN_M)
        return result

    def proves_empty(self, box):
        xyz = np.asarray(box['center_m']) + SAMPLE_OFFSETS*np.asarray(box['dims_m'])
        return bool(self.free(xyz).all())

    def carve(self, box):
        """Remove the parts of a box the camera saw as empty; None when nothing is left."""
        center, dims = np.asarray(box['center_m'], dtype=float), np.asarray(box['dims_m'], dtype=float)
        samples = center+CARVE_OFFSETS*dims
        free = self.free(samples)
        if not free.any():
            return box
        left = samples[~free]
        if not len(left):
            return None
        cell = dims/np.array([4., 4., 2.])
        low = np.maximum(left.min(axis=0)-cell/2, center-dims/2)
        high = np.minimum(left.max(axis=0)+cell/2, center+dims/2)
        return dict(box, center_m=((low+high)/2).tolist(), dims_m=(high-low).tolist())


class ObjectMemory:
    """One object track. Only fresh observations contribute to classification."""
    def __init__(self, base_origin=(0., 0., 0.)):
        self.base_origin = np.asarray(base_origin, dtype=float)
        self.history = deque()
        self.box = None
        self.last_seen = None
        self.kind = None
        self.velocity = np.zeros(3)
        self.cleared_by = 'none'
        self.last_event = {}
        self.updated = None

    def observe(self, box, center, frame, time_s):
        if box is not None:
            self._seen(box, center, time_s)
        age = None if self.last_seen is None else max(0., time_s-self.last_seen)
        heading_in = False
        if box is None and self.box is not None:
            # The remembered box is a running state: propagate, then carve with this frame's depth.
            dt = max(0., time_s-self.updated)
            center = np.asarray(self.box['center_m'], dtype=float)
            dims = np.asarray(self.box['dims_m'], dtype=float)
            if self.kind == 'MOVING':
                # The object keeps its real size out of view; only its position is predicted.
                heading_in = not self.moving_away(center)
                center = center+self.velocity*dt
            self.box = frame.carve(dict(self.box, center_m=center.tolist(), dims_m=dims.tolist()))
            self.updated = time_s
            if self.box is None:
                self._clear('seen_empty')
            elif self.kind == 'MOVING' and age > MOVING_MEMORY_S+1e-9:
                self._clear('moved_on')
            elif self.kind == 'MOVING':
                delta = np.abs(np.asarray(self.box['center_m'])-self.base_origin)
                nearest = np.maximum(delta-np.asarray(self.box['dims_m'])/2, 0.)
                if np.linalg.norm(nearest) > REACH_M:
                    self._clear('out_of_reach')
        current = None if self.box is None else dict(self.box)
        self.last_event = {'time_s': time_s, 'class': self.kind, 'since_seen_s': age,
                           'cleared_by': self.cleared_by, 'fresh_seen': box is not None,
                           'retained': current is not None, 'top_rule': False,
                           'moving_away': bool(self.kind == 'MOVING' and current is not None and not heading_in
                                               and box is None),
                           'heading_in': bool(box is None and current is not None and heading_in),
                           'growth_rule': False}
        return current, self.velocity.copy(), self.last_event

    def moving_away(self, center):
        return float(np.dot(self.velocity[:2], center[:2]-self.base_origin[:2])) > 0.

    def _seen(self, box, center, time_s):
        center = np.array(center, dtype=float)
        if self.history and time_s-self.history[-1][0] > WINDOW_S:
            self.history.clear()
        self.history.append((time_s, center))
        cutoff = time_s-WINDOW_S
        while len(self.history) > 1 and self.history[1][0] <= cutoff+1e-9:
            self.history.popleft()
        oldest, reference = self.history[0]
        if oldest < cutoff and len(self.history) > 1:
            newer, sample = self.history[1]
            reference = reference+(sample-reference)*(cutoff-oldest)/(newer-oldest)
            oldest = cutoff
        elapsed = time_s-oldest
        if elapsed > 0.:
            displacement = center-reference
            # Speed, not distance: a short track (< 0.5 s seen) must not look static just because it
            # had little time to move. 0.02 m per 0.5 s = 0.04 m/s.
            speed = np.linalg.norm(displacement)/elapsed
            self.kind = 'STATIC' if speed < STATIC_DISTANCE_M/WINDOW_S else 'MOVING'
            self.velocity = displacement/elapsed if self.kind == 'MOVING' else np.zeros(3)
        elif self.box is None:
            self.kind = 'STATIC'
        self.box = dict(center_m=list(box['center_m']), dims_m=list(box['dims_m']))
        self.last_seen = time_s
        self.updated = time_s
        self.cleared_by = 'none'

    def _clear(self, reason):
        self.box = None
        self.history.clear()
        self.velocity = np.zeros(3)
        self.cleared_by = reason


def renderer():
    """Run in the existing Newton environment and pass full-resolution evidence."""
    import newton_camera_moving_rod_worker as worker

    mount, _ = worker.camera_common.read_mount_config(ROOT/'config/sim_camera_mount.yaml')
    scene = worker.MovingRodCamera(Path.home()/'newton_ws/ros_bridge_assets/ur5_robotiq.newton.urdf',
                                  mount, 'cuda:0', horizontal_bar=True, bar_length=.30,
                                  vertical_obstacle=True)
    fy = worker.HEIGHT/(2*np.tan(worker.VERTICAL_FOV_RAD/2))
    intrinsics = [fy, fy, worker.WIDTH/2, worker.HEIGHT/2]
    print('NEWTON_MOVING_ROD_WORKER_READY {"scene_builds":1}', flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        if request.get('command') == 'shutdown':
            return
        capture = scene.capture(np.asarray(request['q'], dtype=np.float32), *request['bar'],
                                vertical=request['rod'])
        transform = capture['camera_transform']
        rotation = worker.camera_common.quaternion_xyzw_to_matrix(transform[3:7])
        optical_to_world = rotation @ np.diag([1., -1., -1.])
        shape_ids = scene.shape_index.numpy()[0, 0]
        arrays = dict(points=capture['points'], pose=transform, preview=capture['depth_preview'],
                      depth=scene.depth.numpy()[0, 0], robot_mask=shape_ids < scene.robot_shape_count,
                      intrinsics=intrinsics, rotation=optical_to_world)
        vertical_pixels = None
        if request['rod'] is not None:
            arrays['rod_points'] = capture['vertical_points']
            _, vertical_shape = scene.vertical_bodies[request['rod'][2]]
            depth = arrays['depth']
            vertical_pixels = int(np.count_nonzero((shape_ids == vertical_shape) & np.isfinite(depth) & (depth > 0.)))
        # Uncompressed transport avoids recompressing the full sensor image on every observation.
        np.savez(request['output'], **arrays)
        metadata = dict(border_touches=capture['border_touches'],
                        border_pixel_counts=capture['border_pixel_counts'],
                        bar_pixels=capture['rod_pixels'],
                        rod_borders=capture.get('vertical_border_counts'), vertical_pixels=vertical_pixels)
        print('NEWTON_MOVING_ROD_WORKER_RESPONSE '+json.dumps(metadata), flush=True)


class FreshPoints:
    def update(self, capture, _time_s):
        return capture.points, 0


class CameraMemory:
    """Reuse v5 fitting and height rules with a v6 world-space track."""
    def __init__(self, policy, center_from_points=None, *, record=False):
        self.policy = policy
        self.policy.cloud = FreshPoints()
        self.center_from_points = center_from_points
        self.track = ObjectMemory()
        self.growth_events = 0
        self.events = [] if record else deque(maxlen=300)
        self.last_event = {}
        self.current = None
        self.display_boxes = (None, None)

    def boxes(self, *args):
        observed, predicted, raised, capped = self.policy.boxes(*args)
        self.display_boxes = observed, predicted
        return observed, predicted, raised, capped

    def __getattr__(self, name):
        return getattr(self.policy, name)

    def observe(self, capture, time_s):
        fit, _, _, v5 = self.policy.observe(capture, time_s)
        fresh = fit if v5['fresh_seen'] else None
        center = None if fresh is None else fresh['center_m']
        if fresh is not None and self.center_from_points is not None:
            center = self.center_from_points(capture.points, center)
        current, velocity, event = self.track.observe(fresh, center, capture.metadata['depth_frame'], time_s)
        self.current = current
        self.last_event = {**v5, **event, 'obstacle_pixels': capture.metadata['obstacle_pixels']}
        self.growth_events += int(event['growth_rule'])
        self.events.append(self.last_event)
        return current, velocity, capture.points, self.last_event


def standing_center(points, fallback, height):
    """Estimate the upright cylinder axis from depth, using its configured height."""
    side = points[points[:, 2] < np.max(points[:, 2])-.004, :2]
    xy = np.asarray(fallback[:2])
    if len(side) >= 3:
        mean = side.mean(axis=0)
        offsets = side-mean
        lhs = np.column_stack((2*offsets, np.ones(len(side))))
        solution, _, rank, _ = np.linalg.lstsq(lhs, np.sum(offsets**2, axis=1), rcond=None)
        if rank == 3:
            center = mean+solution[:2]
            radius_squared = np.sum(solution[:2]**2)+solution[2]
            if .020**2 < radius_squared < .030**2:
                xy = center
    return [*xy, height/2]


def path_target(tool, goal):
    """Aim the inner tip along the tool-to-goal segment, below the tool."""
    delta = np.asarray(goal)-tool
    distance = np.linalg.norm(delta)
    target = np.asarray(tool, dtype=float)+delta*min(1., .15/max(distance, .15))
    target[2] = np.clip(target[2], .15, tool[2]-.12)
    return target


def visible_path_target(tool, goal, frame, margin=.10):
    """Nearest point on the tool -> goal path, at or below path_target height, whose bar tip and the
    first 5 cm of bar behind it (+X) project inside the image with a margin. None if the camera sees
    no part of the path: then this poke would test nothing, so it is skipped."""
    tool = np.asarray(tool, dtype=float)
    delta = np.asarray(goal, dtype=float)-tool
    distance = float(np.linalg.norm(delta))
    height, width = frame.depth.shape
    low = np.array([margin*width, margin*height])
    high = np.array([(1-margin)*width, (1-margin)*height])
    for along in np.arange(.05, max(distance, .05)+1e-9, .01):
        point = tool+delta*min(1., along/max(distance, 1e-9))
        for z in np.arange(min(point[2], tool[2]-.12), .15-1e-9, -.01):
            tip = np.array([point[0], point[1], z])
            ok = True
            for sample in (tip, tip+[.05, 0., 0.]):
                inside, xy = tip_projection(frame, sample)
                if not inside or np.any(np.asarray(xy) < low) or np.any(np.asarray(xy) > high):
                    ok = False
                    break
            if ok:
                return tip
    return None


def tip_projection(frame, tip):
    optical = (np.asarray(tip)-frame.origin) @ frame.rotation
    if not np.isfinite(optical).all() or optical[2] <= 0.:
        return False, None
    fx, fy, cx, cy = frame.intrinsics
    xy = optical[:2]/optical[2]*[fx, fy]+[cx, cy]
    height, width = frame.depth.shape
    inside = bool(np.all((xy >= 0.) & (xy < [width, height])))
    return inside, xy.tolist()


def record_poke_frame(poke, frame, time_s, pixels):
    if pixels:
        intervals = poke['seen_intervals_s']
        if intervals and time_s-intervals[-1][1] <= 1/30+1e-8:
            intervals[-1][1] = time_s
        else:
            intervals.append([time_s, time_s])
    if poke['peak_projection'] is None and time_s >= poke['closest_time_s']-1e-9:
        inside, xy = tip_projection(frame, poke['target_m'])
        poke['peak_projection'] = dict(frame_time_s=time_s, in_image=inside, pixel_xy=xy)
        print('V6_POKE_PEAK '+json.dumps(poke), flush=True)


def memory_label(event):
    age = event.get('since_seen_s')
    kind = event.get('class') or '—'
    age_text = '—' if age is None else f'{age:.2f} s'
    motion = ' · leaving' if event.get('moving_away') else (' · heading in' if event.get('heading_in') else '')
    return f"{kind}{motion} · age {age_text} · {event.get('cleared_by', 'none')}"


def paint(handle, label, box, event):
    label.value = memory_label(event)
    handle.visible = box is not None
    if box is not None:
        handle.color = COLORS[event['class']]
        handle.position = tuple(box['center_m'])
        handle.dimensions = tuple(box['dims_m'])


def demo_types():
    """Load cuRobo only in the controller process."""
    import subprocess
    from types import SimpleNamespace

    from curobo_moving_bar_poke_demo import (
        DANCE_PERIOD_S, DANCE_SPEED_MPS, DANCE_TRAVEL_M, DemoBarWorker, LiveDance,
        LiveVerticalRod, ROD_UNKNOWN_TOP_M, dance_center,
    )
    from curobo_moving_bar_v5_viewer import LivePoke
    from curobo_moving_bar_poke_height import LowerPokeMemory, scene
    from curobo_moving_bar_v5 import base, v2
    from curobo_moving_rod_v5_rules import HeightMemory

    class V6Worker(DemoBarWorker):
        def start(self):
            self.process = subprocess.Popen(
                [str(self.python), str(Path(__file__).resolve()), '--capture-worker'],
                cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
            )
            self._read_prefix('NEWTON_MOVING_ROD_WORKER_READY ')

        def capture_xy(self, q, center, *, vertical_rod=None):
            self.counter += 1
            output = self.temp_dir/'optical.npz'
            request = dict(q=q.tolist(), bar=list(center), rod=vertical_rod, output=str(output))
            self.process.stdin.write(json.dumps(request)+'\n')
            self.process.stdin.flush()
            metadata = json.loads(self._read_prefix('NEWTON_MOVING_ROD_WORKER_RESPONSE '))
            with np.load(output) as arrays:
                pose = arrays['pose']
                frame = DepthFrame(arrays['depth'], arrays['robot_mask'], tuple(arrays['intrinsics']),
                                   pose[:3], arrays['rotation'])
                bar_pixels = metadata['bar_pixels']
                camera = v2.sixth.fourth.Capture(
                    q, arrays['points'], pose,
                    {**metadata, 'depth_frame': frame, 'rod_pixels': bar_pixels, 'obstacle_pixels': bar_pixels},
                )
                vertical = None
                if vertical_rod is not None:
                    borders = metadata['rod_borders']
                    vertical = v2.sixth.fourth.Capture(
                        q, arrays['rod_points'], pose,
                        dict(depth_frame=frame, border_touches={k: v > 0 for k, v in borders.items()},
                             border_pixel_counts=borders, obstacle_pixels=metadata['vertical_pixels']),
                    )
                return v2.RodCapture(camera, arrays['preview'], vertical)

    class V6Vertical(LiveVerticalRod):
        def __init__(self, app):
            self.app = app
            super().__init__(app.vertical, lambda: app.graphics.visible.value)

        def reset_memory(self):
            super().reset_memory()
            self.memory = CameraMemory(HeightMemory(ROD_UNKNOWN_TOP_M), self.measure_center,
                                       record=self.app.recording)

        def measure_center(self, points, fallback):
            return standing_center(points, fallback, self.pose[2])

        def capture(self, worker, q, center, time_s):
            capture = super().capture(worker, q, center, time_s)
            self.app.last_depth_frame = capture.camera.metadata['depth_frame']
            poke = self.app.active_poke
            if poke is not None and time_s >= poke['start_s']-1e-9:
                metadata = capture.camera.metadata
                record_poke_frame(poke, metadata['depth_frame'], time_s, metadata['obstacle_pixels'])
            if self.app.recording:
                self.app.camera_frames.append(camera_sample(capture, time_s, v2.sixth.fourth.matrix_to_wxyz))
                if self.app.checkpoint == 'Sofa':
                    self.check_sofa_capture(capture.vertical_camera, time_s)
            return capture

        def check_sofa_capture(self, camera, time_s):
            pixels = camera.metadata['obstacle_pixels']
            if time_s == 0.:
                if not self.memory.last_event['fresh_seen'] or not pixels:
                    raise RuntimeError('Sofa did not produce a rod observation at startup')
                print(f'V6_SOFA seen at startup ({pixels} pixels)', flush=True)
            elif not pixels and self.app.first_unseen is None:
                self.app.first_unseen = time_s
                print(f'V6_SOFA first unseen at {time_s:.3f} simulated seconds', flush=True)

        def update(self, mpc, family, q, mask, horizon):
            super().update(mpc, family, q, mask, horizon)
            app = self.app
            bar = app.bar_memory if family == 'E' else None
            recording = app.recording and family == app.checkpoint_mode
            tracks = {}
            for name, memory in (('bar', bar), ('rod', self.memory if family == 'E' else None)):
                box = None if memory is None else memory.current
                event = {} if memory is None else memory.last_event
                paint(*app.memory_handles[name], box, event)
                if recording:
                    observed, predicted = (None, None) if memory is None else memory.display_boxes
                    if family == 'B' and name == 'rod':
                        x, y, height = self.pose
                        observed = predicted = dict(center_m=[x, y, height/2], dims_m=[.05, .05, height])
                    tracks[name] = dict(box=box, event=event, observed=observed, predicted=predicted,
                                        velocity_mps=([0.]*3 if memory is None else memory.track.velocity.tolist()))
            if recording:
                app.active_mask = mask
                bar_center = app.read_rod().copy()
                true_velocity = (np.zeros(3) if app.last_bar is None else
                                 (bar_center-app.last_bar)/CONTROL_DT)
                app.last_bar = bar_center
                app.rows.append(dict(time_s=round(app.clock, 3), q=q.tolist(), bar=bar_center.tolist(),
                                     rod=self.pose, tracks=tracks, camera_index=len(app.camera_frames)-1,
                                     true_bar_velocity_mps=true_velocity.tolist(), horizon_s=horizon))

    class V6Live(LiveDance):
        worker_type = V6Worker
        version = 'v6 object memory'

        def __init__(self, args, baseline, checkpoint=None, mode='E'):
            self.recording = checkpoint is not None
            self.checkpoint = checkpoint
            self.checkpoint_mode = mode
            self.pokes = []
            self.active_poke = None
            self.rows = []
            self.camera_frames = []
            self.last_bar = None
            self.first_unseen = None
            self.goal_entries = 0
            self.at_goal = False
            self.minimum = np.inf
            self.contacts = {'bar': 0, 'rod': 0, 'floor': 0}
            self.touching = set()
            self.goal = v2.sixth.goal_from_scene().position.detach().cpu().numpy().reshape(-1, 3)[0]
            self.active_mask = None
            super().__init__(args, baseline)

        def obstacle_handles(self, scene_api):
            handles = super().obstacle_handles(scene_api)
            self.vertical.observed.wireframe = True
            self.vertical.gizmo.position = (*SOFA_ROD_XY, 0.)
            self.server.gui.add_markdown('Memory fill: blue = STATIC, orange = MOVING. Orange outline = observed box.')
            self.memory_handles = {
                name: (
                    scene_api.add_box(f'/v6/{name}', dimensions=(.01,)*3, opacity=.48, visible=False),
                    self.server.gui.add_text(f'{name.title()} memory', '—', disabled=True),
                ) for name in ('bar', 'rod')
            }
            return handles

        def extra_obstacle(self):
            self.observed.wireframe = True
            with self.poke_lock:
                self.dance_started = None
                self.active_poke = None
            self.active_vertical = V6Vertical(self)
            return self.active_vertical

        def camera_memory(self, fixed_top):
            self.bar_memory = CameraMemory(LowerPokeMemory(fixed_top), record=self.recording)
            return self.bar_memory

        def move_dance(self, tool, sim_clock):
            with self.poke_lock:
                if self.dancing:
                    if self.dance_started is None or sim_clock-self.dance_started >= DANCE_PERIOD_S-1e-9:
                        frame = getattr(self, 'last_depth_frame', None)
                        if frame is None:
                            # No camera image yet: wait for the first one before aiming.
                            self.gizmo.position = (2., .28, .20)
                            return
                        self.dance_started = sim_clock
                        visible = visible_path_target(tool, self.goal, frame)
                        self.dance_target = path_target(tool, self.goal) if visible is None else visible
                        self.active_poke = dict(start_s=sim_clock, target_m=self.dance_target.tolist(),
                                                aimed_in_view=visible is not None,
                                                closest_time_s=sim_clock+DANCE_TRAVEL_M/DANCE_SPEED_MPS,
                                                peak_projection=None, seen_intervals_s=[])
                        if self.recording:
                            self.pokes.append(self.active_poke)
                    if not self.active_poke.get('aimed_in_view', True):
                        # The camera sees no part of the path: park the bar and log it, never poke blind.
                        self.gizmo.position = (2., .28, .20)
                        return
                    position = dance_center(self.dance_target, sim_clock-self.dance_started)
                    self.gizmo.position = (*position[:2], self.dance_target[2])
                    return
                self.active_poke = None
            LivePoke.before_obstacle_step(self, tool, sim_clock)

        def before_obstacle_step(self, tool, sim_clock):
            self.clock = sim_clock
            self.move_dance(tool, sim_clock)
            if self.recording and self.current['group'] == self.checkpoint_mode:
                near_goal = np.linalg.norm(tool-self.goal) <= v2.sixth.GOAL_TOLERANCE_M
                self.goal_entries += int(near_goal and not self.at_goal)
                self.at_goal = near_goal
                if sim_clock >= self.args.sim_seconds-base.v4.CONTROL_DT-1e-9:
                    self.stop_event.set()

        def truth_clearance(self, spheres, center):
            clearance = super().truth_clearance(spheres, center)
            if self.recording and self.current['group'] == self.checkpoint_mode:
                rod = self.active_vertical.clearance(spheres)
                floor = np.min(spheres[self.active_mask, 2]-spheres[self.active_mask, 3])
                distances = {'bar': clearance, 'rod': rod, 'floor': floor}
                self.minimum = min(self.minimum, *distances.values())
                touches = {part for part, distance in distances.items() if distance < 0.}
                for part in touches-self.touching:
                    self.contacts[part] += 1
                self.touching = touches
            return clearance

    return SimpleNamespace(Live=V6Live, Worker=V6Worker, v2=v2, base=base, scene=scene)


def camera_sample(capture, time_s, matrix_to_wxyz):
    """Store the actual Newton observation without the clearance-only pixel mask."""
    frame = capture.camera.metadata['depth_frame']
    points = capture.camera.points
    if capture.vertical_camera is not None:
        points = np.concatenate((points, capture.vertical_camera.points))
    preview = capture.depth_preview
    encoded = base64.b64encode(zlib.compress(preview.tobytes())).decode('ascii')
    height, width = frame.depth.shape
    return dict(time_s=time_s, position_m=frame.origin.tolist(),
                wxyz=matrix_to_wxyz(frame.rotation).tolist(),
                fov_rad=float(2*np.arctan(height/(2*frame.intrinsics[1]))),
                aspect=width/height, points_m=points.tolist(),
                depth_shape=preview.shape, depth_zlib=encoded)


def decode_depth(frame):
    raw = zlib.decompress(base64.b64decode(frame['depth_zlib']))
    return np.frombuffer(raw, dtype=np.uint8).reshape(frame['depth_shape'])


def write_checkpoint(path, cases):
    import gzip

    def write(payload):
        with gzip.open(tmp, 'wt') as stream:
            json.dump(payload, stream, separators=(',', ':'), allow_nan=False)

    tmp = path.with_suffix('.tmp')
    write(cases)
    if tmp.stat().st_size > REPLAY_SIZE_LIMIT:
        # 30 Hz observations become 10 Hz; keep control poses and events intact.
        thinned = {
            name: (case if case.get('camera_stride') else
                   dict(case, camera_frames=case['camera_frames'][::3],
                        frames=[dict(row, camera_index=row['camera_index']//3) for row in case['frames']]))
            for name, case in cases.items()
        }
        write(thinned)
    tmp.replace(path)


def checkpoint_run(types, args, baseline, name, mode='E', *, rod_xy=SOFA_ROD_XY):
    from curobo_moving_bar_poke_demo import wait_for_demo

    app = types.Live(args, baseline, name, mode)
    try:
        if name == 'Car':
            app.toggle_dance()
        else:
            app.gizmo.position = (2., .28, .20)
            app.vertical.gizmo.position = (*rod_xy, 0.)
            app.vertical.enabled.value = True
        app.selector.value = mode
        wait_for_demo(app, lambda: True if app.current['group'] == mode and
                      app.current['state'] == 'stopped' else None, f'{name} checkpoint', timeout=180.)
        app.thread.join(timeout=5.)
        if app.thread.is_alive():
            raise RuntimeError(f'{name} checkpoint thread did not stop')
        memory = app.bar_memory if name == 'Car' else app.active_vertical.memory
        events = memory.events if mode == 'E' else []
        losses = [event for event in events if event['retained'] and not event['fresh_seen']]
        clears = [event for i, event in enumerate(events)
                  if event['cleared_by'] != 'none' and (i == 0 or events[i-1]['cleared_by'] == 'none')]
        summary = dict(mode=mode, first_unseen_s=app.first_unseen, contacts=app.contacts,
                       reaches=app.goal_entries, minimum_clearance_m=float(app.minimum),
                       observed_frames=sum(event['fresh_seen'] for event in events),
                       moving_frames=sum(event['class'] == 'MOVING' for event in events),
                       max_retained_static_age_s=max((e['since_seen_s'] for e in losses if e['class'] == 'STATIC'),
                                                     default=0.),
                       clears=clears, duration_s=app.current['sim_time_s'])
        print(f'V6_RESULT {name} {mode} '+json.dumps(summary), flush=True)
        if name == 'Car':
            for poke in app.pokes:
                print('V6_CAR_POKE '+json.dumps(poke), flush=True)
        if name == 'Sofa' and mode == 'E' and app.first_unseen is None:
            raise RuntimeError('Sofa stayed in the camera view for the entire checkpoint')
        return dict(summary=summary, frames=app.rows, camera_frames=app.camera_frames, observations=events,
                    pokes=app.pokes)
    finally:
        if not app.closed:
            app.close()


def render_truth_replay(types, args, case):
    """Passively render B's recorded poses after truth-box control."""
    import tempfile

    with tempfile.TemporaryDirectory(prefix='v6-truth-view-') as tmp:
        worker = types.Worker(args.newton_python, Path(tmp))
        try:
            worker.start()
            for i, row in enumerate(case['frames']):
                if i % 4 == 0:
                    capture = worker.capture_xy(np.asarray(row['q'], dtype=np.float32), row['bar'],
                                                vertical_rod=row['rod'])
                    sample = camera_sample(capture, row['time_s'], types.v2.sixth.fourth.matrix_to_wxyz)
                    case['camera_frames'].append(sample)
                row['camera_index'] = len(case['camera_frames'])-1
        finally:
            worker.close()
    case['camera_stride'] = 4



def replay(types, args, cases):
    import threading

    from curobo.types import ContentPath
    from curobo.viewer import ViserVisualizer
    from curobo_moving_rod_v4_visuals import PredictionVisuals

    types.base.v41.v3.check_viewer_port(args.port)
    viewer = ViserVisualizer(content_path=ContentPath(robot_config_file=str(args.robot_config)),
                              connect_ip='127.0.0.1', connect_port=args.port,
                              add_control_frames=True, visualize_robot_spheres=False, add_robot_to_scene=True)
    server = viewer._server
    stop = threading.Event()
    try:
        if server._websock_server._port != args.port:
            raise RuntimeError(f'viewer port {args.port} occupied')
        graphics = PredictionVisuals(server, 0., obstacle=types.scene)
        bar = server.scene.add_cylinder('/truth/bar', radius=types.scene.RADIUS, height=types.scene.LENGTH,
                                         wxyz=types.scene.ROTATION_WXYZ, color=(235, 55, 45))
        rod = server.scene.add_cylinder('/truth/rod', radius=.025, height=.50, color=(235, 55, 45))
        selector = server.gui.add_dropdown('Checkpoint', options=tuple(cases), initial_value=next(iter(cases)))
        max_time = max(case['frames'][-1]['time_s'] for case in cases.values())
        slider = server.gui.add_slider('Simulated time s', min=0., max=max_time, step=CONTROL_DT, initial_value=0.)
        play = server.gui.add_button('Play')
        status = server.gui.add_text('Result', '', disabled=True)
        handles = {name: (
            server.scene.add_box(f'/v6/{name}', dimensions=(.01,)*3, opacity=.48, visible=False),
            server.gui.add_text(f'{name.title()} memory', '—', disabled=True),
        ) for name in ('bar', 'rod')}
        server.gui.add_markdown('Memory fill: blue = STATIC, orange = MOVING. Orange outline = observed box.')
        first_camera = cases[selector.value]['camera_frames'][0]
        frustum = server.scene.add_camera_frustum('/camera/live', fov=first_camera['fov_rad'],
                                                  aspect=first_camera['aspect'], scale=.13)
        cloud = server.scene.add_point_cloud('/camera/points', points=np.zeros((0, 3)),
                                             colors=(220, 0, 220), point_size=.011)
        panel = server.gui.add_image(decode_depth(first_camera), label='Simulated depth')
        outlines = {}
        for name in handles:
            outlines[name] = tuple(
                server.scene.add_box(f'/{kind}/{name}', dimensions=(.01,)*3, color=color,
                                     opacity=.85, wireframe=True, visible=False)
                for kind, color in (('observed', (255, 145, 35)), ('predicted', (80, 150, 255)))
            )
        displayed_camera = None

        def show(_event=None):
            nonlocal displayed_camera
            case = cases[selector.value]
            row = case['frames'][min(round(slider.value/CONTROL_DT), len(case['frames'])-1)]
            # Record and replay the world pose actually used by the live MPC.
            q = np.asarray(row['q'], dtype=np.float32)
            viewer.set_joint_state(types.v2.sixth.arm_state(q).squeeze(0))
            bar.position = tuple(row['bar'])
            rod.visible = row['rod'] is not None
            if row['rod'] is not None:
                x, y, height = row['rod']
                rod.height = height
                rod.position = (x, y, height/2)
            token = selector.value, row['camera_index']
            if token != displayed_camera:
                camera = case['camera_frames'][row['camera_index']]
                frustum.position, frustum.wxyz = tuple(camera['position_m']), tuple(camera['wxyz'])
                cloud.points = np.asarray(camera['points_m'], dtype=np.float32).reshape(-1, 3)
                panel.image = decode_depth(camera)
                displayed_camera = token
            for name, track in row['tracks'].items():
                paint(*handles[name], track['box'], track['event'])
                for key, handle in zip(('observed', 'predicted'), outlines[name]):
                    fit = track[key]
                    handle.visible = fit is not None and (key == 'observed' or graphics.visible.value)
                    if fit is not None:
                        handle.position = tuple(fit['center_m'])
                        handle.dimensions = tuple(fit['dims_m'])
            focus = 'rod' if row['rod'] is not None else 'bar'
            if focus == 'rod':
                graphics.ghost.height, graphics.ghost.radius = rod.height, rod.radius
                graphics.ghost.wxyz = (1., 0., 0., 0.)
                truth = np.zeros(3)
            else:
                graphics.ghost.height, graphics.ghost.radius = bar.height, bar.radius
                graphics.ghost.wxyz = types.scene.ROTATION_WXYZ
                truth = row['true_bar_velocity_mps']
            track = row['tracks'][focus]
            graphics.update(track['observed'], track['velocity_mps'], truth, row['horizon_s'],
                            row['horizon_s'] > 0., outlines[focus][1])
            summary = case['summary']
            status.value = f"{summary['reaches']} reaches · {sum(summary['contacts'].values())} contacts"

        def selected(_event=None):
            slider.value = 0.
            show()

        lock = threading.Lock()

        def run(_event=None):
            if not lock.acquire(blocking=False):
                return
            try:
                name = selector.value
                for row in cases[name]['frames']:
                    if stop.wait(CONTROL_DT) or selector.value != name:
                        break
                    slider.value = row['time_s']
            finally:
                lock.release()

        selector.on_update(selected)
        slider.on_update(show)
        graphics.visible.on_update(show)
        play.on_click(run)
        show()
        if args.viewer_smoke:
            for name, case in cases.items():
                selector.value = name
                focus = 'rod' if case['frames'][0]['rod'] is not None else 'bar'
                point_row = next((row for row in case['frames']
                                  if case['camera_frames'][row['camera_index']]['points_m']), None)
                if point_row is not None:
                    slider.value = point_row['time_s']
                    assert len(cloud.points)
                visible_row = next((row for row in case['frames'] if row['tracks'][focus]['observed']), None)
                if visible_row is not None:
                    slider.value = visible_row['time_s']
                    assert outlines[focus][0].visible
                    assert outlines[focus][1].visible and graphics.ghost.visible
                    graphics.visible.value = False
                    assert not outlines[focus][1].visible and not graphics.ghost.visible
                    graphics.visible.value = True
                    assert outlines[focus][1].visible and graphics.ghost.visible
                else:
                    assert not outlines[focus][0].visible
                slider.value = case['frames'][-1]['time_s']
                assert slider.value > 0. and selector.value == name
            car = cases['Car']
            with_arrow = next((row for row in car['frames'] if row['tracks']['bar']['observed']
                               and np.linalg.norm(row['tracks']['bar']['velocity_mps']) > 1e-9), None)
            if with_arrow is not None:
                selector.value = 'Car'
                slider.value = with_arrow['time_s']
                assert graphics.arrow.visible
            print(f'V6_REPLAY_SMOKE camera, boxes and prediction toggle passed; '
                  f'nonzero arrow sample={with_arrow is not None}', flush=True)
        print(f'V6_REPLAY http://127.0.0.1:{args.port}', flush=True)
        types.base.wait_for_viewer(args.duration)
    finally:
        stop.set()
        server.stop()


def main():
    import gzip
    import os

    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    for mode in ('smoke', 'replay', 'interactive'):
        modes.add_argument(f'--{mode}', action='store_true')
    parser.add_argument('--port', type=int, default=int(os.environ.get('NEWTON_VIEWER_PORT', '30006')))
    parser.add_argument('--duration', type=float, default=0., help='Viewer wall-time limit in seconds')
    parser.add_argument('--sim-seconds', type=float, default=9., help='Simulated seconds per checkpoint')
    parser.add_argument('--viewer-smoke', action='store_true')
    parser.add_argument('--checkpoint', type=Path, default=CHECKPOINT)
    parser.add_argument('--robot-config', type=Path,
                        default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    parser.add_argument('--newton-python', type=Path, default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args = parser.parse_args()
    if args.viewer_smoke and not args.replay:
        parser.error('--viewer-smoke requires --replay')
    if not np.isfinite(args.sim_seconds) or args.sim_seconds < 3.:
        parser.error('--sim-seconds must be at least 3')
    types = demo_types()
    if args.replay:
        with gzip.open(args.checkpoint, 'rt') as stream:
            cases = json.load(stream)
        replay(types, args, cases)
        return
    baseline = json.loads(types.base.tuned.original.BASELINE_FILE.read_text())
    types.base.tuned.original.configure_scene(baseline)
    types.base.v41.v3.check_viewer_port(args.port)
    if args.smoke:
        print('V6_CHECKPOINT Car E, Sofa B and Sofa E; estimated wall time 2–4 minutes', flush=True)
        cases = {'Car': checkpoint_run(types, args, baseline, 'Car')}
        write_checkpoint(args.checkpoint, cases)
        truth = checkpoint_run(types, args, baseline, 'Sofa', 'B')
        render_truth_replay(types, args, truth)
        cases['Sofa B'] = truth
        write_checkpoint(args.checkpoint, cases)
        if not truth['summary']['reaches']:
            raise RuntimeError('Sofa B did not establish a reachable rod position')
        cases['Sofa'] = checkpoint_run(types, args, baseline, 'Sofa')
        write_checkpoint(args.checkpoint, cases)
        print(f'V6_CHECKPOINT {args.checkpoint}', flush=True)
    else:
        app = types.Live(args, baseline)
        try:
            print(f'V6_INTERACTIVE http://127.0.0.1:{args.port}', flush=True)
            types.base.wait_for_viewer(args.duration)
        finally:
            if not app.closed:
                app.close()


if __name__ == '__main__':
    if sys.argv[1:] == ['--capture-worker']:
        renderer()
    else:
        main()
