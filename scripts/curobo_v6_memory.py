#!/usr/bin/env python3
"""Camera-observed static and moving obstacle memory and Newton checkpoint."""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from itertools import product
import json
from pathlib import Path
import sys

import numpy as np

WINDOW_S = .5
STATIC_DISTANCE_M = .02
GROWTH_MPS = .20
EMPTY_MARGIN_M = .03
REACH_M = 1.
COLORS = {'STATIC': (65, 135, 245), 'MOVING': (255, 145, 35)}
SAMPLE_OFFSETS = np.array([*product((-.5, .5), repeat=3), (0., 0., 0.)])
ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = ROOT.parents[1]/'v6-memory-checkpoint.json.gz'


@dataclass
class DepthFrame:
    """Optical depth (positive forward), pose from optical coordinates to world."""
    depth: np.ndarray
    robot_mask: np.ndarray
    intrinsics: tuple[float, float, float, float]
    origin: np.ndarray
    rotation: np.ndarray

    def proves_empty(self, box):
        xyz = np.asarray(box['center_m']) + SAMPLE_OFFSETS*np.asarray(box['dims_m'])
        optical = (xyz-self.origin) @ self.rotation
        z = optical[:, 2]
        if not np.isfinite(optical).all() or np.any(z <= 0.):
            return False
        fx, fy, cx, cy = self.intrinsics
        uv = optical[:, :2]/z[:, None]*[fx, fy]+[cx, cy]
        height, width = self.depth.shape
        if np.any(uv < 0.) or np.any(uv >= [width, height]):
            return False
        x, y = np.floor(uv).astype(int).T
        depth = self.depth[y, x]
        return bool(np.all(~self.robot_mask[y, x] & np.isfinite(depth) &
                           (depth > z+EMPTY_MARGIN_M)))


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

    def observe(self, box, center, frame, time_s):
        if box is not None:
            self._seen(box, center, time_s)
        age = None if self.last_seen is None else max(0., time_s-self.last_seen)
        current = self.box
        if box is None and self.box is not None:
            if self.kind == 'MOVING':
                current = dict(self.box,
                               center_m=(np.asarray(self.box['center_m'])+self.velocity*age).tolist(),
                               dims_m=(np.asarray(self.box['dims_m'])+2*GROWTH_MPS*age*np.array([1., 1., 0.])).tolist())
            if frame.proves_empty(current):
                self._clear('seen_empty')
            elif self.kind == 'MOVING':
                delta = np.abs(np.asarray(current['center_m'])-self.base_origin)
                nearest = np.maximum(delta-np.asarray(current['dims_m'])/2, 0.)
                if np.linalg.norm(nearest) > REACH_M:
                    self._clear('out_of_reach')
            if self.box is None:
                current = None
        self.last_event = {'time_s': time_s, 'class': self.kind, 'since_seen_s': age,
                           'cleared_by': self.cleared_by, 'fresh_seen': box is not None,
                           'retained': current is not None,
                           'top_rule': False, 'growth_rule': bool(
                               box is None and current is not None and self.kind == 'MOVING')}
        return current, self.velocity.copy(), self.last_event

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
            self.kind = 'STATIC' if np.linalg.norm(displacement) < STATIC_DISTANCE_M else 'MOVING'
            self.velocity = displacement/elapsed if self.kind == 'MOVING' else np.zeros(3)
        elif self.box is None:
            self.kind = 'STATIC'
        self.box = dict(center_m=list(box['center_m']), dims_m=list(box['dims_m']))
        self.last_seen = time_s
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
        if request['rod'] is not None:
            arrays['rod_points'] = capture['vertical_points']
        # Uncompressed transport avoids recompressing the full sensor image on every observation.
        np.savez(request['output'], **arrays)
        metadata = dict(border_touches=capture['border_touches'],
                        border_pixel_counts=capture['border_pixel_counts'],
                        rod_pixels=capture['rod_pixels'],
                        rod_borders=capture.get('vertical_border_counts'))
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
        self.last_event = {**v5, **event}
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


def memory_label(event):
    age = event.get('since_seen_s')
    kind = event.get('class') or '—'
    age_text = '—' if age is None else f'{age:.2f} s'
    return f"{kind} · age {age_text} · {event.get('cleared_by', 'none')}"


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

    from curobo_moving_bar_poke_demo import DemoBarWorker, LiveDance, LiveVerticalRod, ROD_UNKNOWN_TOP_M
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
                camera = v2.sixth.fourth.Capture(q, arrays['points'], pose,
                                               {**metadata, 'depth_frame': frame})
                vertical = None
                if vertical_rod is not None:
                    borders = metadata['rod_borders']
                    vertical = v2.sixth.fourth.Capture(
                        q, arrays['rod_points'], pose,
                        dict(depth_frame=frame, border_touches={k: v > 0 for k, v in borders.items()},
                             border_pixel_counts=borders),
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

        def update(self, mpc, family, q, mask, horizon):
            super().update(mpc, family, q, mask, horizon)
            app = self.app
            app.observed.opacity = 0. if family == 'E' else .42
            self.controls.observed.opacity = app.observed.opacity
            bar = app.bar_memory if family == 'E' else None
            tracks = {}
            for name, memory in (('bar', bar), ('rod', self.memory if family == 'E' else None)):
                box = None if memory is None else memory.current
                event = {} if memory is None else memory.last_event
                paint(*app.memory_handles[name], box, event)
                tracks[name] = dict(box=box, event=event)
            if app.recording and family == 'E':
                app.active_mask = mask
                app.rows.append(dict(time_s=round(app.clock, 3), q=q.tolist(), bar=app.read_rod().tolist(),
                                     rod=self.pose, tracks=tracks))

    class V6Live(LiveDance):
        worker_type = V6Worker
        version = 'v6 object memory'

        def __init__(self, args, baseline, checkpoint=None):
            self.recording = checkpoint is not None
            self.checkpoint = checkpoint
            self.rows = []
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
            self.memory_handles = {
                name: (
                    scene_api.add_box(f'/v6/{name}', dimensions=(.01,)*3, opacity=.48, visible=False),
                    self.server.gui.add_text(f'{name.title()} memory', '—', disabled=True),
                ) for name in ('bar', 'rod')
            }
            return handles

        def camera_memory(self, fixed_top):
            self.bar_memory = CameraMemory(LowerPokeMemory(fixed_top), record=self.recording)
            return self.bar_memory

        def extra_obstacle(self):
            with self.poke_lock:
                self.dance_started = None
            self.active_vertical = V6Vertical(self)
            return self.active_vertical

        def before_obstacle_step(self, tool, sim_clock):
            self.clock = sim_clock
            super().before_obstacle_step(tool, sim_clock)
            if self.recording and self.current['group'] == 'E':
                near_goal = np.linalg.norm(tool-self.goal) <= v2.sixth.GOAL_TOLERANCE_M
                self.goal_entries += int(near_goal and not self.at_goal)
                self.at_goal = near_goal
                if sim_clock >= self.args.sim_seconds-base.v4.CONTROL_DT-1e-9:
                    self.stop_event.set()

        def truth_clearance(self, spheres, center):
            clearance = super().truth_clearance(spheres, center)
            if self.recording and self.current['group'] == 'E':
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


def checkpoint_run(types, args, baseline, name):
    from curobo_moving_bar_poke_demo import wait_for_demo

    app = types.Live(args, baseline, name)
    try:
        if name == 'Car':
            app.toggle_dance()
        else:
            app.gizmo.position = (2., .28, .20)
            app.vertical.gizmo.position = (.38, .44, 0.)
            app.vertical.enabled.value = True
        app.selector.value = 'E'
        wait_for_demo(app, lambda: True if app.current['group'] == 'E' and
                      app.current['state'] == 'stopped' else None, f'{name} checkpoint', timeout=180.)
        app.thread.join(timeout=5.)
        if app.thread.is_alive():
            raise RuntimeError(f'{name} checkpoint thread did not stop')
        memory = app.bar_memory if name == 'Car' else app.active_vertical.memory
        events = memory.events
        losses = [event for event in events if event['retained'] and not event['fresh_seen']]
        clears = [event for i, event in enumerate(events)
                  if event['cleared_by'] != 'none' and (i == 0 or events[i-1]['cleared_by'] == 'none')]
        summary = dict(contacts=app.contacts, reaches=app.goal_entries, minimum_clearance_m=float(app.minimum),
                       observed_frames=sum(event['fresh_seen'] for event in events),
                       moving_frames=sum(event['class'] == 'MOVING' for event in events),
                       max_retained_static_age_s=max((e['since_seen_s'] for e in losses if e['class'] == 'STATIC'),
                                                     default=0.),
                       clears=clears, duration_s=app.current['sim_time_s'])
        print(f'V6_{name.upper()} '+json.dumps(summary), flush=True)
        return dict(summary=summary, frames=app.rows, observations=events)
    finally:
        if not app.closed:
            app.close()


def replay(types, args, cases):
    import threading

    from curobo.types import ContentPath
    from curobo.viewer import ViserVisualizer

    types.base.v41.v3.check_viewer_port(args.port)
    viewer = ViserVisualizer(content_path=ContentPath(robot_config_file=str(args.robot_config)),
                              connect_ip='127.0.0.1', connect_port=args.port,
                              add_control_frames=True, visualize_robot_spheres=False, add_robot_to_scene=True)
    server = viewer._server
    stop = threading.Event()
    try:
        if server._websock_server._port != args.port:
            raise RuntimeError(f'viewer port {args.port} occupied')
        server.scene.add_box('/truth/floor', dimensions=(3., 3., .05),
                                     position=(0., 0., -.025), color=(120, 130, 140), opacity=.20)
        bar = server.scene.add_cylinder('/truth/bar', radius=types.scene.RADIUS, height=types.scene.LENGTH,
                                         wxyz=types.scene.ROTATION_WXYZ, color=(235, 55, 45))
        rod = server.scene.add_cylinder('/truth/rod', radius=.025, height=.50, color=(235, 55, 45))
        selector = server.gui.add_dropdown('Checkpoint', options=tuple(cases), initial_value=next(iter(cases)))
        max_time = max(case['frames'][-1]['time_s'] for case in cases.values())
        slider = server.gui.add_slider('Simulated time s', min=0., max=max_time, step=.025, initial_value=0.)
        play = server.gui.add_button('Play')
        status = server.gui.add_text('Result', '', disabled=True)
        handles = {name: (
            server.scene.add_box(f'/v6/{name}', dimensions=(.01,)*3, opacity=.48, visible=False),
            server.gui.add_text(f'{name.title()} memory', '—', disabled=True),
        ) for name in ('bar', 'rod')}
        server.gui.add_markdown('Blue = STATIC. Orange = MOVING.')

        def show(_event=None):
            case = cases[selector.value]
            row = case['frames'][min(round(slider.value/.025), len(case['frames'])-1)]
            # Record and replay the world pose actually used by the live MPC.
            q = np.asarray(row['q'], dtype=np.float32)
            viewer.set_joint_state(types.v2.sixth.arm_state(q).squeeze(0))
            bar.position = tuple(row['bar'])
            rod.visible = row['rod'] is not None
            if row['rod'] is not None:
                x, y, height = row['rod']
                rod.height = height
                rod.position = (x, y, height/2)
            for name, track in row['tracks'].items():
                paint(*handles[name], track['box'], track['event'])
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
                    if stop.wait(.025) or selector.value != name:
                        break
                    slider.value = row['time_s']
            finally:
                lock.release()

        selector.on_update(selected)
        slider.on_update(show)
        play.on_click(run)
        show()
        if args.viewer_smoke:
            for name, case in cases.items():
                selector.value = name
                slider.value = case['frames'][-1]['time_s']
                assert slider.value > 0. and selector.value == name
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
        print('V6_CHECKPOINT two E runs, estimated wall time 1–3 minutes', flush=True)
        cases = {}
        for name in ('Car', 'Sofa'):
            cases[name] = checkpoint_run(types, args, baseline, name)
            tmp = args.checkpoint.with_suffix('.tmp')
            with gzip.open(tmp, 'wt') as stream:
                json.dump(cases, stream, separators=(',', ':'), allow_nan=False)
            tmp.replace(args.checkpoint)
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
