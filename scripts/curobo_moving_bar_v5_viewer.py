"""Viser views for the scripted bar checkpoint and the XYZ drag scene."""
import functools
import json
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

import curobo_moving_bar_poke_v5_scene as poke
import curobo_moving_bar_v5_scene as bar
import curobo_moving_rod_v42 as base
from curobo_moving_rod_mpc import render_frames
from curobo_moving_rod_v4_live import LiveGroups, smoke
from curobo_moving_rod_v4_viewer import FrameCache, start_viewer
from curobo_moving_rod_v5_rules import FloorPlane

GROUPS = ('A', 'B', 'E')


class LiveBar(LiveGroups):
    preflight_file = base.v41.PREFLIGHT_FILE
    camera_memory = staticmethod(bar.BarMemory)
    worker_type = bar.BarWorker
    floor_mask = staticmethod(base.v41.nonbase_mask)
    plane_factory = staticmethod(FloorPlane)
    group_options = GROUPS
    obstacle = bar
    version = 'v5 horizontal bar'
    drag_help = 'Drag the horizontal bar in X, Y or Z. A ignores it, B uses its true box, E uses depth.'

    @staticmethod
    def make_mpc(config, settings):
        return base.v41.make_mpc(config, settings, floor_activation_m=base.FLOOR_ACTIVATION_M)

    def obstacle_handles(self, scene):
        cylinder = scene.add_cylinder('/truth/bar', radius=self.obstacle.RADIUS, height=self.obstacle.LENGTH,
                                      wxyz=self.obstacle.ROTATION_WXYZ, color=(235, 55, 45), position=self.obstacle.START)
        handle = scene.add_transform_controls('/controls/bar', position=self.obstacle.START, scale=.12,
                                              active_axes=(True, True, True), disable_rotations=True)
        return cylinder, handle

    def read_rod(self):
        position = np.asarray(self.gizmo.position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('bar translation must contain three finite coordinates')
        self.rod.position = tuple(map(float, position))
        return position

    def truth_box(self, center):
        return self.obstacle.fit_at_center(center)

    def truth_clearance(self, spheres, center):
        return float(self.obstacle.clearances(spheres[None], np.asarray(center)[None])[0])


class LivePoke(LiveBar):
    obstacle = poke
    camera_memory = staticmethod(poke.BarMemory)
    worker_type = poke.BarWorker
    version = 'v5 short bar poke'
    drag_help = 'Drag in X, Y or Z, or click Poke to extend and retract once at the current Y/Z.'

    def __init__(self, args, baseline):
        self.poke_lock = threading.Lock()
        self.poke_elapsed = None
        self.poke_samples = []
        super().__init__(args, baseline)
        self.poke_button = self.server.gui.add_button('Poke')
        self.poke_button.on_click(self.request_poke)

    def request_poke(self, _event=None):
        with self.poke_lock:
            if self.poke_elapsed is not None:
                return
            self.poke_yz = tuple(map(float, self.read_rod()[1:]))
            self.poke_samples.clear()
            self.poke_elapsed = 0.
            self.poke_button.disabled = True

    def before_obstacle_step(self):
        with self.poke_lock:
            if self.poke_elapsed is None:
                return
            elapsed, scene = self.poke_elapsed, self.obstacle
            duration = scene.period(scene.SPEED)
            x = scene.at(elapsed, scene.SPEED) if elapsed < duration else scene.START[0]
            self.gizmo.position = (float(x), *self.poke_yz)
            self.poke_samples.append(tuple(map(float, self.gizmo.position)))
            if elapsed >= duration-1e-9:
                self.poke_elapsed = None
                self.poke_button.disabled = False
            else:
                self.poke_elapsed = elapsed+base.v4.CONTROL_DT

    def verify_poke(self):
        self.gizmo.position = (.9, .31, .38)
        self.request_poke()
        deadline = time.monotonic()+35.
        while self.poke_elapsed is not None and time.monotonic() < deadline:
            if self.current and self.current['error']:
                raise RuntimeError(self.current['error'])
            time.sleep(.05)
        points = np.asarray(self.poke_samples)
        if self.poke_elapsed is not None or len(points) < 2:
            raise RuntimeError('interactive poke timed out')
        np.testing.assert_allclose(points[:, 1:], np.tile([.31, .38], (len(points), 1)))
        np.testing.assert_allclose(points[[0, -1], 0], [1., 1.])
        np.testing.assert_allclose(points[:, 0].min(), .60)
        return {'samples': len(points), 'min_center_x_m': float(points[:, 0].min()), 'yz_m': [.31, .38]}


def watch(args, baseline, result_path, *, scene=bar, live_class=None):
    base.v41.v3.check_viewer_port(args.port)
    if args.interactive:
        app = (live_class or LiveBar)(args, baseline)
        try:
            print(f'BAR_INTERACTIVE http://127.0.0.1:{args.port}', flush=True)
            if args.viewer_smoke:
                initial_center = app.read_rod().copy()
                np.testing.assert_allclose(initial_center, scene.START)
                poke_check = app.verify_poke() if isinstance(app, LivePoke) else None
                outcome = smoke(app, drag_position=(.52, .22, .37))
                outcome['initial_center_m'] = initial_center.tolist()
                if poke_check is not None:
                    outcome["poke_verified"] = poke_check
                print('BAR_LIVE_SMOKE ' + json.dumps(outcome), flush=True)
                return 0 if outcome['all_passed'] else 2
            base.wait_for_viewer(args.duration)
        finally:
            if not app.closed:
                app.close()
        return 0

    report = json.loads(result_path.read_text())
    if not report.get('complete'):
        raise RuntimeError('complete the horizontal bar checkpoint first')
    with tempfile.TemporaryDirectory(prefix='bar-replay-') as temp:
        worker = scene.BarWorker(args.newton_python, Path(temp))
        viewer = None
        try:
            frames = FrameCache(worker, report, render=functools.partial(render_frames, obstacle_at=scene.at))
            frames[next(iter(report['cases']))]
            viewer, checks = start_viewer(args.robot_config, report, frames, args.port, groups=GROUPS,
                                          verify=args.viewer_smoke, floor_mask=base.v41.nonbase_mask,
                                          obstacle=scene)
            print(f'BAR_REPLAY http://127.0.0.1:{args.port}', flush=True)
            if args.viewer_smoke and not checks['all_passed']:
                return 2
            base.wait_for_viewer(args.duration)
        finally:
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:
                    thread.join(timeout=10)
                viewer._server.stop()
            worker.close()
    return 0
