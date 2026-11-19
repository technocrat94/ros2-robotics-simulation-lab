"""Viser views for the scripted bar checkpoint and the XYZ drag scene."""
import functools
import json
import tempfile
from pathlib import Path

import numpy as np

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
        cylinder = scene.add_cylinder('/truth/bar', radius=bar.RADIUS, height=bar.LENGTH,
                                      wxyz=bar.ROTATION_WXYZ, color=(235, 55, 45), position=bar.START)
        handle = scene.add_transform_controls('/controls/bar', position=bar.START, scale=.12,
                                              active_axes=(True, True, True), disable_rotations=True)
        return cylinder, handle

    def read_rod(self):
        position = np.asarray(self.gizmo.position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('bar translation must contain three finite coordinates')
        self.rod.position = tuple(map(float, position))
        return position

    def truth_box(self, center):
        return bar.fit_at_center(center)

    @staticmethod
    def truth_clearance(spheres, center):
        return float(bar.clearances(spheres[None], np.asarray(center)[None])[0])


def watch(args, baseline, result_path):
    base.v41.v3.check_viewer_port(args.port)
    if args.interactive:
        app = LiveBar(args, baseline)
        try:
            print(f'BAR_INTERACTIVE http://127.0.0.1:{args.port}', flush=True)
            if args.viewer_smoke:
                outcome = smoke(app, drag_position=(.52, .22, .37))
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
        worker = bar.BarWorker(args.newton_python, Path(temp))
        viewer = None
        try:
            frames = FrameCache(worker, report, render=functools.partial(render_frames, obstacle_at=bar.at))
            frames[next(iter(report['cases']))]
            viewer, checks = start_viewer(args.robot_config, report, frames, args.port, groups=GROUPS,
                                          verify=args.viewer_smoke, floor_mask=base.v41.nonbase_mask,
                                          obstacle=bar)
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
