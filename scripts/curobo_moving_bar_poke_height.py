#!/usr/bin/env python3
"""Measure depth visibility and run the lowered short-bar checkpoint."""
from functools import partial
from pathlib import Path
import tempfile

import numpy as np

import curobo_moving_bar_poke_v5_scene as poke
from curobo_moving_bar_v5_scene import covers_fit
from curobo_moving_bar_v5 import main, saved_case, tuned, v2
from curobo_moving_bar_v5_viewer import LivePoke

HEIGHTS = (.20, .15, .10)
CHECKPOINT_HEIGHT = .20
SHORT_BAR_MIN_POINTS = 8
SCAN = v2.RESULTS_DIR/'moving_bar_poke_height.scan.json'
RESULT = v2.RESULTS_DIR/'moving_bar_poke_height.checkpoint.json'


class PokeHeight:
    LENGTH = poke.LENGTH
    RADIUS = poke.RADIUS
    ROTATION_WXYZ = poke.ROTATION_WXYZ
    MOTION_AXIS = poke.MOTION_AXIS
    MOTION = poke.MOTION
    SPEED = poke.SPEED
    at = staticmethod(poke.at)
    period = staticmethod(poke.period)
    fit_at_center = staticmethod(poke.fit_at_center)
    clearances = staticmethod(poke.clearances)

    def __init__(self, height, memory_type):
        self.START = (*poke.START[:2], height)
        self.BarMemory = memory_type

    def center(self, x):
        return float(x), *self.START[1:]

    def fit(self, x):
        return self.fit_at_center(self.center(x))

    def distances(self, spheres, xs):
        return self.clearances(spheres, np.asarray([self.center(x) for x in xs]))

    def covers(self, box, x):
        return covers_fit(box, self.fit(x))

    def BarWorker(self, python, temp_dir):
        worker = poke.BarWorker(python, temp_dir)
        worker.center = self.center
        return worker


class MeasuredBarMemory(poke.BarMemory):
    def observe(self, capture, time):
        box, velocity, points, event = super().observe(capture, time)
        # Shape IDs are used only to measure renderer visibility.
        event.update(bar_pixels=capture.metadata['rod_pixels'],
                     fresh_voxels=len(capture.points))
        return box, velocity, points, event


class LowerPokeMemory(MeasuredBarMemory):
    # A partially visible short bar leaves very few distinct 8 mm voxels.
    fit_points = staticmethod(partial(v2.sixth.fit_with_unknown_extent,
                                      min_points=SHORT_BAR_MIN_POINTS))


def height_row(height, a, e):
    frames = [{key: event[key] for key in ('time_s', 'bar_pixels', 'fresh_voxels', 'fresh_seen')}
              for event in e['perception_events']]
    contact = next((event['time_s'] for event in a['collision_events'] if event['kind'] == 'rod'), None)
    fit = next((frame['time_s'] for frame in frames if frame['fresh_seen']), None)
    lead = contact-fit if contact is not None and fit is not None else None
    return dict(height_m=height, a_bar_contacts=a['summary']['rod_collisions'],
                a_first_contact_s=contact, e_depth_frames=len(frames),
                e_frames_with_bar_pixels=sum(f['bar_pixels'] > 0 for f in frames),
                e_frames_with_fit=sum(f['fresh_seen'] for f in frames),
                e_first_fit_s=fit, fit_lead_s=lead, frames=frames)


def scan_heights(args, baseline, preflight):
    print('HEIGHT_SCAN expected wall time about 2–4 minutes (six cases)', flush=True)
    settings = preflight['frozen_config']
    report = dict(complete=False, heights=[], selected_height_m=None,
                  min_fit_points=v2.sixth.MIN_POINTS)
    for height in HEIGHTS:
        scene = PokeHeight(height, MeasuredBarMemory)
        a = saved_case(args.robot_config, settings, baseline, 'A', scene=scene)['case']
        if a['summary']['stop_reason']:
            raise RuntimeError(a['summary']['stop_reason'])
        with tempfile.TemporaryDirectory(prefix='bar-height-') as temp:
            worker = scene.BarWorker(args.newton_python, Path(temp))
            try:
                e = saved_case(args.robot_config, settings, baseline, 'E', worker, scene=scene)['case']
            finally:
                worker.close()
        if e['summary']['stop_reason']:
            raise RuntimeError(e['summary']['stop_reason'])
        row = height_row(height, a, e)
        report['heights'].append(row)
        tuned.save_json(SCAN, report)
        print('HEIGHT_SCAN', {k: v for k, v in row.items() if k != 'frames'}, flush=True)
    report['selected_height_m'] = max((row['height_m'] for row in report['heights']
                                      if row['fit_lead_s'] is not None and row['fit_lead_s'] > 0),
                                     default=None)
    report['complete'] = True
    tuned.save_json(SCAN, report)
    print('HEIGHT_SELECTED', report['selected_height_m'], flush=True)
    return 0


scene = PokeHeight(CHECKPOINT_HEIGHT, LowerPokeMemory)


class LiveLowerPoke(LivePoke):
    obstacle = scene
    camera_memory = staticmethod(scene.BarMemory)
    worker_type = staticmethod(scene.BarWorker)
    version = 'v5 short bar poke at lower Z'


if __name__ == '__main__':
    raise SystemExit(main(scene=scene, result=RESULT, live_class=LiveLowerPoke, scan_heights=scan_heights))
