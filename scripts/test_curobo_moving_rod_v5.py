#!/usr/bin/env python3
"""CPU checks for v5 height decisions and the executor's braking envelope."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

source = Path(__file__).with_name('curobo_moving_rod_v5_rules.py')
tree = ast.parse(source.read_text())
classes = [node for node in tree.body if isinstance(node, ast.ClassDef)]


def fitted(points, _capture, _top):
    if not len(points):
        return None, None
    high = float(np.max(points[:, 2]))+.02
    return dict(center_m=[.5, .25, high/2], dims_m=[.05, .05, high]), None


class Cloud:
    def update(self, capture, time):
        return capture.points, None


def capture(z=None, cropped=False):
    points = np.array([[.5, .25, z]]) if z is not None else np.empty((0, 3))
    return SimpleNamespace(points=points, metadata={'border_touches': {'top': cropped}})


env = dict(np=np, copy=copy, CONTROL_DT=.025, MAX_UNSEEN_SPEED_MPS=.20, MEMORY_TTL_S=1.,
           v2=SimpleNamespace(TimedPointCloud=Cloud, sixth=SimpleNamespace(fit_with_unknown_extent=fitted)),
           torch=SimpleNamespace(as_tensor=lambda x, **_: np.asarray(x), float32=np.float32))
exec(compile(ast.Module(body=classes, type_ignores=[]), str(source), 'exec'), env)


def top(box):
    return box['center_m'][2]+box['dims_m'][2]/2


class V5Rules(unittest.TestCase):
    def test_crop_can_raise_only_if_footprint_does_not_overlap(self):
        memory = env['HeightMemory'](.545)
        fit, *_ = memory.observe(capture(.15, cropped=True), 0.)
        box, raised, capped = memory.apply_height(fit, np.array([[0., 0., .2, .05]]))
        self.assertAlmostEqual(top(box), .545)
        self.assertTrue(raised)
        self.assertFalse(capped)
        box, raised, capped = memory.apply_height(fit, np.array([[.5, .25, .18, .03]]))
        self.assertAlmostEqual(top(box), .13)
        self.assertFalse(raised)
        self.assertTrue(capped)

    def test_height_memory_retains_maximum_through_loss_and_drops_stale_box(self):
        memory = env['HeightMemory'](.545)
        memory.observe(capture(.25), 0.)
        memory.observe(capture(.14, cropped=True), .1)
        fit, _, _, event = memory.observe(capture(), .6)
        self.assertTrue(event['growth_rule'])
        self.assertAlmostEqual(fit['dims_m'][0], .25)
        box, raised, _ = memory.apply_height(fit, np.empty((0, 4)))
        self.assertAlmostEqual(top(box), .27)
        self.assertFalse(raised)
        self.assertIsNone(memory.observe(capture(), 1.101)[0])

    def test_box_is_removed_when_cap_would_go_below_bottom(self):
        memory = env['HeightMemory'](.545)
        fit, *_ = memory.observe(capture(.20), 0.)
        box, _, capped = memory.apply_height(fit, np.array([[.5, .25, .03, .03]]))
        self.assertIsNone(box)
        self.assertTrue(capped)

    def test_floor_scaling_checks_brake_and_retains_acceleration_bound(self):
        # One sphere's bottom is a linear function of joint 0.
        env['sphere_series'] = lambda _robot, poses: np.stack(
            (np.zeros(len(poses)), np.zeros(len(poses)), poses[:, 0], np.zeros(len(poses))), axis=1)[:, None, :]
        plane = env['FloorPlane'](None, np.array([True]), np.array([[-1.]*6, [1.]*6]),
                                  np.array([[-10.]*6, [10.]*6]))
        q = np.array([.04, 0., 0., 0., 0., 0.])
        previous = np.array([-.5, 0., 0., 0., 0., 0.])
        wanted = np.array([-.75, 0., 0., 0., 0., 0.])
        self.assertLess(plane.braking_positions(q, wanted)[:, 0].min(), .005)
        result = plane.filter(q, previous, wanted)
        self.assertGreaterEqual(plane.braking_positions(q, result)[:, 0].min(), .005)
        self.assertLessEqual(np.max(np.abs(result-previous)), .25+1e-9)
        self.assertEqual(plane.acts, 1)
        self.assertLess(plane.last_factor, 1.)


if __name__ == '__main__':
    unittest.main()
