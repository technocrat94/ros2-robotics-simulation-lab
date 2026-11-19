"""Geometry and image-border regressions for the horizontal bar."""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


class MemoryStub:
    def __init__(self, fixed_top):
        self.fixed_top = fixed_top

    def observe(self, capture, _time):
        box = dict(center_m=[.6, .2, .35], dims_m=[.2, .05, .06])
        return box, np.zeros(3), capture.points, {'fresh_seen': True}


path = Path(__file__).with_name('curobo_moving_bar_v5_scene.py')
tree = ast.parse(path.read_text())
nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.Assign))
         or isinstance(n, ast.ClassDef) and n.name == 'BarMemory']
scope = dict(np=np, math=math, copy=copy, HeightMemory=MemoryStub)
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), scope)


class BarContracts(unittest.TestCase):
    def test_scripted_lane_reflects_on_y_only(self):
        at, center = scope['at'], scope['center']
        np.testing.assert_allclose([center(at(t, .1)) for t in (0, 2.5, 5, 7.5, 10)], [
            [.5, .05, .35], [.5, .30, .35], [.5, .55, .35], [.5, .30, .35], [.5, .05, .35],
        ])

    def test_truth_clearance_uses_length_on_x_and_radius_on_yz(self):
        spheres = np.array([[[.79, .2, .35, .005]], [[.84, .2, .35, .005]],
                            [[.5, .23, .35, .005]]])
        np.testing.assert_allclose(scope['distances'](spheres, [.2]*3), [-.015, .035, .005])

    def test_fresh_border_extends_about_seen_center_without_vertical_fallback(self):
        memory = scope['BarMemory'](.545)
        capture = SimpleNamespace(points=np.array([[.63, .2, .35], [.75, .2, .36]]),
                                  metadata={'border_touches': {'top': True}})
        box, _, _, event = memory.observe(capture, 0.)
        self.assertEqual(memory.fixed_top, 0.)
        self.assertTrue(event['bar_border_length_extended'])
        self.assertAlmostEqual(box['center_m'][0], .69)
        self.assertEqual(box['dims_m'], [.60, .05, .06])
        self.assertEqual(memory.last_box, box)
        capture.metadata['border_touches']['top'] = False
        box, _, _, event = memory.observe(capture, .1)
        self.assertFalse(event['bar_border_length_extended'])
        self.assertAlmostEqual(box['dims_m'][0], .2)
        self.assertEqual(memory.border_length_events, 1)


if __name__ == '__main__':
    unittest.main()
