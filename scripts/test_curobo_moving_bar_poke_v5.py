"""Check the poke timing and the short-bar geometry without GPU dependencies."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

path = Path(__file__).with_name('curobo_moving_bar_poke_v5_scene.py')
tree = ast.parse(path.read_text())
scope = {'np': np, 'sweep': SimpleNamespace(
    fit_at_center=lambda position, *, length: dict(center_m=position, dims_m=[length, .04, .04]),
    BarMemory=object, BarWorker=object,
)}
exec(compile(ast.Module(body=[node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))],
                        type_ignores=[]), str(path), 'exec'), scope)


class PokeContracts(unittest.TestCase):
    def test_tip_moves_holds_retracts_waits_and_repeats(self):
        length = scope['LENGTH']
        times = [0., 2., 4., 4.25, 4.5, 6.5, 8.5, 9., 9.5, 11.5]
        tips = [scope['at'](t, .10)-length/2 for t in times]
        np.testing.assert_allclose(tips, [.85, .65, .45, .45, .45, .65, .85, .85, .85, .65])
        self.assertAlmostEqual(scope['period'](.10), 9.5)

    def test_center_and_geometry_are_shared_with_perception_and_renderer(self):
        np.testing.assert_allclose(scope['center'](scope['at'](0., .10)), [1., .28, .35])
        self.assertEqual(scope['fit'](.6)['dims_m'], [.30, .04, .04])
        self.assertEqual(scope['BarWorker'].length, .30)
        self.assertEqual(scope['BarMemory'].length, .30)


if __name__ == '__main__':
    unittest.main()
