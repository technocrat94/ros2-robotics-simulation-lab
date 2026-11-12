#!/usr/bin/env python3
"""CPU contract checks for swept prediction and the reflected rod clock."""
import ast
import copy
import argparse
import contextlib
import io
import json
import tempfile
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path
import unittest

import numpy as np


# Load pure helpers without importing GPU-only cuRobo initialization.
source = Path(__file__).with_name("curobo_camera_moving_rod.py")
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[node for node in tree.body if isinstance(node, ast.FunctionDef)
                          and node.name in {"predicted_fit", "rod_x_at", "main"}], type_ignores=[])
scope = dict(np=np, copy=copy, ROD_X_MIN_M=0.30, ROD_X_MAX_M=0.70)
exec(compile(helpers, str(source), "exec"), scope)


class MovingRodContracts(unittest.TestCase):
    def test_prediction_encloses_current_and_future_box(self):
        fit = {"center_m": [0.4, 0.2, 0.15], "dims_m": [0.05, 0.05, 0.3]}
        original = copy.deepcopy(fit)
        for velocity in ([0.1, 0, 0], [-0.2, 0, 0], [0, 0, 0], [0.1, -0.2, 0.03]):
            box = scope["predicted_fit"](fit, velocity, 0.4)
            lo = np.array(box["center_m"]) - np.array(box["dims_m"])/2
            hi = np.array(box["center_m"]) + np.array(box["dims_m"])/2
            for fraction in (0, 0.5, 1):
                center = np.array(fit["center_m"]) + np.array(velocity)*0.4*2*fraction
                self.assertTrue(np.all(center-np.array(fit["dims_m"])/2 >= lo-1e-12))
                self.assertTrue(np.all(center+np.array(fit["dims_m"])/2 <= hi+1e-12))
        self.assertEqual(fit, original)
        self.assertIsNone(scope["predicted_fit"](None, [0, 0, 0], 1))

    def test_viewer_writes_only_with_explicit_save_flag(self):
        for save in (False, True):
            worker = Mock(log=[])
            planner = Mock()
            viewer = Mock()
            writer = Mock()
            cfg = SimpleNamespace(from_robot_yaml_file=Mock())
            result = {"cases": {}, "raised_start": {"joints_rad": [0.] * 6},
                      "fastest_speed_e_survives_mps": None}
            runtime = dict(scope, argparse=argparse, Path=Path, tempfile=tempfile,
                           json=json, time=SimpleNamespace(monotonic=Mock(side_effect=[0., 2.])),
                           Kinematics=Mock(), KinematicsCfg=cfg,
                           sixth=SimpleNamespace(preflight=SimpleNamespace(TOOL_FRAME="tool0")),
                           MovingRodWorker=Mock(return_value=worker),
                           build_experiment=Mock(return_value=(result, {}, planner)),
                           start_viewer=Mock(return_value=(viewer, lambda: {"all_passed": True})),
                           save_results=writer)
            exec(compile(helpers, str(source), "exec"), runtime)
            arguments = ["moving-rod", "--duration", "1"] + (["--save-results"] if save else [])
            with patch("sys.argv", arguments), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(runtime["main"](), 0)
            self.assertEqual(writer.call_count, int(save))
            worker.close.assert_called_once()
            planner.destroy.assert_called_once()
            viewer._server.stop.assert_called_once()

    def test_clock_reflects_at_both_limits(self):
        x = scope["rod_x_at"]
        for t, expected in ((0, 0.7), (2, 0.5), (4, 0.3), (6, 0.5), (8, 0.7)):
            self.assertAlmostEqual(x(t, 0.1, 0.7, -1), expected)
        for t in np.linspace(0, 100, 1001):
            self.assertGreaterEqual(x(t, 0.2, 0.3, 1), 0.3-1e-12)
            self.assertLessEqual(x(t, 0.2, 0.3, 1), 0.7+1e-12)


if __name__ == "__main__":
    unittest.main()
