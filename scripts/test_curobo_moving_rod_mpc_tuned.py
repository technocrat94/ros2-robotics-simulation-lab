#!/usr/bin/env python3
"""CPU contracts for no-rod qualification and perception-error scoring."""
import argparse
import ast
import contextlib
import io
import os
from pathlib import Path
from typing import Any
import unittest
from unittest.mock import patch

import numpy as np

source = Path(__file__).with_name("curobo_moving_rod_mpc_tuned.py")
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                          and n.name in {"box_error", "worst_frames", "qualifies_reference", "main"}], type_ignores=[])
scope = dict(np=np, Any=Any, argparse=argparse, os=os, Path=Path)
exec(compile(helpers, str(source), "exec"), scope)


class TunedMPCContracts(unittest.TestCase):
    def test_reach_and_velocity_are_both_required(self):
        qualify = scope["qualifies_reference"]
        self.assertFalse(qualify(dict(time_to_first_goal_s=None, within_joint_velocity_limits=True), 1.75))
        self.assertFalse(qualify(dict(time_to_first_goal_s=1.5, within_joint_velocity_limits=False), 1.75))
        self.assertFalse(qualify(dict(time_to_first_goal_s=1.8, within_joint_velocity_limits=True), 1.75))
        self.assertTrue(qualify(dict(time_to_first_goal_s=1.75, within_joint_velocity_limits=True), 1.75))

    def test_partial_view_and_offset_reduce_box_coverage(self):
        truth = dict(center_m=[.5,.245,.125], dims_m=[.05,.05,.25])
        fit = dict(center_m=[.54,.245,.05], dims_m=[.05,.05,.1])
        error = scope["box_error"](fit, truth)
        self.assertAlmostEqual(error["missing_height_m"], .15)
        self.assertAlmostEqual(error["missing_top_m"], .15)
        np.testing.assert_allclose(error["missing_width_xy_m"], [.04,0], atol=1e-7)
        np.testing.assert_allclose(error["center_offset_xyz_m"], [.04,0,-.075], atol=1e-7)
        full = scope["box_error"](truth, truth)
        self.assertEqual(full["missing_height_m"], 0)
        self.assertEqual(full["center_offset_norm_m"], 0)

    def test_missing_box_and_worst_frame_tracking(self):
        truth = dict(center_m=[.5,.245,.125], dims_m=[.05,.05,.25])
        missing = dict(step=1, **scope["box_error"](None,truth))
        full = dict(step=2, **scope["box_error"](truth,truth))
        worst = scope["worst_frames"]([full,missing])
        self.assertEqual(worst["missing_box_frames"], 1)
        self.assertEqual(worst["height"]["step"], 1)
        self.assertIsNone(missing["center_offset_norm_m"])

    def test_replay_is_read_only(self):
        for conflict in ("--save-results", "--reference"):
            with patch("sys.argv", ["tuned", "--replay", conflict]), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    scope["main"]()
                self.assertEqual(caught.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
