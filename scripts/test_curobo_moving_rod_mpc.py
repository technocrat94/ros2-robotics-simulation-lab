#!/usr/bin/env python3
"""CPU checks for continuous command integration and replay write protection."""
import argparse
import ast
import contextlib
import io
import os
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

source = Path(__file__).with_name("curobo_moving_rod_mpc.py")
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                          and n.name in {"integrate_command", "command_index", "main"}], type_ignores=[])
scope = dict(np=np, math=__import__("math"), CONTROL_DT=.025, argparse=argparse,
             Path=Path, os=os)
exec(compile(helpers, str(source), "exec"), scope)


class ContinuousMPCContracts(unittest.TestCase):
    def test_previous_command_keeps_moving_during_compute(self):
        integrate = scope["integrate_command"]
        q = np.zeros(6)
        limits = np.array([[-1.]*6, [1.]*6])
        times, positions = integrate(q, np.full(6, .1), .04, limits)
        np.testing.assert_allclose(positions[0], q)
        np.testing.assert_allclose(positions[-1], np.full(6, .004))
        self.assertLessEqual(np.max(np.diff(times)), .025)
        # Applying the next command starts from the computed endpoint, with no jump.
        _, following = integrate(positions[-1], np.full(6, -.2), .03, limits)
        np.testing.assert_allclose(following[0], positions[-1])
        np.testing.assert_allclose(following[-1], np.full(6, -.002))

    def test_current_state_boundary_is_not_reissued_as_zero_velocity(self):
        choose = scope["command_index"]
        self.assertEqual(choose(.020, 4), 1)
        self.assertEqual(choose(.030, 4), 2)
        self.assertEqual(choose(.200, 4), 3)

    def test_limits_and_zero_duration(self):
        integrate = scope["integrate_command"]
        q = np.full(6, .99)
        times, positions = integrate(q, np.ones(6), .1, np.array([[-1.]*6, [1.]*6]))
        self.assertAlmostEqual(times[-1], .1)
        self.assertTrue(np.all(positions <= 1))
        self.assertTrue(np.all(positions >= .99))
        times, positions = integrate(q, np.ones(6), 0, np.array([[-1.]*6, [1.]*6]))
        self.assertEqual(positions.shape, (0, 6))
        self.assertEqual(len(times), 0)

    def test_replay_cannot_overwrite_evidence(self):
        with patch("sys.argv", ["mpc", "--replay", "--save-results"]), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                scope["main"]()
            self.assertEqual(caught.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
