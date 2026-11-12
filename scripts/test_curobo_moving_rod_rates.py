#!/usr/bin/env python3
"""CPU checks for phase exclusion and descriptive rate denominators."""
import ast
from pathlib import Path
from typing import Any
import unittest

import numpy as np

source = Path(__file__).with_name("curobo_moving_rod_rates.py")
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                          and n.name == "summarize"], type_ignores=[])
scope = dict(np=np, Any=Any, SPEEDS=(.05,.10), PHASES=(.36,.44,.52,.60,.68))
exec(compile(helpers,str(source),"exec"),scope)


class RatesContracts(unittest.TestCase):
    def test_invalid_phases_are_retained_but_excluded(self):
        cases = {}
        for speed in scope["SPEEDS"]:
            for phase in scope["PHASES"]:
                valid = speed == .05 and phase == .36
                for mode in ("A","B","E"):
                    summary = dict(mode=mode, rod_speed_mps=speed, initial_x_m=phase,
                                   true_collisions=int(valid and mode=="A"),
                                   reaches=4 if mode=="B" else 0,
                                   minimum_true_clearance_m=.01 if valid else -10.,
                                   max_joint_velocity_radps=[.2 if valid else 999.]*6,
                                   max_sampled_joint_acceleration_radps2=[2. if valid else 999.]*6)
                    cases[f"{speed}/{phase}/{mode}"] = dict(summary=summary,camera_frame_errors=[])
        r=dict(cases=cases)
        scope["summarize"](r)
        b=next(x for x in r["rates"] if x["mode"]=="B" and x["speed_mps"]==.05)
        self.assertEqual((b["successes"],b["valid_phases"]),(1,1))
        self.assertEqual(b["success_rate"],1.)
        self.assertEqual(b["mean_reaches"],4.)
        self.assertEqual(b["worst_clearance_m"],.01)
        self.assertEqual(b["max_joint_speed_radps"],.2)
        self.assertEqual(len(r["per_phase"]),10)
        self.assertEqual(sum(x["status"]=="INVALID_SCENE" for x in r["per_phase"]),9)
        # Collision-free stalling is still a failure.
        e=next(x for x in r["rates"] if x["mode"]=="E" and x["speed_mps"]==.05)
        self.assertEqual(e["success_rate"],0.)
        # An empty denominator is undefined, never 0% or 100%.
        for row in r["rates"]:
            if row["speed_mps"]==.10:
                self.assertIsNone(row["success_rate"])
                self.assertIsNone(row["worst_clearance_m"])


if __name__ == "__main__":
    unittest.main()
