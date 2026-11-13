#!/usr/bin/env python3
"""CPU contracts for latency separation and invalid-scene denominators."""
import argparse
import ast
import contextlib
import io
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

source = Path(__file__).with_name("curobo_moving_rod_latency.py")
tree = ast.parse(source.read_text())
helpers = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                          and n.name in ("summarize", "main")], type_ignores=[])
groups = ("A20","B20","B50","E20","E50")
scope = dict(np=np, GROUPS=groups, rates=SimpleNamespace(SPEEDS=(.05,.10), PHASES=(.36,.60)),
             argparse=argparse, Path=Path, os=os)
exec(compile(helpers,str(source),"exec"),scope)


class LatencyContracts(unittest.TestCase):
    def test_validity_metrics_and_repeat_comparison(self):
        results = dict(cases={}, repeats={})
        for speed in (.05,.10):
            for phase in (.36,.60):
                valid = phase == .60
                for group in groups:
                    summary = dict(mode=group, rod_speed_mps=speed, initial_x_m=phase,
                        true_collisions=int(valid and group=="A20"),
                        survives=group=="B20", minimum_true_clearance_m=.01 if valid else -999.)
                    results["cases"][f"{speed}/{phase}/{group}"] = dict(summary=summary,
                        cycles=[dict(measured_compute_ms=x) for x in ([10.,30.] if valid else [999.])])
        for g in ("B20","E20"):
            results["repeats"][g] = dict(summary=dict(survives=True, minimum_true_clearance_m=.02))
        scope["summarize"](results)
        self.assertEqual(len(results["per_phase"]),4)
        for row in results["rates"]:
            self.assertEqual(row["valid_phases"],1)
            self.assertEqual(row["worst_clearance_m"],.01)
            self.assertEqual(row["real_compute_median_ms"],20.)
            self.assertEqual(row["real_compute_p95_ms"],29.)
            self.assertEqual(row["successes"],int(row["group"]=="B20"))
        self.assertAlmostEqual(results["repeatability"][0]["clearance_delta_m"],.01)
        self.assertFalse(results["repeatability"][1]["first_success"])
        self.assertTrue(results["repeatability"][1]["repeat_success"])
        # Empty validity denominators retain rows and have undefined clearance/timing.
        for c in results["cases"].values():
            c["summary"]["true_collisions"] = 0
        scope["summarize"](results)
        self.assertTrue(all(r["valid_phases"]==0 and r["worst_clearance_m"] is None
                            and r["real_compute_median_ms"] is None for r in results["rates"]))

    def test_saved_clock_and_validity(self):
        path = source.parents[1]/"docs/experiments/sim-camera-curobo/results/moving_rod_latency.json"
        if not path.exists():
            self.skipTest("GPU evidence not present")
        results = json.loads(path.read_text())
        self.assertTrue(results["complete"])
        self.assertEqual(len(results["cases"]),50)
        self.assertEqual(len(results["repeats"]),2)
        for c in [*results["cases"].values(), *results["repeats"].values()]:
            summary = c["summary"]
            fixed = summary["counted_latency_s"]
            self.assertAlmostEqual(summary["simulated_duration_s"],20.)
            self.assertEqual(summary["steps"],800 if fixed==.02 else 400)
            self.assertEqual(summary["joint_position_clamped_intervals"],0)
            self.assertEqual(len(c["camera_frame_errors"]),summary["steps"] if summary["mode"][0]=="E" else 0)
            for step in c["cycles"]:
                self.assertAlmostEqual(step["compute_counted_s"],fixed)
                self.assertAlmostEqual(step["prediction_horizon_s"],fixed+.025)
                self.assertEqual(step["command_index_after_latency"],3)
                self.assertAlmostEqual(step["sim_time_end_s"]-step["sim_time_start_s"],max(.025,fixed))
                np.testing.assert_allclose(step["q_after_compute_rad"],
                    np.array(step["q_at_capture_rad"])+np.array(step["previous_velocity_radps"])*fixed,
                    atol=1e-6,rtol=0)
        for row in results["per_phase"]:
            self.assertEqual(row["status"]=="VALID_SCENE",row["cases"]["A20"]["true_collisions"]>0)

    def test_replay_cannot_write(self):
        with patch("sys.argv",["latency","--replay","--save-results"]), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                scope["main"]()
            self.assertEqual(caught.exception.code,2)

    def test_clock_contract_expressions(self):
        # Evaluate the actual loop expressions for short/long measured times.
        rt = ast.parse(source.with_name("curobo_moving_rod_rates.py").read_text())
        fn = next(n for n in rt.body if isinstance(n,ast.FunctionDef) and n.name=="run_case")
        assignment = next(n for n in ast.walk(fn) if isinstance(n,ast.Assign)
                          and any(isinstance(t,ast.Name) and t.id=="counted_compute" for t in n.targets))
        expression = compile(ast.Expression(assignment.value),"clock","eval")
        for measured in (.007,.080):
            self.assertEqual(eval(expression,dict(compute_s=measured,counted_latency_s=None)), measured)
            for fixed in (.020,.050):
                self.assertEqual(eval(expression,dict(compute_s=measured,counted_latency_s=fixed)),fixed)
        calls = [n for n in ast.walk(fn) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="advance"]
        for fixed, expected_period in ((.020,.025),(.050,.050)):
            env=dict(min=min,max=max,counted_compute=fixed,duration=20.,clock=0.,CONTROL_DT=.025)
            durations = [eval(compile(ast.Expression(c.args[0]),"advance","eval"),env) for c in calls]
            self.assertAlmostEqual(sum(durations), expected_period)
        index = next(n.value for n in ast.walk(fn) if isinstance(n,ast.Assign)
                     and any(isinstance(t,ast.Name) and t.id=="index" for t in n.targets))
        self.assertNotIn("compute_s", ast.unparse(index))
        # Both the controller index and prediction horizon use the fixed value.
        self.assertIn("counted_compute",ast.unparse(index))
        self.assertIn("expected_compute = counted_compute",ast.unparse(fn))


if __name__ == "__main__":
    unittest.main()
