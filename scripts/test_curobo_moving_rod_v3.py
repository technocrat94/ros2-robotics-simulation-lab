#!/usr/bin/env python3
"""GPU-free tests for frozen v3 plant, prediction and repeat denominators."""
import argparse
import ast
import contextlib
import copy
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch, MagicMock
import numpy as np

source=Path(__file__).with_name("curobo_moving_rod_v3.py")
tree=ast.parse(source.read_text())
functions={"limited_command","stretch_fit","horizon_seconds","summarize","main","check_viewer_port"}
module=ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in functions],type_ignores=[])
scope=dict(np=np,copy=copy,CONTROL_DT=.025,OBS_HZ=30,argparse=argparse,os=os,Path=Path,
           rates=SimpleNamespace(SPEEDS=(.05,.10),PHASES=(.36,.60)))
exec(compile(module,str(source),"exec"),scope)


class V3Rules(unittest.TestCase):
    def test_acceleration_and_velocity_clamp(self):
        command=scope["limited_command"]
        speed=np.array([[-1.]*6,[1.]*6]);acc=np.array([[-3.]*6,[3.]*6])
        velocity=np.zeros(6)
        for sign in [1]*50+[-1]*100+[1]*100:
            new=command(np.full(6,sign*99.),velocity,speed,acc)
            self.assertLessEqual(np.max(np.abs(new)),1.+1e-10)
            self.assertLessEqual(np.max(np.abs(new-velocity))/.025,3.+1e-10)
            velocity=new

    def test_prediction_is_full_horizon_single_sweep(self):
        horizon=scope["horizon_seconds"](SimpleNamespace(config=SimpleNamespace(
            optimization_dt=.02,metrics_rollout_config=SimpleNamespace(
                transition_model_cfg=SimpleNamespace(horizon=81,interpolation_steps=4)))))
        self.assertEqual(horizon,.4)
        fit=dict(center_m=[.5,.245,.125],dims_m=[.05,.05,.25])
        result=scope["stretch_fit"](fit,[-.1,0,0],horizon+1/30)
        np.testing.assert_allclose(result["dims_m"],[.093333333333,.05,.25])
        self.assertEqual(fit["center_m"],[.5,.245,.125])
        self.assertIsNone(scope["stretch_fit"](None,[1,0,0],1.))

    def test_a_any_repeat_validates_all_repeats(self):
        results=dict(cases={})
        for speed in (.05,.10):
            for phase in (.36,.60):
                for repeat in range(1,6):
                    for mode in ("A","B","E"):
                        valid=phase==.60
                        s=dict(mode=mode,rod_speed_mps=speed,initial_x_m=phase,repeat=repeat,
                            true_collisions=int(valid and mode=="A" and repeat==5),
                            survives=mode=="B",minimum_true_clearance_m=.01 if valid else -999.,
                            max_joint_velocity_radps=[.5 if valid else 999.]*6,
                            max_sampled_joint_acceleration_radps2=[2. if valid else 999.]*6,
                            within_joint_velocity_limits=True,within_joint_acceleration_limits=True,
                            joint_position_clamped_intervals=0)
                        results["cases"][f"{speed}/{phase}/{repeat}/{mode}"]=dict(summary=s,
                            cycles=[dict(measured_compute_ms=20. if valid else 999.)])
        scope["summarize"](results)
        for r in results["rates"]:
            self.assertEqual(r["total_runs"],5)
            self.assertEqual(r["successes"],5 if r["mode"]=="B" else 0)
            self.assertEqual(r["collision_runs"],1 if r["mode"]=="A" else 0)
            self.assertEqual(r["real_compute_median_ms"],20.)
            self.assertEqual(r["max_joint_speed_radps"],.5)
            self.assertEqual(r["median_worst_clearance_m"],.01)
        self.assertEqual(len(results["cases"]),60)

    def test_exact_observation_times_and_historical_pose(self):
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="run_case")
        block=next(n for n in ast.walk(fn) if isinstance(n,ast.While))
        code=compile(ast.Module(body=[block],type_ignores=[]),str(source),"exec")
        captured=[]
        def capture(q,x):
            captured.append((q.copy(),x))
            return SimpleNamespace(camera=SimpleNamespace(points=np.array([[x,0.,0.]])))
        fake=SimpleNamespace(ROD_START_X_M=.36,
            rod_x_at=lambda t,speed,*unused:t*speed,
            exact_rod_fit=lambda x:dict(center_m=[x,0.,0.],dims_m=[.05,.05,.25]),
            sixth=SimpleNamespace(fit_with_unknown_extent=lambda points,*unused:(dict(center_m=points[0].tolist(),dims_m=[.05,.05,.25]),None)))
        for mode in ("B","E"):
            captured.clear()
            env=dict(np=np,no_rod=False,mode=mode,next_observation=0,OBS_HZ=30,observation_times=[],
                observation_updated=False,v2=fake,speed=.1,previous_frame=None,estimated_velocity=np.zeros(3),fit=None,
                worker=SimpleNamespace(capture=capture),cloud=SimpleNamespace(update=lambda c,t:(c.points,None)),
                fixed_top=.545,frame_errors=[],box_error=lambda *args:{})
            held=0
            for i in range(800):
                t=i*.025
                env.update(clock=t,step=i+1,traces=[dict(time_s=max(0.,t-j*.005),q=[max(0.,t-j*.005)]*6) for j in range(11,-1,-1)])
                old=env["next_observation"]
                exec(code,env)
                held+=env["next_observation"]==old
            self.assertEqual(env["next_observation"],600)
            self.assertEqual(held,200)
            np.testing.assert_allclose(env["observation_times"],np.arange(600)/30,atol=1e-12)
            self.assertAlmostEqual(env["estimated_velocity"][0],.1)
            if mode=="E":
                self.assertEqual(len(captured),600)
                for i,(q,x) in enumerate(captured):
                    np.testing.assert_allclose(q,[i/30]*6,atol=1e-12)
                    self.assertAlmostEqual(x,.1*i/30)

    def test_saved_dataset_rules(self):
        path=source.parents[1]/"docs/experiments/sim-camera-curobo/results/moving_rod_v3.json"
        if not path.exists():self.skipTest("GPU evidence not present")
        results=json.loads(path.read_text())
        if not results["complete"]:self.skipTest("measurement still in progress")
        self.assertEqual(len(results["cases"]),150)
        for case in results["cases"].values():
            s=case["summary"]
            self.assertEqual(s["steps"],800)
            self.assertEqual(len(case["trace"]),801)
            self.assertTrue(s["within_joint_velocity_limits"])
            self.assertTrue(s["within_joint_acceleration_limits"])
            self.assertEqual(s["joint_position_clamped_intervals"],0)
            observations=s["observation_times_s"]
            self.assertEqual(len(observations),0 if s["mode"]=="A" else 600)
            if observations:np.testing.assert_allclose(observations,np.arange(600)/30,atol=1e-8,rtol=0)
            previous=np.zeros(6)
            for i,step in enumerate(case["cycles"]):
                self.assertAlmostEqual(step["compute_counted_s"],.025)
                self.assertAlmostEqual(step["prediction_horizon_s"],.4+1/30)
                self.assertEqual(step["command_index_after_latency"],3)
                command=np.array(step["new_velocity_radps"])
                self.assertLessEqual(np.max(np.abs(command)),np.pi+2e-7)
                self.assertLessEqual(np.max(np.abs(command-previous))/.025,10.+1e-6)
                np.testing.assert_allclose(case["trace"][i+1]["q"],np.array(case["trace"][i]["q"])+previous*.025,atol=2e-8,rtol=0)
                previous=command
        for row in results["per_phase"]:
            a=[c["summary"] for c in results["cases"].values() if c["summary"]["mode"]=="A" and c["summary"]["rod_speed_mps"]==row["speed_mps"] and c["summary"]["initial_x_m"]==row["initial_x_m"]]
            self.assertEqual(len(a),5)
            self.assertEqual(row["status"]=="VALID_SCENE",any(c["true_collisions"]>0 for c in a))

    def test_occupied_viewer_port_fails_early(self):
        probe=MagicMock()
        probe.__enter__.return_value.bind.side_effect=OSError("address in use")
        scope["socket"]=SimpleNamespace(AF_INET=2,SOCK_STREAM=1,socket=lambda *args:probe)
        with self.assertRaisesRegex(RuntimeError,"viewer port 30001 unavailable"):
            scope["check_viewer_port"](30001)
        probe.__exit__.assert_called_once()

    def test_view_modes_do_not_write(self):
        for mode in ("--replay","--interactive"):
            with patch("sys.argv",["v3",mode,"--save-results"]),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:scope["main"]()
                self.assertEqual(caught.exception.code,2)


if __name__=="__main__":unittest.main()
