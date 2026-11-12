#!/usr/bin/env python3
"""CPU lifecycle and XY-protocol regressions without GPU or Viser startup."""
import ast
import argparse
import contextlib
import io
import os
from unittest.mock import patch
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from typing import Any
import unittest

import numpy as np

source=Path(__file__).with_name("curobo_moving_rod_live.py")
tree=ast.parse(source.read_text())
helpers=ast.Module(body=[n for n in tree.body if (isinstance(n,ast.FunctionDef)
    and n.name in {"floor_center","cylinder_clearance"}) or (isinstance(n,ast.ClassDef) and n.name=="LiveGroups")],type_ignores=[])
scope=dict(np=np,Any=Any,threading=threading,time=time,
           v2=SimpleNamespace(ROD_TOP_M=.25,ROD_RADIUS_M=.025))
exec(compile(helpers,str(source),"exec"),scope)


class LiveContracts(unittest.TestCase):
    def test_live_mode_cannot_write_or_regenerate_reports(self):
        tuned_source=source.with_name("curobo_moving_rod_mpc_tuned.py")
        main=next(n for n in ast.parse(tuned_source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=="main")
        runtime=dict(argparse=argparse,Path=Path,os=os)
        exec(compile(ast.Module(body=[main],type_ignores=[]),str(tuned_source),"exec"),runtime)
        for conflict in ("--save-results","--reference","--replay","--headless"):
            with patch("sys.argv",["live","--interactive",conflict]),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:runtime["main"]()
                self.assertEqual(caught.exception.code,2)

    def test_floor_and_cylinder_geometry(self):
        center=scope["floor_center"]([.5,.2,99.])
        np.testing.assert_allclose(center,[.5,.2,.125])
        spheres=np.array([[.6,.2,.125,.01]])
        self.assertAlmostEqual(scope["cylinder_clearance"](spheres,center),.065)
        with self.assertRaises(ValueError):scope["floor_center"]([float('nan'),0,0])

    def test_default_worker_y_is_preserved_and_live_y_is_not_sticky(self):
        worker_source=source.with_name("newton_camera_moving_rod_worker.py")
        tree=ast.parse(worker_source.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=="MovingRodCamera")
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=="_assign_pose")
        globals_=dict(np=np,ROD_Y_M=.245,ROD_CENTER_Z_M=.125,newton=SimpleNamespace(eval_fk=lambda *a:None))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(worker_source),"exec"),globals_)
        class Array:
            def __init__(self,a):self.a=np.array(a,dtype=np.float32)
            def assign(self,a):self.a=np.array(a,dtype=np.float32)
            def numpy(self):return self.a.copy()
        model=SimpleNamespace(joint_q=Array(np.zeros(8)),joint_qd=None,bvh_refit_shapes=lambda state:None)
        fake=SimpleNamespace(q_template=np.arange(8,dtype=np.float32),model=model,
                             state=SimpleNamespace(body_q=Array(np.zeros((1,7)))),rod_body=0)
        assign=globals_["_assign_pose"]
        assign(fake,np.zeros(6),.5)
        np.testing.assert_allclose(fake.state.body_q.a[0,:3],[.5,.245,.125])
        assign(fake,np.zeros(6),.4,.22)
        np.testing.assert_allclose(fake.state.body_q.a[0,:3],[.4,.22,.125])
        assign(fake,np.zeros(6),.6)
        np.testing.assert_allclose(fake.state.body_q.a[0,:3],[.6,.245,.125])
        self.assertEqual(fake.model.joint_q.a[6],6.)

    def fake_app(self, close_ok=True):
        app=scope["LiveGroups"].__new__(scope["LiveGroups"])
        app.switch_lock=threading.Lock();app.state_lock=threading.Lock()
        app.thread=None;app.stop_event=threading.Event();app.current=None
        app.history=[];app.switches=[];app.closed=False
        app.active_loops=0;app.max_active_loops=0
        for key in ("group_text","state_text","error_text","collisions_text","depth","points","observed","predicted"):
            setattr(app,key,SimpleNamespace(value="",visible=True))
        app.server=SimpleNamespace(stop=lambda:None)
        def loop(mode,stop,record):
            with app.state_lock:
                app.active_loops+=1;app.max_active_loops=max(app.max_active_loops,app.active_loops)
            record["state"]="running"
            record["worker_pid"]=42 if mode=="E" else None
            stop.wait()
            time.sleep(.01)  # Cleanup must finish before a successor starts.
            record["worker_exitcode"]=0 if mode=="E" else None
            record["worker_stopped"]=close_ok
            record["state"]="stopped";record["stopped_at"]=time.monotonic()
            with app.state_lock:app.active_loops-=1
        app._run=loop
        return app

    def wait_running(self,app):
        deadline=time.monotonic()+1
        while app.current["state"]!="running":
            if time.monotonic()>deadline:self.fail("fake loop did not start")
            time.sleep(.001)

    def test_group_switch_waits_for_loop_and_worker_cleanup(self):
        app=self.fake_app()
        try:
            for mode in ("A","E","B"):
                app.switch(mode);self.wait_running(app)
            self.assertEqual(app.max_active_loops,1)
            self.assertEqual(len(app.switches),2)
            self.assertEqual(app.switches[-1]["old_worker_exitcode"],0)
            self.assertTrue(all(x["old_thread_stopped"] and x["old_worker_stopped"] and
                                x["old_stopped_at"]<=x["new_requested_at"] for x in app.switches))
        finally:app.close()
        self.assertFalse(app.thread.is_alive())

    def test_failed_worker_cleanup_refuses_second_loop(self):
        app=self.fake_app(close_ok=False)
        app.switch("E");self.wait_running(app)
        try:
            with self.assertRaises(RuntimeError):app.switch("B")
            self.assertEqual(len(app.history),1)
            self.assertFalse(app.thread.is_alive())
        finally:
            app.current["worker_stopped"]=True
            app.close()


if __name__=="__main__":unittest.main()
