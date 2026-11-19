#!/usr/bin/env python3
"""GPU-free contracts for v4 conservative occupancy, floor and denominators."""
import ast,copy,json
from pathlib import Path
from types import SimpleNamespace
from typing import Any
import unittest
import numpy as np

root=Path(__file__).parent
rules=ast.parse((root/'curobo_moving_rod_v4_rules.py').read_text())
class Cloud:
    def update(self,capture,t):return capture.points,0

def old_fit(points,capture,top):
    if len(points)<3:return None,{}
    low=np.min(points,axis=0);high=np.max(points,axis=0)+[.01,.01,.02]
    return dict(center_m=((low+high)/2).tolist(),dims_m=np.maximum(high-low,.01).tolist(),points=len(points)),{}

fake=SimpleNamespace(TimedPointCloud=Cloud,sixth=SimpleNamespace(fit_with_unknown_extent=old_fit),
    ROD_RADIUS_M=.025,ROD_TOP_M=.25,ROD_CENTER_Z_M=.125,ROD_Y_M=.245,
    exact_rod_fit=lambda x:dict(center_m=[x,.245,.125],dims_m=[.05,.05,.25]))
scope=dict(np=np,copy=copy,v2=fake,MAX_UNSEEN_SPEED_MPS=.2,MEMORY_TTL_S=1.,BASE_LINK_NAMES={'world','base_link','base_link_inertia'})
module=ast.Module(body=[n for n in rules.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and getattr(n,'name','')!='sphere_series'],type_ignores=[])
exec(compile(module,'v4rules','exec'),scope)

def capture(border=False,shift=0,visible=True):
    points=np.array([[.4+shift,.22,.02],[.42+shift,.23,.1],[.41+shift,.24,.08]]) if visible else np.empty((0,3))
    return SimpleNamespace(points=points,metadata=dict(border_touches=dict(left=border)),camera_transform=np.array([0,0,99,0,0,0,1]))

class V4Contracts(unittest.TestCase):
    def test_observation_pose_is_exact_when_a_joint_hits_its_limit(self):
        sample=scope['sample_held_pose']
        limits=np.array([[-1.]*6,[1.]*6])
        actual=sample([.99]*6,[1.]*6,.02,limits)
        np.testing.assert_allclose(actual,[1.]*6)
        # Linear interpolation of the clipped endpoints would incorrectly give .998.
        linear=.99+(1.-.99)*(.02/.025)
        self.assertGreater(actual[0],linear)
        np.testing.assert_allclose(sample([0.]*6,[.1]*6,.02,limits),[.002]*6)

    def test_border_top_is_fixed_world_and_counted_once(self):
        memory=scope['ConservativeRodMemory'](.545)
        fit,_,_,event=memory.observe(capture(True),0.)
        self.assertAlmostEqual(fit['center_m'][2]+fit['dims_m'][2]/2,.545)
        self.assertTrue(event['top_rule']);self.assertEqual(memory.top_events,1)
        m=scope['ConservativeRodMemory'](.545)
        plain,_,_,event=m.observe(capture(False),0.)
        self.assertAlmostEqual(plain['center_m'][2]+plain['dims_m'][2]/2,.12)
        self.assertFalse(event['top_rule'])

    def test_lost_view_expands_each_side_and_drops_after_one_second(self):
        m=scope['ConservativeRodMemory'](.545)
        seen,_,_,_=m.observe(capture(True),0.)
        grown,velocity,_,event=m.observe(capture(visible=False),.5)
        np.testing.assert_allclose(np.array(grown['dims_m'])[:2],np.array(seen['dims_m'])[:2]+.2)
        np.testing.assert_allclose(grown['center_m'],seen['center_m'])
        np.testing.assert_allclose(velocity,0.)
        self.assertEqual(event['growth_each_side_xy_m'],[.1,.1]);self.assertTrue(event['growth_rule'])
        self.assertAlmostEqual(grown['center_m'][2]+grown['dims_m'][2]/2,.545)
        self.assertIsNotNone(m.observe(capture(visible=False),1.)[0])
        self.assertIsNone(m.observe(capture(visible=False),1.00001)[0])
        self.assertEqual(m.growth_events,2)

    def test_reappearance_resets_growth_and_fresh_velocity(self):
        m=scope['ConservativeRodMemory'](.545)
        m.observe(capture(False),0.)
        m.observe(capture(visible=False),.5)
        fit,velocity,_,event=m.observe(capture(False,.06),.6)
        self.assertFalse(event['growth_rule']);self.assertEqual(event['since_seen_s'],0.)
        self.assertAlmostEqual(velocity[0],.1)
        self.assertLess(fit['dims_m'][0],.1)

    def test_floor_excludes_only_base_and_detects_other_links(self):
        def tensor(a):return SimpleNamespace(detach=lambda:SimpleNamespace(cpu=lambda:SimpleNamespace(numpy=lambda:np.array(a))))
        robot=SimpleNamespace(kinematics_config=SimpleNamespace(link_sphere_idx_map=tensor([0,1]),link_name_to_idx_map={'base_link_inertia':0,'shoulder_link':1}))
        mask=scope['nonbase_mask'](robot)
        np.testing.assert_equal(mask,[False,True])
        spheres=np.array([[[0,0,-1,.1],[0,0,.2,.05]]])
        self.assertAlmostEqual(scope['floor_distances'](spheres,mask,0)[0],.15)
        spheres[0,1,2]=.02
        self.assertLess(scope['floor_distances'](spheres,mask,0)[0],0.)

    def test_coverage_catches_missing_and_unseen_top(self):
        covers=scope['box_covers_rod']
        self.assertFalse(covers(None,.5))
        self.assertTrue(covers(fake.exact_rod_fit(.5),.5))
        self.assertFalse(covers(dict(center_m=[.5,.245,.05],dims_m=[.05,.05,.1]),.5))

    def test_prediction_graphics_use_recorded_velocity_and_toggle_only_display(self):
        visual_tree=ast.parse((root/'curobo_moving_rod_v4_visuals.py').read_text())
        mod=ast.Module(body=[n for n in visual_tree.body if isinstance(n,ast.ClassDef)],type_ignores=[])
        env=dict(np=np)
        exec(compile(mod,'visuals','exec'),env)
        class Handle:
            def __init__(self,**kwargs):self.__dict__.update(kwargs)
            def on_update(self,callback):self.callback=callback
        class Scene:
            def add_box(self,*a,**kw):return Handle(**kw)
            def add_cylinder(self,*a,**kw):return Handle(**kw)
            def add_arrows(self,*a,**kw):return Handle(**kw)
        class Gui:
            def add_markdown(self,*a):pass
            def add_checkbox(self,*a,**kw):return Handle(value=kw['initial_value'])
            def add_text(self,*a,**kw):return Handle(value='')
        graphics=env['PredictionVisuals'](SimpleNamespace(scene=Scene(),gui=Gui()),0.)
        box=Handle(visible=True)
        fit=dict(center_m=[.5,.245,.125],dims_m=[.05,.05,.25])
        graphics.update(fit,[-.1,0,0],[.1,0,0],.4+1/30,True,box)
        np.testing.assert_allclose(graphics.ghost.position,[.456666666667,.245,.125])
        np.testing.assert_allclose(graphics.arrow.points[0,1],[.456666666667,.245,.125],atol=1e-7)
        self.assertTrue(box.visible);self.assertTrue(graphics.arrow.visible)
        graphics.visible.value=False;graphics.refresh()
        self.assertFalse(box.visible);self.assertFalse(graphics.ghost.visible)
        graphics.visible.value=True;graphics.update(fit,[-.1,0,0],[.1,0,0],.4+1/30,False,box)
        self.assertFalse(box.visible);self.assertFalse(graphics.arrow.visible)
        self.assertIn('0.100 / 0.100',graphics.speed.value)

    def test_saved_evidence(self):
        path=root.parent/'docs/experiments/sim-camera-curobo/results/moving_rod_v4.json'
        if not path.exists():self.skipTest('GPU evidence absent')
        r=json.loads(path.read_text())
        if not r['complete']:self.skipTest('formal batch still running')
        self.assertEqual(len(r['cases']),75)
        bounds=np.array(r['loaded_config']['position_limits_rad'])
        for c in r['cases'].values():
            s=c['summary'];family=s['mode'].split('-')[0]
            self.assertEqual(s['steps'],800);self.assertAlmostEqual(s['simulated_duration_s'],20.)
            self.assertTrue(s['within_joint_velocity_limits']);self.assertTrue(s['within_joint_acceleration_limits'])
            q=np.array([p['q'] for p in c['trace']]);self.assertEqual(q.shape,(801,6))
            commands=np.array([p['new_velocity_radps'] for p in c['cycles']])
            previous=np.vstack((np.zeros((1,6)),commands[:-1]))
            np.testing.assert_allclose(q[1:],np.clip(q[:-1]+previous*.025,bounds[0],bounds[1]),atol=2e-8,rtol=0)
            self.assertLessEqual(np.max(np.abs(commands)),np.pi+2e-7)
            self.assertLessEqual(np.max(np.abs(commands-previous))/.025,10+1e-6)
            observation_times=s['observation_times_s']
            self.assertEqual(len(observation_times),0 if family=='A' else 600)
            if observation_times:np.testing.assert_allclose(observation_times,np.arange(600)/30,atol=1e-8,rtol=0)
            for cy in c['cycles']:
                self.assertAlmostEqual(cy['compute_counted_s'],.025)
                self.assertEqual(cy['command_index_after_latency'],3)
                self.assertAlmostEqual(cy['prediction_horizon_s'],.4+1/30 if s['prediction'] else 0.)
            self.assertEqual(s['floor_contacts'],sum(e['kind']=='floor' for e in c['collision_events']))
            self.assertEqual(s['rod_collisions'],sum(e['kind']=='rod' for e in c['collision_events']))
            if family=='E':
                events=c['perception_events'];self.assertEqual(len(events),600)
                self.assertEqual(s['top_rule_events'],sum(e['top_rule'] for e in events))
                self.assertEqual(s['growth_rule_events'],sum(e['growth_rule'] for e in events))
                for e in events:
                    if e['growth_rule']:
                        self.assertLessEqual(e['since_seen_s'],1+1e-9)
                        np.testing.assert_allclose(e['growth_each_side_xy_m'],[.2*e['since_seen_s']]*2,atol=1e-10)
        for p in r['per_phase']:
            a=next(c['summary'] for c in r['cases'].values() if c['summary']['mode']=='A' and c['summary']['rod_speed_mps']==p['speed_mps'] and c['summary']['initial_x_m']==p['initial_x_m'])
            self.assertEqual(p['status']=='VALID_SCENE',a['rod_collisions']>0)

    def test_invalid_a_excludes_every_group_and_counters(self):
        tree=ast.parse((root/'curobo_moving_rod_v4.py').read_text())
        fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='summarize')
        groups=('A','B','B-nopred','E','E-nopred')
        env=dict(np=np,SPEEDS=(.05,),PHASES=(.36,.44),GROUPS=groups,CONTROL_DT=.025)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'summary','exec'),env)
        r=dict(cases={})
        for phase in (.36,.44):
            for group in groups:
                valid=phase==.44
                s=dict(mode=group,rod_speed_mps=.05,initial_x_m=phase,true_collisions=int(group=='A'),rod_collisions=int(valid and group=='A'),survives=group.startswith('B'),minimum_true_clearance_m=.003 if valid else -999.,floor_contacts=0 if valid else 999,top_rule_events=2 if valid else 999,growth_rule_events=3 if valid else 999,rod_onsets_no_box=0 if valid else 999,rod_onsets_uncovered_box=0 if valid else 999,joint_position_clamped_intervals=0,within_joint_velocity_limits=True,within_joint_acceleration_limits=True)
                r['cases'][f'{phase}/{group}']=dict(summary=s,trace=[dict(q=[0.]*6),dict(q=[0.]*6)])
        env['summarize'](r)
        self.assertEqual(len(r['cases']),10)
        for row in r['rates']:
            self.assertEqual(row['valid_runs'],1);self.assertEqual(row['floor_contacts'],0)
            self.assertEqual(row['top_rule_events'],2);self.assertEqual(row['growth_rule_events'],3)
            self.assertEqual(row['worst_clearance_m'],.003)

if __name__=='__main__':unittest.main()
