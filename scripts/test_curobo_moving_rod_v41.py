#!/usr/bin/env python3
"""CPU contracts for the v4.1 stop gate and floor-only sphere exclusion."""
import ast,hashlib,json
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

root=Path(__file__).parent
scope=dict(np=np,FLOOR_EXCLUDED_LINKS={'world','base','base_link','base_link_inertia','shoulder_link'})
tree=ast.parse((root/'curobo_moving_rod_v41.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('nonbase_mask','require_preflight')],type_ignores=[]),'v41','exec'),scope)

class FakeScene:
    def __init__(self,data,checker=None,device_cfg=None,scene_model=None):
        self.data,self.checker,self.device_cfg,self.scene_model=data,checker,device_cfg,scene_model
    def get_sphere_distance(self,state,buffer,*a,**kw):return np.maximum(state.robot_spheres[...,3],0)
    get_sphere_collision=get_sphere_distance
    get_swept_sphere_distance=get_sphere_distance
    get_swept_sphere_collision=get_sphere_distance

fake_torch=SimpleNamespace(as_tensor=lambda a,**kw:np.asarray(a,dtype=bool),bool=bool,
    cat=lambda a,dim:np.concatenate(a,axis=dim),where=np.where,ones_like=np.ones_like)
cscope=dict(SceneCollision=FakeScene,torch=fake_torch,KinematicsState=lambda **kw:SimpleNamespace(**kw),
    CollisionBuffer=SimpleNamespace(from_shape=lambda shape,device:SimpleNamespace(shape=shape)))
ctree=ast.parse((root/'curobo_moving_rod_v41_collision.py').read_text())
exec(compile(ast.Module(body=[n for n in ctree.body if isinstance(n,ast.ClassDef)],type_ignores=[]),'floorquery','exec'),cscope)

class V41Contracts(unittest.TestCase):
    def test_exclude_base_shoulder_not_upper_arm(self):
        data=SimpleNamespace(detach=lambda:SimpleNamespace(cpu=lambda:SimpleNamespace(numpy=lambda:np.array([0,1,2,2]))))
        robot=SimpleNamespace(kinematics_config=SimpleNamespace(link_sphere_idx_map=data,
            link_name_to_idx_map={'base_link_inertia':0,'shoulder_link':1,'upper_arm_link':2}))
        np.testing.assert_equal(scope['nonbase_mask'](robot),[False,False,True,True])

    def test_gate_rejects_failure_subthreshold_and_nonfinite(self):
        gate=scope['require_preflight']
        for result in (dict(passed=False),dict(passed=True),
            dict(passed=True,endpoint_floor_clearance_m=dict(start=.03,raised_start=.03,goal=.02999)),
            dict(passed=True,endpoint_floor_clearance_m=dict(start=.03,raised_start=.03,goal=float('inf')))):
            with self.assertRaisesRegex(RuntimeError,'STOP'):gate(result)
        gate(dict(passed=True,endpoint_floor_clearance_m=dict(start=.03,raised_start=.04,goal=.05)))

    def test_floor_mask_does_not_change_rod_geometry_and_buffers(self):
        cfg=SimpleNamespace(device='cpu')
        scene=cscope['FloorMaskedScene'](FakeScene('rod',device_cfg=cfg),FakeScene('floor',device_cfg=cfg),[False,False,True])
        spheres=np.array([[[[0,0,0,.1],[0,0,0,.2],[0,0,0,.3]]]])
        state=SimpleNamespace(robot_spheres=spheres.copy());buffer=object()
        np.testing.assert_allclose(scene.get_sphere_distance(state,buffer),[[[.1,.2,.6]]])
        np.testing.assert_array_equal(state.robot_spheres,spheres)
        floor,floor_buffer=scene._floor_inputs(state,buffer)
        np.testing.assert_allclose(floor.robot_spheres[...,3],[[[-1.,-1.,.3]]])
        self.assertIsNot(floor_buffer,buffer)
        self.assertIs(floor_buffer,scene._floor_inputs(state,buffer)[1])
        self.assertIsNot(floor_buffer,scene._floor_inputs(state,object())[1])
        np.testing.assert_allclose(scene.get_swept_sphere_distance(state,buffer),[[[.1,.2,.6]]])

    def test_saved_preflight_stops_motion(self):
        p=root.parent/'docs/experiments/sim-camera-curobo/results/moving_rod_v41.preflight.json'
        if not p.exists():self.skipTest('preflight evidence absent')
        r=json.loads(p.read_text())
        self.assertFalse(r['passed']);self.assertTrue(r['goal_unchanged']);self.assertEqual(r['motion_runs'],0)
        self.assertEqual(r['excluded_links_with_spheres'],['base_link_inertia','shoulder_link'])
        self.assertEqual((r['excluded_spheres'],r['scored_spheres']),(11,101))
        self.assertEqual(r['floor_dims_m'],[10.,10.,2.]);self.assertEqual(r['floor_center_m'],[0.,0.,-1.])
        for name,clearance in r['endpoint_floor_clearance_m'].items():
            self.assertLess(clearance,.03);self.assertGreater(clearance,0.)
            self.assertEqual(r['controlling_spheres'][name]['link'],'upper_arm_link')
        with self.assertRaises(RuntimeError):scope['require_preflight'](r)

    def test_real_factory_is_blocked_before_robot_or_mpc_setup(self):
        path=root.parent/'docs/experiments/sim-camera-curobo/results/moving_rod_v41.preflight.json'
        if not path.exists():self.skipTest('preflight evidence absent')
        env=dict(scope,PREFLIGHT_FILE=path,json=json)
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='make_mpc')
        exec(compile(ast.Module(body=[function],type_ignores=[]),'factory','exec'),env)
        with self.assertRaisesRegex(RuntimeError,'STOP'):
            env['make_mpc'](Path('/not-opened-before-gate'),{})

    def test_saved_rescore_counts_match_runs_and_sources(self):
        p=root.parent/'docs/experiments/sim-camera-curobo/results/moving_rod_v41.old_floor_rescore.json'
        if not p.exists():self.skipTest('rescore evidence absent')
        r=json.loads(p.read_text());self.assertFalse(r['rerun']);self.assertEqual(len(r['runs']),180)
        for name,sha in r['source_sha256'].items():
            self.assertEqual(hashlib.sha256((p.parent/name).read_bytes()).hexdigest(),sha)
        for row in r['summary']:
            runs=[x for x in r['runs'] if x['dataset']==row['dataset'] and x['group']==row['group']]
            self.assertEqual(row['runs'],len(runs))
            self.assertEqual(row['floor_contact_runs'],sum(x['floor_contact'] for x in runs))
            self.assertEqual(row['deepest_depth_m'],max(x['deepest_depth_m'] for x in runs))

    def test_saved_gpu_static_check(self):
        p=root.parent/'docs/experiments/sim-camera-curobo/results/moving_rod_v41.cost_check.json'
        if not p.exists():self.skipTest('GPU static evidence absent')
        r=json.loads(p.read_text());self.assertTrue(r['all_passed']);self.assertEqual(r['robot_motion_runs'],0)
        self.assertEqual(r['base_floor']['cost'],0.);self.assertEqual(r['shoulder_floor']['cost'],0.)
        self.assertGreater(r['excluded_base_still_collides_with_rod']['cost'],0.)
        self.assertGreater(r['thin_slab_below_midpoint']['gradient'][2],0.)
        self.assertLess(r['two_metre_floor']['gradient'][2],0.)
        self.assertTrue(r['cuda_graph_forward_backward'])

if __name__=='__main__':unittest.main()
