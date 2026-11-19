#!/usr/bin/env python3
"""Static collision/gradient/CUDA-graph contract; no robot motion.

Use the existing cuRobo environment. --save-results writes only v4.1 evidence.
"""
import sys,json
from pathlib import Path
import torch,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from curobo._src.geom.collision.collision_scene import SceneCollision,SceneCollisionCfg
from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from curobo._src.geom.types import SceneCfg
from curobo._src.types.device_cfg import DeviceCfg
from curobo._src.robot.kinematics.kinematics_state import KinematicsState
from curobo_moving_rod_v41_collision import FloorMaskedScene
from curobo.types import Pose
def main():
    D=DeviceCfg();cfg=SceneCollisionCfg(device_cfg=D,scene_model=SceneCfg.create({'cuboid':{'camera_obstacle':{'dims':[.1,.1,.1],'pose':[.6,.245,.5,1.,0.,0.,0.]}}}))
    scene=FloorMaskedScene.create(cfg,[False,False,True]);scene.enable_obstacle('camera_obstacle',False)
    weight=torch.ones(1,device='cuda');activation=torch.full((1,),.03,device='cuda')

    def query(which,slot,z,rod=False):
        spheres=torch.tensor([[[[.6,.245,.5,-1.],[.6,.245,.5,-1.],[.6,.245,.5,-1.]]]],device='cuda')
        spheres[0,0,slot]=torch.tensor([.66 if rod else .6,.245,z,.02],device='cuda')
        spheres.requires_grad_()
        buffer=CollisionBuffer.from_shape(spheres.shape,D)
        value=which.get_sphere_distance(KinematicsState(robot_spheres=spheres),buffer,weight,activation,return_loss=True).sum()
        value.backward()
        return dict(cost=float(value.item()),gradient=spheres.grad[0,0,slot,:3].detach().cpu().tolist())

    base=query(scene,0,-.2);shoulder=query(scene,1,-.2);arm=query(scene,2,-.2)
    assert base['cost']==0 and shoulder['cost']==0
    assert np.allclose(base['gradient'],0) and np.allclose(shoulder['gradient'],0)
    assert arm['cost']>0 and arm['gradient'][2]<0
    scene.enable_obstacle('camera_obstacle',True)
    rod=query(scene,0,.5,True)
    assert rod['cost']>0 and abs(rod['gradient'][0])>1e-6
    scene.enable_obstacle('camera_obstacle',False)
    thin=SceneCollision.from_config(SceneCollisionCfg(device_cfg=D,scene_model=SceneCfg.create({'cuboid':{'floor':{'dims':[10.,10.,.1],'pose':[0.,0.,-.05,1.,0.,0.,0.]}}})))
    old=query(thin,2,-.08);thick=query(scene,2,-.08)
    assert old['gradient'][2]>0 and thick['gradient'][2]<0
    # Capture/replay the exact differentiable two-query path, before any MPC motion.
    x=torch.tensor([[[[.6,.245,-.2,.02],[.6,.245,-.2,.02],[.6,.245,-.2,.02]]]],device='cuda',requires_grad=True)
    b=CollisionBuffer.from_shape(x.shape,D)
    stream=torch.cuda.Stream();stream.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(stream):
     for _ in range(3):
      x.grad=None
      scene.get_sphere_distance(KinematicsState(robot_spheres=x),b,weight,activation,return_loss=True).sum().backward()
    torch.cuda.current_stream().wait_stream(stream)
    x.grad=None;graph=torch.cuda.CUDAGraph()
    with torch.cuda.graph(graph):
     loss=scene.get_sphere_distance(KinematicsState(robot_spheres=x),b,weight,activation,return_loss=True).sum()
     loss.backward()
    graph.replay();torch.cuda.synchronize()
    assert torch.all(x.grad[0,0,:2]==0) and x.grad[0,0,2,2]<0
    r=dict(all_passed=True,base_floor=base,shoulder_floor=shoulder,moving_link_floor=arm,
           excluded_base_still_collides_with_rod=rod,thin_slab_below_midpoint=old,two_metre_floor=thick,
           cuda_graph_forward_backward=True,robot_motion_runs=0)
    print('V41_STATIC_COST '+json.dumps(r),flush=True)
    if '--save-results' in sys.argv:
        (Path(__file__).resolve().parent.parent/'docs/experiments/sim-camera-curobo/results/moving_rod_v41.cost_check.json').write_text(json.dumps(r,indent=2)+'\n')


if __name__=="__main__":main()
