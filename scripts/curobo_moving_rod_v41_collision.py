"""v4.1 per-obstacle floor filtering; rod collision geometry stays complete."""
import torch
from curobo._src.geom.collision.collision_scene import SceneCollision,SceneCollisionCfg
from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from curobo._src.geom.types import SceneCfg
from curobo._src.robot.kinematics.kinematics_state import KinematicsState


class FloorMaskedScene(SceneCollision):
    """Two differentiable native queries: full rod spheres + filtered floor spheres.

    Passed through MPC's supported scene_collision_checker constructor argument.
    Separate buffers prevent either query overwriting the other's gradient.
    No library files, robot geometry, or live obstacle-enable flags are patched.
    """
    def __init__(self,rod_scene,floor_scene,active_mask,*,floor_activation_m=None):
        super().__init__(data=rod_scene.data,checker=rod_scene.checker,
                         device_cfg=rod_scene.device_cfg,scene_model=rod_scene.scene_model)
        self.floor_scene=floor_scene
        self.active_mask=torch.as_tensor(active_mask,device=self.device_cfg.device,dtype=torch.bool).reshape(1,1,-1,1)
        self._floor_buffers={}
        self.floor_activation = (None if floor_activation_m is None
                                 else self.device_cfg.to_device([floor_activation_m]))

    @classmethod
    def create(cls,rod_cfg,active_mask,*,floor_activation_m=None):
        rod=SceneCollision.from_config(rod_cfg)
        floor=SceneCollision.from_config(SceneCollisionCfg(device_cfg=rod_cfg.device_cfg,
            scene_model=SceneCfg.create({'cuboid':{'floor':{'dims':[10.,10.,2.],
                'pose':[0.,0.,-1.,1.,0.,0.,0.]}}})))
        return cls(rod,floor,active_mask,floor_activation_m=floor_activation_m)

    def _floor_inputs(self,state,buffer):
        spheres=state.robot_spheres
        masked=torch.cat((spheres[...,:3],torch.where(self.active_mask,spheres[...,3:4],-torch.ones_like(spheres[...,3:4]))),dim=-1)
        key=id(buffer)
        if key not in self._floor_buffers:
            self._floor_buffers[key]=(buffer,CollisionBuffer.from_shape(spheres.shape,self.device_cfg))
        return KinematicsState(robot_spheres=masked),self._floor_buffers[key][1]

    def _combined(self,method,state,buffer,*args,**kwargs):
        rod=getattr(super(),method)(state,buffer,*args,**kwargs)
        filtered,floor_buffer=self._floor_inputs(state,buffer)
        if self.floor_activation is not None:
            # A zero-margin feasibility query still checks physical contact.
            if 'activation_distance' in kwargs:
                kwargs['activation_distance'] = torch.minimum(kwargs['activation_distance'], self.floor_activation)
            else:
                args = (args[0], torch.minimum(args[1], self.floor_activation), *args[2:])
        floor=getattr(self.floor_scene,method)(filtered,floor_buffer,*args,**kwargs)
        return rod+floor

    def get_sphere_distance(self,state,collision_buffer,*args,**kwargs):
        return self._combined('get_sphere_distance',state,collision_buffer,*args,**kwargs)

    def get_sphere_collision(self,state,collision_buffer,*args,**kwargs):
        return self._combined('get_sphere_collision',state,collision_buffer,*args,**kwargs)

    def get_swept_sphere_distance(self,state,collision_buffer,*args,**kwargs):
        return self._combined('get_swept_sphere_distance',state,collision_buffer,*args,**kwargs)

    def get_swept_sphere_collision(self,state,collision_buffer,*args,**kwargs):
        return self._combined('get_swept_sphere_collision',state,collision_buffer,*args,**kwargs)

    def get_sphere_distance_raw(self,spheres,collision_buffer,*args,**kwargs):
        return self.get_sphere_distance(KinematicsState(robot_spheres=spheres),collision_buffer,*args,**kwargs)

    def get_swept_sphere_distance_raw(self,spheres,collision_buffer,*args,**kwargs):
        return self.get_swept_sphere_distance(KinematicsState(robot_spheres=spheres),collision_buffer,*args,**kwargs)
