"""v4-only floor geometry and conservative camera memory; v3 stays unchanged."""
from __future__ import annotations
import copy
import numpy as np
import torch
import curobo_moving_rod_v3 as v3

v2=v3.v2
MAX_UNSEEN_SPEED_MPS=.20  # Task assumption, applied in +/- X and +/- Y.
MEMORY_TTL_S=1.
BASE_LINK_NAMES={"world","base","base_link","base_link_inertia"}


def sample_held_pose(q,velocity,elapsed,limits):
    """Exact pose of the existing clipped held-velocity plant, including a joint stop."""
    return np.clip(np.asarray(q)+np.asarray(velocity)*max(0.,elapsed),limits[0],limits[1])


def nonbase_mask(robot):
    cfg=robot.kinematics_config
    indices=cfg.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
    excluded={int(i) for name,i in cfg.link_name_to_idx_map.items() if name in BASE_LINK_NAMES}
    mask=~np.isin(indices,list(excluded))
    if not mask.any() or mask.all():raise RuntimeError("cannot identify base spheres for floor exclusion")
    return mask


def sphere_series(robot,positions):
    state=v2.JointState.from_position(positions.reshape(-1,6),joint_names=list(v2.sixth.preflight.ARM_JOINT_NAMES))
    return robot.compute_kinematics(state).robot_spheres.detach().cpu().numpy().reshape(len(positions),-1,4)


def cylinder_distances(spheres,xs):
    radial=np.linalg.norm(spheres[:,:,:2]-np.stack((xs,np.full(len(xs),v2.ROD_Y_M)),axis=-1)[:,None,:],axis=-1)-v2.ROD_RADIUS_M
    axial=np.abs(spheres[:,:,2]-v2.ROD_CENTER_Z_M)-v2.ROD_TOP_M/2
    pair=np.stack((radial,axial),axis=-1)
    return np.min(np.linalg.norm(np.maximum(pair,0),axis=-1)+np.minimum(np.max(pair,axis=-1),0)-spheres[:,:,3],axis=1)


def floor_distances(spheres,mask,top):
    active=spheres[:,mask]
    return np.min(active[:,:,2]-active[:,:,3]-top,axis=1)


def force_unknown_top(fit,top):
    result=copy.deepcopy(fit)
    low=np.asarray(result['center_m'])-np.asarray(result['dims_m'])/2
    high=np.asarray(result['center_m'])+np.asarray(result['dims_m'])/2
    high[2]=max(top,high[2])
    result['center_m']=((low+high)/2).tolist();result['dims_m']=(high-low).tolist()
    result['fitted_top_z_m']=float(high[2])
    return result


class ConservativeRodMemory:
    def __init__(self,fixed_top):
        self.fixed_top=fixed_top
        self.cloud=v2.TimedPointCloud()
        self.last_seen=None
        self.last_box=None
        self.previous_center=None
        self.top_events=0
        self.growth_events=0
        self.last_event={}

    def observe(self,capture,t):
        points,_=self.cloud.update(capture,t)
        fresh,_=v2.sixth.fit_with_unknown_extent(capture.points,capture,self.fixed_top)
        border=bool(fresh is not None and any(capture.metadata.get('border_touches',{}).values()))
        estimate=np.zeros(3)
        event=dict(time_s=float(t),fresh_seen=fresh is not None,top_rule=border,growth_rule=False,
                   since_seen_s=None,growth_each_side_xy_m=[0.,0.],grown_dims_xy_m=None)
        if fresh is not None:
            fit,_=v2.sixth.fit_with_unknown_extent(points,capture,self.fixed_top)
            if fit is None:raise RuntimeError('fresh fit missing from accumulated cloud')
            if border:
                fit=force_unknown_top(fit,self.fixed_top);self.top_events+=1
            center=np.asarray(fresh['center_m'])
            if self.previous_center is not None:
                estimate=(center-self.previous_center[1])/(t-self.previous_center[0])
            self.previous_center=(t,center.copy())
            self.last_seen=t;self.last_box=copy.deepcopy(fit)
            event['since_seen_s']=0.
        elif self.last_seen is not None and t-self.last_seen<=MEMORY_TTL_S+1e-9:
            age=max(0.,t-self.last_seen)
            fit=copy.deepcopy(self.last_box)
            dims=np.asarray(fit['dims_m']);dims[:2]+=2*MAX_UNSEEN_SPEED_MPS*age
            fit['dims_m']=dims.tolist()
            event.update(growth_rule=age>0,since_seen_s=age,
                         growth_each_side_xy_m=[MAX_UNSEEN_SPEED_MPS*age]*2,grown_dims_xy_m=dims[:2].tolist())
            self.growth_events+=int(age>0)
        else:
            fit=None
            if self.last_seen is not None:event['since_seen_s']=t-self.last_seen
        self.last_event=event
        return fit,estimate,points,event


def box_covers_rod(fit,x):
    if fit is None:return False
    truth=v2.exact_rod_fit(x)
    c,d=np.asarray(fit['center_m']),np.asarray(fit['dims_m'])
    tc,td=np.asarray(truth['center_m']),np.asarray(truth['dims_m'])
    return bool(np.all(c-d/2<=tc-td/2+1e-6) and np.all(c+d/2>=tc+td/2-1e-6))
