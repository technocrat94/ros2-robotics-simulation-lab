#!/usr/bin/env python3
"""v4.1 floor fix and mandatory preflight gate, without altering old evidence."""
from __future__ import annotations
import argparse,hashlib,json,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
import curobo_moving_rod_v4 as v4
from curobo_moving_rod_v41_collision import FloorMaskedScene

v2,v3,tuned=v4.v2,v4.v3,v4.tuned
ENDPOINT_GATE_M = .020
QUALIFICATION_DURATION_S = 10.0
QUALIFICATION_FILE = v2.RESULTS_DIR / 'moving_rod_v41.qualification.json'
FLOOR_EXCLUDED_LINKS={'world','base','base_link','base_link_inertia','shoulder_link'}
PREFLIGHT_FILE=v2.RESULTS_DIR/'moving_rod_v41.preflight.json'
RESCORE_FILE=v2.RESULTS_DIR/'moving_rod_v41.old_floor_rescore.json'


def nonbase_mask(robot):
    cfg=robot.kinematics_config
    ids=cfg.link_sphere_idx_map.detach().cpu().numpy().reshape(-1)
    excluded={int(i) for name,i in cfg.link_name_to_idx_map.items() if name in FLOOR_EXCLUDED_LINKS}
    mask=~np.isin(ids,list(excluded))
    if not mask.any():raise RuntimeError('no floor-scored spheres')
    return mask


def require_preflight(report):
    clearance=report.get('endpoint_floor_clearance_m',{})
    valid = (
        report.get('passed')
        and report.get('endpoint_gate_m') == ENDPOINT_GATE_M
        and set(clearance) == {'start', 'raised_start', 'goal'}
        and all(np.isfinite(value) and value >= ENDPOINT_GATE_M for value in clearance.values())
    )
    if not valid:
        raise RuntimeError(f'STOP: scored endpoint floor clearance is below {ENDPOINT_GATE_M:.3f} m')


def make_mpc(config,settings,*,floor_activation_m=None):
    """Fixed v3 MPC plus two-metre floor, filtered only for floor cost."""
    report=json.loads(PREFLIGHT_FILE.read_text());require_preflight(report)
    if report['robot_config_sha256']!=hashlib.sha256(config.read_bytes()).hexdigest() or settings!=report['frozen_config'] or report['reference_sha256']!=hashlib.sha256(v3.REFERENCE_FILE.read_bytes()).hexdigest():
        raise RuntimeError('preflight robot/config differs; requalify before any motion')
    robot_cfg,wrist=v2.sixth.limited_robot_cfg(config,v2.sixth.CAMERA_FRONT_SEED)
    cfg=tuned.ModelPredictiveControlCfg.create(robot=robot_cfg,
        scene_model={'cuboid':{'camera_obstacle':{'dims':[.01,.01,.01],'pose':[0.,0.,-10.,1.,0.,0.,0.]}}},
        optimization_dt=settings.get('optimization_dt',tuned.OPTIMIZATION_DT),interpolation_steps=4,use_cuda_graph=True,
        optimizer_collision_activation_distance=.03,warm_start_optimization_num_iters=200,cold_start_optimization_num_iters=300,
        squared_l2_regularization_weight=[.01,settings['acceleration_regularization'],10.,0.,0.],
        non_terminal_tool_pose_weight_factor=settings['non_terminal_tool_pose_weight_factor'])
    active=nonbase_mask(SimpleNamespace(kinematics_config=robot_cfg.kinematics.kinematics_config))
    scene=FloorMaskedScene.create(cfg.core_cfg.scene_collision_cfg,active,floor_activation_m=floor_activation_m)
    mpc=tuned.ModelPredictiveControl(cfg,scene_collision_checker=scene)
    mpc._v4_floor_top=0.
    return mpc,dict(floor_dims_m=[10.,10.,2.],floor_center_m=[0.,0.,-1.],floor_top_m=0.,
        floor_excluded_links=report['excluded_links_with_spheres'],wrist_3_limit=wrist,
        floor_filter='separate differentiable floor query; original full sphere set retained for rod')


def preflight(config,save):
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text());tuned.original.configure_scene(baseline)
    ref=json.loads(v3.REFERENCE_FILE.read_text())
    if ref['robot_config_sha256']!=hashlib.sha256(config.read_bytes()).hexdigest():raise RuntimeError('robot config changed')
    robot=v2.Kinematics(v2.KinematicsCfg.from_robot_yaml_file(str(config),tool_frames=[v2.sixth.preflight.TOOL_FRAME]))
    states={'start':np.array(v2.sixth.CAMERA_FRONT_SEED),
            'raised_start':np.array(baseline['raised_start']['joints_rad']),
            'goal':np.array(ref['attempts'][-1]['case']['trace'][-1]['q'])}
    names=list(states);samples=list(states.values())
    for joint in range(6):
        for sign in (-1,1):
            q=states['start'].copy();q[joint]+=sign*.2;samples.append(q)
    spheres=v4.sphere_series(robot,torch.as_tensor(np.array(samples),device='cuda',dtype=torch.float32))
    cfg=robot.kinematics_config;ids=cfg.link_sphere_idx_map.detach().cpu().numpy()
    links=[]
    for link,idx in cfg.link_name_to_idx_map.items():
        selected=ids==idx
        if selected.any():
            clear=np.min(spheres[:3,selected,2]-spheres[:3,selected,3],axis=1)
            links.append(dict(link=link,spheres=int(selected.sum()),endpoint_floor_clearance_m=dict(zip(names,map(float,clear))),
                sampled_z_variation_m=float(np.max(np.ptp(spheres[:,selected,2],axis=0))),excluded=link in FLOOR_EXCLUDED_LINKS))
    active=nonbase_mask(robot)
    clearance=v4.floor_distances(spheres[:3],active,0.)
    controlling=[]
    indices=np.flatnonzero(active)
    for row in spheres[:3]:
        i=indices[int(np.argmin(row[active,2]-row[active,3]))]
        link=next(name for name,idx in cfg.link_name_to_idx_map.items() if idx==ids[i])
        controlling.append(dict(link=link,sphere_index=int(i),sphere_world_m=row[i].tolist()))
    result=dict(schema='4.1',date='2026-11-19',complete=True,passed=bool(np.isfinite(clearance).all() and np.all(clearance>=ENDPOINT_GATE_M)),
        endpoint_gate_m=ENDPOINT_GATE_M,floor_top_m=0.,floor_dims_m=[10.,10.,2.],floor_center_m=[0.,0.,-1.],activation_distance_m=.03,
        excluded_links_with_spheres=[x['link'] for x in links if x['excluded']],
        excluded_spheres=int(np.sum(~active)),scored_spheres=int(np.sum(active)),
        endpoint_floor_clearance_m=dict(zip(names,map(float,clearance))),controlling_spheres=dict(zip(names,controlling)),
        links=links,goal_unchanged=True,goal_source='unchanged v3 qualified joint state (existing 6 mm tolerance)',
        reference_sha256=hashlib.sha256(v3.REFERENCE_FILE.read_bytes()).hexdigest(),
        robot_config_sha256=ref['robot_config_sha256'],frozen_config=ref['selected_config'],
        motion_runs=0)
    if save:tuned.save_json(PREFLIGHT_FILE,result)
    print('V41_PREFLIGHT '+json.dumps({k:result[k] for k in ('passed','excluded_links_with_spheres','excluded_spheres','scored_spheres','endpoint_floor_clearance_m')}),flush=True)
    return result


def rescore(config,save):
    print('V41_RESCORE expected wall time: 1-2 minutes; saved FK only, no motion rerun',flush=True)
    began=time.monotonic()
    robot=v2.Kinematics(v2.KinematicsCfg.from_robot_yaml_file(str(config),tool_frames=[v2.sixth.preflight.TOOL_FRAME]))
    active=nonbase_mask(robot)
    rows=[];runs=[];hashes={}
    for dataset,filename in (('rates','moving_rod_rates.json'),('v3','moving_rod_v3.json')):
        path=v2.RESULTS_DIR/filename
        hashes[filename]=hashlib.sha256(path.read_bytes()).hexdigest()
        old=json.loads(path.read_text())
        for subset,cases in ((dataset,old['cases']),(dataset+' supplementary A',old.get('extra_A_runs',{}))):
            for name,case in cases.items():
                trace=case['trace'];minimum=float('inf');contact_samples=0;episodes=0;prior=False;link_at_min=None
                for offset in range(0,len(trace),512):
                    q=torch.as_tensor(np.array([s['q'] for s in trace[offset:offset+512]]),device='cuda',dtype=torch.float32)
                    spheres=v4.sphere_series(robot,q)
                    dist=v4.floor_distances(spheres,active,0.)
                    if float(np.min(dist))<minimum:
                        minimum=float(np.min(dist))
                        row=spheres[int(np.argmin(dist))]
                        ids=robot.kinematics_config.link_sphere_idx_map.detach().cpu().numpy()
                        indices=np.flatnonzero(active);i=indices[int(np.argmin(row[active,2]-row[active,3]))]
                        link_at_min=next(link for link,idx in robot.kinematics_config.link_name_to_idx_map.items() if idx==ids[i])
                    for x in dist:
                        hit=bool(x<0);contact_samples+=int(hit);episodes+=int(hit and not prior);prior=hit
                runs.append(dict(dataset=subset,case=name,group=case['summary']['mode'],speed_mps=case['summary']['rod_speed_mps'],
                    initial_x_m=case['summary']['initial_x_m'],samples=len(trace),floor_contact=minimum<0,
                    floor_episodes=episodes,contact_samples=contact_samples,minimum_floor_clearance_m=minimum,
                    deepest_depth_m=max(0.,-minimum),deepest_link=link_at_min))
            for group in ('A','B','E'):
                selected=[r for r in runs if r['dataset']==subset and r['group']==group]
                if selected:
                    rows.append(dict(dataset=subset,group=group,runs=len(selected),floor_contact_runs=sum(r['floor_contact'] for r in selected),
                        deepest_depth_m=max(r['deepest_depth_m'] for r in selected)))
    result=dict(schema='4.1',date='2026-11-19',complete=True,rerun=False,excluded_links=['base_link_inertia','shoulder_link'],
        floor_top_m=0.,source_sha256=hashes,source_commits=dict(rates='7527503',v3='56ea2c3'),
        sampling='only saved trace samples, no interpolation; therefore depth is a sampled lower bound and may miss between-sample contacts',
        elapsed_s=time.monotonic()-began,summary=rows,runs=runs)
    if save:
        tuned.save_json(RESCORE_FILE,result)
        RESCORE_FILE.with_suffix('.log').write_text(json.dumps({k:result[k] for k in ('excluded_links','sampling','elapsed_s','summary')},indent=2)+'\n')
    print('V41_OLD_FLOOR_RESCORE '+json.dumps(rows),flush=True)
    return result


def floor_guard(clearance_m):
    if clearance_m < 0:
        return f'floor penetration {abs(clearance_m)*1000:.3f} mm'
    return None


def goal_distances(curve):
    closest = min(curve, key=lambda sample: sample['distance_m'])
    return dict(
        initial_goal_distance_m=curve[0]['distance_m'],
        minimum_goal_distance_m=closest['distance_m'],
        minimum_goal_distance_time_s=closest['time_s'],
        final_goal_distance_m=curve[-1]['distance_m'],
        goal_tolerance_m=v2.sixth.GOAL_TOLERANCE_M,
    )


def qualify(config):
    report = preflight(config, save=True)
    require_preflight(report)
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    phase = baseline['rod']['regular_initial_x_m']
    start = np.asarray(baseline['raised_start']['joints_rad'], dtype=np.float32)
    began = time.monotonic()
    mpc, _ = make_mpc(config, report['frozen_config'])
    try:
        mpc.setup(v4.joint_state(start, np.zeros(6)))
        case = v4.run_case(
            mpc, start, 0., False, 'A', None, baseline,
            duration=QUALIFICATION_DURATION_S, no_rod=True, stop_at_goal=True,
            floor_mask=nonbase_mask(mpc.kinematics), step_guard=floor_guard,
        )
    finally:
        mpc.destroy()
    case['summary'].update(initial_x_m=phase, repeat=1, wall_time_s=time.monotonic()-began)
    summary = case['summary']
    failure = summary['stop_reason']
    if failure is None and not summary['survives']:
        failure = f"Goal not reached within {QUALIFICATION_DURATION_S:g} simulated seconds"
    result = dict(
        passed=failure is None, failure=failure, endpoint_gate_m=ENDPOINT_GATE_M,
        preflight_sha256=hashlib.sha256(PREFLIGHT_FILE.read_bytes()).hexdigest(),
        goal=goal_distances(case['distance_to_goal']), case=v4.compact_case(case),
    )
    tuned.save_json(QUALIFICATION_FILE, result)
    metrics = {key: summary[key] for key in (
        'reaches', 'floor_contacts', 'minimum_floor_clearance_m', 'time_to_first_goal_s',
        'simulated_duration_s', 'wall_time_s',
    )}
    print('V41_QUALIFICATION ' + json.dumps(dict(metrics, failure=failure, **result['goal'])), flush=True)
    return result['passed']


def main():
    p=argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--rescore', action='store_true')
    mode.add_argument('--qualify', action='store_true')
    p.add_argument('--save-results',action='store_true')
    p.add_argument('--robot-config',type=Path,default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    args=p.parse_args()
    if args.rescore:rescore(args.robot_config,args.save_results);return 0
    if args.qualify:
        if not args.save_results:
            p.error('--qualify requires --save-results to checkpoint preflight and qualification')
        return 0 if qualify(args.robot_config) else 2
    report=preflight(args.robot_config,args.save_results)
    return 0 if report['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
