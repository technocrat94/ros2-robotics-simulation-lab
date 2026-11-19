#!/usr/bin/env python3
"""v4 floor, conservative unseen extent and prediction-on/off MPC experiments."""
from __future__ import annotations
import argparse, copy, hashlib, json, math, os, tempfile, time
from pathlib import Path
from typing import Any, Callable
import numpy as np
import torch
import curobo_moving_rod_v3 as v3
from curobo_moving_rod_mpc import CONTROL_DT, integrate_command, joint_state, update_box
from curobo_moving_rod_v4_rules import (nonbase_mask,sphere_series,cylinder_distances,floor_distances,
    ConservativeRodMemory,box_covers_rod,MAX_UNSEEN_SPEED_MPS,sample_held_pose)

tuned,v2=v3.tuned,v3.v2
ModelPredictiveControl=tuned.ModelPredictiveControl
horizon_seconds,stretch_fit,limited_command=v3.horizon_seconds,v3.stretch_fit,v3.limited_command
OBS_HZ=30
GROUPS=("A","B","B-nopred","E","E-nopred")
SPEEDS=(.05,.10,.20)
PHASES=v3.rates.PHASES
RESULT_FILE=v2.RESULTS_DIR/"moving_rod_v4.json"
PREFLIGHT_FILE=v2.RESULTS_DIR/"moving_rod_v4.preflight.json"


def small_fit(fit):
    return None if fit is None else {k:fit[k] for k in ("center_m","dims_m","points") if k in fit}


def make_mpc(config: Path, settings: dict[str, Any]) -> tuple[ModelPredictiveControl, dict[str, Any]]:
    """Build a private in-memory config, preserving all original controller settings."""
    floor_top=json.loads(PREFLIGHT_FILE.read_text())["floor_top_m"]
    robot_cfg, wrist = v2.sixth.limited_robot_cfg(config, v2.sixth.CAMERA_FRONT_SEED)
    cfg = tuned.ModelPredictiveControlCfg.create(
        robot=robot_cfg, scene_model={"cuboid": {"floor": {"dims":[10.,10.,.1],"pose":[0.,0.,floor_top-.05,1.,0.,0.,0.]},"camera_obstacle": {
            "dims": [.01,.01,.01], "pose": [0.,0.,-10.,1.,0.,0.,0.]}}},
        optimization_dt=settings.get("optimization_dt", v3.tuned.OPTIMIZATION_DT), interpolation_steps=4, use_cuda_graph=True,
        optimizer_collision_activation_distance=.03,
        warm_start_optimization_num_iters=200, cold_start_optimization_num_iters=300,
        squared_l2_regularization_weight=[.01, settings["acceleration_regularization"], 10., 0., 0.],
        non_terminal_tool_pose_weight_factor=settings["non_terminal_tool_pose_weight_factor"],
    )
    cost = cfg.optimizer_rollout_configs[0].cost_cfg
    transition = cfg.metrics_rollout_config.transition_model_cfg
    report = {
        "wrist_3_limit": wrist,
        "regularization_velocity_acceleration_jerk_torque_energy": cost.cspace_cfg.squared_l2_regularization_weight.detach().cpu().tolist(),
        "cspace_bounds_cost_weights": cost.cspace_cfg.weight.detach().cpu().tolist(),
        "tool_pose_weights": cost.tool_pose_cfg.weight.detach().cpu().tolist(),
        "non_terminal_tool_pose_weight_factor": cfg.non_terminal_tool_pose_weight_factor,
        "terminal_pose_convergence_tolerance": list(cost.tool_pose_cfg._terminal_pose_convergence_tolerance),
        "retime_regularization_weights": cost.cspace_cfg.retime_regularization_weights,
        "optimization_dt_s": cfg.optimization_dt, "command_dt_s": CONTROL_DT,
        "mpc_interpolation_dt_s": cfg.optimization_dt/4,
        "control_points": transition.n_knots, "state_horizon": transition.horizon,
        "vel_scale": transition.vel_scale,
        "position_tolerance_m": cfg.position_tolerance, "task_reach_tolerance_m": v2.sixth.GOAL_TOLERANCE_M,
    }
    mpc=ModelPredictiveControl(cfg)
    mpc._v4_floor_top=floor_top
    limits=mpc.kinematics.get_joint_limits()
    report.update(floor_top_m=floor_top,mpc_horizon_s=horizon_seconds(mpc),
        position_limits_rad=limits.position.detach().cpu().tolist(),
        velocity_limits_radps=limits.velocity.detach().cpu().tolist(),
        acceleration_limits_radps2=limits.acceleration.detach().cpu().tolist())
    return mpc, report


def run_case(mpc: ModelPredictiveControl, start: np.ndarray, speed: float,
             prediction: bool, mode: str, worker: v2.MovingRodWorker,
             baseline: dict[str, Any], duration: float = 20.,
             no_rod: bool = False, stop_at_goal: bool = False, *,
             floor_mask: np.ndarray | None = None,
             step_guard: Callable[[float], str | None] | None = None) -> dict[str, Any]:
    """Run the held-velocity plant, optionally stopping after a scored step."""
    family=mode.split("-")[0]
    mask=nonbase_mask(mpc.kinematics) if floor_mask is None else floor_mask
    floor_top=float(mpc._v4_floor_top)
    rod_active=floor_active=False
    rod_episodes=floor_episodes=floor_samples=0
    collision_events=[]
    perception_events=[]
    q = start.copy()
    velocity = np.zeros(6)
    acceleration = np.zeros(6)
    state = joint_state(q, velocity)
    update_box(mpc, None)
    mpc.reset_robot(state)
    raised_goal = mpc.compute_kinematics(state).tool_poses.as_goal(mpc.tool_frames)
    goal = v2.sixth.goal_from_scene()
    mpc.update_goal_tool_poses(goal, run_ik=False)
    # Cold start and graph capture belong to setup, outside the measured case.
    mpc.optimize_action_sequence(state)
    limits = mpc.kinematics.get_joint_limits()
    position_limits = limits.position.detach().cpu().numpy()
    velocity_limits = limits.velocity.detach().cpu().numpy()
    accel_limits = limits.acceleration.detach().cpu().numpy()
    horizon_s = horizon_seconds(mpc)
    next_observation = 0
    observation_times = []
    fit = None
    estimated_velocity = np.zeros(3)
    clock = 0.
    last_segment_q=q.copy()
    last_segment_velocity=np.zeros(6)
    last_segment_time=0.
    target = "goal"
    goal_pending = False
    target_points = {
        "goal": goal.position.detach().cpu().numpy().reshape(-1, 3)[0],
        "raised_start": v2.sixth.fourth.tool_position(mpc.kinematics, state.position),
    }
    traces = [{"time_s": 0., "q": q.tolist()}]
    cycles = []
    camera_memory=ConservativeRodMemory(baseline["border_rule"]["fixed_unknown_top_m"])
    previous_frame = None
    expected_compute = CONTROL_DT
    collision_active = False
    episodes = {"moving": 0, "holding": 0}
    samples = {"moving": 0, "holding": 0}
    near_directions = {"away": 0, "toward": 0, "stationary": 0}
    near_robot_rates, near_relative_rates = [], []
    minimum = math.inf
    minimum_floor = math.inf
    tall_box_active = False
    floor_box_samples = dict(tall=0, grown=0, either=0, both=0)
    stop_reason = None
    reaches = 0
    infeasible = 0
    limit_clamps = 0
    first_reach = None
    max_velocity = np.zeros(6)
    max_acceleration = np.zeros(6)
    max_proposed_velocity = np.zeros(6)
    velocity_clip_steps = 0
    distance_curve = [{"time_s": 0., "distance_m": float(np.linalg.norm(target_points["goal"]-target_points["raised_start"]))}]
    frame_errors = []
    fixed_top = baseline["border_rule"]["fixed_unknown_top_m"]

    def advance(duration: float) -> None:
        nonlocal clock, q, minimum, collision_active, limit_clamps, rod_active, floor_active, rod_episodes, floor_episodes, floor_samples, last_segment_q, last_segment_velocity, last_segment_time
        nonlocal minimum_floor
        offsets, positions = integrate_command(q, velocity, duration, position_limits)
        if len(offsets) == 0:
            return
        last_segment_q=q.copy();last_segment_velocity=velocity.copy();last_segment_time=clock
        xs = np.array([v2.rod_x_at(clock+t, speed, v2.ROD_START_X_M, -1.) for t in offsets])
        tensors = torch.as_tensor(positions, device="cuda", dtype=torch.float32)
        spheres=sphere_series(mpc.kinematics,tensors)
        rod_truth=np.full(len(positions),1000.) if no_rod else cylinder_distances(spheres,xs)
        floor_truth=floor_distances(spheres,mask,floor_top)
        if step_guard is not None:
            if not (np.isfinite(floor_truth).all() and np.isfinite(rod_truth).all()):
                raise RuntimeError("non-finite clearance")
            minimum_floor = min(minimum_floor, float(np.min(floor_truth)))
        truth=np.minimum(rod_truth,floor_truth)
        stationary_rod=np.full(len(positions)-1,1000.) if no_rod else cylinder_distances(spheres[:-1],xs[1:])
        stationary=np.minimum(stationary_rod,floor_truth[:-1])
        for i in range(1, len(offsets)):
            moving = bool(np.max(np.abs(positions[i]-positions[i-1])) > 1e-7)
            phase = "moving" if moving else "holding"
            clearance = float(truth[i])
            minimum = min(minimum, clearance, float(truth[i-1]))
            colliding = clearance < 0
            if colliding:
                samples[phase] += 1
                if not collision_active:
                    episodes[phase] += 1
            rod_contact=rod_truth[i]<0
            floor_contact=floor_truth[i]<0
            if rod_contact and not rod_active:
                rod_episodes+=1
                actual_x=float(xs[i])
                collision_events.append(dict(time_s=float(clock+offsets[i]),kind="rod",
                    no_box=fit is None,observed_covers_rod=box_covers_rod(fit,actual_x),
                    predicted_covers_rod=box_covers_rod(predicted,actual_x),
                    fit=small_fit(fit),predicted_fit=small_fit(predicted),
                    truth_x_m=actual_x,clearance_m=float(rod_truth[i]),
                    perception_event=copy.deepcopy(camera_memory.last_event)))
            if floor_contact and not floor_active:
                floor_episodes+=1
                collision_events.append(dict(time_s=float(clock+offsets[i]),kind="floor",clearance_m=float(floor_truth[i])))
            if floor_contact and step_guard is not None and family == 'E':
                tall = fit is not None and tall_box_active
                grown = fit is not None and camera_memory.last_event.get('growth_rule', False)
                floor_box_samples['tall'] += int(tall)
                floor_box_samples['grown'] += int(grown)
                floor_box_samples['either'] += int(tall or grown)
                floor_box_samples['both'] += int(tall and grown)
                if not floor_active:
                    collision_events[-1].update(tall_box_active=tall, grown_box_active=grown)
            floor_samples+=int(floor_contact)
            rod_active=rod_contact;floor_active=floor_contact
            collision_active = colliding
            robot_rate = (truth[i]-stationary[i-1]) / (offsets[i]-offsets[i-1])
            relative_rate = (truth[i]-truth[i-1]) / (offsets[i]-offsets[i-1])
            if min(truth[i], truth[i-1]) <= .05:
                direction = "away" if robot_rate > 1e-4 else "toward" if robot_rate < -1e-4 else "stationary"
                near_directions[direction] += 1
                near_robot_rates.append(float(robot_rate))
                near_relative_rates.append(float(relative_rate))
            traces.append({"time_s": float(clock+offsets[i]), "q": positions[i].tolist(),
                           "clearance_m": clearance, "phase": phase})
        unconstrained = q + velocity * duration
        limit_clamps += int(np.any(np.abs(unconstrained-positions[-1]) > 1e-7))
        q = positions[-1].copy()
        clock += duration

    for step in range(1, 20001):
        if clock >= duration - 1e-9:
            break
        captured_time = clock
        snapshot_q = q.copy()
        began = time.perf_counter()
        if goal_pending:
            mpc.update_goal_tool_poses(raised_goal if target == "raised_start" else goal, run_ik=False)
            goal_pending = False
        x = v2.rod_x_at(clock, speed, v2.ROD_START_X_M, -1.)
        observation_updated = False
        while not no_rod and family != "A" and next_observation/OBS_HZ <= clock+1e-9:
            observed_time = next_observation/OBS_HZ
            next_observation += 1
            observation_updated = True
            observation_times.append(observed_time)
            observed_x = v2.rod_x_at(observed_time, speed, v2.ROD_START_X_M, -1.)
            # Reconstruct the held-velocity plant at the exact 30 Hz observation time.
            observed_q=sample_held_pose(last_segment_q,last_segment_velocity,observed_time-last_segment_time,position_limits)
            if family == "B":
                fit = v2.exact_rod_fit(observed_x)
                center = np.asarray(fit["center_m"])
                fresh = fit
            else:
                capture = worker.capture(observed_q, observed_x).camera
                fit, estimated_velocity, points, event = camera_memory.observe(capture,observed_time)
                if event['fresh_seen']:
                    tall_box_active = event['top_rule']
                perception_events.append(event)
                continue
            estimated_velocity = np.zeros(3)
            if fresh is not None:
                if previous_frame is not None:
                    estimated_velocity = (center-previous_frame[1])/(observed_time-previous_frame[0])
                previous_frame = (observed_time,center.copy())
        predicted = stretch_fit(fit, estimated_velocity, horizon_s+1/OBS_HZ) if prediction else fit
        update_box(mpc, predicted)
        state = joint_state(snapshot_q, velocity)
        # Between discrete command updates the held-velocity plant has zero acceleration.
        # The finite command delta is constrained and audited separately below.
        torch.cuda.synchronize()
        solve_began = time.perf_counter()
        result = mpc.optimize_action_sequence(state)
        torch.cuda.synchronize()
        solve_s = time.perf_counter()-solve_began
        command_sequence = result.action_sequence.velocity.detach().cpu().numpy().reshape(-1, 6)
        if step_guard is not None and not np.isfinite(command_sequence).all():
            raise RuntimeError("non-finite MPC command sequence")
        feasible = bool(result.success.all())
        compute_s = time.perf_counter()-began
        counted_compute = CONTROL_DT
        # Keep executing the OLD command throughout sensing and optimization.
        previous_velocity = velocity.copy()
        advance(min(counted_compute, duration-clock))
        q_after_compute = q.copy()
        index = min(max(1, int(math.ceil(counted_compute / (mpc.config.optimization_dt/4)))), len(command_sequence)-1)
        proposed = command_sequence[index]
        finite = bool(np.isfinite(proposed).all())
        if not finite:
            raise RuntimeError("non-finite MPC velocity")
        max_proposed_velocity = np.maximum(max_proposed_velocity, np.abs(proposed))
        velocity_clip_steps += int(np.any(proposed < velocity_limits[0]-1e-6) or np.any(proposed > velocity_limits[1]+1e-6))
        velocity = limited_command(proposed, previous_velocity, velocity_limits, accel_limits)
        acceleration = (velocity-previous_velocity)/CONTROL_DT
        # Finite infeasible MPC actions are intentionally executed in this simulation.
        # Gating on feasibility would recreate the stop-and-go freeze under test.
        if not feasible:
            infeasible += 1
        advance(min(max(0., CONTROL_DT-counted_compute), duration-clock))
        tool = v2.sixth.fourth.tool_position(mpc.kinematics, torch.as_tensor(q, device="cuda", dtype=torch.float32))
        target_error = float(np.linalg.norm(tool-target_points[target]))
        distance_curve.append({"time_s": clock, "distance_m": float(np.linalg.norm(tool-target_points["goal"]))})
        max_velocity = np.maximum(max_velocity, np.maximum(np.abs(previous_velocity), np.abs(velocity)))
        period = max(clock-captured_time, 1e-9)
        max_acceleration = np.maximum(max_acceleration, np.abs(velocity-previous_velocity)/period)
        reached = bool(target_error <= v2.sixth.GOAL_TOLERANCE_M)
        if reached:
            if first_reach is None:
                first_reach = clock
            reaches += 1
            target = "raised_start" if target == "goal" else "goal"
            goal_pending = True
        cycles.append({
            "step": step, "sim_time_start_s": captured_time, "sim_time_end_s": clock,
            "target_after_step": target, "target_reached": reached,
            "measured_compute_ms": compute_s*1000, "solver_ms": solve_s*1000,
            "compute_counted_s": min(counted_compute, duration-captured_time),
            "q_at_capture_rad": snapshot_q.tolist(),
            "q_after_compute_rad": q_after_compute.tolist(),
            "previous_velocity_radps": previous_velocity.tolist(),
            "new_velocity_radps": velocity.tolist(), "command_index_after_latency": index,
            "feasible": feasible, "finite": finite,
            "fit": v2.compact_fit(fit), "predicted_fit": v2.compact_fit(predicted),
            "velocity_estimate_mps": estimated_velocity.tolist(),
            "perception_event": copy.deepcopy(camera_memory.last_event) if family=="E" else None,
            "prediction_horizon_s": horizon_s+1/OBS_HZ if prediction else 0.,
            "observation_updated": observation_updated,
            "observation_time_s": observation_times[-1] if observation_times else None,
        })
        expected_compute = counted_compute
        if step_guard is not None:
            if not np.isfinite(tool).all():
                raise RuntimeError("non-finite tool position")
            stop_reason = step_guard(minimum_floor)
            if stop_reason is not None:
                break
        if stop_at_goal and reached:
            break
    if stop_reason is None and not (stop_at_goal and first_reach is not None) and clock < duration - 1e-6:
        raise RuntimeError(f"MPC clock incomplete: {clock}")
    compute_ms = np.array([c["measured_compute_ms"] for c in cycles])
    solve_ms = np.array([c["solver_ms"] for c in cycles])
    periods = np.array([c["sim_time_end_s"]-c["sim_time_start_s"] for c in cycles])
    summary = {
        "mode": mode, "prediction": prediction, "rod_speed_mps": speed,
        "controller": "cuRobo MPC tuned, held joint velocity during computation",
        "reaches": reaches, "true_collisions": sum(episodes.values()),
        "collision_episodes_by_onset_phase": episodes, "collision_samples_by_phase": samples,
        "minimum_true_clearance_m": None if no_rod else minimum, "steps": len(cycles), "simulated_duration_s": clock,
        "compute_median_ms": float(np.median(compute_ms)), "compute_p95_ms": float(np.percentile(compute_ms,95)),
        "solver_median_ms": float(np.median(solve_ms)), "solver_p95_ms": float(np.percentile(solve_ms,95)),
        "step_period_median_ms": float(np.median(periods)*1000),
        "step_period_p95_ms": float(np.percentile(periods,95)*1000),
        "achieved_step_rate_hz": len(cycles)/clock,
        "within_50mm_robot_direction_samples": near_directions,
        "within_50mm_mean_robot_clearance_rate_mps": float(np.mean(near_robot_rates)) if near_robot_rates else None,
        "within_50mm_mean_relative_clearance_rate_mps": float(np.mean(near_relative_rates)) if near_relative_rates else None,
        "infeasible_steps": infeasible, "joint_position_clamped_intervals": limit_clamps,
        "survives": sum(episodes.values()) == 0 and reaches > 0,
        "time_to_first_goal_s": first_reach,
        "max_joint_velocity_radps": max_velocity.tolist(),
        "max_sampled_joint_acceleration_radps2": max_acceleration.tolist(),
        "max_proposed_joint_velocity_radps": max_proposed_velocity.tolist(),
        "velocity_limits_radps": velocity_limits.tolist(),
        "within_joint_velocity_limits": bool(np.all(max_velocity <= np.maximum(np.abs(velocity_limits[0]), np.abs(velocity_limits[1])) + 1e-6)),
        "velocity_clipped_steps": velocity_clip_steps,
        "worst_camera_frames": {},
        "floor_top_m":floor_top,"floor_contacts":floor_episodes,"floor_contact_samples":floor_samples,
        "rod_collisions":rod_episodes,"top_rule_events":camera_memory.top_events,
        "growth_rule_events":camera_memory.growth_events,
        "rod_onsets_no_box":sum(e.get("no_box",False) for e in collision_events if e["kind"]=="rod"),
        "rod_onsets_uncovered_box":sum(not e["observed_covers_rod"] and not e["no_box"] for e in collision_events if e["kind"]=="rod"),
        "acceleration_limits_radps2": accel_limits.tolist(),
        "within_joint_acceleration_limits": bool(np.all(max_acceleration <= np.maximum(np.abs(accel_limits[0]),np.abs(accel_limits[1]))+1e-6)),
        "mpc_horizon_s": horizon_s, "observation_times_s": observation_times,
    }
    if step_guard is not None:
        summary.update(minimum_floor_clearance_m=minimum_floor, stop_reason=stop_reason,
                       floor_contact_box_samples=floor_box_samples)
    boundary_times = {round(c["sim_time_end_s"],9) for c in cycles} | {0.}
    traces = [p for p in traces if round(p["time_s"],9) in boundary_times]
    return {"summary": summary, "cycles": cycles, "trace": traces,
            "distance_to_goal": distance_curve, "camera_frame_errors": frame_errors,"perception_events":perception_events,"collision_events":collision_events}


def preflight(config,save):
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    ref=json.loads(v3.REFERENCE_FILE.read_text())
    if ref['robot_config_sha256']!=hashlib.sha256(config.read_bytes()).hexdigest():
        raise RuntimeError('robot config differs from frozen v3 qualification')
    robot=v2.Kinematics(v2.KinematicsCfg.from_robot_yaml_file(str(config),tool_frames=[v2.sixth.preflight.TOOL_FRAME]))
    mask=nonbase_mask(robot)
    states={'start':np.array(v2.sixth.CAMERA_FRONT_SEED),'raised_start':np.array(baseline['raised_start']['joints_rad']),
            'goal':np.array(ref['attempts'][-1]['case']['trace'][-1]['q'])}
    spheres={n:sphere_series(robot,torch.as_tensor(q.reshape(1,-1),device='cuda',dtype=torch.float32)) for n,q in states.items()}
    base_bottom=float(np.min(spheres['start'][0,~mask,2]-spheres['start'][0,~mask,3]))
    floor_top=0. if base_bottom>1e-6 else min(0.,base_bottom-.001)
    clear={n:float(floor_distances(s,mask,floor_top)[0]) for n,s in spheres.items()}
    if min(clear.values())<=0:raise RuntimeError('endpoint floor collision: '+json.dumps(clear))
    result=dict(complete=True,date='2026-11-19',floor_top_m=floor_top,base_bottom_m=base_bottom,
        floor_dims_m=[10.,10.,.1],base_spheres_excluded_from_score=int(np.sum(~mask)),
        nonbase_spheres_scored=int(np.sum(mask)),endpoint_floor_clearance_m=clear,
        goal_source='v3 qualified no-rod goal joint state, within 6 mm task tolerance',
        goal_tool_error_m=float(np.linalg.norm(v2.sixth.fourth.tool_position(robot,torch.as_tensor(states['goal'],device='cuda',dtype=torch.float32))-v2.sixth.goal_from_scene().position.detach().cpu().numpy().reshape(-1,3)[0])),
        frozen_config=ref['selected_config'],robot_config_sha256=ref['robot_config_sha256'],
        floor_choice='z=0 unless base sphere touches/penetrates; then 1 mm below its lowest surface',
        truth_sampling_s=CONTROL_DT)
    if result['goal_tool_error_m']>.006+1e-6:raise RuntimeError('qualified goal mismatch')
    if save:tuned.save_json(PREFLIGHT_FILE,result)
    print('V4_PREFLIGHT '+json.dumps(result),flush=True)
    return result


def compact_case(case):
    velocities=[c['velocity_estimate_mps'] for c in case['cycles']]
    events=[c['perception_event'] for c in case['cycles']]
    perceptions=case.pop('perception_events');collisions=case.pop('collision_events')
    result=v3.compact_case(case)
    for c,velocity,event in zip(result['cycles'],velocities,events):
        c['velocity_estimate_mps']=[round(float(v),8) for v in velocity]
        # A compact held-event reference avoids repeating growing-box diagnostics.
        c['perception_event_index']=max(0,round((c['observation_time_s'] or 0)*OBS_HZ)) if event else None
    result['perception_events']=perceptions;result['collision_events']=collisions
    return result


def summarize(results):
    for case in results["cases"].values():
        q=np.asarray([p["q"] for p in case["trace"]])
        actual_velocity=np.diff(q,axis=0)/CONTROL_DT
        actual_acceleration=np.diff(np.vstack((np.zeros((1,6)),actual_velocity)),axis=0)/CONTROL_DT
        case["summary"]["max_trace_joint_speed_radps"]=float(np.max(np.abs(actual_velocity)))
        case["summary"]["max_trace_sampled_acceleration_radps2"]=float(np.max(np.abs(actual_acceleration)))
    rows=[];table=[]
    for speed in SPEEDS:
        for phase in PHASES:
            group=[c for c in results['cases'].values() if c['summary']['rod_speed_mps']==speed and c['summary']['initial_x_m']==phase]
            a=next(c['summary'] for c in group if c['summary']['mode']=='A')
            # Preserve the rod-scene validity test; floor contacts still fail the run.
            status='VALID_SCENE' if a['rod_collisions']>0 else 'INVALID_SCENE'
            for c in group:c['summary']['scene_status']=status
            rows.append(dict(speed_mps=speed,initial_x_m=phase,status=status))
        for mode in GROUPS:
            cs=[c for c in results['cases'].values() if c['summary']['rod_speed_mps']==speed and c['summary']['mode']==mode and c['summary']['scene_status']=='VALID_SCENE']
            ss=[c['summary'] for c in cs]
            table.append(dict(mode=mode,speed_mps=speed,valid_runs=len(ss),successes=sum(s['survives'] for s in ss),
                collision_runs=sum(s['true_collisions']>0 for s in ss),
                worst_clearance_m=min((s['minimum_true_clearance_m'] for s in ss),default=None),
                floor_contact_runs=sum(s['floor_contacts']>0 for s in ss),floor_contacts=sum(s['floor_contacts'] for s in ss),
                top_rule_events=sum(s['top_rule_events'] for s in ss),growth_rule_events=sum(s['growth_rule_events'] for s in ss),
                rod_onsets_no_box=sum(s['rod_onsets_no_box'] for s in ss),
                rod_onsets_uncovered_box=sum(s['rod_onsets_uncovered_box'] for s in ss),
                joint_position_clamped_intervals=sum(s['joint_position_clamped_intervals'] for s in ss),
                joint_position_clamped_runs=sum(s['joint_position_clamped_intervals']>0 for s in ss),
                max_trace_sampled_acceleration_radps2=max((s['max_trace_sampled_acceleration_radps2'] for s in ss),default=None),
                command_limits_pass=all(s['within_joint_velocity_limits'] and s['within_joint_acceleration_limits'] for s in ss),
                limits_pass=all(s['within_joint_velocity_limits'] and s['within_joint_acceleration_limits'] and s['joint_position_clamped_intervals']==0 for s in ss)))
    results['per_phase']=rows;results['rates']=table


def build(config,worker,save):
    pf=json.loads(PREFLIGHT_FILE.read_text())
    ref=json.loads(v3.REFERENCE_FILE.read_text())
    if not pf['complete'] or pf['frozen_config']!=ref['selected_config'] or pf['robot_config_sha256']!=hashlib.sha256(config.read_bytes()).hexdigest():
        raise RuntimeError('run v4 preflight first')
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text());tuned.original.configure_scene(baseline)
    start=np.array(baseline['raised_start']['joints_rad'],dtype=np.float32)
    results=dict(schema=4,date='2026-11-19',complete=False,frozen_config=ref['selected_config'],
        v3_reference_sha256=hashlib.sha256(v3.REFERENCE_FILE.read_bytes()).hexdigest(),preflight=pf,
        rod=baseline['rod'],raised_start=baseline['raised_start'],groups=GROUPS,speeds_mps=SPEEDS,phases_m=PHASES,
        validity_rule="A must collide with the rod; floor contact is a failure but does not validate rod placement",
        repeats=1,duration_per_case_s=20.,control_period_s=CONTROL_DT,counted_compute_s=CONTROL_DT,observation_hz=OBS_HZ,
        observation_pose_rule="exact clipped held-velocity segment at n/30 seconds, including position saturation",
        unseen_max_speed_assumption_mps=MAX_UNSEEN_SPEED_MPS,growth='each side +/- X/Y expands 0.20*age; full dimensions +2*0.20*age; drop after 1 s',
        cases={})
    for speed in SPEEDS:
        for phase in PHASES:
            v2.ROD_START_X_M=phase
            for mode in GROUPS:
                family=mode.split('-')[0];prediction=not mode.endswith('-nopred')
                if family=='E' and worker.process is None:worker.start();worker.capture(start,phase)
                mpc,loaded=make_mpc(config,ref['selected_config']);results['loaded_config']=loaded
                try:
                    mpc.setup(joint_state(start,np.zeros(6)))
                    case=run_case(mpc,start,speed,prediction,mode,worker,baseline)
                finally:mpc.destroy()
                case['summary'].update(initial_x_m=phase,repeat=1)
                name=f'v4 {mode} {speed:.2f} m/s X={phase:.2f}'
                results['cases'][name]=compact_case(case)
                print(f'V4_CASE {len(results["cases"]):02d}/75 {name} success={case["summary"]["survives"]} collisions={case["summary"]["true_collisions"]} floor={case["summary"]["floor_contacts"]}',flush=True)
                if save:tuned.save_json(RESULT_FILE,results)
    summarize(results);results['complete']=True
    return results


def main():
    parser=argparse.ArgumentParser()
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--preflight',action='store_true');modes.add_argument('--replay',action='store_true');modes.add_argument('--interactive',action='store_true')
    parser.add_argument('--interactive-smoke',action='store_true');parser.add_argument('--headless',action='store_true');parser.add_argument('--save-results',action='store_true')
    parser.add_argument('--viewer-smoke',action='store_true')
    parser.add_argument('--duration',type=float,default=0.)
    parser.add_argument('--port',type=int,default=int(os.environ.get('NEWTON_VIEWER_PORT','30000')))
    parser.add_argument('--robot-config',type=Path,default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    parser.add_argument('--newton-python',type=Path,default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args=parser.parse_args()
    if (args.replay or args.interactive) and (args.save_results or args.headless):parser.error('watch modes cannot save or be headless')
    if args.viewer_smoke and not args.replay:parser.error('viewer smoke requires replay')
    if args.interactive_smoke and not args.interactive:parser.error('smoke requires interactive')
    if not args.headless and not args.preflight:v3.check_viewer_port(args.port)
    if args.interactive:
        from curobo_moving_rod_v4_live import live_main
        return live_main(args)
    viewer=None
    with tempfile.TemporaryDirectory(prefix='rod-v4-') as temp:
        worker=v2.MovingRodWorker(args.newton_python,Path(temp))
        try:
            if args.preflight:preflight(args.robot_config,args.save_results);return 0
            results=json.loads(RESULT_FILE.read_text()) if args.replay else build(args.robot_config,worker,args.save_results)
            if not results['complete']:raise RuntimeError('incomplete v4 checkpoint')
            if args.save_results:
                tuned.save_json(RESULT_FILE,results)
                RESULT_FILE.with_suffix('.log').write_text(json.dumps({k:results[k] for k in ('frozen_config','preflight','rates','per_phase')},indent=2)+'\n')
                print('V4_RESULTS_SAVED '+json.dumps(results['rates']),flush=True)
            if not args.headless:
                from curobo_moving_rod_v4_viewer import start_viewer,FrameCache
                tuned.original.configure_scene(results)
                frames=FrameCache(worker,results)
                frames[next(iter(results['cases']))]
                viewer,report=start_viewer(args.robot_config,results,frames,args.port,verify=args.viewer_smoke)
                if args.viewer_smoke and not report['all_passed']:raise RuntimeError('v4 replay callbacks failed')
                began=time.monotonic()
                while args.duration<=0 or time.monotonic()-began<args.duration:time.sleep(.25)
        finally:
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:thread.join(timeout=10)
                viewer._server.stop()
            worker.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
