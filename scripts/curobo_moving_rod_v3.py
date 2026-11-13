#!/usr/bin/env python3
"""Frozen v3 rules: bounded command updates, fixed clock, exact 30 Hz observations."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import math
import os
import socket
from pathlib import Path
import tempfile
import time
from typing import Any
import numpy as np
import torch
import curobo_moving_rod_rates as rates
from curobo_moving_rod_mpc import CONTROL_DT, integrate_command, joint_state, update_box
from curobo_moving_rod_mpc_tuned import box_error, worst_frames

tuned, v2 = rates.tuned, rates.v2
ModelPredictiveControl = rates.ModelPredictiveControl
OBS_HZ = 30
REPEATS = 5
RESULT_FILE = v2.RESULTS_DIR/"moving_rod_v3.json"
REFERENCE_FILE = v2.RESULTS_DIR/"moving_rod_v3.reference.json"


def horizon_seconds(mpc):
    transition = mpc.config.metrics_rollout_config.transition_model_cfg
    return (transition.horizon-1)*mpc.config.optimization_dt/transition.interpolation_steps


def stretch_fit(fit, velocity, horizon_s):
    if fit is None:
        return None
    result = copy.deepcopy(fit)
    displacement = np.asarray(velocity)*horizon_s
    result["center_m"] = (np.asarray(fit["center_m"])+displacement/2).tolist()
    result["dims_m"] = (np.asarray(fit["dims_m"])+np.abs(displacement)).tolist()
    return result


def limited_command(proposed, previous, speed_limits, acceleration_limits):
    target = np.clip(proposed,speed_limits[0],speed_limits[1])
    return previous+np.clip(target-previous,acceleration_limits[0]*CONTROL_DT,acceleration_limits[1]*CONTROL_DT)


def make_mpc(config, settings):
    mpc, loaded = tuned.make_mpc(config,settings)
    limits = mpc.kinematics.get_joint_limits()
    loaded["velocity_limits_radps"] = limits.velocity.detach().cpu().tolist()
    loaded["acceleration_limits_radps2"] = limits.acceleration.detach().cpu().tolist()
    loaded["acceleration_source"] = "robot YAML kinematics.cspace.max_acceleration, scaled by acceleration_scale; same robot config supplied to MPC"
    loaded["mpc_horizon_s"] = horizon_seconds(mpc)
    loaded["horizon_formula"] = "(transition.horizon - 1) * optimization_dt / transition.interpolation_steps"
    loaded["prediction_horizon_s"] = horizon_seconds(mpc)+1/OBS_HZ
    return mpc,loaded


def run_case(mpc: ModelPredictiveControl, start: np.ndarray, speed: float,
             prediction: bool, mode: str, worker: v2.MovingRodWorker,
             baseline: dict[str, Any], duration: float = 20.,
             no_rod: bool = False, stop_at_goal: bool = False) -> dict[str, Any]:
    """Run an ideal joint-velocity plant while previous commands execute during compute."""
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
    target = "goal"
    goal_pending = False
    target_points = {
        "goal": goal.position.detach().cpu().numpy().reshape(-1, 3)[0],
        "raised_start": v2.sixth.fourth.tool_position(mpc.kinematics, state.position),
    }
    traces = [{"time_s": 0., "q": q.tolist()}]
    cycles = []
    cloud = v2.TimedPointCloud()
    previous_frame = None
    expected_compute = CONTROL_DT
    collision_active = False
    episodes = {"moving": 0, "holding": 0}
    samples = {"moving": 0, "holding": 0}
    near_directions = {"away": 0, "toward": 0, "stationary": 0}
    near_robot_rates, near_relative_rates = [], []
    minimum = math.inf
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
        nonlocal clock, q, minimum, collision_active, limit_clamps
        offsets, positions = integrate_command(q, velocity, duration, position_limits)
        if len(offsets) == 0:
            return
        xs = np.array([v2.rod_x_at(clock+t, speed, v2.ROD_START_X_M, -1.) for t in offsets])
        tensors = torch.as_tensor(positions, device="cuda", dtype=torch.float32)
        truth = np.full(len(positions), 1000.) if no_rod else v2.rod_clearance_series(mpc.kinematics, tensors, xs)
        # At the same new rod position, separate robot motion from rod motion.
        stationary = np.full(len(positions)-1, 1000.) if no_rod else v2.rod_clearance_series(mpc.kinematics, tensors[:-1], xs[1:])
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
        while not no_rod and mode != "A" and next_observation/OBS_HZ <= clock+1e-9:
            observed_time = next_observation/OBS_HZ
            next_observation += 1
            observation_updated = True
            observation_times.append(observed_time)
            observed_x = v2.rod_x_at(observed_time, speed, v2.ROD_START_X_M, -1.)
            # Reconstruct the held-velocity plant at the exact 30 Hz observation time.
            times = [p["time_s"] for p in traces[-12:]]
            local = traces[-12:]
            observed_q = np.array([np.interp(observed_time,times,[p["q"][j] for p in local]) for j in range(6)])
            if mode == "B":
                fit = v2.exact_rod_fit(observed_x)
                center = np.asarray(fit["center_m"])
                fresh = fit
            else:
                capture = worker.capture(observed_q, observed_x).camera
                points, _ = cloud.update(capture, observed_time)
                fit, _ = v2.sixth.fit_with_unknown_extent(points, capture, fixed_top)
                fresh, _ = v2.sixth.fit_with_unknown_extent(capture.points, capture, fixed_top)
                center = np.asarray(fresh["center_m"]) if fresh is not None else None
                frame_errors.append({"step":step,"time_s":observed_time,
                                     **box_error(fit,v2.exact_rod_fit(observed_x))})
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
            "prediction_horizon_s": horizon_s+1/OBS_HZ if prediction else 0.,
            "observation_updated": observation_updated,
            "observation_time_s": observation_times[-1] if observation_times else None,
        })
        expected_compute = counted_compute
        if stop_at_goal and reached:
            break
    if not (stop_at_goal and first_reach is not None) and clock < duration - 1e-6:
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
        "worst_camera_frames": worst_frames(frame_errors),
        "acceleration_limits_radps2": accel_limits.tolist(),
        "within_joint_acceleration_limits": bool(np.all(max_acceleration <= np.maximum(np.abs(accel_limits[0]),np.abs(accel_limits[1]))+1e-6)),
        "mpc_horizon_s": horizon_s, "observation_times_s": observation_times,
    }
    boundary_times = {round(c["sim_time_end_s"],9) for c in cycles} | {0.}
    traces = [p for p in traces if round(p["time_s"],9) in boundary_times]
    return {"summary": summary, "cycles": cycles, "trace": traces,
            "distance_to_goal": distance_curve, "camera_frame_errors": frame_errors}


def compact_case(case):
    # Control-boundary states reconstruct the exactly held velocity between updates.
    keep = {"step","sim_time_start_s","sim_time_end_s","measured_compute_ms","compute_counted_s",
            "new_velocity_radps","command_index_after_latency","feasible","fit","predicted_fit",
            "prediction_horizon_s","observation_updated","observation_time_s"}
    case["cycles"] = [{k:v for k,v in c.items() if k in keep} for c in case["cycles"]]
    for cycle in case["cycles"]:
        for key in ("fit","predicted_fit"):
            fit=cycle[key]
            if fit is not None:cycle[key]={k:fit[k] for k in ("center_m","dims_m","points") if k in fit}
    case.pop("distance_to_goal",None)
    case.pop("camera_frame_errors",None)
    def rounded(value):
        if isinstance(value,float): return round(value,8)
        if isinstance(value,list): return [rounded(v) for v in value]
        if isinstance(value,dict): return {k:rounded(v) for k,v in value.items()}
        return value
    return rounded(case)


def reference(config,worker,save):
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    start=np.array(baseline["raised_start"]["joints_rad"],dtype=np.float32)
    attempts=[]
    candidates=[tuned.NO_ROD_SELECTED_MPC]+[
        {**tuned.NO_ROD_SELECTED_MPC,"name":f"v3_no_rod_pose_{factor}",
         "non_terminal_tool_pose_weight_factor":factor} for factor in (.1,1.,.01,3.)]
    for settings in candidates:
        mpc,loaded=make_mpc(config,settings)
        try:
            mpc.setup(joint_state(start,np.zeros(6)))
            case=run_case(mpc,start,0.,False,"A",worker,baseline,duration=10.,no_rod=True,stop_at_goal=True)
        finally:
            mpc.destroy()
        summary=case["summary"]
        qualifies=bool(summary["time_to_first_goal_s"] is not None and summary["time_to_first_goal_s"]<=10.
                       and summary["within_joint_velocity_limits"] and summary["within_joint_acceleration_limits"]
                       and summary["joint_position_clamped_intervals"]==0)
        attempts.append(dict(settings=settings,loaded_config=loaded,qualifies=qualifies,case=compact_case(case)))
        if qualifies:
            result=dict(complete=True,selected_config=settings,attempts=attempts,
                        robot_config_sha256=hashlib.sha256(config.read_bytes()).hexdigest())
            if save: tuned.save_json(REFERENCE_FILE,result)
            print("V3_REFERENCE_PASSED "+json.dumps(summary),flush=True)
            return result
    raise RuntimeError("predeclared no-rod candidates failed; rod measurement forbidden")


def summarize(results):
    rows,table=[],[]
    for speed in rates.SPEEDS:
        for phase in rates.PHASES:
            selected=[c for c in results["cases"].values() if c["summary"]["rod_speed_mps"]==speed
                      and c["summary"]["initial_x_m"]==phase]
            valid=any(c["summary"]["true_collisions"]>0 for c in selected if c["summary"]["mode"]=="A")
            status="VALID_SCENE" if valid else "INVALID_SCENE"
            for c in selected: c["summary"]["scene_status"]=status
            rows.append(dict(speed_mps=speed,initial_x_m=phase,status=status))
        for mode in ("A","B","E"):
            selected=[c for c in results["cases"].values() if c["summary"]["rod_speed_mps"]==speed
                      and c["summary"]["mode"]==mode and c["summary"]["scene_status"]=="VALID_SCENE"]
            ss=[c["summary"] for c in selected]
            measured=[cy["measured_compute_ms"] for c in selected for cy in c["cycles"]]
            clearances=[s["minimum_true_clearance_m"] for s in ss]
            table.append(dict(mode=mode,speed_mps=speed,total_runs=len(ss),
                successes=sum(s["survives"] for s in ss),collision_runs=sum(s["true_collisions"]>0 for s in ss),
                worst_clearance_m=min(clearances) if ss else None,
                median_worst_clearance_m=float(np.median(clearances)) if ss else None,
                max_joint_speed_radps=max(max(s["max_joint_velocity_radps"]) for s in ss) if ss else None,
                max_acceleration_radps2=max(max(s["max_sampled_joint_acceleration_radps2"]) for s in ss) if ss else None,
                limits_pass=all(s["within_joint_velocity_limits"] and s["within_joint_acceleration_limits"]
                                and s["joint_position_clamped_intervals"]==0 for s in ss),
                real_compute_median_ms=float(np.median(measured)) if measured else None,
                real_compute_p95_ms=float(np.percentile(measured,95)) if measured else None))
    results["per_phase"],results["rates"]=rows,table


def build(config,worker,save):
    ref=json.loads(REFERENCE_FILE.read_text())
    if not ref["complete"] or ref["robot_config_sha256"]!=hashlib.sha256(config.read_bytes()).hexdigest():
        raise RuntimeError("run v3 no-rod reference first with this robot config")
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    start=np.array(baseline["raised_start"]["joints_rad"],dtype=np.float32)
    results=dict(schema=3,date="2026-11-13",complete=False,frozen_config=ref["selected_config"],
        reference_sha256=hashlib.sha256(REFERENCE_FILE.read_bytes()).hexdigest(),
        rod=baseline["rod"],raised_start=baseline["raised_start"],phases_m=rates.PHASES,speeds_mps=rates.SPEEDS,
        repeats=REPEATS,duration_per_case_s=20.,control_period_s=CONTROL_DT,counted_compute_s=CONTROL_DT,
        observation_hz=OBS_HZ,observation_rule="exact n/30 sim-time poses; consume latest frame at control ticks; hold fit/velocity between frames",
        prediction_rule="single velocity sweep over full MPC horizon plus 1/30 s; no old factor of two",
        plant="held joint velocity; command delta bounded by config acceleration times 25 ms; ideal discrete plant",
        case_order="speed, phase, repeat 1..5, A only repeat 1, B/E every repeat; fresh MPC instance each run",
        storage=dict(float_decimal_places=8,trace="exact control-boundary states; dense collision scoring unchanged; linear held-velocity interpolation",
                     fit="center, dimensions, point count; repeated renderer metadata omitted"),cases={})
    for speed in rates.SPEEDS:
        for phase in rates.PHASES:
            v2.ROD_START_X_M=phase
            for repeat in range(1,REPEATS+1):
                for mode in (("A","B","E") if repeat==1 else ("B","E")):
                    if mode=="E" and worker.process is None:
                        worker.start();worker.capture(start,phase)
                    name=f"v3 {mode} {speed:.2f} m/s X={phase:.2f} repeat={repeat}"
                    mpc,loaded=make_mpc(config,ref["selected_config"])
                    results["loaded_config"]=loaded
                    try:
                        mpc.setup(joint_state(start,np.zeros(6)))
                        case=run_case(mpc,start,speed,True,mode,worker,baseline)
                    finally:
                        mpc.destroy()
                    case["summary"].update(initial_x_m=phase,repeat=repeat)
                    print(f"V3_CASE_DONE {name} success={case['summary']['survives']} collisions={case['summary']['true_collisions']}",flush=True)
                    results["cases"][name]=compact_case(case)
                    if save:tuned.save_json(RESULT_FILE,results)
    summarize(results);results["complete"]=True
    return results


def check_viewer_port(port):
    """Fail before expensive offline rendering when the requested port is occupied."""
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1",port))
    except OSError as error:
        raise RuntimeError(f"viewer port {port} unavailable; choose NEWTON_VIEWER_PORT") from error


def main():
    parser=argparse.ArgumentParser()
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument("--reference",action="store_true")
    modes.add_argument("--replay",action="store_true")
    modes.add_argument("--interactive",action="store_true")
    parser.add_argument("--interactive-smoke",action="store_true")
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--save-results",action="store_true")
    parser.add_argument("--duration",type=float,default=0.)
    parser.add_argument("--port",type=int,default=int(os.environ.get("NEWTON_VIEWER_PORT","30000")))
    parser.add_argument("--robot-config",type=Path,default=Path.home()/".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml")
    parser.add_argument("--newton-python",type=Path,default=Path.home()/"newton_ws/.venv-cpu/bin/python")
    args=parser.parse_args()
    if (args.replay or args.interactive) and args.save_results:parser.error("view modes cannot write results")
    if (args.replay or args.interactive) and args.headless:parser.error("view modes require a viewer")
    if args.interactive_smoke and not args.interactive:parser.error("smoke requires interactive")
    if not args.headless and not args.reference:check_viewer_port(args.port)
    if args.interactive:
        from curobo_moving_rod_v3_live import live_main
        return live_main(args)
    viewer=None
    with tempfile.TemporaryDirectory(prefix="rod-v3-") as temp:
        worker=v2.MovingRodWorker(args.newton_python,Path(temp))
        try:
            if args.reference:
                reference(args.robot_config,worker,args.save_results)
                return 0
            results=json.loads(RESULT_FILE.read_text()) if args.replay else build(args.robot_config,worker,args.save_results)
            if not results["complete"]:raise RuntimeError("incomplete v3 results")
            if args.save_results:
                tuned.save_json(RESULT_FILE,results)
                RESULT_FILE.with_suffix(".log").write_text(json.dumps({k:results[k] for k in ("frozen_config","loaded_config","rates","per_phase")},indent=2)+"\n")
                print("V3_RESULTS_SAVED "+json.dumps(results["rates"]),flush=True)
            if not args.headless:
                from curobo_moving_rod_v3_viewer import start_viewer
                tuned.original.configure_scene(results)
                frames=rates.render_frames(worker,results)
                worker.close()
                viewer,report=start_viewer(args.robot_config,results,frames,args.port)
                if not report["all_passed"]:raise RuntimeError("v3 replay verification failed")
                began=time.monotonic()
                while args.duration<=0 or time.monotonic()-began<args.duration:time.sleep(.25)
        finally:
            worker.close()
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:thread.join(timeout=5)
                viewer._server.stop()
    return 0


if __name__=="__main__":raise SystemExit(main())
