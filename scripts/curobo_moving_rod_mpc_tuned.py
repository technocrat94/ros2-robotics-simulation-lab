#!/usr/bin/env python3
"""Tune MPC on a rod-free reference before testing a frozen controller on the rod."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any

import numpy as np
import torch
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg

import curobo_moving_rod_mpc as original
from curobo_moving_rod_mpc import CONTROL_DT, OPTIMIZATION_DT, integrate_command, joint_state, command_index, update_box

v2 = original.v2
RESULT_FILE = v2.RESULTS_DIR / "moving_rod_mpc_tuned.json"
REFERENCE_FILE = v2.RESULTS_DIR / "moving_rod_mpc_tuned.reference.json"
DEFAULT_SETTINGS = {"acceleration_regularization": 10000., "non_terminal_tool_pose_weight_factor": .001}
# Frozen only after the no-rod reference passes; rod results never select this block.
NO_ROD_SELECTED_MPC = {
    "name": "mpc_no_rod_dt_0.02_nonterminal_0.3",
    "acceleration_regularization": 10000.,
    "non_terminal_tool_pose_weight_factor": .3,
    "optimization_dt": .02,
}


def make_mpc(config: Path, settings: dict[str, Any]) -> tuple[ModelPredictiveControl, dict[str, Any]]:
    """Build a private in-memory config, preserving all original controller settings."""
    robot_cfg, wrist = v2.sixth.limited_robot_cfg(config, v2.sixth.CAMERA_FRONT_SEED)
    cfg = ModelPredictiveControlCfg.create(
        robot=robot_cfg, scene_model={"cuboid": {"camera_obstacle": {
            "dims": [.01,.01,.01], "pose": [0.,0.,-10.,1.,0.,0.,0.]}}},
        optimization_dt=settings.get("optimization_dt", OPTIMIZATION_DT), interpolation_steps=4, use_cuda_graph=True,
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
    return ModelPredictiveControl(cfg), report


def box_error(fit: dict[str, Any] | None, truth: dict[str, Any]) -> dict[str, Any]:
    """Score perceived-box shortfall without using truth in the E controller."""
    if fit is None:
        return {"missing_box": True, "missing_height_m": truth["dims_m"][2],
                "missing_width_xy_m": truth["dims_m"][:2], "center_offset_xyz_m": None,
                "center_offset_norm_m": None, "missing_top_m": truth["dims_m"][2],
                "true_center_m": truth["center_m"], "true_dims_m": truth["dims_m"]}
    c, d = np.asarray(fit["center_m"]), np.asarray(fit["dims_m"])
    tc, td = np.asarray(truth["center_m"]), np.asarray(truth["dims_m"])
    lo, hi, tl, th = c-d/2, c+d/2, tc-td/2, tc+td/2
    overlap = np.maximum(0., np.minimum(hi, th)-np.maximum(lo, tl))
    return {"missing_box": False, "missing_height_m": float(max(0., td[2]-overlap[2])),
            "missing_width_xy_m": np.maximum(0., td[:2]-overlap[:2]).tolist(),
            "missing_top_m": float(max(0., th[2]-hi[2])),
            "center_offset_xyz_m": (c-tc).tolist(), "center_offset_norm_m": float(np.linalg.norm(c-tc)),
            "observed_center_m": c.tolist(), "observed_dims_m": d.tolist(),
            "true_center_m": tc.tolist(), "true_dims_m": td.tolist()}


def worst_frames(frames: list[dict[str, Any]]) -> dict[str, Any]:
    """Keep worst height, width and center-offset observations and missing-frame count."""
    if not frames:
        return {}
    return {"missing_box_frames": sum(f["missing_box"] for f in frames),
            "height": max(frames, key=lambda f: f["missing_height_m"]),
            "width": max(frames, key=lambda f: max(f["missing_width_xy_m"])),
            "offset": max(frames, key=lambda f: f["center_offset_norm_m"] or 0.)}


def run_case(mpc: ModelPredictiveControl, start: np.ndarray, speed: float,
             prediction: bool, mode: str, worker: v2.MovingRodWorker,
             baseline: dict[str, Any], duration: float = 20.,
             no_rod: bool = False, stop_at_goal: bool = False) -> dict[str, Any]:
    """Run an ideal joint-velocity plant while previous commands execute during compute."""
    q = start.copy()
    velocity = np.zeros(6)
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
    expected_compute = .025
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
        estimated_velocity = np.zeros(3)
        if no_rod:
            fit = None
        elif mode == "B":
            fit = v2.exact_rod_fit(x)
            estimated_velocity[0] = (v2.rod_x_at(clock+.001, speed, v2.ROD_START_X_M, -1.)-x)/.001
        else:
            capture = worker.capture(q, x).camera
            points, _ = cloud.update(capture, clock)
            fit, _ = v2.sixth.fit_with_unknown_extent(points, capture, fixed_top)
            fresh, _ = v2.sixth.fit_with_unknown_extent(capture.points, capture, fixed_top)
            if fresh is not None:
                center = np.asarray(fresh["center_m"])
                if previous_frame is not None and clock > previous_frame[0]:
                    estimated_velocity = (center-previous_frame[1])/(clock-previous_frame[0])
                previous_frame = (clock, center)
        predicted = v2.predicted_fit(fit, estimated_velocity, expected_compute + CONTROL_DT) if prediction else fit
        update_box(mpc, predicted)
        state = joint_state(snapshot_q, velocity)
        torch.cuda.synchronize()
        solve_began = time.perf_counter()
        result = mpc.optimize_action_sequence(state)
        torch.cuda.synchronize()
        solve_s = time.perf_counter()-solve_began
        command_sequence = result.action_sequence.velocity.detach().cpu().numpy().reshape(-1, 6)
        feasible = bool(result.success.all())
        compute_s = time.perf_counter()-began
        # Keep executing the OLD command throughout sensing and optimization.
        previous_velocity = velocity.copy()
        advance(min(compute_s, duration-clock))
        q_after_compute = q.copy()
        index = min(max(1, int(math.ceil(compute_s / (mpc.config.optimization_dt/4)))), len(command_sequence)-1)
        proposed = command_sequence[index]
        finite = bool(np.isfinite(proposed).all())
        if not finite:
            raise RuntimeError("non-finite MPC velocity")
        max_proposed_velocity = np.maximum(max_proposed_velocity, np.abs(proposed))
        velocity_clip_steps += int(np.any(proposed < velocity_limits[0]-1e-6) or np.any(proposed > velocity_limits[1]+1e-6))
        velocity = np.clip(proposed, velocity_limits[0], velocity_limits[1]) if finite else np.zeros(6)
        velocity = np.where(((q <= position_limits[0]) & (velocity < 0)) |
                            ((q >= position_limits[1]) & (velocity > 0)), 0, velocity)
        # Finite infeasible MPC actions are intentionally executed in this simulation.
        # Gating on feasibility would recreate the stop-and-go freeze under test.
        if not feasible:
            infeasible += 1
        advance(min(max(0., CONTROL_DT-compute_s), duration-clock))
        tool = v2.sixth.fourth.tool_position(mpc.kinematics, torch.as_tensor(q, device="cuda", dtype=torch.float32))
        target_error = float(np.linalg.norm(tool-target_points[target]))
        distance_curve.append({"time_s": clock, "distance_m": float(np.linalg.norm(tool-target_points["goal"]))})
        max_velocity = np.maximum(max_velocity, np.maximum(np.abs(previous_velocity), np.abs(velocity)))
        period = max(clock-captured_time, 1e-9)
        max_acceleration = np.maximum(max_acceleration, np.abs(velocity-previous_velocity)/period)
        reached = bool(target_error <= v2.sixth.GOAL_TOLERANCE_M)
        if mode == "E":
            frame_errors.append({"step": step, "time_s": captured_time,
                                 **box_error(fit, v2.exact_rod_fit(x))})
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
            "compute_counted_s": min(compute_s, duration-captured_time),
            "q_at_capture_rad": snapshot_q.tolist(),
            "q_after_compute_rad": q_after_compute.tolist(),
            "previous_velocity_radps": previous_velocity.tolist(),
            "new_velocity_radps": velocity.tolist(), "command_index_after_latency": index,
            "feasible": feasible, "finite": finite,
            "fit": v2.compact_fit(fit), "predicted_fit": v2.compact_fit(predicted),
            "velocity_estimate_mps": estimated_velocity.tolist(),
            "prediction_horizon_s": expected_compute+CONTROL_DT if prediction else 0.,
        })
        expected_compute = compute_s
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
    }
    print("MOVING_ROD_MPC_CASE " + json.dumps(summary), flush=True)
    return {"summary": summary, "cycles": cycles, "trace": traces,
            "distance_to_goal": distance_curve, "camera_frame_errors": frame_errors}


def motion_planner_reference(config: Path, baseline: dict[str, Any]) -> dict[str, Any]:
    """Measure the no-obstacle v2 path on its own 25 ms trajectory timing."""
    planner, _ = v2.sixth.make_loop_planner(config, v2.sixth.CAMERA_FRONT_SEED)
    start = np.asarray(baseline["raised_start"]["joints_rad"], dtype=np.float32)
    goal = v2.sixth.goal_from_scene()
    try:
        began = time.perf_counter()
        result = planner.plan_pose(goal, v2.sixth.arm_state(start))
        torch.cuda.synchronize()
        planning_s = time.perf_counter()-began
        positions = v2.sixth.fourth.active_trajectory(result)
        if positions is None:
            raise RuntimeError("no-rod MotionPlanner reference failed")
        tools = v2.sixth.fourth.tool_points(planner.kinematics, positions)
        target = goal.position.detach().cpu().numpy().reshape(-1,3)[0]
        distances = np.linalg.norm(tools-target, axis=1)
        indices = np.flatnonzero(distances <= .006)
        if not len(indices):
            raise RuntimeError("MotionPlanner reference never enters 6 mm")
        times = np.arange(len(positions))*CONTROL_DT
        return {"time_to_goal_s": float(times[indices[0]]), "planning_s_excluded": planning_s,
                "path_duration_s": float(times[-1]), "interpolation_dt_s": CONTROL_DT,
                "distance_to_goal": [{"time_s": float(t), "distance_m": float(d)} for t,d in zip(times,distances)],
                "q": positions.detach().cpu().numpy().tolist()}
    finally:
        planner.destroy()


def single_reference(config: Path, baseline: dict[str, Any], settings: dict[str, Any],
                     duration: float) -> dict[str, Any]:
    """Evaluate settings with obstacles disabled; never look at rod outcomes."""
    start = np.array(baseline["raised_start"]["joints_rad"], dtype=np.float32)
    mpc, loaded = make_mpc(config, settings)
    try:
        mpc.setup(joint_state(start, np.zeros(6)))
        result = run_case(mpc, start, .10, False, "N", None, baseline,
                          duration=duration, no_rod=True, stop_at_goal=True)
        result["settings"] = settings
        result["loaded_config"] = loaded
        return result
    finally:
        mpc.destroy()


def qualifies_reference(summary: dict[str, Any], deadline: float) -> bool:
    """Require a measured reach before the deadline and bounded joint velocity."""
    reach = summary["time_to_first_goal_s"]
    return reach is not None and reach <= deadline and summary["within_joint_velocity_limits"]


def settings_name(settings: dict[str, Any]) -> str:
    """Name a candidate solely from its no-rod settings."""
    if "optimization_dt" in settings and settings["non_terminal_tool_pose_weight_factor"] != .001:
        return f"mpc_no_rod_dt_{settings['optimization_dt']:g}_nonterminal_{settings['non_terminal_tool_pose_weight_factor']:g}"
    for key in ("optimization_dt", "acceleration_regularization", "non_terminal_tool_pose_weight_factor"):
        if settings.get(key, DEFAULT_SETTINGS.get(key)) != DEFAULT_SETTINGS.get(key):
            return f"mpc_no_rod_{key}_{settings[key]:g}"
    return "mpc_original"


def reference_experiment(config: Path, baseline: dict[str, Any]) -> dict[str, Any]:
    """Select the first no-rod-qualified candidate, trying single changes before pairs."""
    reference = motion_planner_reference(config, baseline)
    print("TUNED_MOTION_REFERENCE " + json.dumps(reference), flush=True)
    original_case = single_reference(config, baseline, DEFAULT_SETTINGS, 40.)
    deadline = 2*reference["time_to_goal_s"]
    attempts = []
    selected = None
    candidates = [("acceleration_regularization", value) for value in (1000., 100., 10., 1.)]
    candidates += [("non_terminal_tool_pose_weight_factor", value) for value in (.01, .1, 1., 10., 2., 3., 5.)]
    candidates += [("optimization_dt", value) for value in (.03, .02, .05, .04, .01)]
    candidate_settings = [{**DEFAULT_SETTINGS, key: value} for key, value in candidates]
    candidate_settings += [{**DEFAULT_SETTINGS, "optimization_dt": .02,
                            "non_terminal_tool_pose_weight_factor": value}
                           for value in (.1, .01, .3, 1.)]
    for settings in candidate_settings:
        case = single_reference(config, baseline, settings, deadline+CONTROL_DT)
        s = case["summary"]
        passed = qualifies_reference(s, deadline)
        case["passes_reference_gate"] = passed
        attempts.append(case)
        print("TUNED_CANDIDATE " + json.dumps({"settings": settings, "passed": passed,
              "time_to_goal_s": s["time_to_first_goal_s"], "final_distance_m": case["distance_to_goal"][-1]["distance_m"]}), flush=True)
        if passed:
            selected = {"name": settings_name(settings), **settings}
            break
    results = {"schema": 1, "date": "2026-11-12", "selection_used_rod": False,
               "motion_planner": reference, "original_mpc": original_case,
               "deadline_s": deadline, "attempts": attempts, "selected_config": selected}
    return results


def build_results(config: Path, baseline: dict[str, Any], worker: Any) -> dict[str, Any]:
    """Retest a frozen no-rod-qualified controller, then conditionally measure E."""
    reference = json.loads(REFERENCE_FILE.read_text())
    if reference["selected_config"] != NO_ROD_SELECTED_MPC:
        raise RuntimeError("Frozen config must equal the no-rod-selected config before rod testing")
    start = np.array(baseline["raised_start"]["joints_rad"], dtype=np.float32)
    mpc, loaded = make_mpc(config, NO_ROD_SELECTED_MPC)
    try:
        mpc.setup(joint_state(start, np.zeros(6)))
        cases = {}
        for speed in v2.ROD_SPEEDS_MPS:
            for prediction in (False, True):
                name = f"Tuned B {'prediction' if prediction else 'current box'} {speed:.2f} m/s"
                cases[name] = run_case(mpc, start, speed, prediction, "B", worker, baseline)
        survivors = [c["summary"] for c in cases.values() if c["summary"]["survives"]]
        gate = any(s["rod_speed_mps"] == .10 for s in survivors)
        camera_cases = []
        if gate:
            for speed in sorted(set(s["rod_speed_mps"] for s in survivors)):
                # Prefer the prediction variant when it survives at this speed.
                prediction = any(s["rod_speed_mps"] == speed and s["prediction"] for s in survivors)
                name = f"Tuned E {'prediction' if prediction else 'current box'} {speed:.2f} m/s"
                cases[name] = run_case(mpc, start, speed, prediction, "E", worker, baseline)
                camera_cases.append(name)
        return {"schema": 1, "date": "2026-11-12", "frozen_config": NO_ROD_SELECTED_MPC,
                "reference_sha256": hashlib.sha256(REFERENCE_FILE.read_bytes()).hexdigest(),
                "no_rod_reference": reference,
                "loaded_config": loaded, "rod": baseline["rod"], "raised_start": baseline["raised_start"],
                "v2_b_comparison": {n:c["summary"] for n,c in baseline["cases"].items() if n.startswith("B ")},
                "camera_gate_0.10_passed": gate, "camera_cases": camera_cases, "cases": cases}
    finally:
        mpc.destroy()


def save_json(path: Path, results: dict[str, Any]) -> None:
    """Store rich machine evidence compactly; human summaries live in the log."""
    # Machine evidence is compact; human-readable summaries live in the log.
    path.write_text(json.dumps(results, separators=(",", ":"))+"\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--save-results", action="store_true")
    parser.add_argument("--duration", type=float, default=0.)
    parser.add_argument("--port", type=int, default=int(os.environ.get("NEWTON_VIEWER_PORT", "30000")))
    parser.add_argument("--robot-config", type=Path, default=Path.home()/".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml")
    parser.add_argument("--newton-python", type=Path, default=Path.home()/"newton_ws/.venv-cpu/bin/python")
    args = parser.parse_args()
    if args.replay and (args.save_results or args.reference):
        parser.error("--replay is read-only and cannot be combined with --save-results or --reference")
    baseline = json.loads(original.BASELINE_FILE.read_text())
    original.configure_scene(baseline)
    if args.reference:
        results = reference_experiment(args.robot_config, baseline)
        if args.save_results:
            save_json(REFERENCE_FILE, results)
        print("TUNED_REFERENCE_SELECTED " + json.dumps(results["selected_config"]), flush=True)
        return 0 if results["selected_config"] is not None else 2
    viewer = None
    results = None
    with tempfile.TemporaryDirectory(prefix="moving-rod-mpc-tuned-") as temp:
        worker = v2.MovingRodWorker(args.newton_python, Path(temp))
        try:
            results = json.loads(RESULT_FILE.read_text()) if args.replay else build_results(args.robot_config, baseline, worker)
            if not args.headless:
                frames = original.render_frames(worker, results)
                viewer, report = original.start_viewer(args.robot_config, results, frames, args.port)
                results["playback_verification"] = report
                if not report["all_passed"]:
                    raise RuntimeError("tuned viewer callback verification failed")
                started = time.monotonic()
                while args.duration <= 0 or time.monotonic()-started < args.duration:
                    time.sleep(.25)
        except KeyboardInterrupt:
            pass
        finally:
            worker.close()
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:
                    thread.join(timeout=5)
                viewer._server.stop()
    if results is None:
        return 130
    if args.save_results:
        save_json(RESULT_FILE, results)
        RESULT_FILE.with_suffix(".log").write_text(json.dumps({
            "frozen_config": results["frozen_config"], "camera_cases": results["camera_cases"],
            "summaries": {n:c["summary"] for n,c in results["cases"].items()}}, indent=2)+"\n")
        print("TUNED_RESULTS_SAVED", flush=True)
    else:
        print("TUNED_RESULTS_NOT_SAVED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
