#!/usr/bin/env python3
"""Fixed-phase A/B/E success rates with the frozen no-rod-qualified MPC."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Any

import numpy as np
import torch
from curobo.model_predictive_control import ModelPredictiveControl
from curobo.types import ContentPath
from curobo.viewer import ViserVisualizer

import curobo_moving_rod_mpc_tuned as tuned
from curobo_moving_rod_mpc import CONTROL_DT, integrate_command, joint_state, update_box
from curobo_moving_rod_mpc_tuned import box_error, worst_frames

v2 = tuned.v2
DURATION = 20.
PHASES = (.36, .44, .52, .60, .68)
SPEEDS = (.05, .10)
RESULT_FILE = v2.RESULTS_DIR / "moving_rod_rates.json"
REPORT_FILE = v2.REPO / "docs/SIM_CAMERA_MOVING_ROD_REPORT.zh-TW.md"


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
        if no_rod or mode == "A":
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


def start_viewer(config: Path, results: dict[str, Any], frames: dict[str, list[dict[str, Any]]],
                 port: int) -> tuple[Any, dict[str, Any]]:
    """Replay selectable MPC cases without writing files."""
    viewer = ViserVisualizer(content_path=ContentPath(robot_config_file=str(config)),
                             connect_ip="127.0.0.1", connect_port=port,
                             add_control_frames=True, visualize_robot_spheres=False, add_robot_to_scene=True)
    server = viewer._server
    if int(server._websock_server._port) != port:
        server.stop()
        raise RuntimeError(f"requested port {port} occupied; refusing alternate binding")
    rod = server.scene.add_cylinder("/truth/rod", radius=v2.ROD_RADIUS_M,
                                   height=v2.ROD_TOP_M, color=(235,55,45))
    points = server.scene.add_point_cloud("/camera/points", points=np.zeros((1,3)),
                                         colors=(220,0,220), point_size=.011)
    observed = server.scene.add_box("/observed/box", dimensions=(.01,.01,.01), color=(255,145,35), opacity=.42)
    predicted = server.scene.add_box("/predicted/box", dimensions=(.01,.01,.01), color=(80,150,255), opacity=.30)
    first = next(iter(frames))
    panel = server.gui.add_image(frames[first][0]["depth"], label="Live simulated depth")
    index = {(c["summary"]["mode"], c["summary"]["rod_speed_mps"], c["summary"]["initial_x_m"]): n
             for n,c in results["cases"].items()}
    group_select = server.gui.add_dropdown("Case", options=("A", "B", "E"), initial_value="A")
    speed_select = server.gui.add_dropdown("Rod speed m/s", options=("0.05", "0.10"), initial_value="0.05")
    phase_select = server.gui.add_dropdown("Initial X m", options=tuple(f"{x:.2f}" for x in PHASES), initial_value="0.36")
    validity = server.gui.add_text("Phase validity", "", disabled=True)

    def selected_name() -> str:
        return index[(group_select.value, float(speed_select.value), float(phase_select.value))]
    controller = server.gui.add_text("Controller", "cuRobo MPC", disabled=True)
    rate = server.gui.add_text("Achieved step rate", "", disabled=True)
    clock_text = server.gui.add_text("Sim time", "0", disabled=True)
    clearance = server.gui.add_text("Minimum true clearance", "", disabled=True)
    error = server.gui.add_text("Playback error", "none", disabled=True)
    play = server.gui.add_button("Play selected case")
    mount = v2.sixth.fourth.read_mount_config(v2.MOUNT_CONFIG)
    # Use a fresh kinematics object so the controller is destroyed before playback.
    robot = v2.Kinematics(v2.KinematicsCfg.from_robot_yaml_file(
        str(config), tool_frames=[v2.sixth.preflight.TOOL_FRAME]))
    frustum = server.scene.add_camera_frustum("/camera/live", fov=v2.DEPTH_FOV_RAD,
                                             aspect=v2.DEPTH_ASPECT, scale=.13)
    busy = threading.Lock()
    stop_playback = threading.Event()
    workers = []

    def show(name: str, frame: dict[str, Any]) -> None:
        summary = results["cases"][name]["summary"]
        controller.value = summary["controller"]
        validity.value = summary["scene_status"]
        rate.value = f"{summary['achieved_step_rate_hz']:.2f} Hz"
        clock_text.value = f"{frame['time_s']:.3f} s"
        clearance.value = f"{summary['minimum_true_clearance_m']:+.4f} m"
        q = np.array(frame["q"], dtype=np.float32)
        viewer.set_joint_state(v2.sixth.arm_state(q).squeeze(0))
        pos, quat = v2.sixth.fourth.camera_pose_for_q(robot, torch.as_tensor(q, device="cuda"), mount)
        frustum.position, frustum.wxyz = tuple(pos), tuple(quat)
        rod.position = (frame["rod_x"], v2.ROD_Y_M, v2.ROD_CENTER_Z_M)
        points.points = frame["points"] if len(frame["points"]) else np.zeros((1,3))
        panel.image = frame["depth"]
        for box, fit in ((observed, frame["fit"]), (predicted, frame["predicted_fit"])):
            box.visible = fit is not None
            if fit:
                box.position, box.dimensions = tuple(fit["center_m"]), tuple(fit["dims_m"])
        # Show only one box when prediction is disabled, avoiding coincident surfaces.
        predicted.visible = summary["prediction"] and frame["predicted_fit"] is not None

    def playback(name: str, fast: bool = False) -> dict[str, Any]:
        with busy:
            try:
                previous_time = 0.
                frame_index = 0
                for sample in results["cases"][name]["trace"]:
                    if stop_playback.is_set():
                        break
                    t = sample["time_s"]
                    while frame_index+1 < len(frames[name]) and frames[name][frame_index+1]["time_s"] <= t:
                        frame_index += 1
                    frame = {**frames[name][frame_index], **sample,
                             "rod_x": v2.rod_x_at(t, results["cases"][name]["summary"]["rod_speed_mps"], results["cases"][name]["summary"]["initial_x_m"], -1.)}
                    if not fast:
                        time.sleep(max(0., t-previous_time))
                    show(name, frame)
                    previous_time = t
                return {"case": name, "error": None,
                        "callback_non_main": threading.current_thread() is not threading.main_thread(),
                        "robot_changed": any(not np.allclose(frames[name][0]["q"], f["q"]) for f in frames[name][1:]),
                        "rod_changed": bool(np.ptp([f["rod_x"] for f in frames[name]]) > 0),
                        "depth_updates": len(frames[name]), "predicted_box_available": any(f["predicted_fit"] for f in frames[name])}
            except Exception as exc:
                error.value = f"{type(exc).__name__}: {exc}"
                return {"case": name, "error": error.value}

    def click(_event: Any) -> None:
        thread = threading.Thread(target=playback, args=(selected_name(),), daemon=True)
        workers.append(thread)
        thread.start()

    play.on_click(click)
    def filter_changed(_event: Any) -> None:
        name = selected_name()
        show(name, frames[name][0])
    for handle in (group_select, speed_select, phase_select):
        handle.on_update(filter_changed)
    show(first, frames[first][0])
    verification = []
    for mode in ("A", "B", "E"):
        group_select.value, speed_select.value, phase_select.value = mode, "0.10", "0.60"
        name = selected_name()
        output = []
        thread = threading.Thread(target=lambda n=name: output.append(playback(n, fast=True)))
        thread.start()
        thread.join(timeout=30)
        if thread.is_alive():
            server.stop()
            raise RuntimeError("MPC playback verification timed out")
        verification.extend(output)
    group_select.value, speed_select.value, phase_select.value = "A", "0.05", "0.36"
    show(first, frames[first][0])
    report = {"cases": verification, "all_passed": all(
        c.get("error") is None and c.get("callback_non_main") and c.get("robot_changed") and c.get("rod_changed")
        for c in verification)}
    print("MOVING_ROD_RATES_VIEWER " + json.dumps(report), flush=True)
    print(f"MOVING_ROD_RATES_URL http://127.0.0.1:{port}", flush=True)
    # Main owns the server and only the playback threads started here.
    viewer._mpc_workers = workers
    viewer._mpc_stop_event = stop_playback
    return viewer, report


def summarize(results: dict[str, Any]) -> None:
    """Exclude only A-noncolliding phases and retain every measured phase row."""
    cases = results["cases"]
    rows = []
    for speed in SPEEDS:
        for phase in PHASES:
            group = {m: next(c for c in cases.values() if c["summary"]["mode"] == m
                            and c["summary"]["rod_speed_mps"] == speed
                            and c["summary"]["initial_x_m"] == phase) for m in ("A","B","E")}
            valid = group["A"]["summary"]["true_collisions"] > 0
            status = "VALID_SCENE" if valid else "INVALID_SCENE"
            for c in group.values():
                c["summary"]["scene_status"] = status
            rows.append({"speed_mps": speed, "initial_x_m": phase, "status": status,
                         "cases": {m:c["summary"] for m,c in group.items()}})
    table = []
    for speed in SPEEDS:
        valid_rows = [r for r in rows if r["speed_mps"] == speed and r["status"] == "VALID_SCENE"]
        for mode in ("A","B","E"):
            ss = [r["cases"][mode] for r in valid_rows]
            successes = sum(s["true_collisions"] == 0 and s["reaches"] > 0 for s in ss)
            table.append({"mode": mode, "speed_mps": speed, "valid_phases": len(ss),
                          "invalid_phases": len(PHASES)-len(ss), "successes": successes,
                          "success_rate": successes/len(ss) if ss else None,
                          "mean_reaches": float(np.mean([s["reaches"] for s in ss])) if ss else None,
                          "collision_episodes": sum(s["true_collisions"] for s in ss) if ss else None,
                          "worst_clearance_m": min(s["minimum_true_clearance_m"] for s in ss) if ss else None,
                          "max_joint_speed_radps": max(max(s["max_joint_velocity_radps"]) for s in ss) if ss else None,
                          "max_sampled_acceleration_radps2": max(max(s["max_sampled_joint_acceleration_radps2"]) for s in ss) if ss else None})
    frames = []
    for name,c in cases.items():
        s = c["summary"]
        if s["mode"] == "E":
            for f in c["camera_frame_errors"]:
                frames.append({**f, "case": name, "speed_mps": s["rod_speed_mps"],
                               "initial_x_m": s["initial_x_m"], "scene_status": s["scene_status"]})
    def rank(f: dict[str, Any]) -> tuple[bool, float]:
        return (f["missing_box"], f["missing_height_m"]+sum(f["missing_width_xy_m"])+(f["center_offset_norm_m"] or 0.))
    results["per_phase"] = rows
    results["rates"] = table
    results["worst_E_frames_all"] = sorted(frames, key=rank, reverse=True)[:3]
    results["worst_E_frames_valid"] = sorted([f for f in frames if f["scene_status"] == "VALID_SCENE"], key=rank, reverse=True)[:3]
    results["E_frame_count"] = len(frames)
    results["worst_frame_ranking"] = "missing box first, then missing height + missing X/Y coverage + center-offset norm (metres); stable chronological ties"


def build_rates(config: Path, worker: v2.MovingRodWorker, checkpoint: bool) -> dict[str, Any]:
    """Run the predeclared 30 cases without selecting or changing settings."""
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    reference = json.loads(tuned.REFERENCE_FILE.read_text())
    if reference["selected_config"] != tuned.NO_ROD_SELECTED_MPC:
        raise RuntimeError("frozen config differs from accepted no-rod qualification")
    tuned.original.configure_scene(baseline)
    start = np.array(baseline["raised_start"]["joints_rad"], dtype=np.float32)
    mpc, loaded = tuned.make_mpc(config, tuned.NO_ROD_SELECTED_MPC)
    results = {"schema": 1, "date": "2026-11-12", "complete": False,
               "frozen_config": tuned.NO_ROD_SELECTED_MPC, "loaded_config": loaded,
               "reference_sha256": hashlib.sha256(tuned.REFERENCE_FILE.read_bytes()).hexdigest(),
               "rod": baseline["rod"], "raised_start": baseline["raised_start"],
               "phases_m": list(PHASES), "speeds_mps": list(SPEEDS), "initial_direction": -1,
               "duration_per_case_s": DURATION, "A_controller": "same frozen MPC; obstacle disabled; truth still scored",
               "E_gate": "old B-at-0.10 gate explicitly lifted for this report; E at both speeds and all phases",
               "case_order": "speed 0.05 then 0.10; phases ascending; A then B then E",
               "clock": "unchanged continuous previous-command execution during measured compute",
               "camera_memory": "unchanged v2 TimedPointCloud, fresh-frame velocity, unknown extent rule",
               "cases": {}}
    try:
        mpc.setup(joint_state(start, np.zeros(6)))
        for speed in SPEEDS:
            for phase in PHASES:
                v2.ROD_START_X_M = phase
                for mode in ("A","B","E"):
                    if mode == "E" and worker.process is None:
                        worker.start()
                        worker.capture(start, phase)  # Renderer warmup is outside measured case time.
                    name = f"Rates {mode} {speed:.2f} m/s X={phase:.2f}"
                    print("RATES_CASE_START " + name, flush=True)
                    case = run_case(mpc, start, speed, mode != "A", mode, worker, baseline)
                    case["summary"]["initial_x_m"] = phase
                    results["cases"][name] = case
                    if checkpoint:
                        tuned.save_json(RESULT_FILE, results)
        summarize(results)
        results["complete"] = True
        return results
    finally:
        mpc.destroy()


def render_frames(worker: v2.MovingRodWorker, results: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    """Render only after rates are saved, using each case's actual initial phase."""
    frames = {}
    for name,case in results["cases"].items():
        v2.ROD_START_X_M = case["summary"]["initial_x_m"]
        frames.update(tuned.original.render_frames(worker, {"cases": {name:case}}))
        print("RATES_RENDERED " + name, flush=True)
    return frames


def save_results(results: dict[str, Any]) -> None:
    """Write the report's machine data and concise descriptive statistics."""
    tuned.save_json(RESULT_FILE, results)
    RESULT_FILE.with_suffix(".log").write_text(json.dumps({
        "frozen_config": results["frozen_config"], "E_gate": results["E_gate"],
        "rates": results["rates"], "per_phase": results["per_phase"],
        "worst_E_frames_all": results["worst_E_frames_all"],
        "worst_E_frames_valid": results["worst_E_frames_valid"]}, indent=2)+"\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-results", action="store_true")
    parser.add_argument("--duration", type=float, default=0.)
    parser.add_argument("--port", type=int, default=int(os.environ.get("NEWTON_VIEWER_PORT", "30000")))
    parser.add_argument("--robot-config", type=Path, default=Path.home()/".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml")
    parser.add_argument("--newton-python", type=Path, default=Path.home()/"newton_ws/.venv-cpu/bin/python")
    args = parser.parse_args()
    if args.replay and args.save_results:
        parser.error("--replay cannot write results")
    viewer = None
    with tempfile.TemporaryDirectory(prefix="moving-rod-rates-") as temp:
        worker = v2.MovingRodWorker(args.newton_python, Path(temp))
        try:
            results = json.loads(RESULT_FILE.read_text()) if args.replay else build_rates(args.robot_config, worker, args.save_results)
            if not results["complete"]:
                raise RuntimeError("incomplete checkpoint cannot be reported or replayed as rates")
            if args.save_results:
                save_results(results)
                print("RATES_RESULTS_SAVED", flush=True)
            if not args.headless:
                tuned.original.configure_scene(results)
                frames = render_frames(worker, results)
                worker.close()
                viewer, report = start_viewer(args.robot_config, results, frames, args.port)
                if not report["all_passed"]:
                    raise RuntimeError("rates viewer callback verification failed")
                began = time.monotonic()
                while args.duration <= 0 or time.monotonic()-began < args.duration:
                    time.sleep(.25)
        except KeyboardInterrupt:
            return 130
        finally:
            worker.close()
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:
                    thread.join(timeout=5)
                viewer._server.stop()
    if not args.save_results:
        print("RATES_RESULTS_NOT_SAVED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
