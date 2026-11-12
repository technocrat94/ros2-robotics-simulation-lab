#!/usr/bin/env python3
"""Compare continuous cuRobo MPC with the accepted moving-rod v2 baseline."""
from __future__ import annotations

import argparse
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
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer

import curobo_camera_moving_rod as v2

RESULT_FILE = v2.RESULTS_DIR / "moving_rod_mpc.json"
BASELINE_FILE = v2.RESULTS_DIR / "moving_rod_v2.json"
CONTROL_DT = 0.025
OPTIMIZATION_DT = CONTROL_DT * 4
DURATION = 20.0


def configure_scene(baseline: dict[str, Any]) -> None:
    """Use the accepted scene values in this process without editing v2."""
    rod = baseline["rod"]
    v2.ROD_Y_M = rod["y_m"]
    v2.ROD_TOP_M = rod["top_m"]
    v2.ROD_CENTER_Z_M = rod["top_m"] / 2
    v2.ROD_X_MIN_M, v2.ROD_X_MAX_M = rod["x_limits_m"]
    v2.ROD_START_X_M = rod["regular_initial_x_m"]


def integrate_command(q: np.ndarray, velocity: np.ndarray, duration: float,
                      limits: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Execute a held velocity without teleportation, sampling at most 25 ms."""
    if duration <= 0:
        return np.empty(0), np.empty((0, len(q)))
    count = max(1, int(math.ceil(duration / CONTROL_DT)))
    times = np.linspace(0, duration, count + 1)
    positions = np.clip(q[None, :] + times[:, None] * velocity[None, :],
                        limits[0], limits[1])
    return times, positions


def command_index(compute_s: float, count: int) -> int:
    """Skip the current-state boundary; choose the first non-stale velocity."""
    return min(max(1, int(math.ceil(compute_s / CONTROL_DT))), count-1)


def update_box(mpc: ModelPredictiveControl, fit: dict[str, Any] | None) -> None:
    """Update one preallocated cuboid in-place, keeping CUDA graph storage."""
    checker = mpc.scene_collision_checker
    if fit is None:
        checker.enable_obstacle("camera_obstacle", False)
        return
    checker.update_obstacle_pose("camera_obstacle", Pose.from_list(
        [*fit["center_m"], 1., 0., 0., 0.]))
    checker.data.cuboids.update_dims("camera_obstacle", torch.as_tensor(
        fit["dims_m"], device="cuda", dtype=torch.float32))
    checker.enable_obstacle("camera_obstacle", True)


def joint_state(q: np.ndarray, velocity: np.ndarray) -> JointState:
    """Return measured ideal velocity-plant feedback for MPC."""
    state = v2.sixth.arm_state(torch.as_tensor(q, device="cuda", dtype=torch.float32))
    state.velocity = torch.as_tensor(velocity, device="cuda", dtype=torch.float32).reshape(1, -1)
    state.acceleration = torch.zeros_like(state.position)
    return state


def make_mpc(robot_config: Path) -> tuple[ModelPredictiveControl, dict[str, Any]]:
    """Keep the v2 wrist limit in memory and enable normal CUDA graphs."""
    robot_cfg, report = v2.sixth.limited_robot_cfg(robot_config, v2.sixth.CAMERA_FRONT_SEED)
    cfg = ModelPredictiveControlCfg.create(
        robot=robot_cfg,
        scene_model={"cuboid": {"camera_obstacle": {
            "dims": [.01, .01, .01], "pose": [0., 0., -10., 1., 0., 0., 0.]}}},
        optimization_dt=OPTIMIZATION_DT, interpolation_steps=4,
        use_cuda_graph=True, optimizer_collision_activation_distance=.03,
        warm_start_optimization_num_iters=200, cold_start_optimization_num_iters=300,
    )
    return ModelPredictiveControl(cfg), report


def survey(mpc: ModelPredictiveControl, start: np.ndarray) -> dict[str, Any]:
    """Measure warm solves and probe API acceptance of a colliding state."""
    state = joint_state(start, np.zeros(6))
    update_box(mpc, None)
    mpc.setup(state)
    mpc.update_goal_tool_poses(v2.sixth.goal_from_scene(), run_ik=False)
    timings = []
    for _ in range(12):
        torch.cuda.synchronize()
        began = time.perf_counter()
        result = mpc.optimize_action_sequence(state)
        torch.cuda.synchronize()
        timings.append((time.perf_counter() - began) * 1000)
    spheres = mpc.compute_kinematics(state).robot_spheres.reshape(-1, 4)
    center = spheres[-1, :3].detach().cpu().numpy()
    fit = {"center_m": center.tolist(), "dims_m": [.05, .05, .05]}
    update_box(mpc, fit)
    initial_clearance = v2.sixth.fourth.state_clearance(
        mpc.kinematics, state.position, center, np.array(fit["dims_m"]))
    mpc.setup(state)  # Explicitly start setup with the obstacle overlapping the robot.
    result = mpc.optimize_action_sequence(state)
    report = {
        "public_class": "curobo.model_predictive_control.ModelPredictiveControl",
        "implementation": "curobo/curobo/_src/solver/solver_mpc.py:MPCSolver",
        "goal": "setup(JointState); update_goal_tool_poses(GoalToolPose, run_ik=False)",
        "obstacles": "scene_collision_checker.update_obstacle_pose; data.cuboids.update_dims; enable_obstacle",
        "gpu": torch.cuda.get_device_name(),
        "warm_solve_median_ms": float(np.median(timings[2:])),
        "warm_solve_p95_ms": float(np.percentile(timings[2:], 95)),
        "warm_solve_rate_hz": 1000 / float(np.median(timings[2:])),
        "initial_collision_box_clearance_m": initial_clearance,
        "collision_setup_returned": True,
        "collision_result_success": bool(result.success.all()),
        "collision_commands_finite": bool(torch.isfinite(result.action_sequence.velocity).all()),
        "collision_recovery_proven": False,
        "deceleration_fallback": "_solve_impl feasibility fallback is guarded by 'and False' in this checkout",
        "optimization_dt_s": OPTIMIZATION_DT,
        "command_dt_s": CONTROL_DT,
        "returned_joint_state_dt_s": result.action_sequence.dt.detach().cpu().tolist(),
        "iterations_warm_cold": [200, 300],
        "cuda_graph": True,
    }
    update_box(mpc, None)
    return report


def run_case(mpc: ModelPredictiveControl, start: np.ndarray, speed: float,
             prediction: bool, mode: str, worker: v2.MovingRodWorker,
             baseline: dict[str, Any]) -> dict[str, Any]:
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
    fixed_top = baseline["border_rule"]["fixed_unknown_top_m"]

    def advance(duration: float) -> None:
        nonlocal clock, q, minimum, collision_active, limit_clamps
        offsets, positions = integrate_command(q, velocity, duration, position_limits)
        if len(offsets) == 0:
            return
        xs = np.array([v2.rod_x_at(clock+t, speed, v2.ROD_START_X_M, -1.) for t in offsets])
        tensors = torch.as_tensor(positions, device="cuda", dtype=torch.float32)
        truth = v2.rod_clearance_series(mpc.kinematics, tensors, xs)
        # At the same new rod position, separate robot motion from rod motion.
        stationary = v2.rod_clearance_series(mpc.kinematics, tensors[:-1], xs[1:])
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
        if clock >= DURATION - 1e-9:
            break
        captured_time = clock
        snapshot_q = q.copy()
        began = time.perf_counter()
        if goal_pending:
            mpc.update_goal_tool_poses(raised_goal if target == "raised_start" else goal, run_ik=False)
            goal_pending = False
        x = v2.rod_x_at(clock, speed, v2.ROD_START_X_M, -1.)
        estimated_velocity = np.zeros(3)
        if mode == "B":
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
        advance(min(compute_s, DURATION-clock))
        q_after_compute = q.copy()
        index = command_index(compute_s, len(command_sequence))
        proposed = command_sequence[index]
        finite = bool(np.isfinite(proposed).all())
        velocity = np.clip(proposed, velocity_limits[0], velocity_limits[1]) if finite else np.zeros(6)
        velocity = np.where(((q <= position_limits[0]) & (velocity < 0)) |
                            ((q >= position_limits[1]) & (velocity > 0)), 0, velocity)
        # Finite infeasible MPC actions are intentionally executed in this simulation.
        # Gating on feasibility would recreate the stop-and-go freeze under test.
        if not feasible:
            infeasible += 1
        advance(min(max(0., CONTROL_DT-compute_s), DURATION-clock))
        tool = v2.sixth.fourth.tool_position(mpc.kinematics, torch.as_tensor(q, device="cuda", dtype=torch.float32))
        reached = bool(np.linalg.norm(tool-target_points[target]) <= v2.sixth.GOAL_TOLERANCE_M)
        if reached:
            reaches += 1
            target = "raised_start" if target == "goal" else "goal"
            goal_pending = True
        cycles.append({
            "step": step, "sim_time_start_s": captured_time, "sim_time_end_s": clock,
            "target_after_step": target, "target_reached": reached,
            "measured_compute_ms": compute_s*1000, "solver_ms": solve_s*1000,
            "compute_counted_s": min(compute_s, DURATION-captured_time),
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
    if clock < DURATION - 1e-6:
        raise RuntimeError(f"MPC clock incomplete: {clock}")
    compute_ms = np.array([c["measured_compute_ms"] for c in cycles])
    solve_ms = np.array([c["solver_ms"] for c in cycles])
    periods = np.array([c["sim_time_end_s"]-c["sim_time_start_s"] for c in cycles])
    summary = {
        "mode": mode, "prediction": prediction, "rod_speed_mps": speed,
        "controller": "cuRobo MPC, held joint velocity during computation",
        "reaches": reaches, "true_collisions": sum(episodes.values()),
        "collision_episodes_by_onset_phase": episodes, "collision_samples_by_phase": samples,
        "minimum_true_clearance_m": minimum, "steps": len(cycles), "simulated_duration_s": clock,
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
    }
    print("MOVING_ROD_MPC_CASE " + json.dumps(summary), flush=True)
    return {"summary": summary, "cycles": cycles, "trace": traces}


def build_results(config: Path, worker: v2.MovingRodWorker,
                  baseline: dict[str, Any]) -> dict[str, Any]:
    """Run six B cases and conditionally one camera-only E case."""
    mpc, wrist_report = make_mpc(config)
    start = np.array(baseline["raised_start"]["joints_rad"], dtype=np.float32)
    try:
        api = survey(mpc, start)
        print("MOVING_ROD_MPC_SURVEY " + json.dumps(api), flush=True)
        cases = {}
        for speed in v2.ROD_SPEEDS_MPS:
            for prediction in (False, True):
                name = f"MPC B {'prediction' if prediction else 'current box'} {speed:.2f} m/s"
                cases[name] = run_case(mpc, start, speed, prediction, "B", worker, baseline)
        survivors = [c for c in cases.values() if c["summary"]["rod_speed_mps"] == .10 and c["summary"]["survives"]]
        camera = {"run": bool(survivors), "reason": "B did not survive 0.10 m/s"}
        if survivors:
            prediction = survivors[-1]["summary"]["prediction"]
            name = f"MPC E {'prediction' if prediction else 'current box'} 0.10 m/s"
            cases[name] = run_case(mpc, start, .10, prediction, "E", worker, baseline)
            camera = {"run": True, "prediction": prediction}
        return {
            "schema": 1, "date": "2026-11-12", "survey": api,
            "rod": baseline["rod"], "raised_start": baseline["raised_start"],
            "scene_preflight": baseline["scene_preflight"], "wrist_3_limit": wrist_report,
            "clock": {"continuous_during_compute": True, "target_period_s": CONTROL_DT,
                      "model": "ideal velocity integrator, previous command during measured compute; remainder to target period after result",
                      "scoring_and_visual_rendering": "offline, excluded as in v2 truth scoring",
                      "acceleration_feedback": "zero; no actuator dynamics or acceleration/jerk certification"},
            "prediction_rule": "v2 directional stretch: 2 * velocity * (previous measured compute + 0.025 s)",
            "v2_b_comparison": {n: c["summary"] for n,c in baseline["cases"].items() if n.startswith("B ")},
            "camera_conditional": camera, "cases": cases,
        }
    finally:
        mpc.destroy()


def render_frames(worker: v2.MovingRodWorker, results: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    """Render replay snapshots offline; B never needs camera data to control."""
    frames = {}
    for name, case in results["cases"].items():
        frames[name] = []
        next_frame = 0.
        cycle_index = 0
        for sample in case["trace"]:
            t = sample["time_s"]
            if t < next_frame and t < DURATION-1e-6:
                continue
            next_frame = t+.20
            while cycle_index+1 < len(case["cycles"]) and case["cycles"][cycle_index+1]["sim_time_start_s"] <= t:
                cycle_index += 1
            cycle = case["cycles"][cycle_index]
            x = v2.rod_x_at(t, case["summary"]["rod_speed_mps"], v2.ROD_START_X_M, -1.)
            capture = worker.capture(np.array(sample["q"], dtype=np.float32), x)
            frames[name].append({**sample, "rod_x": x, "points": capture.camera.points,
                                 "depth": capture.depth_preview, "fit": cycle["fit"],
                                 "predicted_fit": cycle["predicted_fit"]})
    return frames


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
    selector = server.gui.add_dropdown("Case", options=tuple(frames), initial_value=first)
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
                             "rod_x": v2.rod_x_at(t, results["cases"][name]["summary"]["rod_speed_mps"], v2.ROD_START_X_M, -1.)}
                    if not fast:
                        time.sleep(max(0., t-previous_time))
                    show(name, frame)
                    previous_time = t
                return {"case": name, "error": None,
                        "callback_non_main": threading.current_thread() is not threading.main_thread(),
                        "robot_changed": not np.allclose(frames[name][0]["q"], frames[name][-1]["q"]),
                        "rod_changed": bool(np.ptp([f["rod_x"] for f in frames[name]]) > 0),
                        "depth_updates": len(frames[name]), "predicted_box_available": any(f["predicted_fit"] for f in frames[name])}
            except Exception as exc:
                error.value = f"{type(exc).__name__}: {exc}"
                return {"case": name, "error": error.value}

    def click(_event: Any) -> None:
        thread = threading.Thread(target=playback, args=(selector.value,), daemon=True)
        workers.append(thread)
        thread.start()

    play.on_click(click)
    selector.on_update(lambda _event: show(selector.value, frames[selector.value][0]))
    show(first, frames[first][0])
    verification = []
    for name in (n for n in frames if "0.10 m/s" in n):
        output = []
        thread = threading.Thread(target=lambda n=name: output.append(playback(n, fast=True)))
        thread.start()
        thread.join(timeout=30)
        if thread.is_alive():
            server.stop()
            raise RuntimeError("MPC playback verification timed out")
        verification.extend(output)
    show(first, frames[first][0])
    report = {"cases": verification, "all_passed": all(
        c.get("error") is None and c.get("callback_non_main") and c.get("robot_changed") and c.get("rod_changed")
        for c in verification)}
    print("MOVING_ROD_MPC_VIEWER " + json.dumps(report), flush=True)
    print(f"MOVING_ROD_MPC_URL http://127.0.0.1:{port}", flush=True)
    # Main owns the server and only the playback threads started here.
    viewer._mpc_workers = workers
    viewer._mpc_stop_event = stop_playback
    return viewer, report


def save_results(results: dict[str, Any]) -> None:
    """Write only the explicitly requested MPC evidence artifacts."""
    RESULT_FILE.write_text(json.dumps(results, indent=2)+"\n")
    RESULT_FILE.with_suffix(".log").write_text("MOVING_ROD_MPC\n" + json.dumps({
        "survey": results["survey"], "summaries": {n:c["summary"] for n,c in results["cases"].items()},
        "camera_conditional": results["camera_conditional"]}, sort_keys=True)+"\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--replay", action="store_true", help="read saved traces; no control experiment or evidence writes")
    parser.add_argument("--save-results", action="store_true")
    parser.add_argument("--duration", type=float, default=0.)
    parser.add_argument("--port", type=int, default=int(os.environ.get("NEWTON_VIEWER_PORT", "30000")))
    parser.add_argument("--robot-config", type=Path, default=Path.home()/".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml")
    parser.add_argument("--newton-python", type=Path, default=Path.home()/"newton_ws/.venv-cpu/bin/python")
    args = parser.parse_args()
    if args.replay and args.save_results:
        parser.error("--replay cannot be combined with --save-results")
    baseline = json.loads(BASELINE_FILE.read_text())
    configure_scene(baseline)
    viewer = None
    results = None
    with tempfile.TemporaryDirectory(prefix="moving-rod-mpc-") as temp:
        worker = v2.MovingRodWorker(args.newton_python, Path(temp))
        try:
            results = json.loads(RESULT_FILE.read_text()) if args.replay else build_results(args.robot_config, worker, baseline)
            if not args.headless:
                frames = render_frames(worker, results)
                viewer, report = start_viewer(args.robot_config, results, frames, args.port)
                results["playback_verification"] = report
                if not report["all_passed"]:
                    raise RuntimeError("MPC viewer verification failed")
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
        save_results(results)
        print("MOVING_ROD_MPC_RESULTS_SAVED", flush=True)
    else:
        print("MOVING_ROD_MPC_RESULTS_NOT_SAVED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
