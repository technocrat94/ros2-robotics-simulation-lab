#!/usr/bin/env python3
"""Camera-only cuRobo replanning against a kinematically moving rod."""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile
import threading
import time
import traceback
from typing import Any

import numpy as np
import torch

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer

import curobo_camera_fifth_pass as sixth


REPO = Path(__file__).resolve().parents[1]
RESULTS_DIR = REPO / "docs/experiments/sim-camera-curobo/results"
MOUNT_CONFIG = REPO / "config/sim_camera_mount.yaml"
ROD_RADIUS_M = 0.025
ROD_TOP_M = 0.35
ROD_CENTER_Z_M = 0.5 * ROD_TOP_M
ROD_Y_M = 0.263
ROD_X_MIN_M = 0.30
ROD_X_MAX_M = 0.70
ROD_START_X_M = 0.60
ROD_SPEEDS_MPS = (0.05, 0.10, 0.20)
SIM_DURATION_S = 20.0
STALE_TTL_S = 1.0
TRAJECTORY_DT_S = 0.025
NO_PLAN_HOLD_S = TRAJECTORY_DT_S
SEED = 1901
DEPTH_FOV_RAD = math.radians(58.0)
DEPTH_ASPECT = 848 / 480


@dataclass
class RodCapture:
    camera: sixth.fourth.Capture
    depth_preview: np.ndarray


class MovingRodWorker:
    def __init__(self, python: Path, temp_dir: Path):
        self.temp_dir = temp_dir
        self.counter = 0
        self.log: list[str] = []
        self.python = python
        self.process = None

    def start(self) -> None:
        self.process = subprocess.Popen(
            [
                str(self.python),
                str(REPO / "scripts/newton_camera_moving_rod_worker.py"),
                "--rod-y", str(ROD_Y_M), "--rod-top", str(ROD_TOP_M),
            ],
            cwd=REPO,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        ready = json.loads(self._read_prefix("NEWTON_MOVING_ROD_WORKER_READY "))
        if ready.get("scene_builds") != 1:
            raise RuntimeError(f"moving-rod worker rebuilt its scene: {ready}")

    def _read_prefix(self, prefix: str) -> str:
        if self.process.stdout is None:
            raise RuntimeError("moving-rod worker stdout is unavailable")
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError(
                    f"moving-rod worker exited before {prefix!r}: "
                    + "".join(self.log[-20:])
                )
            self.log.append(line)
            if line.startswith(prefix):
                return line[len(prefix) :].strip()

    def capture(self, q: np.ndarray | torch.Tensor, rod_x_m: float) -> RodCapture:
        if self.process is None:
            self.start()
        self.counter += 1
        output = self.temp_dir / f"moving_rod_{self.counter:05d}.npz"
        request = {
            "id": self.counter,
            "arm_q": np.asarray(
                q.detach().cpu().numpy() if isinstance(q, torch.Tensor) else q,
                dtype=float,
            ).tolist(),
            "rod_x_m": rod_x_m,
            "output": str(output),
        }
        if self.process.stdin is None:
            raise RuntimeError("moving-rod worker stdin is unavailable")
        self.process.stdin.write(json.dumps(request) + "\n")
        self.process.stdin.flush()
        response = json.loads(
            self._read_prefix("NEWTON_MOVING_ROD_WORKER_RESPONSE ")
        )
        if response.get("scene_builds") != 1:
            raise RuntimeError(f"worker scene was rebuilt: {response}")
        arrays = np.load(output)
        capture = sixth.fourth.Capture(
            q=np.asarray(request["arm_q"], dtype=np.float32),
            points=arrays["points"].copy(),
            camera_transform=arrays["camera_transform"].copy(),
            metadata=response,
        )
        preview = arrays["depth_preview"].copy()
        arrays.close()
        output.unlink()
        return RodCapture(capture, preview)

    def close(self) -> None:
        if self.process is not None and self.process.poll() is None and self.process.stdin is not None:
            self.process.stdin.write(json.dumps({"command": "shutdown"}) + "\n")
            self.process.stdin.flush()
            self.process.wait(timeout=10.0)


class TimedPointCloud:
    def __init__(self) -> None:
        self.points = np.empty((0, 3), dtype=np.float32)
        self.times = np.empty((0,), dtype=np.float64)

    @staticmethod
    def _inside_view(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return np.empty((0,), dtype=bool)
        rotation = sixth.fourth.wxyz_matrix(transform[[6, 3, 4, 5]])
        camera = (points - transform[:3]) @ rotation
        depth = -camera[:, 2]
        half_height = math.tan(0.5 * DEPTH_FOV_RAD)
        half_width = half_height * DEPTH_ASPECT
        return (
            (depth > 1.0e-6)
            & (np.abs(camera[:, 0]) <= depth * half_width)
            & (np.abs(camera[:, 1]) <= depth * half_height)
        )

    def update(
        self, capture: sixth.fourth.Capture, sim_time_s: float
    ) -> tuple[np.ndarray, int]:
        inside = self._inside_view(self.points, capture.camera_transform)
        young = sim_time_s - self.times <= STALE_TTL_S + 1.0e-9
        keep = (~inside) & young
        retained = self.points[keep]
        retained_times = self.times[keep]
        current = np.asarray(capture.points, dtype=np.float32)
        current_times = np.full(len(current), sim_time_s, dtype=np.float64)
        self.points = np.concatenate((retained, current), axis=0)
        self.times = np.concatenate((retained_times, current_times), axis=0)
        return self.points.copy(), int(len(retained))


def rod_x_at(
    sim_time_s: float,
    speed_mps: float,
    start_x_m: float,
    initial_direction: float,
) -> float:
    length = ROD_X_MAX_M - ROD_X_MIN_M
    unfolded = (
        start_x_m - ROD_X_MIN_M + initial_direction * speed_mps * sim_time_s
    ) % (2.0 * length)
    if unfolded <= length:
        return ROD_X_MIN_M + unfolded
    return ROD_X_MAX_M - (unfolded - length)


def rod_clearance_series(
    robot: Kinematics,
    positions: torch.Tensor,
    rod_x_values: np.ndarray,
) -> np.ndarray:
    state = JointState.from_position(
        positions.reshape(-1, len(sixth.preflight.ARM_JOINT_NAMES)),
        joint_names=list(sixth.preflight.ARM_JOINT_NAMES),
    )
    spheres = robot.compute_kinematics(state).robot_spheres
    if spheres is None:
        raise RuntimeError("cuRobo did not return collision spheres")
    values = spheres.detach().cpu().numpy().reshape(len(positions), -1, 4)
    rod_x = np.asarray(rod_x_values, dtype=np.float64).reshape(-1, 1)
    radial = (
        np.sqrt(
            (values[:, :, 0] - rod_x) ** 2
            + (values[:, :, 1] - ROD_Y_M) ** 2
        )
        - ROD_RADIUS_M
    )
    axial = np.abs(values[:, :, 2] - ROD_CENTER_Z_M) - 0.5 * ROD_TOP_M
    pair = np.stack((radial, axial), axis=-1)
    outside = np.linalg.norm(np.maximum(pair, 0.0), axis=-1)
    inside = np.minimum(np.maximum(radial, axial), 0.0)
    signed = outside + inside - values[:, :, 3]
    return np.min(signed, axis=1)


def sampled_hold_clearances(
    robot: Kinematics,
    q: torch.Tensor,
    start_s: float,
    duration_s: float,
    speed_mps: float,
    start_x_m: float,
    direction: float,
) -> np.ndarray:
    count = max(2, int(math.ceil(duration_s / TRAJECTORY_DT_S)) + 1)
    times = np.linspace(start_s, start_s + duration_s, count)
    positions = q.reshape(1, -1).repeat(count, 1)
    rod_x = np.asarray(
        [rod_x_at(t, speed_mps, start_x_m, direction) for t in times]
    )
    return rod_clearance_series(robot, positions, rod_x)


def compact_fit(fit: dict[str, Any] | None) -> dict[str, Any] | None:
    if fit is None:
        return None
    return {
        "center_m": fit["center_m"],
        "dims_m": fit["dims_m"],
        "fitted_top_z_m": fit.get("fitted_top_z_m"),
        "points": fit.get("points"),
        "unknown_extension": fit.get("unknown_extension"),
        "height_rule": fit.get("height_rule"),
    }


@dataclass(frozen=True)
class CaseConfig:
    name: str
    mode: str
    speed_mps: float
    count_compute_time: bool = True
    blind: bool = False

    @property
    def start_x_m(self) -> float:
        return ROD_X_MIN_M if self.blind else min(ROD_START_X_M, ROD_X_MAX_M)

    @property
    def direction(self) -> float:
        return 1.0 if self.blind else -1.0


def case_matrix() -> list[CaseConfig]:
    cases: list[CaseConfig] = []
    for speed in ROD_SPEEDS_MPS:
        cases.extend(
            (
                CaseConfig(f"A no planner obstacle {speed:.2f} m/s", "A", speed),
                CaseConfig(f"B god view {speed:.2f} m/s", "B", speed),
                CaseConfig(f"E camera only {speed:.2f} m/s", "E", speed),
            )
        )
    cases.append(
        CaseConfig(
            "E camera only 0.10 m/s zero compute latency",
            "E",
            0.10,
            count_compute_time=False,
        )
    )
    cases.append(
        CaseConfig(
            "E blind entry 0.10 m/s",
            "E",
            0.10,
            count_compute_time=True,
            blind=True,
        )
    )
    return cases


def exact_rod_fit(rod_x_m: float) -> dict[str, Any]:
    return {
        "center_m": [rod_x_m, ROD_Y_M, ROD_CENTER_Z_M],
        "dims_m": [2.0 * ROD_RADIUS_M, 2.0 * ROD_RADIUS_M, ROD_TOP_M],
        "fitted_top_z_m": ROD_TOP_M,
        "points": None,
        "unknown_extension": {"active": False, "directions": []},
        "height_rule": {"source": "case B exact truth"},
    }


def select_fair_scene(robot, start_q, goal_q, baseline, camera_z):
    global ROD_Y_M, ROD_TOP_M, ROD_CENTER_Z_M, ROD_X_MIN_M, ROD_X_MAX_M
    rejected = []
    camera_position, camera_wxyz = sixth.fourth.camera_pose_for_q(
        robot, torch.as_tensor(start_q, device="cuda"),
        sixth.fourth.read_mount_config(MOUNT_CONFIG))
    # Undo the Viser-only pi-X conversion before testing Newton's -Z view.
    sensor_rotation = sixth.fourth.wxyz_matrix(camera_wxyz) @ np.diag((1., -1., -1.))
    sensor_wxyz = sixth.fourth.matrix_to_wxyz(sensor_rotation)
    camera_transform = np.r_[camera_position, sensor_wxyz[[1, 2, 3, 0]]]
    candidates = [(0.263, 0.30, 0.70, 0.35)]
    candidates += [(y, lo, hi, top)
                   for top in (0.30, 0.25, 0.20, 0.35)
                   for y in np.arange(0.12, 0.421, 0.025)
                   for lo, hi in ((0.30, 0.70), (0.35, 0.65), (0.40, 0.60))]
    for y, lo, hi, top in candidates:
        ROD_Y_M, ROD_TOP_M, ROD_CENTER_Z_M = float(y), top, top / 2
        ROD_X_MIN_M, ROD_X_MAX_M = lo, hi
        xs = np.linspace(lo, hi, int(round((hi-lo)/0.001)) + 1)
        endpoints = []
        for q in (start_q, goal_q):
            q = torch.as_tensor(q, device="cuda", dtype=torch.float32)
            endpoints.append(float(np.min(rod_clearance_series(
                robot, q.reshape(1,-1).repeat(len(xs),1), xs))) - 0.0005)
        # Existence of a baseline intersection is a necessary preflight filter;
        # actual A runs below additionally validate the timed controller.
        baseline_min = min(float(np.min(rod_clearance_series(
            robot, baseline, np.full(len(baseline), x)))) for x in xs[::10])
        record = dict(y_m=float(y), x_limits_m=[lo, hi], top_m=top,
                      endpoint_clearance_lower_bounds_m=endpoints,
                      baseline_min_clearance_m=baseline_min,
                      required_baseline_penetration_m=0.030)
        # Conservative visibility filter on a dense enclosing box at the low-X
        # entry. Actual zero rod pixels are checked with Newton before cases.
        entry_points = np.array([[lo+dx, y+dy, z]
            for dx in (-ROD_RADIUS_M, 0, ROD_RADIUS_M)
            for dy in (-ROD_RADIUS_M, 0, ROD_RADIUS_M)
            for z in np.linspace(0.012, top, 20)])
        blind = not np.any(TimedPointCloud._inside_view(entry_points, camera_transform))
        record["blind_entry_geometric_filter"] = bool(blind)
        if min(endpoints) >= 0.025 and top < camera_z and baseline_min <= -0.030 and blind:
            return {"status": "VALID_PREFLIGHT", "chosen": record,
                    "sweep_spacing_m": 0.001, "lipschitz_allowance_m": 0.0005,
                    "rejected": rejected}
        record["status"] = "INVALID_SCENE"
        rejected.append(record)
    raise RuntimeError("INVALID_SCENE: no lane meets endpoint and baseline tests: " + json.dumps(rejected))


def predicted_fit(fit, velocity, horizon_s):
    if fit is None:
        return None
    result = copy.deepcopy(fit)
    displacement = np.asarray(velocity) * horizon_s * 2.0
    result["center_m"] = (np.asarray(fit["center_m"]) + displacement/2).tolist()
    result["dims_m"] = (np.asarray(fit["dims_m"]) + np.abs(displacement)).tolist()
    return result


def upward_goal(robot, current):
    pose = robot.compute_kinematics(sixth.arm_state(current)).tool_poses[sixth.preflight.TOOL_FRAME]
    position = pose.position.detach().cpu().reshape(-1, 3)[0].numpy().copy()
    position[2] += 0.10
    quaternion = pose.quaternion.detach().cpu().reshape(-1, 4)[0].numpy()
    return GoalToolPose.from_poses(
        {sixth.preflight.TOOL_FRAME: Pose.from_list([*position.tolist(), *quaternion.tolist()])},
        ordered_tool_frames=[sixth.preflight.TOOL_FRAME], num_goalset=1)


def run_case(
    config: CaseConfig,
    worker: MovingRodWorker,
    planner: Any,
    robot: Kinematics,
    start_q: np.ndarray,
    goal_q: torch.Tensor,
    raised_goal: GoalToolPose,
    goal: GoalToolPose,
    fixed_unknown_top_m: float,
    wrist_limits: tuple[float, float],
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    torch.manual_seed(SEED)
    sixth.fourth.update_fit_in_place(planner, None)
    current = torch.as_tensor(start_q, device="cuda", dtype=torch.float32)
    start_tensor = current.detach().clone()
    cloud = TimedPointCloud()
    previous_observation = None
    expected_compute_s = 0.20
    expected_segment_s = 0.20
    escape_counts = {"raised_start": 0, "straight_up": 0, "all_failed": 0}
    collision_samples = {"moving": 0, "holding": 0}
    collision_episodes_by_phase = {"moving": 0, "holding": 0}
    sim_time_s = 0.0
    target_name = "goal"
    reaches = 0
    collision_episodes = 0
    collision_active = False
    minimum_clearance = math.inf
    stopped_no_plan_s = 0.0
    first_seen_time_s: float | None = None
    first_seen_distance_m: float | None = None
    planner_failures: list[str] = []
    whole_cycle_ms: list[float] = []
    compact_cycles: list[dict[str, Any]] = []
    visuals: list[dict[str, Any]] = []

    def ingest_clearances(values: np.ndarray, phase="holding") -> None:
        nonlocal collision_active, collision_episodes, minimum_clearance
        for value in np.asarray(values, dtype=float):
            minimum_clearance = min(minimum_clearance, float(value))
            colliding = value < 0.0
            if colliding:
                collision_samples[phase] += 1
            if colliding and not collision_active:
                collision_episodes += 1
                collision_episodes_by_phase[phase] += 1
            collision_active = colliding

    initial_x = rod_x_at(
        0.0, config.speed_mps, config.start_x_m, config.direction
    )
    ingest_clearances(
        rod_clearance_series(robot, current.reshape(1, -1), np.array([initial_x]))
    )

    for cycle in range(1, 1001):
        if sim_time_s >= SIM_DURATION_S - 1.0e-9:
            break
        cycle_start_s = sim_time_s
        rod_x_capture = rod_x_at(
            sim_time_s,
            config.speed_mps,
            config.start_x_m,
            config.direction,
        )
        compute_started = time.perf_counter()
        capture = worker.capture(current, rod_x_capture)
        points, stale_points = cloud.update(capture.camera, sim_time_s)
        if first_seen_time_s is None and len(capture.camera.points):
            first_seen_time_s = sim_time_s
            first_seen_distance_m = float(
                rod_clearance_series(
                    robot,
                    current.reshape(1, -1),
                    np.array([rod_x_capture]),
                )[0]
            )

        if config.mode == "A":
            fit = None
        elif config.mode == "B":
            fit = exact_rod_fit(rod_x_capture)
        else:
            fit, _extension = sixth.fit_with_unknown_extent(
                points, capture.camera, fixed_unknown_top_m
            )
        observed_fit = fit
        velocity = np.zeros(3)
        if config.mode == "B":
            dt = 0.001
            velocity[0] = (rod_x_at(sim_time_s + dt, config.speed_mps,
                                   config.start_x_m, config.direction) - rod_x_capture)/dt
        elif config.mode == "E":
            # Estimate only from fresh-frame boxes, never stale cloud or truth.
            fresh, _ = sixth.fit_with_unknown_extent(
                capture.camera.points, capture.camera, fixed_unknown_top_m)
            if fresh is not None:
                center = np.asarray(fresh["center_m"])
                if previous_observation is not None:
                    old_time, old_center = previous_observation
                    if sim_time_s > old_time:
                        velocity = (center-old_center)/(sim_time_s-old_time)
                previous_observation = (sim_time_s, center)
        prediction_horizon_s = (expected_compute_s if config.count_compute_time else 0.0) + expected_segment_s
        fit = predicted_fit(fit, velocity, prediction_horizon_s)
        sixth.fourth.update_fit_in_place(planner, fit)
        current_fit_clearance = (
            None
            if fit is None
            else sixth.fourth.state_clearance(
                robot,
                current,
                np.asarray(fit["center_m"]),
                np.asarray(fit["dims_m"]),
            )
        )
        planned_target = target_name
        target_pose = goal if target_name == "goal" else raised_goal
        target_q = goal_q if target_name == "goal" else start_tensor
        target_fit_clearance = (
            None
            if fit is None
            else sixth.fourth.state_clearance(
                robot,
                target_q,
                np.asarray(fit["center_m"]),
                np.asarray(fit["dims_m"]),
            )
        )
        advance_rad = (
            sixth.NEAR_ADVANCE_RAD
            if current_fit_clearance is not None
            and current_fit_clearance < sixth.NEAR_BOX_CLEARANCE_M
            else sixth.FAR_ADVANCE_RAD
        )
        torch.cuda.synchronize()
        plan_started = time.perf_counter()
        plan_result = planner.plan_pose(target_pose, sixth.arm_state(current))
        torch.cuda.synchronize()
        plan_ms = (time.perf_counter() - plan_started) * 1000.0
        positions = sixth.fourth.active_trajectory(plan_result)
        planner_failure = sixth.fourth.failure_status(plan_result)
        escape = None
        escape_attempts = []
        if planner_failure:
            planner_failures.append(planner_failure)
        if config.mode != "A" and (positions is None or
                (current_fit_clearance is not None and current_fit_clearance < 0.030)):
            positions = None
            for escape_name, escape_pose in (("raised_start", raised_goal),
                                              ("straight_up", upward_goal(robot, current))):
                retreat = planner.plan_pose(escape_pose, sixth.arm_state(current))
                positions = sixth.fourth.active_trajectory(retreat)
                escape_attempts.append({"target": escape_name,
                    "failure": sixth.fourth.failure_status(retreat)})
                if positions is not None:
                    escape = escape_name
                    escape_counts[escape_name] += 1
                    break
            if positions is None:
                escape = "all_failed"
                escape_counts[escape] += 1
        torch.cuda.synchronize()
        measured_compute_s = time.perf_counter() - compute_started
        expected_compute_s = measured_compute_s
        simulated_compute_s = measured_compute_s if config.count_compute_time else 0.0
        simulated_compute_s = min(
            simulated_compute_s, SIM_DURATION_S - sim_time_s
        )
        if simulated_compute_s > 0.0:
            ingest_clearances(
                sampled_hold_clearances(
                    robot,
                    current,
                    sim_time_s,
                    simulated_compute_s,
                    config.speed_mps,
                    config.start_x_m,
                    config.direction,
                )
            )
            sim_time_s += simulated_compute_s

        q_start = current.detach().cpu().numpy().copy()
        executed = torch.empty((0, 6), device="cuda")
        execution_duration_s = 0.0
        stop_reason: str | None = None
        reached_this_cycle = False
        if positions is None:
            failure = planner_failure or "NO_PLAN"
            planner_failures.append(failure)
            stop_reason = failure
            hold_s = min(NO_PLAN_HOLD_S, SIM_DURATION_S - sim_time_s)
            if hold_s > 0.0:
                ingest_clearances(
                    sampled_hold_clearances(
                        robot,
                        current,
                        sim_time_s,
                        hold_s,
                        config.speed_mps,
                        config.start_x_m,
                        config.direction,
                    )
                )
                sim_time_s += hold_s
            stopped_no_plan_s += simulated_compute_s + hold_s
            execution_duration_s = hold_s
        else:
            wrist = positions[:, sixth.WRIST_3_INDEX]
            if bool(
                torch.any(wrist < wrist_limits[0] - 1.0e-5)
                or torch.any(wrist > wrist_limits[1] + 1.0e-5)
            ):
                planner_failures.append("WRIST_LIMIT_VIOLATION")
                stop_reason = "WRIST_LIMIT_VIOLATION"
                hold_s = min(NO_PLAN_HOLD_S, SIM_DURATION_S - sim_time_s)
                if hold_s > 0.0:
                    ingest_clearances(
                        sampled_hold_clearances(
                            robot,
                            current,
                            sim_time_s,
                            hold_s,
                            config.speed_mps,
                            config.start_x_m,
                            config.direction,
                        )
                    )
                    sim_time_s += hold_s
                stopped_no_plan_s += simulated_compute_s + hold_s
                execution_duration_s = hold_s
            else:
                index = sixth.fourth.advance_index(positions, advance_rad)
                maximum_steps = int(
                    math.floor((SIM_DURATION_S - sim_time_s) / TRAJECTORY_DT_S)
                )
                index = min(index, maximum_steps)
                executed = positions[: index + 1].detach().clone()
                execution_duration_s = max(0, len(executed) - 1) * TRAJECTORY_DT_S
                if len(executed):
                    sample_times = sim_time_s + np.arange(len(executed)) * TRAJECTORY_DT_S
                    rod_x_path = np.asarray(
                        [
                            rod_x_at(
                                float(t),
                                config.speed_mps,
                                config.start_x_m,
                                config.direction,
                            )
                            for t in sample_times
                        ]
                    )
                    truth = rod_clearance_series(robot, executed, rod_x_path)
                    changes = torch.max(torch.abs(executed[1:] - executed[:-1]), dim=1).values
                    moving = np.r_[False, changes.detach().cpu().numpy() > 1.0e-7]
                    for clearance, is_moving in zip(truth, moving):
                        ingest_clearances(np.array([clearance]),
                                          "moving" if is_moving else "holding")
                    current = executed[-1].detach().clone()
                sim_time_s += execution_duration_s
                target_error = float(
                    np.linalg.norm(
                        sixth.fourth.tool_position(robot, current)
                        - sixth.fourth.tool_position(robot, target_q)
                    )
                )
                expected_segment_s = execution_duration_s
                if escape is None and (index == len(positions) - 1 or target_error <= sixth.GOAL_TOLERANCE_M):
                    reaches += 1
                    reached_this_cycle = True
                    stop_reason = "TARGET_REACHED"
                    target_name = "raised_start" if target_name == "goal" else "goal"

        # A time-parameterized trajectory can legitimately have no complete
        # control interval left in the final fractional part of the case.  Keep
        # the rod and truth clock moving through that remainder while the arm
        # holds its last commanded state.
        if sim_time_s <= cycle_start_s + 1.0e-12:
            hold_s = min(NO_PLAN_HOLD_S, SIM_DURATION_S - sim_time_s)
            if hold_s > 0.0:
                ingest_clearances(
                    sampled_hold_clearances(
                        robot,
                        current,
                        sim_time_s,
                        hold_s,
                        config.speed_mps,
                        config.start_x_m,
                        config.direction,
                    )
                )
                sim_time_s += hold_s
                execution_duration_s += hold_s
                stop_reason = stop_reason or "FINAL_CLOCK_HOLD"

        cycle_total_ms = measured_compute_s * 1000.0 + execution_duration_s * 1000.0
        whole_cycle_ms.append(cycle_total_ms)
        cycle_record = {
            "cycle": cycle,
            "sim_time_start_s": cycle_start_s,
            "sim_time_end_s": sim_time_s,
            "rod_x_at_capture_m": rod_x_capture,
            "target": planned_target,
            "capture_points": int(len(capture.camera.points)),
            "merged_points": int(len(points)),
            "stale_points_used": stale_points,
            "fit": compact_fit(observed_fit),
            "predicted_fit": compact_fit(fit),
            "velocity_mps": velocity.tolist(),
            "prediction_horizon_s": prediction_horizon_s,
            "prediction_multiplier": 2.0,
            "escape": escape,
            "escape_attempts": escape_attempts,
            "current_to_fit_clearance_m": current_fit_clearance,
            "target_to_fit_clearance_m": target_fit_clearance,
            "measured_compute_ms": measured_compute_s * 1000.0,
            "simulated_compute_s": simulated_compute_s,
            "plan_ms": plan_ms,
            "execution_duration_s": execution_duration_s,
            "executed_states": int(len(executed)),
            "whole_cycle_ms": cycle_total_ms,
            "planner_failure_status": planner_failure,
            "stop_reason": stop_reason,
            "minimum_true_clearance_so_far_m": minimum_clearance,
            "collision_episodes_so_far": collision_episodes,
            "reaches_so_far": reaches,
        }
        compact_cycles.append(cycle_record)
        visuals.append(
            {
                "cycle": cycle,
                "sim_time_start_s": cycle_start_s,
                "simulated_compute_s": simulated_compute_s,
                "execution_duration_s": execution_duration_s,
                "q_start": q_start,
                "executed": executed.detach().cpu().numpy(),
                "points": points[:3000].copy(),
                "fit": observed_fit,
                "predicted_fit": fit,
                "escape": escape,
                "depth_preview": capture.depth_preview,
                "rod_x_at_capture_m": rod_x_capture,
                "stop_reason": stop_reason,
                "minimum_true_clearance_m": minimum_clearance,
            }
        )
        if sim_time_s <= cycle_start_s + 1.0e-12:
            raise RuntimeError(f"case clock did not advance: {config.name}")

    if not compact_cycles or sim_time_s < SIM_DURATION_S - 1.0e-6:
        raise RuntimeError(
            f"case did not complete {SIM_DURATION_S:.1f} simulated seconds: "
            f"{config.name}, t={sim_time_s:.6f}"
        )
    summary = {
        "mode": config.mode,
        "rod_speed_mps": config.speed_mps,
        "blind_entry": config.blind,
        "compute_time_counted": config.count_compute_time,
        "simulated_duration_s": sim_time_s,
        "cycles": len(compact_cycles),
        "reaches": reaches,
        "true_collisions": collision_episodes,
        "collision_samples_by_phase": collision_samples,
        "collision_episodes_by_onset_phase": collision_episodes_by_phase,
        "escapes": escape_counts,
        "minimum_true_clearance_m": minimum_clearance,
        "stopped_with_no_plan_s": stopped_no_plan_s,
        "whole_cycle_median_ms": float(statistics.median(whole_cycle_ms)),
        "whole_cycle_p95_ms": float(np.percentile(whole_cycle_ms, 95.0)),
        "first_seen_sim_time_s": first_seen_time_s,
        "rod_to_arm_distance_when_first_seen_m": first_seen_distance_m,
        "planner_failure_statuses": sorted(set(planner_failures)),
        "arm_motion_during_compute": "none; stop-and-go",
        "old_point_rule": (
            "replace points inside the current frustum; retain outside-frustum "
            f"points for at most {STALE_TTL_S:.1f} simulated seconds"
        ),
    }
    return summary, compact_cycles, visuals


def start_viewer(
    robot_config: Path,
    robot: Kinematics,
    start_q: np.ndarray,
    cases: dict[str, dict[str, Any]],
    port: int,
) -> tuple[ViserVisualizer, Any]:
    mount = sixth.fourth.read_mount_config(MOUNT_CONFIG)
    viewer = ViserVisualizer(
        content_path=ContentPath(robot_config_file=str(robot_config)),
        connect_ip="127.0.0.1",
        connect_port=port,
        add_control_frames=True,
        visualize_robot_spheres=False,
        add_robot_to_scene=True,
    )
    server = viewer._server
    actual_port = int(server._websock_server._port)
    if actual_port != port:
        server.stop()
        raise RuntimeError(
            f"Viser requested 127.0.0.1:{port} but selected {actual_port}"
        )
    viewer.set_joint_state(sixth.arm_state(start_q).squeeze(0))
    rod = server.scene.add_cylinder(
        "/truth/moving_rod",
        radius=ROD_RADIUS_M,
        height=ROD_TOP_M,
        color=(235, 55, 45),
        opacity=0.65,
        position=(ROD_X_MAX_M, ROD_Y_M, ROD_CENTER_Z_M),
    )
    fit_box = server.scene.add_box(
        "/perceived/fitted_box",
        dimensions=(0.01, 0.01, 0.01),
        position=(0.0, 0.0, -1.0),
        color=(255, 145, 35),
        opacity=0.42,
    )
    predicted_box = server.scene.add_box(
        "/prediction/box", dimensions=(0.01, 0.01, 0.01),
        color=(80, 150, 255), opacity=0.30)
    escape_text = server.gui.add_text("Escape", "", disabled=True)
    cloud = server.scene.add_point_cloud(
        "/camera/points",
        points=np.zeros((1, 3)),
        colors=(220, 0, 220),
        point_size=0.011,
        point_shape="circle",
    )
    camera_position, camera_wxyz = sixth.fourth.camera_pose_for_q(
        robot, torch.as_tensor(start_q, device="cuda"), mount
    )
    frustum = server.scene.add_camera_frustum(
        "/camera/live",
        fov=DEPTH_FOV_RAD,
        aspect=DEPTH_ASPECT,
        scale=0.13,
        color=(255, 255, 255),
        position=tuple(camera_position),
        wxyz=tuple(camera_wxyz),
    )
    first_name = next(iter(cases))
    first_preview = cases[first_name]["visuals"][0]["depth_preview"]
    depth_panel = server.gui.add_image(
        first_preview,
        label="Live simulated depth",
        format="jpeg",
        jpeg_quality=80,
    )
    selector = server.gui.add_dropdown(
        "Case", options=tuple(cases), initial_value=first_name
    )
    case_text = server.gui.add_text("Current case", first_name, disabled=True)
    speed_text = server.gui.add_text("Rod speed", "", disabled=True)
    sim_time_text = server.gui.add_text("Sim time", "0.000 s", disabled=True)
    cycle_text = server.gui.add_text("Cycle", "1", disabled=True)
    stop_text = server.gui.add_text("Stop reason", "", disabled=True)
    clearance_text = server.gui.add_text("Minimum true clearance", "", disabled=True)
    playback_error = server.gui.add_text("Playback error", "none", disabled=True)
    play = server.gui.add_button("Play selected case")
    verification_mode = threading.Event()
    playback_done = threading.Event()
    playback_record: dict[str, Any] = {}

    def set_q(q: np.ndarray) -> None:
        q_tensor = torch.as_tensor(q, device="cuda", dtype=torch.float32)
        viewer.set_joint_state(sixth.arm_state(q_tensor).squeeze(0))
        position, wxyz = sixth.fourth.camera_pose_for_q(robot, q_tensor, mount)
        frustum.position = tuple(position)
        frustum.wxyz = tuple(wxyz)

    def show_record(name: str, record: dict[str, Any]) -> None:
        case = cases[name]
        config: CaseConfig = case["config"]
        case_text.value = name
        speed_text.value = f"{config.speed_mps:.2f} m/s"
        sim_time_text.value = f"{record['sim_time_start_s']:.3f} s"
        cycle_text.value = str(record["cycle"])
        stop_text.value = str(record.get("stop_reason") or "RUNNING")
        clearance_text.value = f"{record['minimum_true_clearance_m']:+.4f} m"
        rod.position = (
            record["rod_x_at_capture_m"],
            ROD_Y_M,
            ROD_CENTER_Z_M,
        )
        points = record["points"]
        cloud.points = points if len(points) else np.zeros((1, 3))
        predicted = record.get("predicted_fit")
        predicted_box.visible = predicted is not None
        if predicted is not None:
            predicted_box.position = tuple(predicted["center_m"])
            predicted_box.dimensions = tuple(predicted["dims_m"])
        escape_text.value = "ESCAPE: " + record["escape"] if record.get("escape") else ""
        fit = record.get("fit")
        fit_box.visible = fit is not None
        if fit is not None:
            fit_box.position = tuple(fit["center_m"])
            fit_box.dimensions = tuple(fit["dims_m"])
        depth_panel.image = record["depth_preview"]
        set_q(record["q_start"])

    def apply_case(_event: Any = None) -> None:
        show_record(selector.value, cases[selector.value]["visuals"][0])

    def playback(_event: Any) -> None:
        name = selector.value
        callback_thread = threading.current_thread()
        playback_done.clear()
        playback_error.value = "none"

        def worker_thread() -> None:
            commanded: list[np.ndarray] = []
            rod_positions: list[float] = []
            depth_checksums: list[int] = []
            try:
                case = cases[name]
                config: CaseConfig = case["config"]
                records = case["visuals"]
                for record in records:
                    show_record(name, record)
                    depth_checksums.append(int(np.sum(record["depth_preview"])))
                    compute_duration = record["simulated_compute_s"]
                    compute_steps = (
                        (0, 1)
                        if verification_mode.is_set()
                        else range(max(1, int(math.ceil(compute_duration / 0.025))) + 1)
                    )
                    for step in compute_steps:
                        fraction = float(step) / max(len(compute_steps) - 1, 1)
                        t = record["sim_time_start_s"] + fraction * compute_duration
                        x = rod_x_at(
                            t,
                            config.speed_mps,
                            config.start_x_m,
                            config.direction,
                        )
                        rod.position = (x, ROD_Y_M, ROD_CENTER_Z_M)
                        sim_time_text.value = f"{t:.3f} s"
                        rod_positions.append(x)
                        if not verification_mode.is_set() and compute_duration > 0.0:
                            time.sleep(compute_duration / max(len(compute_steps) - 1, 1))
                    executed = record["executed"]
                    if len(executed):
                        indices = (
                            (0, len(executed) - 1)
                            if verification_mode.is_set() and len(executed) > 1
                            else range(len(executed))
                        )
                        for index in indices:
                            t = (
                                record["sim_time_start_s"]
                                + compute_duration
                                + index * TRAJECTORY_DT_S
                            )
                            x = rod_x_at(
                                t,
                                config.speed_mps,
                                config.start_x_m,
                                config.direction,
                            )
                            rod.position = (x, ROD_Y_M, ROD_CENTER_Z_M)
                            sim_time_text.value = f"{t:.3f} s"
                            set_q(executed[index])
                            commanded.append(executed[index].copy())
                            rod_positions.append(x)
                            if not verification_mode.is_set():
                                time.sleep(TRAJECTORY_DT_S)
                playback_record.clear()
                playback_record.update(
                    {
                        "case": name,
                        "callback_non_main": callback_thread
                        is not threading.main_thread(),
                        "states_commanded": len(commanded),
                        "robot_changed": bool(
                            len(commanded) > 1
                            and not np.allclose(commanded[0], commanded[-1])
                        ),
                        "rod_changed": bool(
                            len(rod_positions) > 1
                            and not np.allclose(rod_positions[0], rod_positions[-1])
                        ),
                        "depth_panel_updated": bool(depth_checksums),
                        "error": None,
                    }
                )
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                playback_error.value = message
                traceback.print_exc(file=__import__("sys").stderr)
                playback_record.clear()
                playback_record.update({"case": name, "error": message})
            finally:
                playback_done.set()

        threading.Thread(
            target=worker_thread,
            name=f"moving-rod-playback-{name}",
            daemon=True,
        ).start()

    selector.on_update(apply_case)
    play.on_click(playback)
    apply_case()
    print(f"SIM_CAMERA_MOVING_ROD_VISER http://127.0.0.1:{actual_port}", flush=True)

    def verify_callbacks() -> dict[str, Any]:
        names = (
            "A no planner obstacle 0.10 m/s",
            "E camera only 0.10 m/s",
            "E blind entry 0.10 m/s",
        )
        records: list[dict[str, Any]] = []
        verification_mode.set()
        try:
            for name in names:
                selector.value = name
                callback = threading.Thread(
                    target=playback,
                    args=(None,),
                    name=f"moving-rod-callback-{name}",
                )
                callback.start()
                callback.join(timeout=5.0)
                if not playback_done.wait(timeout=30.0):
                    records.append({"case": name, "error": "playback timed out"})
                else:
                    records.append(dict(playback_record))
        finally:
            verification_mode.clear()
        return {
            "cases": records,
            "all_passed": all(
                record.get("callback_non_main")
                and record.get("robot_changed")
                and record.get("rod_changed")
                and record.get("depth_panel_updated")
                and record.get("error") is None
                for record in records
            ),
            "default_case": first_name,
            "playback_error": playback_error.value,
        }

    return viewer, verify_callbacks


def build_experiment(
    worker: MovingRodWorker,
    robot: Kinematics,
    robot_config: Path,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]], Any]:
    goal = sixth.goal_from_scene()
    planner, wrist_report = sixth.make_loop_planner(
        robot_config, sixth.CAMERA_FRONT_SEED
    )
    start_q, start_report = sixth.solve_raised_start(robot, planner)
    wrist_report["solved_start_matches_limit_center"] = bool(
        abs(start_q[sixth.WRIST_3_INDEX] - wrist_report["start_rad"]) <= 1.0e-6
    )
    sixth.fourth.update_fit_in_place(planner, None)
    baseline_result = planner.plan_pose(goal, sixth.arm_state(start_q))
    baseline = sixth.fourth.active_trajectory(baseline_result)
    if baseline is None:
        planner.destroy()
        raise RuntimeError(
            "raised no-obstacle baseline failed: "
            + str(sixth.fourth.failure_status(baseline_result))
        )
    baseline = baseline.detach().clone()
    goal_q = baseline[-1]
    scene_preflight = select_fair_scene(robot, start_q, goal_q, baseline,
                                        start_report["camera_world_m"][2])
    print("MOVING_ROD_SCENE " + json.dumps(scene_preflight), flush=True)
    worker.start()
    worker.capture(start_q, ROD_X_MAX_M)  # Warm renderer outside the case clock.
    entry = worker.capture(start_q, ROD_X_MIN_M)
    entry_pixels = int(entry.camera.metadata["rod_pixels"])
    scene_preflight["blind_entry_initial_rod_pixels"] = entry_pixels
    if entry_pixels:
        raise RuntimeError(f"INVALID_SCENE: blind entry initially has {entry_pixels} rod pixels")
    raised_goal = sixth.raised_pose_goal(robot)
    fixed_unknown_top_m = (
        float(start_report["camera_world_m"][2])
        + sixth.FIXED_UNKNOWN_TOP_ABOVE_START_CAMERA_M
    )
    wrist_limits = tuple(wrist_report["effective_planner_limits_rad"])

    results: dict[str, Any] = {
        "schema": 2,
        "scene_preflight": scene_preflight,
        "date": "2026-11-12",
        "simulated_duration_per_case_s": SIM_DURATION_S,
        "rod": {
            "shape": "vertical cylinder",
            "radius_m": ROD_RADIUS_M,
            "top_m": ROD_TOP_M,
            "y_m": ROD_Y_M,
            "x_limits_m": [ROD_X_MIN_M, ROD_X_MAX_M],
            "regular_initial_x_m": min(ROD_START_X_M, ROD_X_MAX_M),
            "blind_initial_x_m": ROD_X_MIN_M,
            "motion": "constant-speed reflected motion; body pose updated each capture",
            "newton_scene_builds": 1,
            "physics_used": False,
        },
        "raised_start": start_report,
        "wrist_3_limit": wrist_report,
        "speed_rule": (
            f"{sixth.NEAR_ADVANCE_RAD:.3f} rad below "
            f"{sixth.NEAR_BOX_CLEARANCE_M:.3f} m fit clearance, otherwise "
            f"{sixth.FAR_ADVANCE_RAD:.3f} rad"
        ),
        "border_rule": {
            "horizontal_margin_m": sixth.HORIZONTAL_UNKNOWN_MARGIN_M,
            "fixed_unknown_top_m": fixed_unknown_top_m,
            "source": "corrected Sixth-pass rule",
        },
        "clock": {
            "one_simulated_clock": True,
            "arm_motion_during_compute": "none; stop-and-go",
            "normal_cycle": "measured compute duration plus executed segment duration",
            "zero_compute_comparison": (
                "measured compute is reported but contributes zero simulated time"
            ),
            "no_plan_hold_s": NO_PLAN_HOLD_S,
        },
        "point_memory": {
            "inside_current_view": "replaced by current frame",
            "outside_current_view_ttl_s": STALE_TTL_S,
        },
        "planner_input_truth_rule": (
            "truth is never planner input except B; truth is otherwise scoring only"
        ),
        "baseline": {
            "states": int(len(baseline)),
            "planner_failure_status": sixth.fourth.failure_status(baseline_result),
        },
        "cases": {},
        "known_limits": [
            "one axis-aligned box for all retained points",
            "floor cut at world z=0.012 m",
            "robot pixels identified by simulator shape index",
            "the eye-in-hand camera sees only in front of and below the gripper",
        ],
    }
    viewer_cases: dict[str, dict[str, Any]] = {}
    for config in case_matrix():
        print(f"MOVING_ROD_CASE_START {config.name}", flush=True)
        summary, cycles, visual = run_case(
            config,
            worker,
            planner,
            robot,
            start_q,
            goal_q,
            raised_goal,
            goal,
            fixed_unknown_top_m,
            wrist_limits,
        )
        results["cases"][config.name] = {
            "summary": summary,
            "cycles": cycles,
        }
        viewer_cases[config.name] = {"config": config, "visuals": visual}
        print(
            "MOVING_ROD_CASE_RESULT "
            + json.dumps({"name": config.name, **summary}, sort_keys=True),
            flush=True,
        )

    results["fastest_speed_b_survives_mps"] = max((
        v["summary"]["rod_speed_mps"] for n, v in results["cases"].items()
        if n.startswith("B ") and v["summary"]["true_collisions"] == 0
        and v["summary"]["reaches"] > 0), default=None)
    surviving_speeds = [
        value["summary"]["rod_speed_mps"]
        for name, value in results["cases"].items()
        if name.startswith("E camera only")
        and "zero compute" not in name
        and value["summary"]["true_collisions"] == 0
        and value["summary"]["reaches"] > 0
    ]
    results["fastest_speed_e_survives_mps"] = (
        max(surviving_speeds) if surviving_speeds else None
    )
    a_cases = [
        value["summary"]
        for name, value in results["cases"].items()
        if name.startswith("A ")
    ]
    results["case_a_scene_evidence"] = {
        "all_a_cases_collide": all(case["true_collisions"] > 0 for case in a_cases),
        "collisions_by_speed": {
            f"{case['rod_speed_mps']:.2f}": case["true_collisions"]
            for case in a_cases
        },
    }
    results["scene_preflight"]["status"] = ("VALID_SCENE" if
        results["case_a_scene_evidence"]["all_a_cases_collide"] else "INVALID_SCENE")
    counted = results["cases"]["E camera only 0.10 m/s"]["summary"]
    zero = results["cases"][
        "E camera only 0.10 m/s zero compute latency"
    ]["summary"]
    results["latency_comparison_0.10_mps"] = {
        "counted": counted,
        "zero_compute": zero,
        "collision_delta_counted_minus_zero": (
            counted["true_collisions"] - zero["true_collisions"]
        ),
        "clearance_delta_counted_minus_zero_m": (
            counted["minimum_true_clearance_m"]
            - zero["minimum_true_clearance_m"]
        ),
    }
    return results, viewer_cases, planner


def save_results(results: dict[str, Any], worker_lines: int) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    json_path = RESULTS_DIR / "moving_rod_v2.json"
    log_path = RESULTS_DIR / "moving_rod_v2.log"
    json_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    lines = [
        "SIM_CAMERA_MOVING_ROD",
        "summary="
        + json.dumps(
            {
                name: value["summary"]
                for name, value in results["cases"].items()
            },
            sort_keys=True,
        ),
        "fastest_speed_b_survives_mps="
        + json.dumps(results["fastest_speed_b_survives_mps"]),
        "fastest_speed_e_survives_mps="
        + json.dumps(results["fastest_speed_e_survives_mps"]),
        "case_a_scene_evidence="
        + json.dumps(results["case_a_scene_evidence"], sort_keys=True),
        "latency_comparison="
        + json.dumps(results["latency_comparison_0.10_mps"], sort_keys=True),
        f"worker_lines={worker_lines}",
    ]
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8084)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--save-results", action="store_true")
    parser.add_argument(
        "--newton-python",
        type=Path,
        default=Path.home() / "newton_ws/.venv-cpu/bin/python",
    )
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=Path.home()
        / ".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml",
    )
    args = parser.parse_args()
    robot = Kinematics(
        KinematicsCfg.from_robot_yaml_file(
            str(args.robot_config), tool_frames=[sixth.preflight.TOOL_FRAME]
        )
    )
    worker_log: list[str] = []
    planner = None
    viewer = None
    with tempfile.TemporaryDirectory(prefix="newton-moving-rod-") as temp:
        worker = MovingRodWorker(args.newton_python, Path(temp))
        try:
            results, viewer_cases, planner = build_experiment(
                worker, robot, args.robot_config
            )
        finally:
            worker.close()
            worker_log = worker.log
    results["newton_worker_lines"] = len(worker_log)
    try:
        if not args.headless:
            viewer, verify = start_viewer(
                args.robot_config,
                robot,
                np.asarray(results["raised_start"]["joints_rad"], dtype=np.float32),
                viewer_cases,
                args.port,
            )
            results["playback_verification"] = verify()
            print(
                "SIM_CAMERA_MOVING_ROD_PLAYBACK "
                + json.dumps(results["playback_verification"], sort_keys=True),
                flush=True,
            )
            if not results["playback_verification"]["all_passed"]:
                raise RuntimeError("moving-rod viewer playback verification failed")
            started = time.monotonic()
            while args.duration <= 0.0 or time.monotonic() - started < args.duration:
                time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer._server.stop()
        if planner is not None:
            planner.destroy()
    if args.save_results:
        save_results(results, len(worker_log))
        print("SIM_CAMERA_MOVING_ROD_RESULTS_SAVED", flush=True)
    else:
        print("SIM_CAMERA_MOVING_ROD_RESULTS_NOT_SAVED", flush=True)
    print(
        "SIM_CAMERA_MOVING_ROD_RESULT "
        + json.dumps(
            {
                "summaries": {
                    name: value["summary"]
                    for name, value in results["cases"].items()
                },
                "fastest_speed_e_survives_mps": results[
                    "fastest_speed_e_survives_mps"
                ],
                "playback_verification": results.get("playback_verification"),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
