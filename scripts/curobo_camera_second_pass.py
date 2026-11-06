#!/usr/bin/env python3
"""Fourth-pass camera planning with the mount facing away from the base."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from typing import Any

import numpy as np
import torch
import curobo

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer


PROTOTYPE = Path(__file__).resolve().parents[1] / "docs/experiments/curobo-v2/prototype"
sys.path.insert(0, str(PROTOTYPE))
import ur5_viser_preflight as preflight  # noqa: E402


REPO = Path(__file__).resolve().parents[1]
ORIGINAL_START_ARM = np.array(
    [-2.6888591245, -2.0617183994, -1.2870012279, 1.7779233188, 4.7123889501, -5.8304517710],
    dtype=np.float32,
)
MOUNT_MATCH_ARM = np.array(
    [0.0, -math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0, -math.pi / 2.0, 0.0],
    dtype=np.float32,
)
EXPERIMENT_TOOL_Z_FLIP_RAD = math.pi
WRIST_3_INDEX = 5
START_ARM = ORIGINAL_START_ARM.copy()
START_ARM[WRIST_3_INDEX] += EXPERIMENT_TOOL_Z_FLIP_RAD
MAIN_CENTER_XY = (0.487, 0.263)
MAIN_DIMS_XY = (0.200, 0.060)
WALL_TOPS_M = (0.18, 0.22, 0.26)
WALL_BOTTOM_M = 0.01
# Fixed deterministic search grid.  Narrow boxes offset in X can intersect a
# robot-link sphere on the route without covering the tool goal.  Y is searched
# explicitly because the requested fallback permits shifting the old blind wall.
BLIND_CANDIDATES = tuple(
    (
        np.array((x, y, z), dtype=np.float64),
        np.array((0.050, 0.040, 0.060), dtype=np.float64),
    )
    for z in (0.38, 0.34, 0.30)
    for y in (0.22, 0.26, 0.30, 0.34, 0.38)
    for x in (0.407, 0.447, 0.487, 0.527, 0.567, 0.607, 0.647)
) + tuple(
    (
        np.array((x, y, 0.130), dtype=np.float64),
        np.array((0.050, 0.040, 0.240), dtype=np.float64),
    )
    for y in (0.38, 0.36, 0.34, 0.32, 0.30, 0.28, 0.26)
    for x in (0.247, 0.287, 0.327, 0.367, 0.407, 0.447, 0.487,
              0.527, 0.567, 0.607, 0.647, 0.687, 0.727)
)
PADDINGS_M = (0.0, 0.02)
FIT_PADDING_XY_M = 0.015
MIN_POINTS = 40
E_ADVANCE_RAD = 0.05
E_MAX_CYCLES = 12
GOAL_TOLERANCE_M = 0.006
MIN_ENDPOINT_CLEARANCE_M = 0.025
MOUNT_CONFIG = REPO / "config/sim_camera_mount.yaml"


def multiply_wxyz(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    value = np.array((
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ))
    return value / np.linalg.norm(value)


def rotate_goal_about_tool_z(quaternion_wxyz: np.ndarray) -> np.ndarray:
    local_z_flip = np.array((
        math.cos(0.5 * EXPERIMENT_TOOL_Z_FLIP_RAD), 0.0, 0.0,
        math.sin(0.5 * EXPERIMENT_TOOL_Z_FLIP_RAD),
    ))
    return multiply_wxyz(np.asarray(quaternion_wxyz), local_z_flip)


def wall_geometry(top_m: float) -> tuple[np.ndarray, np.ndarray]:
    height = top_m - WALL_BOTTOM_M
    center = np.array((*MAIN_CENTER_XY, WALL_BOTTOM_M + 0.5 * height), dtype=np.float64)
    dims = np.array((*MAIN_DIMS_XY, height), dtype=np.float64)
    return center, dims


def arm_state(q: np.ndarray | torch.Tensor) -> JointState:
    value = torch.as_tensor(q, device="cuda", dtype=torch.float32).reshape(1, -1)
    return JointState.from_position(value, joint_names=list(preflight.ARM_JOINT_NAMES))


def pose_goal(robot: Kinematics, q: np.ndarray | torch.Tensor) -> GoalToolPose:
    poses = robot.compute_kinematics(arm_state(q)).tool_poses[preflight.TOOL_FRAME]
    position = poses.position.detach().cpu().reshape(-1, 3)[0].tolist()
    quaternion = poses.quaternion.detach().cpu().reshape(-1, 4)[0].tolist()
    pose = Pose.from_list([*position, *quaternion])
    return GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )


def observation_goals(robot: Kinematics) -> tuple[tuple[GoalToolPose, ...], GoalToolPose]:
    """Create wall-independent raised/downward observation poses from start FK."""
    poses = robot.compute_kinematics(arm_state(START_ARM)).tool_poses[
        preflight.TOOL_FRAME
    ]
    start_position = poses.position.detach().cpu().reshape(-1, 3)[0].numpy()
    quaternion = poses.quaternion.detach().cpu().reshape(-1, 4)[0].numpy()

    def goal(offset: tuple[float, float, float]) -> GoalToolPose:
        position = start_position + np.asarray(offset)
        pose = Pose.from_list([*position.tolist(), *quaternion.tolist()])
        return GoalToolPose.from_poses(
            {preflight.TOOL_FRAME: pose},
            ordered_tool_frames=[preflight.TOOL_FRAME],
            num_goalset=1,
        )

    overview = goal((0.0, 0.0, 0.20))
    standard = (
        overview,
        goal((0.12, 0.0, 0.20)),
        goal((-0.12, 0.0, 0.20)),
    )
    return standard, overview


def active_trajectory(result: Any) -> torch.Tensor | None:
    if result is None or result.success is None or not result.success.any():
        return None
    trajectory = preflight.effective_trajectory(
        result.interpolated_trajectory, result.interpolated_last_tstep
    ).reorder(list(preflight.ARM_JOINT_NAMES))
    return trajectory.position.reshape(-1, len(preflight.ARM_JOINT_NAMES))


def failure_status(result: Any) -> str | None:
    """Return a stable failure label for every unsuccessful planner call."""
    if active_trajectory(result) is not None:
        return None
    if result is None:
        return "NO_RESULT_IK_OR_GRAPH_FAILED"
    success = getattr(result, "success", None)
    if success is None:
        return "RESULT_WITHOUT_SUCCESS_FIELD"
    return "TRAJOPT_FAILED"


def fit_cuboid(points: np.ndarray, z_padding_m: float) -> dict[str, Any] | None:
    if len(points) < MIN_POINTS:
        return None
    low = np.min(points, axis=0) - np.array((FIT_PADDING_XY_M, FIT_PADDING_XY_M, z_padding_m))
    high = np.max(points, axis=0) + np.array((FIT_PADDING_XY_M, FIT_PADDING_XY_M, z_padding_m))
    low[2] = max(float(low[2]), 0.0)
    return {
        "center_m": (0.5 * (low + high)).tolist(),
        "dims_m": np.maximum(high - low, 0.01).tolist(),
        "highest_seen_z_m": float(np.max(points[:, 2])),
        "fitted_top_z_m": float(high[2]),
        "points": int(len(points)),
        "z_padding_m": z_padding_m,
    }


def planner_for_fit(robot_config: Path, fit: dict[str, Any] | None):
    cuboids: dict[str, Any] = {}
    if fit is not None:
        cuboids["camera_obstacle"] = {
            "dims": fit["dims_m"],
            "pose": [*fit["center_m"], 1.0, 0.0, 0.0, 0.0],
        }
    return preflight.create_pick_planner(robot_config, {"cuboid": cuboids})


def reusable_fit_planner(robot_config: Path):
    """Warm one planner with a cuboid slot that E updates in-place."""
    placeholder = {
        "center_m": [0.0, 0.0, -10.0],
        "dims_m": [0.01, 0.01, 0.01],
    }
    planner = planner_for_fit(robot_config, placeholder)
    planner.scene_collision_checker.enable_obstacle("camera_obstacle", False)
    return planner


def update_fit_in_place(planner: Any, fit: dict[str, Any] | None) -> None:
    checker = planner.scene_collision_checker
    if fit is None:
        checker.enable_obstacle("camera_obstacle", False)
        return
    checker.update_obstacle_pose(
        "camera_obstacle",
        Pose.from_list([*fit["center_m"], 1.0, 0.0, 0.0, 0.0]),
    )
    if checker.data.cuboids is None:
        raise RuntimeError("cuRobo cuboid scene data is unavailable")
    checker.data.cuboids.update_dims(
        "camera_obstacle",
        torch.as_tensor(fit["dims_m"], device="cuda", dtype=torch.float32),
    )
    checker.enable_obstacle("camera_obstacle", True)
    if planner.graph_planner is not None:
        planner.graph_planner.reset_buffer()


def clearance_series(
    robot: Kinematics,
    positions: torch.Tensor,
    center: np.ndarray,
    dims: np.ndarray,
) -> np.ndarray:
    state = JointState.from_position(
        positions.reshape(-1, len(preflight.ARM_JOINT_NAMES)),
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    spheres = robot.compute_kinematics(state).robot_spheres
    if spheres is None:
        raise RuntimeError("cuRobo did not return collision spheres")
    values = spheres.detach().cpu().numpy()
    relative = np.abs(values[..., :3] - center) - 0.5 * dims
    outside = np.linalg.norm(np.maximum(relative, 0.0), axis=-1)
    inside = np.minimum(np.max(relative, axis=-1), 0.0)
    # Kinematics returns [batch, horizon?, spheres, 4].  Collapse every
    # non-state dimension so callers always receive scalar clearance per q.
    signed = outside + inside - values[..., 3]
    return np.min(signed.reshape(len(positions), -1), axis=1)


def state_clearance(
    robot: Kinematics, q: np.ndarray | torch.Tensor, center: np.ndarray, dims: np.ndarray
) -> float:
    value = clearance_series(robot, arm_state(q).position, center, dims)
    return float(value[0])


def scene_validity(
    robot: Kinematics,
    baseline_path: torch.Tensor,
    center: np.ndarray,
    dims: np.ndarray,
) -> dict[str, Any]:
    start = state_clearance(robot, baseline_path[0], center, dims)
    goal = state_clearance(robot, baseline_path[-1], center, dims)
    along_values = clearance_series(robot, baseline_path, center, dims)
    interior = along_values[1:-1] if len(along_values) > 2 else along_values
    along = float(np.min(interior))
    reasons: list[str] = []
    if start < MIN_ENDPOINT_CLEARANCE_M:
        reasons.append(
            f"start clearance {start:.6f} m is below {MIN_ENDPOINT_CLEARANCE_M:.6f} m"
        )
    if goal < MIN_ENDPOINT_CLEARANCE_M:
        reasons.append(
            f"goal clearance {goal:.6f} m is below {MIN_ENDPOINT_CLEARANCE_M:.6f} m"
        )
    if along >= 0.0:
        reasons.append(f"no-obstacle path does not collide (along-path {along:.6f} m)")
    return {
        "status": "VALID" if not reasons else "INVALID_SCENE",
        "valid": not reasons,
        "start_clearance_m": start,
        "goal_clearance_m": goal,
        "along_path_clearance_m": along,
        "reasons": reasons,
    }


def joint_path_length(positions: torch.Tensor) -> float:
    values = positions.detach().cpu().numpy()
    return float(np.linalg.norm(np.diff(values, axis=0), axis=1).sum())


def tool_position(robot: Kinematics, q: torch.Tensor) -> np.ndarray:
    pose = robot.compute_kinematics(arm_state(q)).tool_poses[preflight.TOOL_FRAME]
    return pose.position.detach().cpu().reshape(-1, 3)[0].numpy()


def tool_points(robot: Kinematics, positions: torch.Tensor) -> np.ndarray:
    state = JointState.from_position(
        positions.reshape(-1, len(preflight.ARM_JOINT_NAMES)),
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    pose = robot.compute_kinematics(state).tool_poses[preflight.TOOL_FRAME]
    return pose.position.detach().cpu().numpy().reshape(-1, 3)


@dataclass
class Capture:
    q: np.ndarray
    points: np.ndarray
    camera_transform: np.ndarray
    metadata: dict[str, Any]


@dataclass(frozen=True)
class MountConfig:
    parent_link: str
    left_m: float
    up_m: float
    forward_m: float
    tilt_deg: float


def read_mount_config(path: Path) -> MountConfig:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return MountConfig(
        parent_link=values["parent_link"], left_m=float(values["left_m"]),
        up_m=float(values["up_m"]), forward_m=float(values["forward_m"]),
        tilt_deg=float(values["tilt_deg"]),
    )


class NewtonWorker:
    def __init__(self, python: Path, temp_dir: Path):
        self.temp_dir = temp_dir
        self.counter = 0
        self.log: list[str] = []
        self.process = subprocess.Popen(
            [str(python), str(REPO / "scripts/newton_camera_capture_worker.py")],
            cwd=REPO,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._read_prefix("NEWTON_CAMERA_WORKER_READY ")

    def _read_prefix(self, prefix: str) -> str:
        if self.process.stdout is None:
            raise RuntimeError("Newton worker stdout is unavailable")
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError(
                    f"Newton worker exited before {prefix!r}: " + "".join(self.log[-20:])
                )
            self.log.append(line)
            if line.startswith(prefix):
                return line[len(prefix) :].strip()

    def capture(self, q: np.ndarray, center: np.ndarray, dims: np.ndarray) -> Capture:
        self.counter += 1
        output = self.temp_dir / f"capture_{self.counter:04d}.npz"
        request = {
            "id": self.counter,
            "arm_q": np.asarray(q, dtype=float).tolist(),
            "center_m": center.tolist(),
            "dims_m": dims.tolist(),
            "output": str(output),
        }
        if self.process.stdin is None:
            raise RuntimeError("Newton worker stdin is unavailable")
        self.process.stdin.write(json.dumps(request) + "\n")
        self.process.stdin.flush()
        response = json.loads(self._read_prefix("NEWTON_CAMERA_WORKER_RESPONSE "))
        arrays = np.load(output)
        capture = Capture(
            q=np.asarray(q, dtype=np.float32).copy(),
            points=arrays["points"].copy(),
            camera_transform=arrays["camera_transform"].copy(),
            metadata=response,
        )
        output.unlink()
        return capture

    def close(self) -> None:
        if self.process.poll() is None and self.process.stdin is not None:
            self.process.stdin.write(json.dumps({"command": "shutdown"}) + "\n")
            self.process.stdin.flush()
            self.process.wait(timeout=10.0)


def concatenate_paths(paths: list[torch.Tensor]) -> torch.Tensor:
    if not paths:
        return torch.as_tensor(START_ARM, device="cuda").reshape(1, -1)
    return torch.cat(paths, dim=0)


def run_scan(
    worker: NewtonWorker,
    robot: Kinematics,
    robot_config: Path,
    center: np.ndarray,
    dims: np.ndarray,
    targets: tuple[GoalToolPose, ...],
    padding_m: float,
) -> dict[str, Any]:
    current = torch.as_tensor(START_ARM, device="cuda", dtype=torch.float32)
    captures = [worker.capture(START_ARM, center, dims)]
    paths: list[torch.Tensor] = []
    scan_min = math.inf
    collision = False
    no_plan = False
    failure_statuses: list[dict[str, Any]] = []
    target_sequence = (*targets, pose_goal(robot, START_ARM))
    for target_index, target in enumerate(target_sequence):
        merged = np.concatenate([capture.points for capture in captures], axis=0)
        fit = fit_cuboid(merged, padding_m)
        planner = planner_for_fit(robot_config, fit)
        result = planner.plan_pose(target, arm_state(current))
        positions = active_trajectory(result)
        del planner
        torch.cuda.empty_cache()
        if positions is None:
            no_plan = True
            failure_statuses.append(
                {"target_index": target_index, "status": failure_status(result)}
            )
            break
        clearances = clearance_series(robot, positions, center, dims)
        scan_min = min(scan_min, float(np.min(clearances)))
        paths.append(positions.detach().clone())
        current = positions[-1].detach().clone()
        if np.min(clearances) < 0.0:
            collision = True
            break
        if target_index < len(targets):
            captures.append(
                worker.capture(current.detach().cpu().numpy(), center, dims)
            )
    return {
        "captures": captures,
        "paths": paths,
        "end_q": current,
        "returned_to_start": bool(
            not collision and not no_plan and len(paths) == len(target_sequence)
        ),
        "scan_collision": collision,
        "scan_no_plan": no_plan,
        "scan_min_clearance_m": None if math.isinf(scan_min) else scan_min,
        "failure_statuses": failure_statuses,
    }


def evaluate_final_paths(
    robot: Kinematics,
    robot_config: Path,
    fit: dict[str, Any] | None,
    center: np.ndarray,
    dims: np.ndarray,
    goal: GoalToolPose,
    repeats: int,
) -> tuple[dict[str, Any], torch.Tensor | None]:
    planner = planner_for_fit(robot_config, fit)
    records: list[dict[str, Any]] = []
    display_path: torch.Tensor | None = None
    for repeat in range(repeats):
        torch.manual_seed(2000 + repeat)
        torch.cuda.synchronize()
        started = time.perf_counter()
        result = planner.plan_pose(goal, arm_state(START_ARM))
        torch.cuda.synchronize()
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        positions = active_trajectory(result)
        record = {
            "repeat": repeat,
            "success": positions is not None,
            "planning_ms": elapsed_ms,
            "true_clearance_m": None,
            "start_clearance_m": state_clearance(robot, START_ARM, center, dims),
            "goal_clearance_m": None,
            "along_path_clearance_m": None,
            "path_length_rad": None,
            "failure_status": failure_status(result),
        }
        if positions is not None:
            values = clearance_series(robot, positions, center, dims)
            interior = values[1:-1] if len(values) > 2 else values
            record["true_clearance_m"] = float(np.min(values))
            record["goal_clearance_m"] = float(values[-1])
            record["along_path_clearance_m"] = float(np.min(interior))
            record["path_length_rad"] = joint_path_length(positions)
            if display_path is None:
                display_path = positions.detach().clone()
        records.append(record)
    del planner
    torch.cuda.empty_cache()
    successes = [record for record in records if record["success"]]
    summary = {
        "attempts": repeats,
        "successes": len(successes),
        "planning_median_ms": float(
            statistics.median(record["planning_ms"] for record in records)
        ),
        "minimum_true_clearance_m": (
            float(min(record["true_clearance_m"] for record in successes))
            if successes
            else None
        ),
        "minimum_along_path_clearance_m": (
            float(min(record["along_path_clearance_m"] for record in successes))
            if successes
            else None
        ),
        "path_length_median_rad": (
            float(statistics.median(record["path_length_rad"] for record in successes))
            if successes
            else None
        ),
        "paths_identical": (
            len(successes) > 1
            and max(record["path_length_rad"] for record in successes)
            - min(record["path_length_rad"] for record in successes)
            < 1.0e-5
        ),
        "records": records,
    }
    return summary, display_path


def merged_points(scan: dict[str, Any]) -> np.ndarray:
    captures: list[Capture] = scan["captures"]
    return np.concatenate([capture.points for capture in captures], axis=0)


def serializable_scan(scan: dict[str, Any]) -> dict[str, Any]:
    captures: list[Capture] = scan["captures"]
    return {
        "capture_count": len(captures),
        "capture_points": [int(len(capture.points)) for capture in captures],
        "capture_ms": [capture.metadata["capture_ms"] for capture in captures],
        "highest_seen_z_m": (
            float(max(np.max(capture.points[:, 2]) for capture in captures if len(capture.points)))
            if any(len(capture.points) for capture in captures)
            else None
        ),
        "scan_collision": scan["scan_collision"],
        "scan_no_plan": scan["scan_no_plan"],
        "scan_min_clearance_m": scan["scan_min_clearance_m"],
        "returned_to_start": scan["returned_to_start"],
        "failure_statuses": scan["failure_statuses"],
    }


def advance_index(positions: torch.Tensor, distance_rad: float) -> int:
    values = positions.detach().cpu().numpy()
    if len(values) <= 1:
        return 0
    cumulative = np.cumsum(np.linalg.norm(np.diff(values, axis=0), axis=1))
    return min(int(np.searchsorted(cumulative, distance_rad)) + 1, len(values) - 1)


def run_look_while_moving(
    worker: NewtonWorker,
    robot: Kinematics,
    robot_config: Path,
    center: np.ndarray,
    dims: np.ndarray,
    goal: GoalToolPose,
    goal_q: torch.Tensor,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    current = torch.as_tensor(START_ARM, device="cuda", dtype=torch.float32)
    accumulated: list[np.ndarray] = []
    history: list[dict[str, Any]] = []
    capture_ms: list[float] = []
    plan_ms: list[float] = []
    cycle_ms: list[float] = []
    minimum_clearance = math.inf
    no_plan_cycles = 0
    first_seen_cycle: int | None = None
    first_seen_clearance: float | None = None
    goal_reached = False
    collision = False
    planner = reusable_fit_planner(robot_config)
    for cycle in range(1, E_MAX_CYCLES + 1):
        cycle_started = time.perf_counter()
        capture = worker.capture(current.detach().cpu().numpy(), center, dims)
        capture_ms.append(float(capture.metadata["capture_ms"]))
        if len(capture.points):
            accumulated.append(capture.points)
            if first_seen_cycle is None:
                first_seen_cycle = cycle
                current_clearance = clearance_series(
                    robot, current.reshape(1, -1), center, dims
                )
                first_seen_clearance = float(np.asarray(current_clearance).reshape(-1)[0])
        points = (
            np.concatenate(accumulated, axis=0)
            if accumulated
            else np.empty((0, 3), dtype=np.float32)
        )
        fit = fit_cuboid(points, 0.02)
        update_fit_in_place(planner, fit)
        current_fit_clearance = (
            None
            if fit is None
            else state_clearance(
                robot, current, np.asarray(fit["center_m"]), np.asarray(fit["dims_m"])
            )
        )
        goal_fit_clearance = (
            None
            if fit is None
            else state_clearance(
                robot, goal_q, np.asarray(fit["center_m"]), np.asarray(fit["dims_m"])
            )
        )
        torch.cuda.synchronize()
        started = time.perf_counter()
        result = planner.plan_pose(goal, arm_state(current))
        torch.cuda.synchronize()
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        plan_ms.append(elapsed_ms)
        positions = active_trajectory(result)
        planner_failure = failure_status(result)
        whole_cycle_ms = (time.perf_counter() - cycle_started) * 1000.0
        cycle_ms.append(whole_cycle_ms)
        record: dict[str, Any] = {
            "cycle": cycle,
            "q": current.detach().cpu().numpy(),
            "capture": capture,
            "points": points.copy(),
            "fit": fit,
            "plan_ms": elapsed_ms,
            "whole_cycle_ms": whole_cycle_ms,
            "planner_failure_status": planner_failure,
            "current_to_fit_clearance_m": current_fit_clearance,
            "goal_to_fit_clearance_m": goal_fit_clearance,
            "executed": None,
            "clearances": None,
        }
        if positions is None:
            no_plan_cycles += 1
            if current_fit_clearance is not None and current_fit_clearance < 0.0:
                record["stop_reason"] = "FITTED_BOX_COVERS_CURRENT_STATE"
            elif goal_fit_clearance is not None and goal_fit_clearance < 0.0:
                record["stop_reason"] = "FITTED_BOX_COVERS_GOAL_STATE"
            else:
                record["stop_reason"] = planner_failure
            history.append(record)
            break
        index = advance_index(positions, E_ADVANCE_RAD)
        executed = positions[: index + 1].detach().clone()
        clearances = clearance_series(robot, executed, center, dims)
        minimum_clearance = min(minimum_clearance, float(np.min(clearances)))
        record["executed"] = executed
        record["clearances"] = clearances
        history.append(record)
        current = executed[-1].detach().clone()
        if np.min(clearances) < 0.0:
            collision = True
            record["stop_reason"] = "TRUE_COLLISION"
            break
        goal_error = float(
            np.linalg.norm(tool_position(robot, current) - tool_position(robot, positions[-1]))
        )
        if index == len(positions) - 1 or goal_error <= GOAL_TOLERANCE_M:
            goal_reached = True
            record["stop_reason"] = "GOAL_REACHED"
            break
    cold_cycle_ms = cycle_ms[0] if cycle_ms else None
    del planner
    torch.cuda.empty_cache()
    without_box_ms = [
        float(record["whole_cycle_ms"]) for record in history if record["fit"] is None
    ]
    with_box_ms = [
        float(record["whole_cycle_ms"]) for record in history if record["fit"] is not None
    ]

    def timing_group(values: list[float]) -> dict[str, Any]:
        return {
            "cycles": len(values),
            "median_ms": float(statistics.median(values)) if values else None,
            "p95_ms": float(np.percentile(values, 95)) if values else None,
        }

    final_stop_reason = history[-1].get("stop_reason") if history else None
    if history and final_stop_reason is None and len(history) == E_MAX_CYCLES:
        final_stop_reason = "MAX_CYCLES_REACHED"
        history[-1]["stop_reason"] = final_stop_reason
    summary = {
        "cycles": len(history),
        "cycles_without_plan": no_plan_cycles,
        "minimum_true_clearance_m": (
            None if math.isinf(minimum_clearance) else minimum_clearance
        ),
        "goal_reached": goal_reached,
        "true_collision": collision,
        "first_seen_cycle": first_seen_cycle,
        "clearance_when_first_seen_m": first_seen_clearance,
        "capture_median_ms": float(statistics.median(capture_ms)),
        "planning_median_ms": float(statistics.median(plan_ms)),
        "whole_cycle_cold_ms": cold_cycle_ms,
        "whole_cycle_warm_median_ms": (
            float(statistics.median(cycle_ms[1:])) if len(cycle_ms) > 1 else None
        ),
        "whole_cycle_without_box": timing_group(without_box_ms),
        "whole_cycle_with_box": timing_group(with_box_ms),
        "planner_reuse": (
            "one warmed planner; camera_obstacle pose and CuboidData dimensions "
            "updated in-place each cycle"
        ),
        "final_stop_reason": final_stop_reason,
        "advance_joint_length_rad": E_ADVANCE_RAD,
    }
    return summary, history


def compact_capture(capture: Capture) -> dict[str, Any]:
    return {
        "q": capture.q.tolist(),
        "points": int(len(capture.points)),
        "camera_transform": capture.camera_transform.tolist(),
        "metadata": capture.metadata,
    }


def make_baseline_path(
    robot_config: Path, goal: GoalToolPose, start_q: np.ndarray = START_ARM
) -> tuple[torch.Tensor, dict[str, Any]]:
    planner = planner_for_fit(robot_config, None)
    torch.manual_seed(1901)
    result = planner.plan_pose(goal, arm_state(start_q))
    path = active_trajectory(result)
    status = failure_status(result)
    del planner
    torch.cuda.empty_cache()
    if path is None:
        raise RuntimeError(f"no-obstacle baseline path failed: {status}")
    return path.detach().clone(), {
        "states": int(len(path)),
        "joint_path_length_rad": joint_path_length(path),
        "failure_status": status,
    }


def select_blind_scene(
    worker: NewtonWorker, robot: Kinematics, baseline_path: torch.Tensor
) -> tuple[np.ndarray, np.ndarray, dict[str, Any], list[dict[str, Any]]]:
    attempts: list[dict[str, Any]] = []
    for center, dims in BLIND_CANDIDATES:
        validity = scene_validity(robot, baseline_path, center, dims)
        record: dict[str, Any] = {
            "center_m": center.tolist(),
            "dims_m": dims.tolist(),
            "validity": validity,
            "start_view_points": None,
        }
        if validity["valid"]:
            capture = worker.capture(START_ARM, center, dims)
            record["start_view_points"] = int(len(capture.points))
            record["start_view_obstacle_pixels"] = int(
                capture.metadata.get("obstacle_pixels", len(capture.points))
            )
            record["model_start_points"] = {
                "flange_m": capture.metadata.get("flange_m"),
                "wrist_1_center_m": capture.metadata.get("wrist_1_center_m"),
                "source": "Newton FK from the loaded URDF model",
            }
            if len(capture.points) == 0:
                attempts.append(record)
                return center.copy(), dims.copy(), validity, attempts
        attempts.append(record)
    raise RuntimeError(
        "no blind-wall candidate both passed validity and was unseen from start: "
        + json.dumps(attempts)
    )


def model_start_points(robot: Kinematics) -> dict[str, Any]:
    q = arm_state(START_ARM).position
    tool = robot.get_link_poses(q, [preflight.TOOL_FRAME])
    result: dict[str, Any] = {
        "flange_tool0_m": tool.position.detach().cpu().reshape(-1, 3)[0].tolist()
    }
    for name in ("ur/wrist_1_link", "wrist_1_link"):
        try:
            pose = robot.get_link_poses(q, [name])
            result["wrist_1_link_name"] = name
            result["wrist_1_center_m"] = (
                pose.position.detach().cpu().reshape(-1, 3)[0].tolist()
            )
            break
        except Exception:
            continue
    if "wrist_1_center_m" not in result:
        result["wrist_1_error"] = "link name not present in loaded model"
    return result


def frame_position(
    robot: Kinematics, q: np.ndarray | torch.Tensor, frame: str
) -> np.ndarray:
    pose = robot.get_link_poses(arm_state(q).position, [frame])
    return pose.position.detach().cpu().reshape(-1, 3)[0].numpy()


def sphere_set_symmetry(
    robot: Kinematics, before_q: np.ndarray, after_q: np.ndarray
) -> dict[str, Any]:
    before = robot.compute_kinematics(arm_state(before_q)).robot_spheres
    after = robot.compute_kinematics(arm_state(after_q)).robot_spheres
    if before is None or after is None:
        raise RuntimeError("robot collision spheres unavailable for symmetry check")
    a = before.detach().cpu().numpy().reshape(-1, 4)
    b = after.detach().cpu().numpy().reshape(-1, 4)
    distances = np.linalg.norm(a[:, None, :3] - b[None, :, :3], axis=2)
    radius_match = np.isclose(a[:, None, 3], b[None, :, 3], atol=1.0e-6)
    distances[~radius_match] = np.inf
    deviation = max(float(np.max(np.min(distances, axis=1))),
                    float(np.max(np.min(distances, axis=0))))
    return {
        "collision_spheres": int(len(a)),
        "bidirectional_max_center_deviation_m": deviation,
        "symmetric_enough": deviation <= 0.002,
        "criterion_m": 0.002,
    }


def experiment_pose_report(
    robot: Kinematics,
    original_goal_q: torch.Tensor,
    flipped_goal_q: torch.Tensor,
) -> dict[str, Any]:
    mount = read_mount_config(MOUNT_CONFIG)

    def pose_pair(q: np.ndarray | torch.Tensor) -> dict[str, Any]:
        camera, _ = camera_pose_for_q(robot, torch.as_tensor(q, device="cuda"), mount)
        return {
            "flange_m": frame_position(robot, q, "flange").tolist(),
            "tool0_m": frame_position(robot, q, preflight.TOOL_FRAME).tolist(),
            "camera_world_m": camera.tolist(),
        }

    base_camera, _ = camera_pose_for_q(
        robot, torch.as_tensor(MOUNT_MATCH_ARM, device="cuda"), mount
    )
    original_camera, _ = camera_pose_for_q(
        robot, torch.as_tensor(ORIGINAL_START_ARM, device="cuda"), mount
    )
    base_flange = frame_position(robot, MOUNT_MATCH_ARM, preflight.TOOL_FRAME)
    original_flange = frame_position(robot, ORIGINAL_START_ARM, preflight.TOOL_FRAME)
    return {
        "tool_z_flip_rad": EXPERIMENT_TOOL_Z_FLIP_RAD,
        "wrist_3_joint_before_rad": float(ORIGINAL_START_ARM[WRIST_3_INDEX]),
        "wrist_3_joint_after_rad": float(START_ARM[WRIST_3_INDEX]),
        "wrist_3_urdf_limits_rad": [-2.0 * math.pi, 2.0 * math.pi],
        "mount_offset_world_checks": {
            "mount_match_arm_camera_minus_flange_m": (base_camera - base_flange).tolist(),
            "original_start_camera_minus_flange_m": (
                original_camera - original_flange
            ).tolist(),
        },
        "start_before": pose_pair(ORIGINAL_START_ARM),
        "start_after": pose_pair(START_ARM),
        "goal_before": pose_pair(original_goal_q),
        "goal_after": pose_pair(flipped_goal_q),
        "gripper_symmetry": sphere_set_symmetry(
            robot, ORIGINAL_START_ARM, START_ARM
        ),
    }


def exact_truth_fit(center: np.ndarray, dims: np.ndarray) -> dict[str, Any]:
    return {
        "center_m": center.tolist(),
        "dims_m": dims.tolist(),
        "highest_seen_z_m": float(center[2] + 0.5 * dims[2]),
        "fitted_top_z_m": float(center[2] + 0.5 * dims[2]),
        "points": 0,
        "z_padding_m": 0.0,
    }


def serializable_e_cycles(history: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "cycle": record["cycle"],
            "capture": compact_capture(record["capture"]),
            "fit": record["fit"],
            "plan_ms": record["plan_ms"],
            "whole_cycle_ms": record["whole_cycle_ms"],
            "planner_failure_status": record["planner_failure_status"],
            "current_to_fit_clearance_m": record["current_to_fit_clearance_m"],
            "goal_to_fit_clearance_m": record["goal_to_fit_clearance_m"],
            "stop_reason": record.get("stop_reason"),
            "executed_states": (
                0 if record["executed"] is None else int(len(record["executed"]))
            ),
            "minimum_true_clearance_m": (
                None
                if record["clearances"] is None
                else float(np.min(record["clearances"]))
            ),
        }
        for record in history
    ]


def build_result_and_visuals(
    worker: NewtonWorker,
    robot: Kinematics,
    robot_config: Path,
    goal: GoalToolPose,
    original_goal: GoalToolPose,
    repeats: int,
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    baseline_path, baseline = make_baseline_path(robot_config, goal)
    original_baseline_path, original_baseline = make_baseline_path(
        robot_config, original_goal, ORIGINAL_START_ARM
    )
    goal_q = baseline_path[-1]
    original_goal_q = original_baseline_path[-1]
    results: dict[str, Any] = {
        "baseline_no_obstacle": {
            "flipped": baseline,
            "original": original_baseline,
        },
        "experiment_pose": experiment_pose_report(robot, original_goal_q, goal_q),
        "model_start_points": model_start_points(robot),
        "padding_sweep": {},
        "height_sweep": {},
        "overview_height_sweep": {},
        "god_view": {},
        "main_case_a": {},
    }
    visuals: dict[str, dict[str, Any]] = {}
    scans: dict[tuple[str, float], dict[str, Any]] = {}
    blind_center, blind_dims, blind_validity, blind_attempts = select_blind_scene(
        worker, robot, baseline_path
    )
    results["blind_scene"] = {
        "center_m": blind_center.tolist(), "dims_m": blind_dims.tolist(),
        "validity": blind_validity, "selection_attempts": blind_attempts,
        "selection": "first deterministic candidate that is valid and unseen from start",
    }
    results["model_start_points"].update(
        blind_attempts[-1].get("model_start_points", {})
    )
    standard_goals, overview_goal = observation_goals(robot)

    for top in WALL_TOPS_M:
        center, dims = wall_geometry(top)
        validity = scene_validity(robot, baseline_path, center, dims)
        results["height_sweep"][f"top_{top:.2f}_m"] = {
            "true_top_z_m": top,
            "validity": validity,
            "status": validity["status"],
        }
        results["overview_height_sweep"][f"top_{top:.2f}_m"] = {
            "true_top_z_m": top,
            "validity": validity,
            "status": validity["status"],
        }
        if not validity["valid"]:
            visuals[f"INVALID_SCENE wall top {top:.2f}"] = {
                "points": np.empty((0, 3)), "fit": None, "center": center,
                "dims": dims, "captures": [],
                "path": torch.empty((0, len(preflight.ARM_JOINT_NAMES)), device="cuda"),
                "clearance": validity["along_path_clearance_m"],
                "status": "INVALID_SCENE: " + "; ".join(validity["reasons"]),
                "playable": False,
            }
            continue
        standard = run_scan(
            worker, robot, robot_config, center, dims,
            standard_goals, 0.02,
        )
        overview = run_scan(
            worker, robot, robot_config, center, dims,
            (overview_goal,), 0.02,
        )
        scans[("standard", top)] = standard
        scans[("overview", top)] = overview

    valid_tops = [top for top in WALL_TOPS_M if ("standard", top) in scans]
    if not valid_tops:
        raise RuntimeError(
            "none of the mandated same-XY main scenes passed the 25 mm validity gate: "
            + json.dumps(results["height_sweep"], sort_keys=True)
        )

    for top in valid_tops:
        center, dims = wall_geometry(top)
        validity = scene_validity(robot, baseline_path, center, dims)
        top_key = f"top_{top:.2f}_m"
        case_a, a_path = evaluate_final_paths(
            robot, robot_config, None, center, dims, goal, repeats
        )
        truth_fit = exact_truth_fit(center, dims)
        case_b, b_path = evaluate_final_paths(
            robot, robot_config, truth_fit, center, dims, goal, repeats
        )
        results["main_case_a"][top_key] = case_a
        results["god_view"][top_key] = {
            "planner_obstacle": "exact truth cuboid (case B only)",
            "final_plan": case_b,
        }
        if top == valid_tops[0]:
            visuals["NO OBSTACLE BASELINE (expected truth collision)"] = {
                "points": np.empty((0, 3)), "fit": None, "center": center,
                "dims": dims, "captures": [], "path": baseline_path,
                "clearance": validity["along_path_clearance_m"],
                "status": "BASELINE: no planner obstacle; collision is expected",
                "playable": True,
            }
        visuals[f"B god view wall top {top:.2f}"] = {
            "points": np.empty((0, 3)), "fit": truth_fit, "center": center,
            "dims": dims, "captures": [],
            "path": b_path if b_path is not None else torch.empty((0, 6), device="cuda"),
            "clearance": case_b["minimum_true_clearance_m"],
            "status": "VALID B GOD VIEW",
            "playable": b_path is not None,
        }

        standard_scan = scans[("standard", top)]
        standard_points = merged_points(standard_scan)
        results["padding_sweep"][top_key] = {}
        for padding in PADDINGS_M:
            fit = fit_cuboid(standard_points, padding)
            final, padding_path = evaluate_final_paths(
                robot, robot_config, fit, center, dims, goal, repeats
            )
            padding_key = f"{int(round(padding * 1000))}_mm"
            passed = bool(
                not standard_scan["scan_collision"]
                and not standard_scan["scan_no_plan"]
                and final["minimum_true_clearance_m"] is not None
                and final["minimum_true_clearance_m"] >= 0.0
            )
            results["padding_sweep"][top_key][padding_key] = {
                "z_padding_m": padding, "fit": fit, "validity": validity,
                "scan": serializable_scan(standard_scan), "final_plan": final,
                "case_pass": passed,
            }
            visuals[f"padding {padding_key} wall top {top:.2f}"] = {
                "points": standard_points, "fit": fit, "center": center,
                "dims": dims, "captures": standard_scan["captures"],
                "path": concatenate_paths([
                    *standard_scan["paths"],
                    *([padding_path] if padding_path is not None else []),
                ]),
                "clearance": final["minimum_true_clearance_m"],
                "status": "PASS" if passed else (
                    "NO PLAN: " + str(final["records"][0]["failure_status"])
                ),
                "playable": True,
            }

        for mode, result_key in (
            ("standard", "height_sweep"),
            ("overview", "overview_height_sweep"),
        ):
            scan = scans[(mode, top)]
            points = merged_points(scan)
            fit = fit_cuboid(points, 0.02)
            final, path = evaluate_final_paths(
                robot, robot_config, fit, center, dims, goal, repeats
            )
            key = f"top_{top:.2f}_m"
            results[result_key][key].update({
                "true_top_z_m": top,
                "validity": validity,
                "status": "VALID",
                "fit": fit,
                "scan": serializable_scan(scan),
                "final_plan": final,
                "top_seen": bool(
                    fit is not None and fit["highest_seen_z_m"] >= top - 0.008
                ),
                "case_pass": bool(
                    not scan["scan_collision"]
                    and not scan["scan_no_plan"]
                    and final["minimum_true_clearance_m"] is not None
                    and final["minimum_true_clearance_m"] >= 0.0
                ),
            })
            visuals[f"{mode} wall top {top:.2f}"] = {
                "points": points,
                "fit": fit,
                "center": center,
                "dims": dims,
                "captures": scan["captures"],
                "path": concatenate_paths([*scan["paths"], *([path] if path is not None else [])]),
                "clearance": final["minimum_true_clearance_m"],
                "status": "VALID",
                "playable": True,
            }

    main_e_results: dict[str, Any] = {}
    main_e_histories: dict[str, list[dict[str, Any]]] = {}
    for top in valid_tops:
        center, dims = wall_geometry(top)
        top_key = f"top_{top:.2f}_m"
        summary, history = run_look_while_moving(
            worker, robot, robot_config, center, dims, goal, goal_q
        )
        main_e_results[top_key] = summary
        main_e_histories[top_key] = history
        paths = [r["executed"] for r in history if r["executed"] is not None]
        visuals[f"E main wall top {top:.2f}"] = {
            "points": history[-1]["points"] if history else np.empty((0, 3)),
            "fit": history[-1]["fit"] if history else None,
            "center": center, "dims": dims,
            "captures": [r["capture"] for r in history],
            "path": concatenate_paths(paths),
            "clearance": summary["minimum_true_clearance_m"],
            "e_history": history,
            "status": summary["final_stop_reason"] or "RUNNING",
            "playable": True,
        }

    blind_start_capture = worker.capture(START_ARM, blind_center, blind_dims)
    blind_one_view_fit = fit_cuboid(blind_start_capture.points, 0.02)
    blind_a, blind_a_path = evaluate_final_paths(
        robot, robot_config, None, blind_center, blind_dims, goal, repeats
    )
    blind_c, blind_c_path = evaluate_final_paths(
        robot, robot_config, blind_one_view_fit, blind_center, blind_dims, goal, repeats
    )
    e_blind, e_blind_history = run_look_while_moving(
        worker, robot, robot_config, blind_center, blind_dims, goal, goal_q
    )
    results["blind_cases"] = {
        "A_no_obstacle": blind_a,
        "C_one_view": {
            "capture": compact_capture(blind_start_capture),
            "fit": blind_one_view_fit,
            "final_plan": blind_c,
        },
    }
    results["look_while_moving"] = {
        "main": main_e_results,
        "blind": e_blind,
        "main_cycles": {
            key: serializable_e_cycles(history)
            for key, history in main_e_histories.items()
        },
        "blind_cycles": serializable_e_cycles(e_blind_history),
    }
    visuals["C blind one-view"] = {
        "points": blind_start_capture.points,
        "fit": blind_one_view_fit, "center": blind_center, "dims": blind_dims,
        "captures": [blind_start_capture],
        "path": blind_c_path if blind_c_path is not None else torch.empty((0, 6), device="cuda"),
        "clearance": blind_c["minimum_true_clearance_m"],
        "status": "VALID C ONE-VIEW",
        "playable": blind_c_path is not None,
    }
    blind_paths = [r["executed"] for r in e_blind_history if r["executed"] is not None]
    visuals["E blind spot"] = {
        "points": e_blind_history[-1]["points"] if e_blind_history else np.empty((0, 3)),
        "fit": e_blind_history[-1]["fit"] if e_blind_history else None,
        "center": blind_center, "dims": blind_dims,
        "captures": [r["capture"] for r in e_blind_history],
        "path": concatenate_paths(blind_paths),
        "clearance": e_blind["minimum_true_clearance_m"],
        "e_history": e_blind_history,
        "status": e_blind["final_stop_reason"] or "RUNNING",
        "playable": True,
    }
    visuals["A blind no obstacle"] = {
        "points": np.empty((0, 3)), "fit": None,
        "center": blind_center, "dims": blind_dims, "captures": [],
        "path": blind_a_path if blind_a_path is not None else torch.empty((0, 6), device="cuda"),
        "clearance": blind_a["minimum_true_clearance_m"],
        "status": "VALID A NO OBSTACLE; collision expected",
        "playable": blind_a_path is not None,
    }
    results["overview_pose"] = {
        "cartesian_offset_from_start_m": [0.0, 0.0, 0.20],
        "standard_high_scan_offsets_m": [
            [0.0, 0.0, 0.20],
            [0.12, 0.0, 0.20],
            [-0.12, 0.0, 0.20],
        ],
        "selection": (
            "fixed Cartesian offsets from the recorded start FK: raise world +Z while "
            "holding the already downward optical orientation, then fixed +/-X at height; "
            "chosen without wall coordinates"
        ),
    }
    results["known_limits"] = [
        "all perceived points fit into one cuboid, so separate obstacles merge",
        "all points at z <= 0.012 m are deleted, including a cable",
        "robot pixels use simulator shape indices unavailable to a real camera",
    ]
    results["valid_main_tops_m"] = valid_tops
    return results, visuals


def matrix_to_wxyz(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        xyzw = np.array(((matrix[2, 1] - matrix[1, 2]) / s, (matrix[0, 2] - matrix[2, 0]) / s, (matrix[1, 0] - matrix[0, 1]) / s, 0.25 * s))
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            xyzw = np.array((0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s))
        elif index == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            xyzw = np.array(((matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s))
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            xyzw = np.array(((matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s, (matrix[1, 0] - matrix[0, 1]) / s))
    xyzw /= np.linalg.norm(xyzw)
    return xyzw[[3, 0, 1, 2]]


def wxyz_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
    )


def camera_pose_for_q(
    robot: Kinematics, q: torch.Tensor, mount: MountConfig
) -> tuple[np.ndarray, np.ndarray]:
    pose = robot.compute_kinematics(arm_state(q)).tool_poses[preflight.TOOL_FRAME]
    position = pose.position.detach().cpu().reshape(-1, 3)[0].numpy()
    quaternion = pose.quaternion.detach().cpu().reshape(-1, 4)[0].numpy()
    rotation = wxyz_matrix(quaternion)
    if mount.parent_link.split("/")[-1] != preflight.TOOL_FRAME.split("/")[-1]:
        raise ValueError(
            f"camera mount parent {mount.parent_link!r} is not {preflight.TOOL_FRAME!r}"
        )
    local_position = np.array((mount.left_m, mount.up_m, mount.forward_m))
    camera_position = position + rotation @ local_position
    forward = np.array((0.0, 0.0, 1.0))
    up = np.array((0.0, 1.0, 0.0))
    transverse = np.array((mount.left_m, mount.up_m, 0.0))
    toward_center = -up if np.linalg.norm(transverse) <= 1.0e-9 else -transverse / np.linalg.norm(transverse)
    tilt = math.radians(mount.tilt_deg)
    optical = math.cos(tilt) * forward + math.sin(tilt) * toward_center
    optical /= np.linalg.norm(optical)
    image_up = up - np.dot(up, optical) * optical
    image_up /= np.linalg.norm(image_up)
    camera_z = -optical
    camera_x = np.cross(image_up, camera_z)
    camera_x /= np.linalg.norm(camera_x)
    camera_rotation = rotation @ np.column_stack((camera_x, image_up, camera_z))
    # Viser camera frustum adds OpenCV's pi-X conversion.
    viser_rotation = camera_rotation @ np.diag((1.0, -1.0, -1.0))
    return camera_position, matrix_to_wxyz(viser_rotation)


def segments(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros((1, 2, 3))
    return np.stack((points[:-1], points[1:]), axis=1)


def start_viewer(
    robot_config: Path,
    robot: Kinematics,
    visuals: dict[str, dict[str, Any]],
    port: int,
) -> tuple[ViserVisualizer, Any]:
    mount = read_mount_config(MOUNT_CONFIG)
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
            f"Viser requested 127.0.0.1:{port} but selected {actual_port}; "
            "the required port is busy"
        )
    server.initial_camera.position = (1.15, 1.10, 0.85)
    server.initial_camera.look_at = (0.48, 0.27, 0.22)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    viewer.set_joint_state(arm_state(START_ARM).squeeze(0))
    first = next(iter(visuals.values()))
    truth = server.scene.add_box(
        "/truth/obstacle", dimensions=tuple(first["dims"]),
        position=tuple(first["center"]), color=(230, 65, 55), opacity=0.55,
    )
    perceived = server.scene.add_box(
        "/perceived/fit", dimensions=(0.01, 0.01, 0.01),
        position=(0.0, 0.0, 0.0), color=(55, 205, 235), opacity=0.35,
    )
    cloud = server.scene.add_point_cloud(
        "/perceived/points", points=np.zeros((1, 3)), colors=(255, 210, 40),
        point_size=0.008, point_shape="circle",
    )
    path = server.scene.add_line_segments(
        "/plans/path", points=np.zeros((1, 2, 3)), colors=(50, 255, 120), line_width=4.0,
    )
    seen_line = server.scene.add_line_segments(
        "/heights/highest_seen", points=np.zeros((1, 2, 3)), colors=(255, 215, 0), line_width=5.0,
    )
    padded_line = server.scene.add_line_segments(
        "/heights/padded_top", points=np.zeros((1, 2, 3)), colors=(255, 60, 220), line_width=5.0,
    )
    camera_position, camera_wxyz = camera_pose_for_q(
        robot, torch.as_tensor(START_ARM, device="cuda"), mount
    )
    live_frustum = server.scene.add_camera_frustum(
        "/camera/live", fov=math.radians(58.0), aspect=848 / 480, scale=0.13,
        color=(255, 255, 255), position=tuple(camera_position), wxyz=tuple(camera_wxyz),
    )
    capture_frustums: dict[str, list[Any]] = {}
    for case_name, case in visuals.items():
        capture_frustums[case_name] = []
        for index, capture in enumerate(case["captures"]):
            transform = capture.camera_transform
            # Newton xyzw camera pose to Viser/OpenCV frustum orientation.
            rotation = wxyz_matrix(transform[[6, 3, 4, 5]]) @ np.diag((1.0, -1.0, -1.0))
            handle = server.scene.add_camera_frustum(
                f"/captures/{case_name}/{index}", fov=math.radians(58.0),
                aspect=848 / 480, scale=0.09, color=(255, 190, 40),
                position=tuple(transform[:3]), wxyz=tuple(matrix_to_wxyz(rotation)),
            )
            capture_frustums[case_name].append(handle)
    default_case = next(
        (
            name for name, case in visuals.items()
            if case.get("playable", True)
            and "BASELINE" not in case.get("status", "")
        ),
        next(iter(visuals)),
    )
    selector = server.gui.add_dropdown(
        "Case", options=tuple(visuals), initial_value=default_case
    )
    show_true = server.gui.add_checkbox("Show true obstacle", initial_value=True)
    clearance_text = server.gui.add_text("True clearance", "", disabled=True)
    height_text = server.gui.add_text("Seen / padded top", "", disabled=True)
    status_text = server.gui.add_text("Case status / no-plan reason", "", disabled=True)
    camera_position_text = server.gui.add_text(
        "Camera world position", "", disabled=True
    )
    playback_error_text = server.gui.add_text("Playback error", "none", disabled=True)
    play = server.gui.add_button("Play selected case")
    playback_done = threading.Event()
    playback_record: dict[str, Any] = {}
    verification_mode = threading.Event()

    def apply_case(_event: Any = None) -> None:
        name = selector.value
        case = visuals[name]
        truth.dimensions = tuple(case["dims"])
        truth.position = tuple(case["center"])
        truth.visible = show_true.value
        points = case["points"]
        cloud.points = points if len(points) else np.zeros((1, 3))
        fit = case["fit"]
        perceived.visible = fit is not None
        if fit is not None:
            perceived.dimensions = tuple(fit["dims_m"])
            perceived.position = tuple(fit["center_m"])
            x0 = fit["center_m"][0] - 0.5 * fit["dims_m"][0]
            x1 = fit["center_m"][0] + 0.5 * fit["dims_m"][0]
            y = fit["center_m"][1]
            seen_line.points = np.asarray([[[x0, y, fit["highest_seen_z_m"]], [x1, y, fit["highest_seen_z_m"]]]])
            padded_line.points = np.asarray([[[x0, y, fit["fitted_top_z_m"]], [x1, y, fit["fitted_top_z_m"]]]])
            height_text.value = f"{fit['highest_seen_z_m']:.3f} / {fit['fitted_top_z_m']:.3f} m"
        seen_line.visible = fit is not None
        padded_line.visible = fit is not None
        playable = case.get("playable", True) and len(case["path"]) > 0
        path.visible = playable
        path.points = (
            segments(tool_points(robot, case["path"]))
            if playable else np.zeros((1, 2, 3))
        )
        value = case["clearance"]
        clearance_text.value = "not measured" if value is None else f"{value:+.4f} m"
        status_text.value = case.get("status", "VALID")
        current_camera, _ = camera_pose_for_q(
            robot, torch.as_tensor(START_ARM, device="cuda"), mount
        )
        camera_position_text.value = "[" + ", ".join(
            f"{value:.4f}" for value in current_camera
        ) + "] m"
        for case_key, handles in capture_frustums.items():
            for handle in handles:
                handle.visible = case_key == name

    def playback(_event: Any) -> None:
        name = selector.value
        case = visuals[name]
        callback_thread = threading.current_thread()
        playback_done.clear()
        playback_error_text.value = "none"
        if not case.get("playable", True) or len(case["path"]) == 0:
            status_text.value = case.get("status", "INVALID_SCENE: playback disabled")
            playback_record.clear()
            playback_record.update({
                "case": name, "callback_thread": callback_thread.name,
                "callback_non_main": callback_thread is not threading.main_thread(),
                "states_commanded": 0, "robot_joint_state_changed": False,
                "frustum_changed": False, "error": None,
                "playback_disabled": True,
            })
            playback_done.set()
            return

        def worker_thread() -> None:
            commanded: list[np.ndarray] = []
            frustums: list[np.ndarray] = []
            try:
                e_history = case.get("e_history")
                sequences: list[tuple[torch.Tensor, np.ndarray | None]] = []
                if e_history:
                    for record in e_history:
                        cloud.points = record["points"] if len(record["points"]) else np.zeros((1, 3))
                        fit = record["fit"]
                        if fit is not None:
                            perceived.visible = True
                            perceived.dimensions = tuple(fit["dims_m"])
                            perceived.position = tuple(fit["center_m"])
                        if record["executed"] is not None:
                            sequences.append((record["executed"], record["clearances"]))
                else:
                    sequences.append((
                        case["path"],
                        clearance_series(robot, case["path"], case["center"], case["dims"]),
                    ))
                for positions, clearances in sequences:
                    iterable = positions
                    if verification_mode.is_set() and len(positions) > 2:
                        iterable = positions[torch.as_tensor([0, len(positions) - 1], device=positions.device)]
                        clearance_indices = (0, len(positions) - 1)
                    else:
                        clearance_indices = range(len(positions))
                    for q, clearance_index in zip(iterable, clearance_indices):
                        viewer.set_joint_state(arm_state(q).squeeze(0))
                        position, wxyz = camera_pose_for_q(robot, q, mount)
                        live_frustum.position = tuple(position)
                        live_frustum.wxyz = tuple(wxyz)
                        camera_position_text.value = "[" + ", ".join(
                            f"{value:.4f}" for value in position
                        ) + "] m"
                        commanded.append(q.detach().cpu().numpy().copy())
                        frustums.append(np.asarray(position).copy())
                        if clearances is not None:
                            clearance_text.value = f"{float(clearances[clearance_index]):+.4f} m"
                        if not verification_mode.is_set():
                            time.sleep(0.025)
                playback_record.clear()
                playback_record.update({
                    "case": name,
                    "callback_thread": callback_thread.name,
                    "callback_non_main": callback_thread is not threading.main_thread(),
                    "worker_thread": threading.current_thread().name,
                    "states_commanded": len(commanded),
                    "robot_joint_state_changed": bool(
                        len(commanded) > 1 and not np.allclose(commanded[0], commanded[-1])
                    ),
                    "frustum_changed": bool(
                        len(frustums) > 1 and not np.allclose(frustums[0], frustums[-1])
                    ),
                    "error": None,
                })
            except Exception as exc:
                message = f"{type(exc).__name__}: {exc}"
                playback_error_text.value = message
                traceback.print_exc(file=sys.stderr)
                playback_record.clear()
                playback_record.update({
                    "case": name, "callback_thread": callback_thread.name,
                    "callback_non_main": callback_thread is not threading.main_thread(),
                    "worker_thread": threading.current_thread().name,
                    "states_commanded": len(commanded), "robot_joint_state_changed": False,
                    "frustum_changed": False, "error": message,
                })
            finally:
                playback_done.set()

        threading.Thread(target=worker_thread, name=f"playback-{name}", daemon=True).start()

    selector.on_update(apply_case)
    show_true.on_update(apply_case)
    play.on_click(playback)
    apply_case()
    print(
        f"SIM_CAMERA_CUROBO_FOURTH_PASS_VISER http://127.0.0.1:{actual_port}",
        flush=True,
    )

    def verify_callbacks() -> dict[str, Any]:
        prefixes = (
            "B god view", "padding ", "standard ", "overview ",
            "E main", "C blind", "E blind", "NO OBSTACLE BASELINE",
        )
        records: list[dict[str, Any]] = []
        verification_mode.set()
        try:
            for prefix in prefixes:
                name = next((key for key in visuals if key.startswith(prefix)), None)
                if name is None:
                    continue
                selector.value = name
                callback = threading.Thread(
                    target=playback, args=(None,), name=f"viser-callback-{prefix.strip()}"
                )
                callback.start()
                callback.join(timeout=5.0)
                if not playback_done.wait(timeout=20.0):
                    records.append({"case": name, "error": "playback timed out"})
                else:
                    records.append(dict(playback_record))
        finally:
            verification_mode.clear()
        return {
            "registered_callback_cases": records,
            "all_passed": bool(records) and all(
                record.get("callback_non_main")
                and record.get("robot_joint_state_changed")
                and record.get("frustum_changed")
                and record.get("error") is None
                for record in records
            ),
            "mount_config_path": str(MOUNT_CONFIG.relative_to(REPO)),
            "invalid_cases_no_path": all(
                len(case["path"]) == 0 and not case.get("playable", True)
                for name, case in visuals.items() if name.startswith("INVALID_SCENE")
            ),
            "default_case": default_case,
        }

    return viewer, verify_callbacks


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--port", type=int, default=8084)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--newton-python", type=Path,
        default=Path.home() / "newton_ws/.venv-cpu/bin/python",
    )
    parser.add_argument(
        "--robot-config", type=Path,
        default=Path.home() / ".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml",
    )
    parser.add_argument(
        "--results-dir", type=Path,
        default=REPO / "docs/experiments/sim-camera-curobo/results",
    )
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error("--repeats must be at least 3")
    robot = Kinematics(
        KinematicsCfg.from_robot_yaml_file(
            str(args.robot_config),
            tool_frames=[preflight.TOOL_FRAME, "flange", "wrist_1_link"],
        )
    )
    _, _, object_center = preflight.load_scene(REPO, "step_c_cable_no_wall")
    goal_position, goal_quaternion = preflight.pregrasp_target(object_center)
    flipped_goal_quaternion = rotate_goal_about_tool_z(goal_quaternion)
    goal_pose = Pose.from_list(
        [*goal_position.tolist(), *flipped_goal_quaternion.tolist()]
    )
    goal = GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: goal_pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )
    original_goal_pose = Pose.from_list(
        [*goal_position.tolist(), *goal_quaternion.tolist()]
    )
    original_goal = GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: original_goal_pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )
    worker_log: list[str] = []
    with tempfile.TemporaryDirectory(prefix="newton-camera-worker-") as temp:
        worker = NewtonWorker(args.newton_python, Path(temp))
        try:
            results, visuals = build_result_and_visuals(
                worker, robot, args.robot_config, goal, original_goal, args.repeats
            )
        finally:
            worker.close()
            worker_log = worker.log
    results.update(
        {
            "schema": 3,
            "date": "2026-11-06",
            "repeats": args.repeats,
            "planner_truth_rule": "truth used only by case B and offline scoring, never C/D/E input",
            "worker_exchange": "persistent Newton subprocess, JSON pipe plus temporary NPZ captures",
            "curobo_version": {
                "python": curobo.__version__,
                "checkout": subprocess.check_output(
                    ["git", "-C", str(REPO / "curobo"), "describe", "--tags", "--always"],
                    text=True,
                ).strip(),
            },
            "playback_root_cause": {
                "cause": (
                    "clearance_series returned shape [N,1]; playback formatted the one-element "
                    "ndarray with +.4f, raising TypeError inside an uncaught daemon thread"
                ),
                "reproduced_exception": (
                    "TypeError: unsupported format string passed to numpy.ndarray.__format__"
                ),
                "fix": (
                    "clearance_series now returns shape [N], formatting casts float, and the "
                    "worker reports exceptions to stderr and the Playback error GUI field"
                ),
            },
        }
    )
    args.results_dir.mkdir(parents=True, exist_ok=True)
    result_path = args.results_dir / "fourth_pass.json"
    log_path = args.results_dir / "fourth_pass.log"
    viewer = None
    try:
        if not args.headless:
            viewer, verify_callbacks = start_viewer(
                args.robot_config, robot, visuals, args.port
            )
            results["playback_verification"] = verify_callbacks()
            started = time.monotonic()
            while args.duration <= 0.0 or time.monotonic() - started < args.duration:
                time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer._server.stop()
    result_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    log_lines = [
        "SIM_CAMERA_CUROBO_FOURTH_PASS",
        f"valid_main_tops={results['valid_main_tops_m']}",
        f"padding_heights={len(results['padding_sweep'])}",
        f"height_cases={len(results['height_sweep'])}",
        f"overview_cases={len(results['overview_height_sweep'])}",
        "main_E=" + json.dumps(results["look_while_moving"]["main"], sort_keys=True),
        "blind_E=" + json.dumps(results["look_while_moving"]["blind"], sort_keys=True),
        "playback=" + json.dumps(results.get("playback_verification"), sort_keys=True),
        f"newton_worker_lines={len(worker_log)}",
    ]
    log_path.write_text("\n".join(log_lines) + "\n", encoding="utf-8")
    print("SIM_CAMERA_CUROBO_FOURTH_PASS_RESULT " + json.dumps(results, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
