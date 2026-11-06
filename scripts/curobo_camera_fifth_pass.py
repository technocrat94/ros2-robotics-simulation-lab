#!/usr/bin/env python3
"""Sixth-pass correction to real-time camera planning unknown extents."""

from __future__ import annotations

import argparse
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

import curobo
import numpy as np
import torch

from curobo._src.motion.motion_planner import MotionPlanner
from curobo._src.motion.motion_planner_cfg import MotionPlannerCfg
from curobo._src.types.robot import RobotCfg
from curobo._src.util.config_io import load_yaml
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer

import curobo_camera_second_pass as fourth


REPO = Path(__file__).resolve().parents[1]
preflight = fourth.preflight
CAMERA_FRONT_SEED = fourth.START_ARM.copy()
WRIST_3_INDEX = 5
RAISED_START_Z_M = 0.20
MIN_ENDPOINT_CLEARANCE_M = 0.025
MAIN_WALL_TOPS_M = (0.22, 0.26, 0.30)
FALLBACK_WALL_TOPS_M = (0.34, 0.38, 0.42, 0.46, 0.50)
WALL_BOTTOM_M = 0.01
MAIN_CENTER_XY = (0.487, 0.263)
MAIN_DIMS_XY = (0.200, 0.060)
FIT_PADDING_XY_M = 0.015
FIT_PADDING_Z_M = 0.020
HORIZONTAL_UNKNOWN_MARGIN_M = 0.030
CAMERA_NEAR_TOP_M = 0.030
FIXED_UNKNOWN_TOP_ABOVE_START_CAMERA_M = 0.100
MIN_POINTS = 40
CYCLE_CAP = 80
FAR_ADVANCE_RAD = 0.050
NEAR_ADVANCE_RAD = 0.020
NEAR_BOX_CLEARANCE_M = 0.050
GOAL_TOLERANCE_M = 0.006
MOUNT_CONFIG = REPO / "config/sim_camera_mount.yaml"


def arm_state(q: np.ndarray | torch.Tensor) -> JointState:
    return fourth.arm_state(q)


def wall_geometry(top_m: float) -> tuple[np.ndarray, np.ndarray]:
    height = top_m - WALL_BOTTOM_M
    return (
        np.array((*MAIN_CENTER_XY, WALL_BOTTOM_M + 0.5 * height), dtype=np.float64),
        np.array((*MAIN_DIMS_XY, height), dtype=np.float64),
    )


def goal_from_scene() -> GoalToolPose:
    _, _, object_center = preflight.load_scene(REPO, "step_c_cable_no_wall")
    position, quaternion = preflight.pregrasp_target(object_center)
    quaternion = fourth.rotate_goal_about_tool_z(quaternion)
    pose = Pose.from_list([*position.tolist(), *quaternion.tolist()])
    return GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )


def raised_pose_goal(robot: Kinematics) -> GoalToolPose:
    pose = robot.compute_kinematics(arm_state(CAMERA_FRONT_SEED)).tool_poses[
        preflight.TOOL_FRAME
    ]
    position = pose.position.detach().cpu().reshape(-1, 3)[0].numpy()
    position[2] += RAISED_START_Z_M
    quaternion = pose.quaternion.detach().cpu().reshape(-1, 4)[0].numpy()
    target = Pose.from_list([*position.tolist(), *quaternion.tolist()])
    return GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: target},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )


def solve_raised_start(
    robot: Kinematics, planner: MotionPlanner
) -> tuple[np.ndarray, dict[str, Any]]:
    """Solve the old overview pose once with the same planner used by the loop."""
    result = planner.plan_pose(raised_pose_goal(robot), arm_state(CAMERA_FRONT_SEED))
    path = fourth.active_trajectory(result)
    status = fourth.failure_status(result)
    if path is None:
        raise RuntimeError(f"raised-start solve failed: {status}")
    q = path[-1].detach().cpu().numpy().astype(np.float32)
    wrist_delta = float(q[WRIST_3_INDEX] - CAMERA_FRONT_SEED[WRIST_3_INDEX])
    if abs(wrist_delta) > 1.0e-6:
        raise RuntimeError(
            "raised-start wrist_3 did not preserve the constrained planner center: "
            f"delta={wrist_delta:.6f} rad"
        )
    mount = fourth.read_mount_config(MOUNT_CONFIG)
    camera, _ = fourth.camera_pose_for_q(
        robot, torch.as_tensor(q, device="cuda"), mount
    )
    return q, {
        "method": (
            "the one warmed, wrist-constrained loop planner solves from the camera-front "
            "seed to world +0.20 m Z with the same tool orientation; the solved state is "
            "used directly as loop start and that solve is not executed as experiment motion"
        ),
        "joints_rad": q.tolist(),
        "wrist_3_seed_rad": float(CAMERA_FRONT_SEED[WRIST_3_INDEX]),
        "wrist_3_solved_rad": float(q[WRIST_3_INDEX]),
        "wrist_3_delta_from_seed_rad": wrist_delta,
        "tool_world_m": fourth.tool_position(
            robot, torch.as_tensor(q, device="cuda")
        ).tolist(),
        "camera_world_m": camera.tolist(),
    }


def limited_robot_cfg(
    robot_config: Path, start_q: np.ndarray
) -> tuple[RobotCfg, dict[str, Any]]:
    """Use cuRobo's typed RobotCfg to narrow planner limits in memory."""
    robot_cfg = RobotCfg.create(load_yaml(str(robot_config)))
    params = robot_cfg.kinematics.kinematics_config
    joint_index = params.joint_limits.joint_names.index("wrist_3_joint")
    original = params.joint_limits.position[:, joint_index].detach().cpu().tolist()
    requested = (
        float(start_q[WRIST_3_INDEX] - math.pi),
        float(start_q[WRIST_3_INDEX] + math.pi),
    )
    lower = max(float(original[0]), requested[0])
    upper = min(float(original[1]), requested[1])
    params.joint_limits.position[0, joint_index] = lower
    params.joint_limits.position[1, joint_index] = upper
    params.cspace.default_joint_position[joint_index] = float(start_q[WRIST_3_INDEX])
    return robot_cfg, {
        "method": (
            "load existing YAML into cuRobo RobotCfg, modify the in-memory JointLimits.position "
            "tensor before MotionPlannerCfg.create; IK, TrajOpt, and PRM receive the same RobotCfg"
        ),
        "joint": "wrist_3_joint",
        "start_rad": float(start_q[WRIST_3_INDEX]),
        "requested_start_plus_minus_pi_rad": list(requested),
        "original_model_limits_rad": original,
        "effective_planner_limits_rad": [lower, upper],
        "source_files_modified": False,
    }


def make_loop_planner(
    robot_config: Path, start_q: np.ndarray
) -> tuple[MotionPlanner, dict[str, Any]]:
    robot_cfg, limit_report = limited_robot_cfg(robot_config, start_q)
    placeholder = {
        "cuboid": {
            "camera_obstacle": {
                "dims": [0.01, 0.01, 0.01],
                "pose": [0.0, 0.0, -10.0, 1.0, 0.0, 0.0, 0.0],
            }
        }
    }
    config = MotionPlannerCfg.create(
        robot=robot_cfg,
        scene_model=placeholder,
        interpolation_dt=0.025,
    )
    planner = MotionPlanner(config)
    print("Warming one sixth-pass cuRobo motion planner...", flush=True)
    planner.warmup(enable_graph=True, num_warmup_iterations=3)
    planner.scene_collision_checker.enable_obstacle("camera_obstacle", False)
    return planner, limit_report


def fit_with_unknown_extent(
    points: np.ndarray,
    capture: fourth.Capture,
    fixed_unknown_top_m: float,
) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    flags = {
        name: bool(capture.metadata.get("border_touches", {}).get(name, False))
        for name in ("top", "bottom", "left", "right")
    }
    extension: dict[str, Any] = {
        "active": False,
        "directions": [],
        "border_touches": flags,
        "border_pixel_counts": capture.metadata.get("border_pixel_counts", {}),
        "horizontal_margin_m": HORIZONTAL_UNKNOWN_MARGIN_M,
        "world_horizontal_directions": {},
        "border_extension_never_changes_height": True,
    }
    if len(points) < MIN_POINTS:
        return None, extension
    low = np.min(points, axis=0) - np.array(
        (FIT_PADDING_XY_M, FIT_PADDING_XY_M, FIT_PADDING_Z_M)
    )
    high = np.max(points, axis=0) + np.array(
        (FIT_PADDING_XY_M, FIT_PADDING_XY_M, FIT_PADDING_Z_M)
    )
    low[2] = max(0.0, float(low[2]))
    rotation = fourth.wxyz_matrix(capture.camera_transform[[6, 3, 4, 5]])
    camera_border_directions = {
        "top": np.array((0.0, 1.0, 0.0)),
        "bottom": np.array((0.0, -1.0, 0.0)),
        "left": np.array((-1.0, 0.0, 0.0)),
        "right": np.array((1.0, 0.0, 0.0)),
    }
    for name, camera_direction in camera_border_directions.items():
        if not flags[name]:
            continue
        world_direction = rotation @ camera_direction
        horizontal = world_direction[:2]
        norm = float(np.linalg.norm(horizontal))
        if norm <= 1.0e-9:
            extension["world_horizontal_directions"][name] = None
            continue
        horizontal = horizontal / norm
        extension["directions"].append(name)
        extension["world_horizontal_directions"][name] = horizontal.tolist()
        delta = HORIZONTAL_UNKNOWN_MARGIN_M * horizontal
        for axis in (0, 1):
            if delta[axis] >= 0.0:
                high[axis] += delta[axis]
            else:
                low[axis] += delta[axis]
    extension["active"] = bool(extension["directions"])

    same_frame_highest = (
        float(np.max(capture.points[:, 2])) if len(capture.points) else None
    )
    camera_height = float(capture.camera_transform[2])
    height_guard_active = bool(
        same_frame_highest is not None
        and abs(camera_height - same_frame_highest) <= CAMERA_NEAR_TOP_M
    )
    observed_padded_top = float(high[2])
    if height_guard_active:
        high[2] = max(observed_padded_top, fixed_unknown_top_m)
    height_rule = {
        "same_frame_highest_obstacle_m": same_frame_highest,
        "same_frame_camera_height_m": camera_height,
        "near_camera_threshold_m": CAMERA_NEAR_TOP_M,
        "guard_active": height_guard_active,
        "fixed_unknown_top_m": fixed_unknown_top_m,
        "observed_padded_top_m": observed_padded_top,
        "fitted_top_m": float(high[2]),
        "follows_current_camera": False,
    }
    return {
        "center_m": (0.5 * (low + high)).tolist(),
        "dims_m": np.maximum(high - low, 0.01).tolist(),
        "highest_seen_z_m": float(np.max(points[:, 2])),
        "fitted_top_z_m": float(high[2]),
        "points": int(len(points)),
        "padding_xy_m": FIT_PADDING_XY_M,
        "padding_z_m": FIT_PADDING_Z_M,
        "unknown_extension": extension,
        "height_rule": height_rule,
    }, extension


def concatenate_executed(
    start_q: np.ndarray, history: list[dict[str, Any]]
) -> torch.Tensor:
    pieces = [
        torch.as_tensor(start_q, device="cuda", dtype=torch.float32).reshape(1, -1)
    ]
    for record in history:
        executed = record.get("executed")
        if executed is not None and len(executed) > 1:
            pieces.append(executed[1:])
    return torch.cat(pieces, dim=0)


def motion_metrics(
    robot: Kinematics, positions: torch.Tensor, start_q: np.ndarray
) -> dict[str, Any]:
    q = positions.detach().cpu().numpy()
    travel = (
        np.sum(np.abs(np.diff(q, axis=0)), axis=0)
        if len(q) > 1
        else np.zeros(q.shape[1], dtype=np.float64)
    )
    tool = fourth.tool_points(robot, positions)
    tool_length = (
        float(np.linalg.norm(np.diff(tool, axis=0), axis=1).sum())
        if len(tool) > 1
        else 0.0
    )
    straight = float(np.linalg.norm(tool[-1] - tool[0]))
    max_tool_height = float(np.max(tool[:, 2]))
    return {
        "states": int(len(q)),
        "joint_travel_rad": {
            name: float(travel[index])
            for index, name in enumerate(preflight.ARM_JOINT_NAMES)
        },
        "max_wrist_3_change_from_start_rad": float(
            np.max(np.abs(q[:, WRIST_3_INDEX] - start_q[WRIST_3_INDEX]))
        ),
        "tool_path_length_m": tool_length,
        "endpoint_straight_line_distance_m": straight,
        "tool_path_over_straight_line": (
            tool_length / straight if straight > 1.0e-9 else None
        ),
        "maximum_tool_height_m": max_tool_height,
        "tool_height_climb_from_start_m": max_tool_height - float(tool[0, 2]),
        "start_tool_m": tool[0].tolist(),
        "end_tool_m": tool[-1].tolist(),
    }


def scene_validity(
    robot: Kinematics,
    baseline: torch.Tensor,
    center: np.ndarray,
    dims: np.ndarray,
) -> dict[str, Any]:
    return fourth.scene_validity(robot, baseline, center, dims)


def first_frame_report(
    capture: fourth.Capture, top_m: float, bottom_m: float
) -> dict[str, Any]:
    if len(capture.points):
        low = float(np.min(capture.points[:, 2]))
        high = float(np.max(capture.points[:, 2]))
        visible = max(0.0, min(high, top_m) - max(low, bottom_m))
        fraction = visible / max(top_m - bottom_m, 1.0e-9)
    else:
        low = high = None
        fraction = 0.0
    return {
        "points": int(len(capture.points)),
        "lowest_seen_z_m": low,
        "highest_seen_z_m": high,
        "true_vertical_fraction_seen": fraction,
        "obstacle_pixels": int(capture.metadata.get("obstacle_pixels", 0)),
        "border_touches": capture.metadata.get("border_touches", {}),
    }


def run_loop(
    worker: fourth.NewtonWorker,
    planner: MotionPlanner,
    robot: Kinematics,
    start_q: np.ndarray,
    goal: GoalToolPose,
    goal_q: torch.Tensor,
    center: np.ndarray,
    dims: np.ndarray,
    wrist_limits: tuple[float, float],
    fixed_unknown_top_m: float,
) -> tuple[dict[str, Any], list[dict[str, Any]], torch.Tensor]:
    fourth.update_fit_in_place(planner, None)
    current = torch.as_tensor(start_q, device="cuda", dtype=torch.float32)
    accumulated: list[np.ndarray] = []
    history: list[dict[str, Any]] = []
    # The start state is part of the executed motion even when the first plan
    # fails, so every valid scene has a meaningful minimum true clearance.
    minimum_clearance = fourth.state_clearance(robot, current, center, dims)
    first_seen_cycle: int | None = None
    first_seen_clearance: float | None = None
    goal_reached = False
    true_collision = False
    final_stop_reason: str | None = None
    final_planner_failure: str | None = None
    cycle_times: list[float] = []
    for cycle in range(1, CYCLE_CAP + 1):
        cycle_started = time.perf_counter()
        capture = worker.capture(current.detach().cpu().numpy(), center, dims)
        if len(capture.points):
            accumulated.append(capture.points)
            if first_seen_cycle is None:
                first_seen_cycle = cycle
                first_seen_clearance = fourth.state_clearance(
                    robot, current, center, dims
                )
        points = (
            np.concatenate(accumulated, axis=0)
            if accumulated
            else np.empty((0, 3), dtype=np.float32)
        )
        fit, extension = fit_with_unknown_extent(
            points, capture, fixed_unknown_top_m
        )
        fourth.update_fit_in_place(planner, fit)
        current_fit_clearance = (
            None
            if fit is None
            else fourth.state_clearance(
                robot,
                current,
                np.asarray(fit["center_m"]),
                np.asarray(fit["dims_m"]),
            )
        )
        goal_fit_clearance = (
            None
            if fit is None
            else fourth.state_clearance(
                robot,
                goal_q,
                np.asarray(fit["center_m"]),
                np.asarray(fit["dims_m"]),
            )
        )
        tool_height_start = float(fourth.tool_position(robot, current)[2])
        box_top = (
            None
            if fit is None
            else float(fit["center_m"][2] + 0.5 * fit["dims_m"][2])
        )
        advance = (
            NEAR_ADVANCE_RAD
            if current_fit_clearance is not None
            and current_fit_clearance < NEAR_BOX_CLEARANCE_M
            else FAR_ADVANCE_RAD
        )
        torch.cuda.synchronize()
        plan_started = time.perf_counter()
        result = planner.plan_pose(goal, arm_state(current))
        torch.cuda.synchronize()
        plan_ms = (time.perf_counter() - plan_started) * 1000.0
        positions = fourth.active_trajectory(result)
        planner_failure = fourth.failure_status(result)
        record: dict[str, Any] = {
            "cycle": cycle,
            "q": current.detach().cpu().numpy(),
            "capture": capture,
            "points": points.copy(),
            "fit": fit,
            "extension": extension,
            "plan_ms": plan_ms,
            "planner_failure_status": planner_failure,
            "current_to_fit_clearance_m": current_fit_clearance,
            "goal_to_fit_clearance_m": goal_fit_clearance,
            "box_top_m": box_top,
            "tool_height_start_m": tool_height_start,
            "tool_height_end_m": tool_height_start,
            "advance_rule": (
                f"{NEAR_ADVANCE_RAD:.3f} rad when current-to-fit clearance is below "
                f"{NEAR_BOX_CLEARANCE_M:.3f} m; otherwise {FAR_ADVANCE_RAD:.3f} rad"
            ),
            "advance_joint_length_rad": advance,
            "executed": None,
            "clearances": None,
            "stop_reason": None,
        }
        if positions is None:
            final_planner_failure = planner_failure
            if current_fit_clearance is not None and current_fit_clearance < 0.0:
                final_stop_reason = "FITTED_BOX_COVERS_CURRENT_STATE"
            elif goal_fit_clearance is not None and goal_fit_clearance < 0.0:
                final_stop_reason = "FITTED_BOX_COVERS_GOAL_STATE"
            else:
                final_stop_reason = planner_failure or "NO_PLAN"
            record["stop_reason"] = final_stop_reason
            record["whole_cycle_ms"] = (time.perf_counter() - cycle_started) * 1000.0
            cycle_times.append(record["whole_cycle_ms"])
            history.append(record)
            break
        wrist_values = positions[:, WRIST_3_INDEX]
        if bool(
            torch.any(wrist_values < wrist_limits[0] - 1.0e-5)
            or torch.any(wrist_values > wrist_limits[1] + 1.0e-5)
        ):
            final_stop_reason = "WRIST_LIMIT_VIOLATION"
            record["stop_reason"] = final_stop_reason
            record["whole_cycle_ms"] = (time.perf_counter() - cycle_started) * 1000.0
            cycle_times.append(record["whole_cycle_ms"])
            history.append(record)
            break
        index = fourth.advance_index(positions, advance)
        executed = positions[: index + 1].detach().clone()
        clearances = fourth.clearance_series(robot, executed, center, dims)
        minimum_clearance = min(minimum_clearance, float(np.min(clearances)))
        record["executed"] = executed
        record["clearances"] = clearances
        current = executed[-1].detach().clone()
        record["tool_height_end_m"] = float(
            fourth.tool_position(robot, current)[2]
        )
        if float(np.min(clearances)) < 0.0:
            true_collision = True
            final_stop_reason = "TRUE_COLLISION"
            record["stop_reason"] = final_stop_reason
        else:
            goal_error = float(
                np.linalg.norm(
                    fourth.tool_position(robot, current)
                    - fourth.tool_position(robot, goal_q)
                )
            )
            if index == len(positions) - 1 or goal_error <= GOAL_TOLERANCE_M:
                goal_reached = True
                final_stop_reason = "GOAL_REACHED"
                record["stop_reason"] = final_stop_reason
        record["whole_cycle_ms"] = (time.perf_counter() - cycle_started) * 1000.0
        cycle_times.append(record["whole_cycle_ms"])
        history.append(record)
        if final_stop_reason is not None:
            break
    if final_stop_reason is None:
        final_stop_reason = "MAX_CYCLES_REACHED"
        if history:
            history[-1]["stop_reason"] = final_stop_reason
    executed_path = concatenate_executed(start_q, history)
    summary = {
        "goal_reached": goal_reached,
        "cycles": len(history),
        "cycle_cap": CYCLE_CAP,
        "final_stop_reason": final_stop_reason,
        "planner_failure_status": final_planner_failure,
        "true_collision": true_collision,
        "minimum_true_clearance_m": minimum_clearance,
        "whole_cycle_median_ms": float(statistics.median(cycle_times)),
        "whole_cycle_p95_ms": float(np.percentile(cycle_times, 95)),
        "first_seen_cycle": first_seen_cycle,
        "clearance_when_first_seen_m": first_seen_clearance,
        "extension_active_cycles": [
            record["cycle"] for record in history if record["extension"]["active"]
        ],
        "motion": motion_metrics(robot, executed_path, start_q),
        "final_stop_diagnostics": (
            None
            if not history
            else {
                "current_to_fit_clearance_m": history[-1][
                    "current_to_fit_clearance_m"
                ],
                "goal_to_fit_clearance_m": history[-1][
                    "goal_to_fit_clearance_m"
                ],
                "planner_failure_status": history[-1][
                    "planner_failure_status"
                ],
                "classification": (
                    "BOX_COVERING_GOAL"
                    if history[-1]["goal_to_fit_clearance_m"] is not None
                    and history[-1]["goal_to_fit_clearance_m"] < 0.0
                    else "NOT_BOX_COVERING_GOAL"
                ),
            }
        ),
    }
    return summary, history, executed_path


def serialize_history(history: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for record in history:
        result.append({
            "cycle": record["cycle"],
            "q": record["q"].tolist(),
            "capture": fourth.compact_capture(record["capture"]),
            "accumulated_points": int(len(record["points"])),
            "fit": record["fit"],
            "extension": record["extension"],
            "plan_ms": record["plan_ms"],
            "whole_cycle_ms": record["whole_cycle_ms"],
            "planner_failure_status": record["planner_failure_status"],
            "current_to_fit_clearance_m": record["current_to_fit_clearance_m"],
            "goal_to_fit_clearance_m": record["goal_to_fit_clearance_m"],
            "box_top_m": record["box_top_m"],
            "tool_height_start_m": record["tool_height_start_m"],
            "tool_height_end_m": record["tool_height_end_m"],
            "advance_rule": record["advance_rule"],
            "advance_joint_length_rad": record["advance_joint_length_rad"],
            "executed_states": (
                0 if record["executed"] is None else int(len(record["executed"]))
            ),
            "minimum_true_clearance_m": (
                None
                if record["clearances"] is None
                else float(np.min(record["clearances"]))
            ),
            "stop_reason": record["stop_reason"],
        })
    return result


def blind_candidates() -> list[tuple[np.ndarray, np.ndarray]]:
    candidates: list[tuple[np.ndarray, np.ndarray]] = []
    for height in (0.30, 0.38, 0.22):
        for y in (0.34, 0.38, 0.30):
            for x in (0.647, 0.567, 0.727):
                candidates.append((
                    np.array((x, y, 0.5 * height), dtype=np.float64),
                    np.array((0.050, 0.040, height), dtype=np.float64),
                ))
    return candidates


def select_blind_wall(
    worker: fourth.NewtonWorker,
    robot: Kinematics,
    start_q: np.ndarray,
    baseline: torch.Tensor,
) -> tuple[tuple[np.ndarray, np.ndarray] | None, list[dict[str, Any]]]:
    attempts: list[dict[str, Any]] = []
    for center, dims in blind_candidates():
        validity = scene_validity(robot, baseline, center, dims)
        record: dict[str, Any] = {
            "center_m": center.tolist(),
            "dims_m": dims.tolist(),
            "standing_on_floor": bool(abs(center[2] - 0.5 * dims[2]) < 1.0e-9),
            "validity": validity,
            "first_frame_points": None,
        }
        if validity["valid"]:
            capture = worker.capture(start_q, center, dims)
            record["first_frame_points"] = int(len(capture.points))
            record["first_frame_obstacle_pixels"] = int(
                capture.metadata.get("obstacle_pixels", 0)
            )
            if len(capture.points) == 0:
                attempts.append(record)
                return (center.copy(), dims.copy()), attempts
        attempts.append(record)
    return None, attempts


def scene_candidates(
    robot: Kinematics, baseline: torch.Tensor
) -> list[dict[str, Any]]:
    scenes: list[dict[str, Any]] = []
    for top in MAIN_WALL_TOPS_M:
        center, dims = wall_geometry(top)
        scenes.append({
            "top_m": top,
            "center": center,
            "dims": dims,
            "validity": scene_validity(robot, baseline, center, dims),
            "source": "mandated",
        })
    highest = scenes[-1]
    if any(
        reason.startswith("no-obstacle path does not collide")
        for reason in highest["validity"]["reasons"]
    ):
        for top in FALLBACK_WALL_TOPS_M:
            center, dims = wall_geometry(top)
            validity = scene_validity(robot, baseline, center, dims)
            scenes.append({
                "top_m": top,
                "center": center,
                "dims": dims,
                "validity": validity,
                "source": "taller fallback after 0.30 m flyover",
            })
            if validity["goal_clearance_m"] < MIN_ENDPOINT_CLEARANCE_M:
                break
            if validity["valid"]:
                break
    return scenes


def segments(points: np.ndarray) -> np.ndarray:
    return fourth.segments(points)


def start_viewer(
    robot_config: Path,
    robot: Kinematics,
    start_q: np.ndarray,
    visuals: dict[str, dict[str, Any]],
    port: int,
) -> tuple[ViserVisualizer, Any]:
    mount = fourth.read_mount_config(MOUNT_CONFIG)
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
            f"Viser requested 127.0.0.1:{port} but selected {actual_port}; required port is busy"
        )
    viewer.set_joint_state(arm_state(start_q).squeeze(0))
    first = next(iter(visuals.values()))
    truth = server.scene.add_box(
        "/truth/obstacle", dimensions=tuple(first["dims"]),
        position=tuple(first["center"]), color=(230, 65, 55), opacity=0.50,
    )
    normal_box = server.scene.add_box(
        "/perceived/normal", dimensions=(0.01, 0.01, 0.01),
        position=(0.0, 0.0, 0.0), color=(55, 205, 235), opacity=0.40,
    )
    extended_box = server.scene.add_box(
        "/perceived/unknown_extended", dimensions=(0.01, 0.01, 0.01),
        position=(0.0, 0.0, 0.0), color=(255, 145, 35), opacity=0.45,
    )
    cloud = server.scene.add_point_cloud(
        "/perceived/points", points=np.zeros((1, 3)), colors=(255, 210, 40),
        point_size=0.008, point_shape="circle",
    )
    path = server.scene.add_line_segments(
        "/plans/path", points=np.zeros((1, 2, 3)), colors=(50, 255, 120), line_width=4.0,
    )
    camera_position, camera_wxyz = fourth.camera_pose_for_q(
        robot, torch.as_tensor(start_q, device="cuda"), mount
    )
    live_frustum = server.scene.add_camera_frustum(
        "/camera/live", fov=math.radians(58.0), aspect=848 / 480, scale=0.13,
        color=(255, 255, 255), position=tuple(camera_position), wxyz=tuple(camera_wxyz),
    )
    default_case = next(
        name for name, case in visuals.items()
        if case.get("valid_e", False)
    )
    selector = server.gui.add_dropdown(
        "Case", options=tuple(visuals), initial_value=default_case
    )
    clearance_text = server.gui.add_text("True clearance", "", disabled=True)
    cycle_text = server.gui.add_text("Current cycle", "", disabled=True)
    box_top_text = server.gui.add_text("Box top height", "", disabled=True)
    tool_height_text = server.gui.add_text("Tool height", "", disabled=True)
    stop_text = server.gui.add_text("Stop reason", "", disabled=True)
    status_text = server.gui.add_text("Case status", "", disabled=True)
    playback_error = server.gui.add_text("Playback error", "none", disabled=True)
    play = server.gui.add_button("Play selected case")
    playback_done = threading.Event()
    playback_record: dict[str, Any] = {}
    verification_mode = threading.Event()

    def show_fit(fit: dict[str, Any] | None, extended: bool) -> None:
        normal_box.visible = fit is not None and not extended
        extended_box.visible = fit is not None and extended
        if fit is not None:
            handle = extended_box if extended else normal_box
            handle.dimensions = tuple(fit["dims_m"])
            handle.position = tuple(fit["center_m"])

    def apply_case(_event: Any = None) -> None:
        case = visuals[selector.value]
        truth.dimensions = tuple(case["dims"])
        truth.position = tuple(case["center"])
        points = case.get("points", np.empty((0, 3)))
        cloud.points = points if len(points) else np.zeros((1, 3))
        fit = case.get("fit")
        extended = bool(
            fit is not None and fit.get("unknown_extension", {}).get("active", False)
        )
        show_fit(fit, extended)
        case_path = case["path"]
        path.visible = len(case_path) > 0
        path.points = (
            segments(fourth.tool_points(robot, case_path))
            if len(case_path) > 0 else np.zeros((1, 2, 3))
        )
        clearance = case.get("clearance")
        clearance_text.value = (
            "not measured" if clearance is None else f"{clearance:+.4f} m"
        )
        cycle_text.value = str(case.get("cycle", "n/a"))
        box_top_text.value = (
            "n/a" if fit is None else f"{float(fit['fitted_top_z_m']):.4f} m"
        )
        tool_height_text.value = (
            f"{float(fourth.tool_position(robot, case_path[0])[2]):.4f} m"
            if len(case_path)
            else "n/a"
        )
        stop_text.value = str(case.get("stop_reason", "n/a"))
        status_text.value = case.get("status", "VALID")

    def playback(_event: Any) -> None:
        name = selector.value
        case = visuals[name]
        callback_thread = threading.current_thread()
        playback_done.clear()
        playback_error.value = "none"

        def worker_thread() -> None:
            commanded: list[np.ndarray] = []
            frustums: list[np.ndarray] = []
            try:
                history = case.get("history")
                if history:
                    sequences = [
                        (record["executed"], record["clearances"], record)
                        for record in history
                    ]
                else:
                    sequences = (
                        [(case["path"], None, None)] if len(case["path"]) else []
                    )
                for positions, clearances, record in sequences:
                    if record is not None:
                        cycle_text.value = str(record["cycle"])
                        stop_text.value = str(record.get("stop_reason") or "RUNNING")
                        cloud.points = (
                            record["points"]
                            if len(record["points"])
                            else np.zeros((1, 3))
                        )
                        show_fit(record["fit"], record["extension"]["active"])
                        box_top_text.value = (
                            "n/a"
                            if record["box_top_m"] is None
                            else f"{record['box_top_m']:.4f} m"
                        )
                        tool_height_text.value = (
                            f"{record['tool_height_start_m']:.4f} m"
                        )
                    if positions is None:
                        continue
                    if verification_mode.is_set() and len(positions) > 2:
                        indices = (0, len(positions) - 1)
                    else:
                        indices = range(len(positions))
                    for index in indices:
                        q = positions[index]
                        viewer.set_joint_state(arm_state(q).squeeze(0))
                        position, wxyz = fourth.camera_pose_for_q(robot, q, mount)
                        live_frustum.position = tuple(position)
                        live_frustum.wxyz = tuple(wxyz)
                        tool_height_text.value = (
                            f"{float(fourth.tool_position(robot, q)[2]):.4f} m"
                        )
                        if clearances is not None:
                            clearance_text.value = f"{float(clearances[index]):+.4f} m"
                        commanded.append(q.detach().cpu().numpy().copy())
                        frustums.append(position.copy())
                        if not verification_mode.is_set():
                            time.sleep(0.025)
                playback_record.clear()
                playback_record.update({
                    "case": name,
                    "callback_non_main": callback_thread is not threading.main_thread(),
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
                playback_error.value = message
                traceback.print_exc(file=__import__("sys").stderr)
                playback_record.clear()
                playback_record.update({"case": name, "error": message})
            finally:
                playback_done.set()

        threading.Thread(
            target=worker_thread, name=f"sixth-playback-{name}", daemon=True
        ).start()

    selector.on_update(apply_case)
    play.on_click(playback)
    apply_case()
    print(f"SIM_CAMERA_CUROBO_SIXTH_PASS_VISER http://127.0.0.1:{actual_port}", flush=True)

    def verify_callbacks() -> dict[str, Any]:
        records: list[dict[str, Any]] = []
        verification_mode.set()
        try:
            for name in visuals:
                if len(visuals[name]["path"]) < 2:
                    continue
                selector.value = name
                callback = threading.Thread(
                    target=playback, args=(None,), name=f"sixth-callback-{name}"
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
            "default_case": default_case,
            "only_e_and_baseline": all(
                name.startswith("E ") or name.startswith("INVALID_SCENE")
                or name == "NO OBSTACLE BASELINE"
                for name in visuals
            ),
        }

    return viewer, verify_callbacks


def build_results(
    worker: fourth.NewtonWorker,
    robot: Kinematics,
    robot_config: Path,
    goal: GoalToolPose,
    prior_blind_search: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, dict[str, Any]], MotionPlanner]:
    planner, limit_report = make_loop_planner(robot_config, CAMERA_FRONT_SEED)
    start_q, start_report = solve_raised_start(robot, planner)
    limit_report["solved_start_matches_limit_center"] = bool(
        abs(start_q[WRIST_3_INDEX] - limit_report["start_rad"]) <= 1.0e-6
    )
    fourth.update_fit_in_place(planner, None)
    baseline_result = planner.plan_pose(goal, arm_state(start_q))
    baseline = fourth.active_trajectory(baseline_result)
    if baseline is None:
        planner.destroy()
        raise RuntimeError(
            "raised no-obstacle baseline failed: " + str(fourth.failure_status(baseline_result))
        )
    baseline = baseline.detach().clone()
    goal_q = baseline[-1]
    baseline_motion = motion_metrics(robot, baseline, start_q)
    baseline_motion["planner_failure_status"] = fourth.failure_status(baseline_result)
    baseline_tool_climb = baseline_motion["tool_height_climb_from_start_m"]
    fixed_unknown_top_m = (
        float(start_report["camera_world_m"][2])
        + FIXED_UNKNOWN_TOP_ABOVE_START_CAMERA_M
    )
    scenes = scene_candidates(robot, baseline)
    results: dict[str, Any] = {
        "schema": 5,
        "date": "2026-11-06",
        "mode": "capture-merge-fit-update-replan-advance loop only; no scan or overview motion",
        "planner_truth_rule": "truth is used only for scene validity and offline scoring",
        "raised_start": start_report,
        "wrist_3_limit": limit_report,
        "speed_rule": (
            f"advance {NEAR_ADVANCE_RAD:.3f} rad when current-to-fit clearance is below "
            f"{NEAR_BOX_CLEARANCE_M:.3f} m; otherwise advance {FAR_ADVANCE_RAD:.3f} rad"
        ),
        "unknown_extent_rule": {
            "all_borders": (
                "transform each same-frame image-border outward direction to world, "
                "project to the horizontal plane, and extend by "
                f"{HORIZONTAL_UNKNOWN_MARGIN_M:.3f} m; border touches never change height"
            ),
            "normal_height": (
                f"highest observed point plus {FIT_PADDING_Z_M:.3f} m padding"
            ),
            "near_camera_exception": (
                f"when the same-frame highest point is within {CAMERA_NEAR_TOP_M:.3f} m "
                "of the camera height, use the fixed raised-start camera height plus "
                f"{FIXED_UNKNOWN_TOP_ABOVE_START_CAMERA_M:.3f} m"
            ),
            "fixed_unknown_top_m": fixed_unknown_top_m,
            "shrink_rule": (
                "horizontal extensions use only current-frame flags and disappear "
                "once that edge is inside"
            ),
        },
        "cycle_cap": CYCLE_CAP,
        "baseline_no_obstacle": {
            "states": int(len(baseline)),
            "motion": baseline_motion,
        },
        "scenes": {},
        "blind_search": {},
        "curobo_version": {
            "python": curobo.__version__,
            "checkout": subprocess.check_output(
                ["git", "-C", str(REPO / "curobo"), "describe", "--tags", "--always"],
                text=True,
            ).strip(),
        },
    }
    visuals: dict[str, dict[str, Any]] = {}
    first_valid: tuple[np.ndarray, np.ndarray] | None = None
    wrist_limits = tuple(limit_report["effective_planner_limits_rad"])
    for scene in scenes:
        top = scene["top_m"]
        key = f"top_{top:.2f}_m"
        center = scene["center"]
        dims = scene["dims"]
        validity = scene["validity"]
        entry: dict[str, Any] = {
            "true_top_z_m": top,
            "source": scene["source"],
            "validity": validity,
            "baseline_minimum_true_clearance_m": float(
                np.min(fourth.clearance_series(robot, baseline, center, dims))
            ),
        }
        if validity["valid"]:
            summary, history, path = run_loop(
                worker, planner, robot, start_q, goal, goal_q,
                center, dims, wrist_limits, fixed_unknown_top_m,
            )
            scene_tool_climb = summary["motion"]["tool_height_climb_from_start_m"]
            excess_climb = scene_tool_climb - baseline_tool_climb
            summary["tool_climb_vs_no_obstacle"] = {
                "scene_climb_m": scene_tool_climb,
                "no_obstacle_climb_m": baseline_tool_climb,
                "excess_m": excess_climb,
                "within_no_obstacle_climb": excess_climb <= 1.0e-4,
                "reason_if_higher": (
                    None
                    if excess_climb <= 1.0e-4
                    else "collision-free detour around the camera-fitted obstacle"
                ),
            }
            entry.update({
                "loop": summary,
                "first_frame": first_frame_report(history[0]["capture"], top, WALL_BOTTOM_M),
                "cycles": serialize_history(history),
            })
            if first_valid is None:
                first_valid = (center, dims)
            visuals[f"E wall top {top:.2f}"] = {
                "valid_e": True,
                "center": center, "dims": dims,
                "points": history[-1]["points"], "fit": history[-1]["fit"],
                "path": path, "history": history,
                "clearance": summary["minimum_true_clearance_m"],
                "cycle": summary["cycles"],
                "stop_reason": summary["final_stop_reason"],
                "status": "VALID E",
            }
        else:
            entry["loop"] = None
            visuals[f"INVALID_SCENE wall top {top:.2f}"] = {
                "valid_e": False,
                "center": center, "dims": dims,
                "points": np.empty((0, 3)), "fit": None,
                "path": torch.empty((0, 6), device="cuda"), "history": None,
                "clearance": validity["along_path_clearance_m"],
                "stop_reason": "INVALID_SCENE",
                "status": "INVALID_SCENE: " + "; ".join(validity["reasons"]),
            }
        results["scenes"][key] = entry
    results["blind_search"] = json.loads(json.dumps(prior_blind_search))
    results["blind_search"]["reused_from"] = (
        "fifth_pass.json; the 27-candidate search was not repeated"
    )
    if first_valid is None:
        planner.destroy()
        raise RuntimeError("no valid main E scene for viewer default")
    baseline_center, baseline_dims = first_valid
    visuals["NO OBSTACLE BASELINE"] = {
        "valid_e": False,
        "center": baseline_center, "dims": baseline_dims,
        "points": np.empty((0, 3)), "fit": None,
        "path": baseline, "history": None,
        "clearance": float(np.min(fourth.clearance_series(
            robot, baseline, baseline_center, baseline_dims
        ))),
        "stop_reason": "BASELINE",
        "status": "NO OBSTACLE BASELINE (truth collision expected)",
    }
    return results, visuals, planner


def main() -> int:
    parser = argparse.ArgumentParser()
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
    fifth_result_path = args.results_dir / "fifth_pass.json"
    if not fifth_result_path.is_file():
        raise RuntimeError(
            f"accepted blind-search source is missing: {fifth_result_path}"
        )
    prior_blind_search = json.loads(
        fifth_result_path.read_text(encoding="utf-8")
    )["blind_search"]
    if (
        prior_blind_search.get("candidate_count") != 27
        or prior_blind_search.get("selected") is not None
        or prior_blind_search.get("result") is not None
    ):
        raise RuntimeError(
            "fifth-pass blind-search result is not the accepted 27-candidate no-result"
        )
    robot = Kinematics(
        KinematicsCfg.from_robot_yaml_file(
            str(args.robot_config), tool_frames=[preflight.TOOL_FRAME]
        )
    )
    goal = goal_from_scene()
    worker_log: list[str] = []
    planner: MotionPlanner | None = None
    with tempfile.TemporaryDirectory(prefix="newton-camera-sixth-") as temp:
        worker = fourth.NewtonWorker(args.newton_python, Path(temp))
        try:
            results, visuals, planner = build_results(
                worker, robot, args.robot_config, goal, prior_blind_search
            )
        finally:
            worker.close()
            worker_log = worker.log
    viewer = None
    try:
        if not args.headless:
            viewer, verify = start_viewer(
                args.robot_config, robot,
                np.asarray(results["raised_start"]["joints_rad"], dtype=np.float32),
                visuals, args.port,
            )
            results["playback_verification"] = verify()
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
    results["newton_worker_lines"] = len(worker_log)
    args.results_dir.mkdir(parents=True, exist_ok=True)
    result_path = args.results_dir / "sixth_pass.json"
    log_path = args.results_dir / "sixth_pass.log"
    result_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    log_lines = [
        "SIM_CAMERA_CUROBO_SIXTH_PASS",
        "raised_start=" + json.dumps(results["raised_start"], sort_keys=True),
        "wrist_limit=" + json.dumps(results["wrist_3_limit"], sort_keys=True),
        "scenes=" + json.dumps({
            key: value.get("loop") for key, value in results["scenes"].items()
        }, sort_keys=True),
        "blind=" + json.dumps(results["blind_search"].get("result"), sort_keys=True),
        "playback=" + json.dumps(results.get("playback_verification"), sort_keys=True),
        f"newton_worker_lines={len(worker_log)}",
    ]
    log_path.write_text("\n".join(log_lines) + "\n", encoding="utf-8")
    print("SIM_CAMERA_CUROBO_SIXTH_PASS_RESULT " + json.dumps(results, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
