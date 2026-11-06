#!/usr/bin/env python3
"""Plan against Newton camera cuboids and compare them with true geometry."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time
from typing import Any

import numpy as np
import torch

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.scene import Cuboid, Scene
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer


PROTOTYPE = Path(__file__).resolve().parents[1] / "docs/experiments/curobo-v2/prototype"
sys.path.insert(0, str(PROTOTYPE))
import ur5_viser_preflight as preflight  # noqa: E402


FIT_PADDING_XY_M = 0.015
FIT_PADDING_Z_M = 0.130
MIN_PERCEIVED_POINTS = 40
CASE_NAMES = ("A no planner obstacle", "B god view", "C start camera", "D 4-pose scan", "blind spot")


def fit_camera_cuboid(points: np.ndarray, name: str) -> tuple[dict[str, Any] | None, float]:
    started = time.perf_counter()
    if len(points) < MIN_PERCEIVED_POINTS:
        return None, (time.perf_counter() - started) * 1000.0
    low = np.min(points, axis=0) - np.array((FIT_PADDING_XY_M, FIT_PADDING_XY_M, FIT_PADDING_Z_M))
    high = np.max(points, axis=0) + np.array((FIT_PADDING_XY_M, FIT_PADDING_XY_M, FIT_PADDING_Z_M))
    low[2] = max(low[2], 0.0)
    center = 0.5 * (low + high)
    dims = np.maximum(high - low, 0.01)
    result = {
        "name": name,
        "center_m": center.tolist(),
        "dims_m": dims.tolist(),
        "points": int(len(points)),
    }
    return result, (time.perf_counter() - started) * 1000.0


def cuboid_entry(name: str, center: np.ndarray, dims: np.ndarray) -> dict[str, list[float]]:
    return {
        "dims": [float(value) for value in dims],
        "pose": [*[float(value) for value in center], 1.0, 0.0, 0.0, 0.0],
    }


def active_trajectory(result: Any) -> JointState | None:
    if result is None or result.success is None or not result.success.any():
        return None
    trajectory = preflight.effective_trajectory(
        result.interpolated_trajectory, result.interpolated_last_tstep
    )
    return trajectory.reorder(list(preflight.ARM_JOINT_NAMES))


def goal_from_pose(pose: Pose) -> GoalToolPose:
    return GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )


def true_cuboid_clearance(
    planner: Any,
    positions: torch.Tensor,
    center: np.ndarray,
    dims: np.ndarray,
) -> float:
    state = JointState.from_position(
        positions.reshape(-1, len(preflight.ARM_JOINT_NAMES)),
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    spheres = planner.kinematics.compute_kinematics(state).robot_spheres
    if spheres is None:
        raise RuntimeError("cuRobo did not return robot collision spheres")
    values = spheres.detach().cpu().numpy().reshape(-1, 4)
    relative = np.abs(values[:, :3] - center) - 0.5 * dims
    outside = np.linalg.norm(np.maximum(relative, 0.0), axis=1)
    inside = np.minimum(np.max(relative, axis=1), 0.0)
    return float(np.min(outside + inside - values[:, 3]))


def joint_path_length(positions: torch.Tensor) -> float:
    values = positions.detach().cpu().numpy().reshape(-1, len(preflight.ARM_JOINT_NAMES))
    return float(np.linalg.norm(np.diff(values, axis=0), axis=1).sum())


def tool_path(planner: Any, positions: torch.Tensor) -> np.ndarray:
    state = JointState.from_position(
        positions.reshape(-1, len(preflight.ARM_JOINT_NAMES)),
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    poses = planner.kinematics.compute_kinematics(state).tool_poses[preflight.TOOL_FRAME]
    return poses.position.detach().cpu().numpy().reshape(-1, 3)


def path_segments(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        points = np.repeat(points[:1], 2, axis=0)
    return np.stack((points[:-1], points[1:]), axis=1)


def quaternion_multiply_xyzw(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def frustum_wxyz(camera_transform: np.ndarray) -> np.ndarray:
    xyzw = quaternion_multiply_xyzw(
        np.asarray(camera_transform[3:7]), np.array((1.0, 0.0, 0.0, 0.0))
    )
    return xyzw[[3, 0, 1, 2]]


def summarize_case(records: list[dict[str, Any]]) -> dict[str, Any]:
    successes = [record for record in records if record["success"]]
    return {
        "attempts": len(records),
        "successes": len(successes),
        "success_rate": len(successes) / len(records),
        "planning_median_ms": float(statistics.median(record["planning_ms"] for record in records)),
        "minimum_true_clearance_m": (
            float(min(record["true_clearance_m"] for record in successes)) if successes else None
        ),
        "path_length_median_rad": (
            float(statistics.median(record["path_length_rad"] for record in successes)) if successes else None
        ),
        "records": records,
    }


def build_viewer(
    robot_config: Path,
    planner: Any,
    start_state: JointState,
    input_data: Any,
    cuboids: dict[str, dict[str, Any] | None],
    results: dict[str, dict[str, Any]],
    trajectories: dict[str, torch.Tensor],
    port: int,
) -> tuple[ViserVisualizer, threading.Event]:
    viewer = ViserVisualizer(
        content_path=ContentPath(robot_config_file=str(robot_config)),
        connect_ip="127.0.0.1",
        connect_port=port,
        add_control_frames=True,
        visualize_robot_spheres=False,
        add_robot_to_scene=True,
    )
    server = viewer._server
    server.initial_camera.position = (1.15, 1.10, 0.85)
    server.initial_camera.look_at = (0.48, 0.27, 0.22)
    server.initial_camera.up = (0.0, 0.0, 1.0)
    viewer.set_joint_state(start_state.squeeze(0))

    main_center, main_dims = input_data["main_center"], input_data["main_dims"]
    blind_center, blind_dims = input_data["blind_center"], input_data["blind_dims"]
    truth_handles = {
        "main": server.scene.add_box(
            "/truth/main", dimensions=tuple(main_dims), position=tuple(main_center),
            color=(230, 70, 55), opacity=0.65,
        ),
        "blind": server.scene.add_box(
            "/truth/blind", dimensions=tuple(blind_dims), position=tuple(blind_center),
            color=(230, 70, 55), opacity=0.65,
        ),
    }
    perceived_handles: dict[str, Any] = {}
    for key, cuboid in cuboids.items():
        if cuboid is None or not key.startswith("perceived"):
            continue
        perceived_handles[key] = server.scene.add_box(
            f"/perceived/{key}", dimensions=tuple(cuboid["dims_m"]),
            position=tuple(cuboid["center_m"]), color=(55, 205, 235), opacity=0.45,
        )

    camera = input_data["main_camera_transforms"][0]
    server.scene.add_camera_frustum(
        "/camera/start_frustum", fov=math.radians(58.0), aspect=848 / 480,
        scale=0.14, color=(255, 215, 0), position=tuple(camera[:3]),
        wxyz=tuple(frustum_wxyz(camera)),
    )
    path_handle = server.scene.add_line_segments(
        "/plans/selected", points=np.zeros((1, 2, 3)), colors=(40, 255, 120), line_width=4.0
    )
    show_true = server.gui.add_checkbox("Show true obstacle", initial_value=True)
    selector = server.gui.add_dropdown("Case", options=CASE_NAMES, initial_value=CASE_NAMES[0])
    clearance = server.gui.add_text("True clearance", "", disabled=True)
    playback = server.gui.add_button("Play selected path")
    stop = threading.Event()

    planner_names = {
        CASE_NAMES[0]: None,
        CASE_NAMES[1]: "true_main",
        CASE_NAMES[2]: "perceived_start",
        CASE_NAMES[3]: "perceived_scan",
        CASE_NAMES[4]: "perceived_blind",
    }

    def update(_event: Any = None) -> None:
        case = selector.value
        truth_handles["main"].visible = bool(show_true.value and case != CASE_NAMES[4])
        truth_handles["blind"].visible = bool(show_true.value and case == CASE_NAMES[4])
        selected = planner_names[case]
        for name, handle in perceived_handles.items():
            handle.visible = name == selected
        positions = trajectories.get(case)
        if positions is not None:
            path_handle.points = path_segments(tool_path(planner, positions))
        else:
            path_handle.points = np.zeros((1, 2, 3))
        value = results[case]["minimum_true_clearance_m"]
        clearance.value = "no successful plan" if value is None else f"{value:+.4f} m"

    def play(_event: Any) -> None:
        case = selector.value
        positions = trajectories.get(case)
        if positions is None:
            return

        def worker() -> None:
            for position in positions[:: max(1, len(positions) // 150)]:
                state = JointState.from_position(
                    position.unsqueeze(0), joint_names=list(preflight.ARM_JOINT_NAMES)
                )
                viewer.set_joint_state(state.squeeze(0))
                time.sleep(0.025)

        threading.Thread(target=worker, daemon=True).start()

    selector.on_update(update)
    show_true.on_update(update)
    playback.on_click(play)
    update()
    print(f"SIM_CAMERA_CUROBO_VISER_URL http://127.0.0.1:{port}", flush=True)
    return viewer, stop


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--port", type=int, default=8084)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--result-json", type=Path)
    parser.add_argument(
        "--robot-config", type=Path,
        default=Path.home() / ".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml",
    )
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error("--repeats must be at least 3")
    data = np.load(args.input)
    capture_metadata = json.loads(str(data["metadata_json"]))
    perceived_start, fit_start_ms = fit_camera_cuboid(data["main_start_points"], "perceived_start")
    perceived_scan, fit_scan_ms = fit_camera_cuboid(data["main_scan_points"], "perceived_scan")
    perceived_blind, fit_blind_ms = fit_camera_cuboid(data["blind_start_points"], "perceived_blind")
    main_center, main_dims = data["main_center"], data["main_dims"]
    blind_center, blind_dims = data["blind_center"], data["blind_dims"]
    cuboids: dict[str, dict[str, Any] | None] = {
        "true_main": {"name": "true_main", "center_m": main_center.tolist(), "dims_m": main_dims.tolist()},
        "perceived_start": perceived_start,
        "perceived_scan": perceived_scan,
        "true_blind": {"name": "true_blind", "center_m": blind_center.tolist(), "dims_m": blind_dims.tolist()},
        "perceived_blind": perceived_blind,
    }
    scene_cuboids = {
        name: cuboid_entry(name, np.asarray(cuboid["center_m"]), np.asarray(cuboid["dims_m"]))
        for name, cuboid in cuboids.items() if cuboid is not None
    }
    scene = Scene(
        cuboid=[
            Cuboid(name=name, dims=value["dims"], pose=value["pose"])
            for name, value in scene_cuboids.items()
        ]
    )
    del scene
    planner = preflight.create_pick_planner(args.robot_config, {"cuboid": scene_cuboids})
    robot = Kinematics(KinematicsCfg.from_robot_yaml_file(str(args.robot_config)))
    repo = preflight.repository_root()
    start_state = preflight.make_initial_joint_state(
        robot, preflight.load_initial_arm_positions(repo)
    ).unsqueeze(0)
    _, _, object_center = preflight.load_scene(repo, "step_c_cable_no_wall")
    goal_position, goal_quaternion = preflight.pregrasp_target(object_center)
    goal = goal_from_pose(
        Pose.from_list([*goal_position.tolist(), *goal_quaternion.tolist()])
    )
    for name in scene_cuboids:
        planner.scene_collision_checker.enable_obstacle(name, False)

    case_obstacles = {
        CASE_NAMES[0]: None,
        CASE_NAMES[1]: "true_main",
        CASE_NAMES[2]: "perceived_start",
        CASE_NAMES[3]: "perceived_scan",
        CASE_NAMES[4]: "perceived_blind" if perceived_blind is not None else None,
    }
    case_truth = {
        **{case: (main_center, main_dims) for case in CASE_NAMES[:4]},
        CASE_NAMES[4]: (blind_center, blind_dims),
    }
    results: dict[str, dict[str, Any]] = {}
    trajectories: dict[str, torch.Tensor] = {}
    for case in CASE_NAMES:
        for name in scene_cuboids:
            planner.scene_collision_checker.enable_obstacle(name, name == case_obstacles[case])
        records: list[dict[str, Any]] = []
        for repeat in range(args.repeats):
            torch.manual_seed(1000 + repeat)
            torch.cuda.synchronize()
            started = time.perf_counter()
            plan_result = planner.plan_pose(goal, start_state)
            torch.cuda.synchronize()
            planning_ms = (time.perf_counter() - started) * 1000.0
            trajectory = active_trajectory(plan_result)
            record: dict[str, Any] = {
                "repeat": repeat,
                "success": trajectory is not None,
                "planning_ms": planning_ms,
                "true_clearance_m": None,
                "path_length_rad": None,
            }
            if trajectory is not None:
                positions = trajectory.position.reshape(-1, len(preflight.ARM_JOINT_NAMES))
                center, dims = case_truth[case]
                record["true_clearance_m"] = true_cuboid_clearance(
                    planner, positions, center, dims
                )
                record["path_length_rad"] = joint_path_length(positions)
                trajectories.setdefault(case, positions.detach().clone())
            records.append(record)
        results[case] = summarize_case(records)

    conversion = {
        "newton_cloud_median_ms": capture_metadata["conversion_median_ms"],
        "fit_start_ms": fit_start_ms,
        "fit_scan_ms": fit_scan_ms,
        "fit_blind_ms": fit_blind_ms,
    }
    output = {
        "curobo_version": "0.8.0 source v0.8.0-43-g78fd485",
        "exchange": "compressed NPZ between separate Newton and cuRobo processes",
        "capture": capture_metadata,
        "conversion": conversion,
        "fitted_cuboids": cuboids,
        "cases": results,
        "no_obstacle_path_collides_true": (
            results[CASE_NAMES[0]]["minimum_true_clearance_m"] is not None
            and results[CASE_NAMES[0]]["minimum_true_clearance_m"] < 0.0
        ),
        "blind_start_view_points": int(len(data["blind_start_points"])),
    }
    print("SIM_CAMERA_CUROBO_RESULT " + json.dumps(output, sort_keys=True), flush=True)
    if args.result_json:
        args.result_json.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")

    viewer = None
    try:
        if not args.headless:
            viewer, _ = build_viewer(
                args.robot_config, planner, start_state, data, cuboids, results,
                trajectories, args.port,
            )
            started = time.monotonic()
            while args.duration <= 0.0 or time.monotonic() - started < args.duration:
                time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer._server.stop()
    required = (
        output["no_obstacle_path_collides_true"]
        and results[CASE_NAMES[1]]["successes"] > 0
        and results[CASE_NAMES[3]]["successes"] > 0
    )
    return 0 if required else 2


if __name__ == "__main__":
    raise SystemExit(main())
