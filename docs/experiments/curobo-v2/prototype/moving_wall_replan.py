#!/usr/bin/env python3
"""Offline cuRobo moving-wall replanning demo for the UR5 cable pre-grasp.

This program intentionally starts neither ROS nor Newton.  It reuses the
accepted cuRobo/Viser preflight helpers while keeping that program unchanged.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import time
from typing import Any

import numpy as np
import torch

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.scene import Scene
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer

import ur5_viser_preflight as preflight


SCENE_PROFILE = "step_c_cable_wall"
WALL_NAME = "step_c_cable_wall"
WALL_CENTER_X_M = 0.487
WALL_AMPLITUDE_X_M = 0.100
WALL_PERIOD_S = 8.0


def wall_x_at_time(elapsed_s: float) -> float:
    """Return the requested sinusoidal wall-center x coordinate."""

    return WALL_CENTER_X_M + WALL_AMPLITUDE_X_M * math.sin(
        2.0 * math.pi * elapsed_s / WALL_PERIOD_S
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="cuRobo-only live replanning around a sinusoidally moving wall."
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument(
        "--start-delay",
        type=float,
        default=0.0,
        help="Viewer-only setup delay before the timed loop (useful for recording).",
    )
    parser.add_argument(
        "--advance-waypoints",
        type=int,
        default=1,
        help="Number of 25 ms plan samples to advance after every successful replan.",
    )
    parser.add_argument("--show-spheres", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--rebuild-model", action="store_true")
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=Path.home() / ".cache" / "ur5_moveit_demo" / "curobo_v2",
    )
    parser.add_argument("--result-json", type=Path)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be greater than zero")
    if args.interval <= 0.0:
        parser.error("--interval must be greater than zero")
    if args.start_delay < 0.0:
        parser.error("--start-delay cannot be negative")
    if args.advance_waypoints < 1:
        parser.error("--advance-waypoints must be at least one")
    return args


def active_trajectory(result: Any) -> JointState | None:
    """Return the valid, arm-only portion of a successful plan."""

    if result is None or result.success is None or not result.success.any():
        return None
    trajectory = preflight.effective_trajectory(
        result.interpolated_trajectory,
        result.interpolated_last_tstep,
    )
    return trajectory.reorder(list(preflight.ARM_JOINT_NAMES))


def trajectory_tool_points(planner: Any, trajectory: JointState) -> np.ndarray:
    """Convert a joint trajectory into world-frame tool positions for Viser."""

    positions = trajectory.position.reshape(-1, len(preflight.ARM_JOINT_NAMES))
    states = JointState.from_position(
        positions,
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    kinematics = planner.kinematics.compute_kinematics(states)
    return (
        kinematics.tool_poses[preflight.TOOL_FRAME]
        .position.detach()
        .cpu()
        .reshape(-1, 3)
        .numpy()
    )


def line_segments(points: np.ndarray) -> np.ndarray:
    """Convert ordered path points to Viser's N x 2 x 3 representation."""

    if len(points) < 2:
        points = np.repeat(points[:1], 2, axis=0)
    return np.stack((points[:-1], points[1:]), axis=1)


def summarize(samples_ms: list[float], successes: int, attempts: int) -> dict[str, Any]:
    """Build the machine-readable final timing summary."""

    ordered = sorted(samples_ms)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "device": "cuda:0",
        "scene_profile": SCENE_PROFILE,
        "wall_center_x_m": WALL_CENTER_X_M,
        "wall_amplitude_x_m": WALL_AMPLITUDE_X_M,
        "wall_period_s": WALL_PERIOD_S,
        "attempts": attempts,
        "successes": successes,
        "success_rate": successes / attempts if attempts else 0.0,
        "mean_replan_ms": statistics.fmean(samples_ms) if samples_ms else None,
        "p95_replan_ms": ordered[p95_index] if ordered else None,
        "max_replan_ms": max(samples_ms) if samples_ms else None,
        "finite_timings": bool(samples_ms) and all(math.isfinite(x) for x in samples_ms),
    }


def main() -> int:
    args = parse_args()
    repo = preflight.repository_root()
    runtime = preflight.verify_cuda_and_source(repo)
    config_path, cache_status = preflight.prepare_robot_model(
        repo,
        args.cache_dir.expanduser().resolve(),
        args.rebuild_model,
    )
    scene, scene_model, object_center = preflight.load_scene(repo, SCENE_PROFILE)
    if WALL_NAME not in scene_model["cuboid"]:
        raise RuntimeError(f"Scene profile {SCENE_PROFILE} has no {WALL_NAME!r} cuboid")

    wall_pose = list(scene_model["cuboid"][WALL_NAME]["pose"])
    planner = preflight.create_pick_planner(config_path, scene_model)
    robot = Kinematics(KinematicsCfg.from_robot_yaml_file(str(config_path)))
    current_state = preflight.make_initial_joint_state(
        robot,
        preflight.load_initial_arm_positions(repo),
    ).unsqueeze(0)
    target_position, target_orientation = preflight.pregrasp_target(object_center)
    target_pose = Pose.from_list(
        [*target_position.tolist(), *target_orientation.tolist()]
    )
    goal = GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: target_pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )

    visualizer = None
    wall_handle = None
    status_handle = None
    if not args.headless:
        visualizer = ViserVisualizer(
            content_path=ContentPath(robot_config_file=str(config_path)),
            connect_ip="0.0.0.0",
            connect_port=args.port,
            add_control_frames=True,
            visualize_robot_spheres=args.show_spheres,
            add_robot_to_scene=True,
        )
        visualizer._server.initial_camera.position = (1.20, 1.15, 0.75)
        visualizer._server.initial_camera.look_at = (0.30, 0.25, 0.25)
        visualizer._server.initial_camera.up = (0.0, 0.0, 1.0)
        visualizer.set_joint_state(current_state.squeeze(0))
        wall_handle = visualizer._server.scene.add_box(
            name=f"/obstacles/{WALL_NAME}",
            dimensions=np.asarray(scene_model["cuboid"][WALL_NAME]["dims"]),
            position=np.asarray(wall_pose[:3]),
            color=(185, 75, 65),
            opacity=0.85,
        )
        visualizer._server.scene.add_icosphere(
            name="/targets/cable_pregrasp",
            position=target_position,
            radius=0.012,
            color=(255, 170, 0),
        )
        status_handle = visualizer._server.gui.add_markdown(
            "**Starting CUDA replanning...** Wall moves along world X."
        )
        print(f"MOVING_WALL_VISER_URL http://127.0.0.1:{args.port}/")

    print(
        "CUROBO_MOVING_WALL_START "
        + json.dumps(
            {
                **runtime,
                **cache_status,
                "duration_s": args.duration,
                "interval_s": args.interval,
                "scene_profile": SCENE_PROFILE,
                "wall_size_m": scene_model["cuboid"][WALL_NAME]["dims"],
                "wall_center_xyz_m": wall_pose[:3],
                "target_position_m": target_position.tolist(),
                "ros": False,
                "newton": False,
            }
        )
    )

    # The first real query initializes lazy CUDA kernels that planner.warmup()
    # does not cover. Prime that path before starting the 30-second timing
    # window so one-time setup is not mislabeled as a periodic replan.
    torch.cuda.synchronize()
    prime_started = time.perf_counter()
    prime_result = planner.plan_pose(goal, current_state)
    torch.cuda.synchronize()
    prime_ms = (time.perf_counter() - prime_started) * 1000.0
    if active_trajectory(prime_result) is None:
        raise RuntimeError("Initial moving-wall planner prime failed")
    print(
        "CUROBO_MOVING_WALL_PRIME "
        + json.dumps({"replan_ms": prime_ms, "success": True}),
        flush=True,
    )
    if args.start_delay > 0.0:
        if status_handle is not None:
            status_handle.content = (
                f"**Ready.** Timed replanning starts in `{args.start_delay:.1f} s`."
            )
        print(f"CUROBO_MOVING_WALL_START_DELAY {args.start_delay:.3f}", flush=True)
        time.sleep(args.start_delay)

    samples_ms: list[float] = []
    successes = 0
    attempts = 0
    started = time.monotonic()
    next_tick = started
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if elapsed >= args.duration:
                break
            if now < next_tick:
                time.sleep(next_tick - now)
            now = time.monotonic()
            elapsed = now - started
            if elapsed >= args.duration:
                break

            wall_x = wall_x_at_time(elapsed)
            wall_pose[0] = wall_x
            curobo_wall_pose = Pose.from_list(wall_pose)
            planner.scene_collision_checker.update_obstacle_pose(
                WALL_NAME,
                curobo_wall_pose,
            )
            if wall_handle is not None:
                wall_handle.position = np.asarray(wall_pose[:3], dtype=np.float64)

            torch.cuda.synchronize()
            plan_started = time.perf_counter()
            result = planner.plan_pose(goal, current_state)
            torch.cuda.synchronize()
            replan_ms = (time.perf_counter() - plan_started) * 1000.0
            samples_ms.append(replan_ms)
            attempts += 1
            trajectory = active_trajectory(result)
            success = trajectory is not None

            waypoint_count = 0
            if trajectory is not None:
                successes += 1
                positions = trajectory.position.reshape(
                    -1, len(preflight.ARM_JOINT_NAMES)
                )
                waypoint_count = int(positions.shape[0])
                advance_index = min(args.advance_waypoints, waypoint_count - 1)
                current_state = JointState.from_position(
                    positions[advance_index].unsqueeze(0),
                    joint_names=list(preflight.ARM_JOINT_NAMES),
                )
                if visualizer is not None:
                    visualizer.set_joint_state(current_state.squeeze(0))
                    if args.show_spheres:
                        visualizer.update_robot_spheres(current_state)
                    tool_points = trajectory_tool_points(planner, trajectory)
                    visualizer._server.scene.add_line_segments(
                        name="/plans/current_tool_path",
                        points=line_segments(tool_points),
                        colors=(40, 220, 255),
                        line_width=4.0,
                    )

            record = {
                "attempt": attempts,
                "elapsed_s": elapsed,
                "wall_x_m": wall_x,
                "replan_ms": replan_ms,
                "success": success,
                "waypoints": waypoint_count,
            }
            print("MOVING_WALL_REPLAN " + json.dumps(record), flush=True)
            if status_handle is not None:
                status_handle.content = (
                    f"**Live CUDA replanning**  \n"
                    f"Attempt: `{attempts}` | Wall x: `{wall_x:+.3f} m`  \n"
                    f"Latest: `{replan_ms:.1f} ms` | Success: `{success}`  \n"
                    f"Running success rate: `{100.0 * successes / attempts:.1f}%`"
                )
            # If a plan overruns its 200 ms slot, resume from the present
            # instead of issuing a burst of stale catch-up queries.
            next_tick = max(next_tick + args.interval, time.monotonic())
    except KeyboardInterrupt:
        print("MOVING_WALL_INTERRUPTED", flush=True)

    summary = summarize(samples_ms, successes, attempts)
    summary.update(
        {
            "requested_duration_s": args.duration,
            "wall_elapsed_s": time.monotonic() - started,
            "interval_s": args.interval,
        }
    )
    print("CUROBO_MOVING_WALL_RESULT " + json.dumps(summary), flush=True)
    if status_handle is not None:
        status_handle.content = (
            "**Demo complete.**  \n"
            f"Mean / p95 / max: `{summary['mean_replan_ms']:.1f}` / "
            f"`{summary['p95_replan_ms']:.1f}` / `{summary['max_replan_ms']:.1f} ms`  \n"
            f"Success rate: `{100.0 * summary['success_rate']:.1f}%`"
        )
    if args.result_json is not None:
        output_path = args.result_json.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"CUROBO_MOVING_WALL_RESULT_JSON {output_path}")
    return 0 if attempts > 0 and successes == attempts else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"CUROBO_MOVING_WALL_ERROR {error}", file=sys.stderr)
        raise SystemExit(1)
