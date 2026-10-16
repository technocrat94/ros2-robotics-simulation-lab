#!/usr/bin/env python3
"""Live cuRobo-only UR5 dodging demo with a route-crossing cube.

The arm shuttles between its recorded start pose and the cable pre-grasp pose.
Every control cycle updates the cube, replans from the simulated current joint
state, and executes one full cycle of trajectory time.  ROS and Newton are not
started and no command is sent to robot hardware.
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

from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.scene import Cuboid, Scene
from curobo.types import ContentPath, GoalToolPose, JointState, Pose
from curobo.viewer import ViserVisualizer

import ur5_viser_preflight as preflight


CUBE_NAME = "route_crossing_cube"
CUBE_SIZE_M = 0.10
CUBE_CENTER_X_M = 0.487
# The handoff suggested z ~= 0.40 m.  With this repository's fitted UR5 +
# Robotiq collision spheres, a 0.10 m cube at z=0.40 sweeps through both end
# configurations (there is no y where both are collision-free).  z=0.52 is
# the nearest tested height that preserves the requested x/y crossing and a
# positive endpoint clearance for the complete y sweep.
CUBE_CENTER_Z_M = 0.52
CUBE_Y_MIN_M = 0.15
CUBE_Y_MAX_M = 0.38
CUBE_PERIOD_S = 6.0
INTERPOLATION_DT_S = 0.025
DISTANCE_ACTIVATION_M = 2.0
GOAL_POSITION_TOLERANCE_M = 0.006


def cube_y_at_time(elapsed_s: float) -> float:
    """Return a smooth back-and-forth cube position spanning both limits."""

    midpoint = 0.5 * (CUBE_Y_MIN_M + CUBE_Y_MAX_M)
    amplitude = 0.5 * (CUBE_Y_MAX_M - CUBE_Y_MIN_M)
    # Begin at the upper limit so the first lazy planner query has an endpoint
    # pose, rather than a route-centre pose, for the moving obstacle.
    return midpoint + amplitude * math.cos(2.0 * math.pi * elapsed_s / CUBE_PERIOD_S)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live CUDA cuRobo replanning while a cube crosses the UR5 route."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run; zero keeps the live viewer running until Ctrl-C.",
    )
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--port", type=int, default=8082)
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
    if args.duration < 0.0:
        parser.error("--duration cannot be negative")
    if args.interval <= 0.0:
        parser.error("--interval must be greater than zero")
    return args


def make_scene(cube_y_m: float) -> tuple[Scene, dict[str, Any], list[float]]:
    """Create a scene containing only the moving cube, never the Step C wall."""

    cube_pose = [
        CUBE_CENTER_X_M,
        cube_y_m,
        CUBE_CENTER_Z_M,
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    scene_model = {
        "cuboid": {
            CUBE_NAME: {
                "dims": [CUBE_SIZE_M, CUBE_SIZE_M, CUBE_SIZE_M],
                "pose": cube_pose,
            }
        }
    }
    scene = Scene(
        cuboid=[
            Cuboid(
                name=CUBE_NAME,
                dims=scene_model["cuboid"][CUBE_NAME]["dims"],
                pose=cube_pose,
            )
        ]
    )
    return scene, scene_model, cube_pose


def goal_from_pose(pose: Pose) -> GoalToolPose:
    return GoalToolPose.from_poses(
        {preflight.TOOL_FRAME: pose},
        ordered_tool_frames=[preflight.TOOL_FRAME],
        num_goalset=1,
    )


def active_trajectory(result: Any) -> JointState | None:
    """Return the valid, arm-only portion of a successful plan."""

    if result is None or result.success is None or not result.success.any():
        return None
    trajectory = preflight.effective_trajectory(
        result.interpolated_trajectory,
        result.interpolated_last_tstep,
    )
    return trajectory.reorder(list(preflight.ARM_JOINT_NAMES))


def trajectory_tool_points(planner: Any, positions: torch.Tensor) -> np.ndarray:
    """Convert joint positions to world-frame tool positions for Viser."""

    state = JointState.from_position(
        positions.reshape(-1, len(preflight.ARM_JOINT_NAMES)),
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    kinematics = planner.kinematics.compute_kinematics(state)
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


def make_collision_buffer(planner: Any, sample_state: JointState) -> CollisionBuffer:
    """Allocate the reusable cuRobo collision-query output buffers."""

    kinematics = planner.kinematics.compute_kinematics(sample_state)
    if kinematics.robot_spheres is None:
        raise RuntimeError("cuRobo kinematics did not produce collision spheres")
    return CollisionBuffer.from_shape(
        kinematics.robot_spheres.shape,
        planner.scene_collision_checker.device_cfg,
    )


def minimum_cube_clearance_m(
    planner: Any,
    positions: torch.Tensor,
    collision_buffer: CollisionBuffer,
) -> float:
    """Measure signed robot-to-cube clearance through cuRobo's checker.

    cuRobo's distance kernel reports a smoothed activation cost per robot
    sphere.  With unit weight and one obstacle, its documented activation
    formula is invertible.  The largest cost corresponds to the smallest
    sphere-surface clearance.  Positive returned values are separation;
    negative values are penetration.
    """

    joint_state = JointState.from_position(
        positions.reshape(-1, len(preflight.ARM_JOINT_NAMES)),
        joint_names=list(preflight.ARM_JOINT_NAMES),
    )
    kinematics = planner.kinematics.compute_kinematics(joint_state)
    spheres = kinematics.robot_spheres
    if spheres is None:
        raise RuntimeError("cuRobo kinematics did not produce collision spheres")
    collision_buffer.resize(spheres.shape, planner.scene_collision_checker.device_cfg)
    device = spheres.device
    weight = torch.ones(1, device=device, dtype=spheres.dtype)
    activation = torch.full(
        (1,), DISTANCE_ACTIVATION_M, device=device, dtype=spheres.dtype
    )
    costs = planner.scene_collision_checker.get_sphere_distance_raw(
        spheres,
        collision_buffer,
        weight,
        activation,
    )
    maximum_cost = float(costs.max().item())
    eta = DISTANCE_ACTIVATION_M
    if maximum_cost <= 0.0:
        return eta
    if maximum_cost <= 0.5 * eta:
        penetration = math.sqrt(2.0 * eta * maximum_cost)
    else:
        penetration = maximum_cost + 0.5 * eta
    return eta - penetration


def tool_position(planner: Any, state: JointState) -> np.ndarray:
    pose = planner.kinematics.compute_kinematics(state).tool_poses[
        preflight.TOOL_FRAME
    ]
    return pose.position.detach().cpu().reshape(-1, 3)[0].numpy()


def summarize(
    samples_ms: list[float],
    successes: int,
    attempts: int,
    clearances_m: list[float],
    goal_switches: int,
) -> dict[str, Any]:
    """Build the machine-readable final result."""

    ordered = sorted(samples_ms)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    minimum_clearance = min(clearances_m) if clearances_m else None
    return {
        "device": "cuda:0",
        "attempts": attempts,
        "successes": successes,
        "success_rate": successes / attempts if attempts else 0.0,
        "mean_replan_ms": statistics.fmean(samples_ms) if samples_ms else None,
        "p95_replan_ms": ordered[p95_index] if ordered else None,
        "max_replan_ms": max(samples_ms) if samples_ms else None,
        "overall_minimum_clearance_m": minimum_clearance,
        "clearance_positive": minimum_clearance is not None and minimum_clearance > 0.0,
        "goal_switches": goal_switches,
        "finite_measurements": bool(samples_ms)
        and all(math.isfinite(value) for value in samples_ms + clearances_m),
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

    initial_cube_y = cube_y_at_time(0.0)
    _scene, scene_model, cube_pose = make_scene(initial_cube_y)
    planner = preflight.create_pick_planner(config_path, scene_model)
    robot = Kinematics(KinematicsCfg.from_robot_yaml_file(str(config_path)))
    current_state = preflight.make_initial_joint_state(
        robot,
        preflight.load_initial_arm_positions(repo),
    ).unsqueeze(0)

    start_kinematics = planner.kinematics.compute_kinematics(current_state)
    start_tool_pose = start_kinematics.tool_poses[preflight.TOOL_FRAME]
    start_position = start_tool_pose.position.detach().cpu().reshape(-1, 3)[0].numpy()
    start_quaternion = (
        start_tool_pose.quaternion.detach().cpu().reshape(-1, 4)[0].numpy()
    )
    _, _, object_center = preflight.load_scene(repo, "step_c_cable_no_wall")
    pregrasp_position, pregrasp_quaternion = preflight.pregrasp_target(object_center)
    target_poses = {
        "cable_pregrasp": Pose.from_list(
            [*pregrasp_position.tolist(), *pregrasp_quaternion.tolist()]
        ),
        "start": Pose.from_list([*start_position.tolist(), *start_quaternion.tolist()]),
    }
    goals = {name: goal_from_pose(pose) for name, pose in target_poses.items()}
    target_positions = {
        "cable_pregrasp": pregrasp_position,
        "start": start_position,
    }
    goal_name = "cable_pregrasp"

    visualizer = None
    cube_handle = None
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
        visualizer._server.initial_camera.position = (1.20, 1.15, 0.80)
        visualizer._server.initial_camera.look_at = (0.35, 0.27, 0.30)
        visualizer._server.initial_camera.up = (0.0, 0.0, 1.0)
        visualizer.set_joint_state(current_state.squeeze(0))
        cube_handle = visualizer._server.scene.add_box(
            name=f"/obstacles/{CUBE_NAME}",
            dimensions=np.asarray([CUBE_SIZE_M] * 3),
            position=np.asarray(cube_pose[:3]),
            color=(220, 65, 55),
            opacity=0.88,
        )
        visualizer._server.scene.add_icosphere(
            name="/targets/start",
            position=start_position,
            radius=0.012,
            color=(80, 180, 255),
        )
        visualizer._server.scene.add_icosphere(
            name="/targets/cable_pregrasp",
            position=pregrasp_position,
            radius=0.012,
            color=(255, 170, 0),
        )
        status_handle = visualizer._server.gui.add_markdown(
            "**Starting CUDA dodge replanning...**"
        )
        print(f"CUROBO_DODGE_VISER_URL http://127.0.0.1:{args.port}/")

    print(
        "CUROBO_DODGE_START "
        + json.dumps(
            {
                **runtime,
                **cache_status,
                "duration_s": args.duration,
                "interval_s": args.interval,
                "trajectory_dt_s": INTERPOLATION_DT_S,
                "cube_size_m": [CUBE_SIZE_M] * 3,
                "cube_xz_m": [CUBE_CENTER_X_M, CUBE_CENTER_Z_M],
                "cube_y_limits_m": [CUBE_Y_MIN_M, CUBE_Y_MAX_M],
                "cube_period_s": CUBE_PERIOD_S,
                "start_tool_position_m": start_position.tolist(),
                "pregrasp_tool_position_m": pregrasp_position.tolist(),
                "ros": False,
                "newton": False,
            }
        ),
        flush=True,
    )

    # Prime lazy planner kernels without the dynamic obstacle.  Its initial
    # upper-limit pose deliberately guards the pre-grasp endpoint, so using it
    # during the one-time warm-up would reject the goal before the live loop.
    planner.scene_collision_checker.enable_obstacle(CUBE_NAME, False)
    torch.cuda.synchronize()
    prime_started = time.perf_counter()
    prime_result = planner.plan_pose(goals[goal_name], current_state)
    torch.cuda.synchronize()
    prime_ms = (time.perf_counter() - prime_started) * 1000.0
    planner.scene_collision_checker.enable_obstacle(CUBE_NAME, True)
    prime_trajectory = active_trajectory(prime_result)
    if prime_trajectory is None:
        raise RuntimeError("Initial cuRobo dodge planner prime failed")
    collision_buffer = make_collision_buffer(planner, current_state)
    prime_clearance = minimum_cube_clearance_m(
        planner, current_state.position, collision_buffer
    )
    print(
        "CUROBO_DODGE_PRIME "
        + json.dumps(
            {
                "replan_ms": prime_ms,
                "success": True,
                "minimum_clearance_m": prime_clearance,
            }
        ),
        flush=True,
    )

    samples_ms: list[float] = []
    clearances_m: list[float] = []
    successes = 0
    attempts = 0
    goal_switches = 0
    started = time.monotonic()
    next_tick = started
    interrupted = False
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            if now < next_tick:
                time.sleep(next_tick - now)
            now = time.monotonic()
            elapsed = now - started
            if args.duration > 0.0 and elapsed >= args.duration:
                break

            cube_y = cube_y_at_time(elapsed)
            cube_pose[1] = cube_y
            planner.scene_collision_checker.update_obstacle_pose(
                CUBE_NAME,
                Pose.from_list(cube_pose),
            )
            if cube_handle is not None:
                cube_handle.position = np.asarray(cube_pose[:3], dtype=np.float64)

            torch.cuda.synchronize()
            plan_started = time.perf_counter()
            result = planner.plan_pose(goals[goal_name], current_state)
            torch.cuda.synchronize()
            replan_ms = (time.perf_counter() - plan_started) * 1000.0
            samples_ms.append(replan_ms)
            attempts += 1
            trajectory = active_trajectory(result)
            success = trajectory is not None
            waypoint_count = 0
            advance_index = 0
            executed_positions = current_state.position

            if trajectory is not None:
                successes += 1
                positions = trajectory.position.reshape(
                    -1, len(preflight.ARM_JOINT_NAMES)
                )
                waypoint_count = int(positions.shape[0])
                advance_steps = max(1, int(round(args.interval / INTERPOLATION_DT_S)))
                advance_index = min(advance_steps, waypoint_count - 1)
                executed_positions = positions[: advance_index + 1]
                current_state = JointState.from_position(
                    positions[advance_index].unsqueeze(0),
                    joint_names=list(preflight.ARM_JOINT_NAMES),
                )

            minimum_clearance = minimum_cube_clearance_m(
                planner,
                executed_positions,
                collision_buffer,
            )
            clearances_m.append(minimum_clearance)

            if trajectory is not None:
                current_tool_position = tool_position(planner, current_state)
                goal_error = float(
                    np.linalg.norm(current_tool_position - target_positions[goal_name])
                )
                if goal_error <= GOAL_POSITION_TOLERANCE_M:
                    goal_name = "start" if goal_name == "cable_pregrasp" else "cable_pregrasp"
                    goal_switches += 1
                if visualizer is not None:
                    visualizer.set_joint_state(current_state.squeeze(0))
                    if args.show_spheres:
                        visualizer.update_robot_spheres(current_state)
                    tool_points = trajectory_tool_points(planner, positions)
                    visualizer._server.scene.add_line_segments(
                        name="/plans/current_tool_path",
                        points=line_segments(tool_points),
                        colors=(40, 220, 255),
                        line_width=4.0,
                    )

            record = {
                "attempt": attempts,
                "elapsed_s": elapsed,
                "cube_y_m": cube_y,
                "goal": goal_name,
                "replan_ms": replan_ms,
                "success": success,
                "minimum_clearance_m": minimum_clearance,
                "waypoints": waypoint_count,
                "advanced_waypoints": advance_index,
            }
            print("CUROBO_DODGE_CYCLE " + json.dumps(record), flush=True)
            if status_handle is not None:
                status_handle.content = (
                    f"**Live CUDA dodge replanning**  \n"
                    f"Cycle: `{attempts}` | Goal: `{goal_name}` | Switches: `{goal_switches}`  \n"
                    f"Latest: `{replan_ms:.1f} ms` | Success: `{success}`  \n"
                    f"Cube y: `{cube_y:.3f} m` | Clearance: `{minimum_clearance:.4f} m`"
                )
            next_tick = max(next_tick + args.interval, time.monotonic())
    except KeyboardInterrupt:
        interrupted = True
        print("CUROBO_DODGE_INTERRUPTED", flush=True)

    summary = summarize(
        samples_ms,
        successes,
        attempts,
        clearances_m,
        goal_switches,
    )
    summary.update(
        {
            "requested_duration_s": args.duration,
            "wall_elapsed_s": time.monotonic() - started,
            "interval_s": args.interval,
            "cube_period_s": CUBE_PERIOD_S,
            "interrupted": interrupted,
        }
    )
    print("CUROBO_DODGE_RESULT " + json.dumps(summary), flush=True)
    if status_handle is not None:
        status_handle.content = (
            "**Demo stopped.**  \n"
            f"Mean / p95 / max: `{summary['mean_replan_ms']:.1f}` / "
            f"`{summary['p95_replan_ms']:.1f}` / `{summary['max_replan_ms']:.1f} ms`  \n"
            f"Success: `{100.0 * summary['success_rate']:.1f}%` | "
            f"Minimum clearance: `{summary['overall_minimum_clearance_m']:.4f} m`"
        )
    if args.result_json is not None:
        output_path = args.result_json.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"CUROBO_DODGE_RESULT_JSON {output_path}")

    passed = (
        attempts > 0
        and successes == attempts
        and summary["finite_measurements"]
        and summary["clearance_positive"]
        and goal_switches > 0
    )
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"CUROBO_DODGE_ERROR {error}", file=sys.stderr)
        raise SystemExit(1)
