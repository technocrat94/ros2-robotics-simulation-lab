#!/usr/bin/env python3
"""Live cuRobo-only UR5 dodging demo with a route-crossing cylinder.

The arm shuttles between its recorded start pose and the cable pre-grasp pose.
Every control cycle updates the cylinder, replans from the simulated current joint
state, and executes one full cycle of trajectory time.  ROS and Newton are not
started and no command is sent to robot hardware.
"""

from __future__ import annotations

import argparse
import hashlib
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


OBSTACLE_NAME = "route_crossing_cylinder"
CYLINDER_RADIUS_M = 0.05
CYLINDER_HEIGHT_M = 0.25
# SceneCollision's generic backend has no analytic cylinder kernel.  Use its
# fast native cuboid primitive as a conservative bounding volume so a 200 ms
# control cycle remains achievable; Viser still renders the requested cylinder.
COLLISION_REPRESENTATION = "conservative-0.10x0.10x0.25-m-cuboid"
# The handoff's 0.30-0.45 m z range intersects the fitted UR5 + Robotiq
# collision spheres at both shuttle endpoints.  This calibrated range keeps
# the vertical 0.25 m cylinder close enough to influence the route while
# allowing all seeded evidence runs to maintain positive clearance.
RANDOM_BOUNDS_M = np.asarray(
    [[0.44, 0.53], [0.15, 0.38], [0.60, 0.68]], dtype=np.float64
)
RANDOM_MAX_SPEED_M_S = 0.15
INTERPOLATION_DT_S = 0.025
DISTANCE_ACTIVATION_M = 0.09
GOAL_POSITION_TOLERANCE_M = 0.006


class SmoothRandomMotion:
    """Seeded random waypoint motion with a smoothstep speed profile."""

    def __init__(self, seed: int):
        self.rng = np.random.default_rng(seed)
        self.segment_start_s = 0.0
        self.start = RANDOM_BOUNDS_M.mean(axis=1)
        self.end = self._new_waypoint()
        self.duration_s = self._segment_duration(self.start, self.end)

    def _new_waypoint(self) -> np.ndarray:
        return self.rng.uniform(RANDOM_BOUNDS_M[:, 0], RANDOM_BOUNDS_M[:, 1])

    @staticmethod
    def _segment_duration(start: np.ndarray, end: np.ndarray) -> float:
        # Smoothstep's maximum derivative is 1.5, so this duration strictly
        # bounds Cartesian speed by RANDOM_MAX_SPEED_M_S.
        distance = float(np.linalg.norm(end - start))
        return max(0.2, 1.5 * distance / RANDOM_MAX_SPEED_M_S)

    def position(self, elapsed_s: float) -> np.ndarray:
        while elapsed_s >= self.segment_start_s + self.duration_s:
            self.segment_start_s += self.duration_s
            self.start = self.end
            self.end = self._new_waypoint()
            self.duration_s = self._segment_duration(self.start, self.end)
        u = (elapsed_s - self.segment_start_s) / self.duration_s
        u = min(max(u, 0.0), 1.0)
        smooth_u = u * u * (3.0 - 2.0 * u)
        return self.start + smooth_u * (self.end - self.start)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live CUDA cuRobo replanning around a moving cylinder."
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run; zero keeps the live viewer running until Ctrl-C.",
    )
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--mode", choices=("random", "interactive"), default="random")
    parser.add_argument("--seed", type=int, default=7)
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
    if args.headless and args.mode == "interactive":
        parser.error("--mode interactive requires the Viser viewer")
    return args


def make_scene(position: np.ndarray) -> tuple[Scene, list[float]]:
    """Create the conservative cylinder bounding volume for cuRobo."""

    obstacle_pose = [*position.tolist(), 1.0, 0.0, 0.0, 0.0]
    return (
        Scene(
            cuboid=[
                Cuboid(
                    name=OBSTACLE_NAME,
                    dims=[
                        2.0 * CYLINDER_RADIUS_M,
                        2.0 * CYLINDER_RADIUS_M,
                        CYLINDER_HEIGHT_M,
                    ],
                    pose=obstacle_pose,
                )
            ]
        ),
        obstacle_pose,
    )


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


def minimum_obstacle_clearance_m(
    planner: Any,
    positions: torch.Tensor,
    collision_buffer: CollisionBuffer,
) -> float:
    """Measure signed robot-to-cylinder clearance through cuRobo's checker.

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


def path_sha256(samples: list[list[float]]) -> str:
    """Hash rounded path samples so independent seeded runs are comparable."""

    normalized = np.round(np.asarray(samples, dtype=np.float64), decimals=6)
    return hashlib.sha256(normalized.tobytes()).hexdigest()


def summarize(
    samples_ms: list[float],
    successes: int,
    attempts: int,
    clearances_m: list[float],
    goal_switches: int,
    obstacle_path: list[list[float]],
    arm_path: list[list[float]],
    sample_times_s: list[float],
) -> dict[str, Any]:
    """Build the machine-readable final result."""

    ordered = sorted(samples_ms)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    minimum_clearance = min(clearances_m) if clearances_m else None
    obstacle_array = np.asarray(obstacle_path, dtype=np.float64)
    time_array = np.asarray(sample_times_s, dtype=np.float64)
    observed_speeds = np.linalg.norm(np.diff(obstacle_array, axis=0), axis=1) / np.diff(
        time_array
    )
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
        "obstacle_path_sha256": path_sha256(obstacle_path),
        "arm_path_sha256": path_sha256(arm_path),
        "obstacle_path_min_m": np.min(obstacle_array, axis=0).tolist(),
        "obstacle_path_max_m": np.max(obstacle_array, axis=0).tolist(),
        "maximum_observed_obstacle_speed_m_s": float(np.max(observed_speeds)),
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

    random_motion = SmoothRandomMotion(args.seed)
    initial_obstacle_position = random_motion.position(0.0)
    scene_model, obstacle_pose = make_scene(initial_obstacle_position)
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
    obstacle_handle = None
    obstacle_controls = None
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
        obstacle_handle = visualizer._server.scene.add_cylinder(
            name=f"/obstacles/{OBSTACLE_NAME}",
            radius=CYLINDER_RADIUS_M,
            height=CYLINDER_HEIGHT_M,
            position=initial_obstacle_position,
            color=(220, 65, 55),
            opacity=0.88,
        )
        if args.mode == "interactive":
            obstacle_controls = visualizer._server.scene.add_transform_controls(
                name="/controls/cylinder",
                position=initial_obstacle_position,
                scale=0.12,
                disable_rotations=True,
                translation_limits=tuple(tuple(row) for row in RANDOM_BOUNDS_M),
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
                "mode": args.mode,
                "seed": args.seed,
                "obstacle": "vertical_cylinder",
                "cylinder_radius_m": CYLINDER_RADIUS_M,
                "cylinder_height_m": CYLINDER_HEIGHT_M,
                "collision_representation": COLLISION_REPRESENTATION,
                "random_bounds_m": RANDOM_BOUNDS_M.tolist(),
                "random_max_speed_m_s": RANDOM_MAX_SPEED_M_S,
                "start_tool_position_m": start_position.tolist(),
                "pregrasp_tool_position_m": pregrasp_position.tolist(),
                "ros": False,
                "newton": False,
            }
        ),
        flush=True,
    )

    # Prime lazy planner kernels without the dynamic obstacle so one-time CUDA
    # initialization is not mislabeled as a periodic replan.
    planner.scene_collision_checker.enable_obstacle(OBSTACLE_NAME, False)
    torch.cuda.synchronize()
    prime_started = time.perf_counter()
    prime_result = planner.plan_pose(goals[goal_name], current_state)
    torch.cuda.synchronize()
    prime_ms = (time.perf_counter() - prime_started) * 1000.0
    planner.scene_collision_checker.enable_obstacle(OBSTACLE_NAME, True)
    prime_trajectory = active_trajectory(prime_result)
    if prime_trajectory is None:
        raise RuntimeError("Initial cuRobo dodge planner prime failed")
    collision_buffer = make_collision_buffer(planner, current_state)
    prime_clearance = minimum_obstacle_clearance_m(
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
    obstacle_path: list[list[float]] = []
    arm_path: list[list[float]] = []
    sample_times_s: list[float] = []
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

            if args.mode == "random":
                obstacle_position = random_motion.position(elapsed)
            else:
                if obstacle_controls is None:
                    raise RuntimeError("interactive controls were not created")
                obstacle_position = np.asarray(obstacle_controls.position, dtype=np.float64)
            obstacle_pose[:3] = obstacle_position.tolist()
            planner.scene_collision_checker.update_obstacle_pose(
                OBSTACLE_NAME,
                Pose.from_list(obstacle_pose),
            )
            if obstacle_handle is not None:
                obstacle_handle.position = obstacle_position

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

            minimum_clearance = minimum_obstacle_clearance_m(
                planner,
                executed_positions,
                collision_buffer,
            )
            clearances_m.append(minimum_clearance)
            current_tool_position = tool_position(planner, current_state)
            obstacle_path.append(obstacle_position.tolist())
            arm_path.append(current_tool_position.tolist())
            sample_times_s.append(elapsed)

            if trajectory is not None:
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
                "obstacle_position_m": obstacle_position.tolist(),
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
                    f"Obstacle xyz: `{np.round(obstacle_position, 3).tolist()}`  \n"
                    f"Clearance: `{minimum_clearance:.4f} m`"
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
        obstacle_path,
        arm_path,
        sample_times_s,
    )
    summary.update(
        {
            "requested_duration_s": args.duration,
            "elapsed_s": time.monotonic() - started,
            "interval_s": args.interval,
            "mode": args.mode,
            "seed": args.seed,
            "collision_representation": COLLISION_REPRESENTATION,
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
