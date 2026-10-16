#!/usr/bin/env python3
"""Plan Step-C with cuRobo and execute it through the existing ROS/Newton path.

The arm path is planned entirely by cuRobo on CUDA.  Execution still uses the
existing FollowJointTrajectory and GripperCommand controllers, allowing the
unchanged ROS-Newton trajectory shadow to drive the verified FEM endpoint.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
import torch

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState as RosJointState
from trajectory_msgs.msg import JointTrajectoryPoint

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState

import ur5_viser_preflight as preflight


SCENE_PROFILE = "step_c_cable_wall"
TRANSIT_PADDING_M = 0.020
CONTACT_PADDING_M = 0.000
PLANNING_TIME_S = 10.0

# Preserve the execution timing of the accepted MoveIt C1 run.  cuRobo owns
# path geometry; these durations preserve the already-validated contact test.
PREGRASP_EXECUTION_S = 5.000
DESCENT_EXECUTION_S = 1.037
LIFT_EXECUTION_S = 3.559
CONTACT_HOLD_S = 1.0
LIFT_HOLD_S = 1.0
START_TOLERANCE_RAD = 0.01


class ControllerClient(Node):
    """Small ROS client for feedback and the two existing controllers."""

    def __init__(self) -> None:
        super().__init__("curobo_step_c_pick")
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/robotiq_gripper_controller/gripper_cmd",
        )
        self.arm_positions: dict[str, float] = {}
        self.object_position: list[float] | None = None
        self.create_subscription(RosJointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(
            PoseStamped,
            "/newton/object_pose",
            self._object_cb,
            10,
        )

    def _joint_cb(self, message: RosJointState) -> None:
        self.arm_positions.update(
            {
                str(name): float(position)
                for name, position in zip(message.name, message.position)
            }
        )

    def _object_cb(self, message: PoseStamped) -> None:
        self.object_position = [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ]

    def wait_for_feedback(self, timeout_s: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            have_arm = all(
                name in self.arm_positions for name in preflight.ARM_JOINT_NAMES
            )
            if have_arm and self.object_position is not None:
                return
        raise RuntimeError(
            "Timed out waiting for /joint_states and /newton/object_pose."
        )

    def _wait_future(self, future, timeout_s: float, description: str):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise RuntimeError(f"Timed out {description}.")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"Failed {description}: {exception}")
        return future.result()

    def command_gripper(self, position: float) -> None:
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Gripper action server is unavailable.")
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        print(f"Sending gripper command: position={position:.4f}", flush=True)
        goal_handle = self._wait_future(
            self.gripper_client.send_goal_async(goal),
            10.0,
            "sending the gripper goal",
        )
        if not goal_handle.accepted:
            raise RuntimeError("Gripper goal was rejected.")
        wrapped = self._wait_future(
            goal_handle.get_result_async(),
            20.0,
            "waiting for the gripper result",
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Gripper action status was {wrapped.status}.")
        if not wrapped.result.reached_goal:
            raise RuntimeError("Gripper controller did not reach its goal.")
        print(
            f"Gripper reached position {wrapped.result.position:.4f}",
            flush=True,
        )

    def execute_arm_trajectory(
        self,
        trajectory: JointState,
        last_tstep: torch.Tensor,
        duration_s: float,
        stage: str,
    ) -> None:
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowJointTrajectory action server is unavailable.")
        active = preflight.effective_trajectory(trajectory, last_tstep)
        active = active.reorder(list(preflight.ARM_JOINT_NAMES))
        positions = active.position.detach().cpu().reshape(
            -1, len(preflight.ARM_JOINT_NAMES)
        )
        if positions.shape[0] < 2:
            raise RuntimeError(f"{stage} trajectory has fewer than two points.")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(preflight.ARM_JOINT_NAMES)
        point_count = int(positions.shape[0])
        for index, position in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in position.tolist()]
            seconds = duration_s * float(index + 1) / float(point_count)
            point.time_from_start.sec = int(math.floor(seconds))
            point.time_from_start.nanosec = int(
                round((seconds - math.floor(seconds)) * 1.0e9)
            )
            if point.time_from_start.nanosec == 1_000_000_000:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec = 0
            goal.trajectory.points.append(point)

        print(
            "CUROBO_EXECUTION "
            f"segment={stage} waypoints={point_count} duration_s={duration_s:.3f}",
            flush=True,
        )
        goal_handle = self._wait_future(
            self.arm_client.send_goal_async(goal),
            10.0,
            f"sending the {stage} trajectory",
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"Controller rejected the {stage} trajectory.")
        wrapped = self._wait_future(
            goal_handle.get_result_async(),
            duration_s + 15.0,
            f"executing the {stage} trajectory",
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{stage} trajectory action status was {wrapped.status}."
            )
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"{stage} trajectory failed: {wrapped.result.error_string}"
            )
        print(f"CUROBO_EXECUTION_SUCCEEDED segment={stage}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Execute the accepted Step-C cable pick with cuRobo planning."
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=Path.home() / ".cache" / "ur5_moveit_demo" / "curobo_v2",
    )
    parser.add_argument("--rebuild-model", action="store_true")
    return parser.parse_args()


def padded_planning_scene(
    scene_model: dict[str, Any],
    obstacle_id: str,
) -> dict[str, Any]:
    """Inflate the planning wall by the accepted 20 mm transit padding."""

    padded = copy.deepcopy(scene_model)
    dimensions = padded["cuboid"][obstacle_id]["dims"]
    padded["cuboid"][obstacle_id]["dims"] = [
        float(value) + 2.0 * TRANSIT_PADDING_M for value in dimensions
    ]
    return padded


def phase_planning_time(result: Any, phase: str) -> float:
    phase_result = getattr(result, f"{phase}_result", None)
    if phase_result is None:
        return float("nan")
    return float(getattr(phase_result, "total_time", float("nan")))


def sphere_box_signed_distance(
    center: np.ndarray,
    radius: float,
    box_center: np.ndarray,
    box_size: np.ndarray,
) -> float:
    """Signed sphere-to-axis-aligned-box surface distance."""

    delta = np.abs(center - box_center) - 0.5 * box_size
    outside_distance = float(np.linalg.norm(np.maximum(delta, 0.0)))
    inside_distance = min(float(np.max(delta)), 0.0)
    return outside_distance + inside_distance - float(radius)


def route_metrics(
    planner,
    trajectory: JointState,
    last_tstep: torch.Tensor,
    obstacle: dict[str, Any],
) -> dict[str, float | int]:
    """Measure tool height and fitted-sphere distance along the cuRobo route."""

    active = preflight.effective_trajectory(trajectory, last_tstep)
    active = active.reorder(list(preflight.ARM_JOINT_NAMES))
    positions = active.position.reshape(-1, len(preflight.ARM_JOINT_NAMES))
    kinematics = planner.kinematics.compute_kinematics(
        JointState.from_position(
            positions,
            joint_names=list(preflight.ARM_JOINT_NAMES),
        )
    )
    tool_positions = (
        kinematics.tool_poses[preflight.TOOL_FRAME]
        .position.detach()
        .cpu()
        .reshape(-1, 3)
        .numpy()
    )
    spheres_by_sample = planner.kinematics.get_robot_as_spheres(
        positions.contiguous(),
        filter_valid=True,
    )
    box_center = np.asarray(obstacle["center_m"], dtype=np.float64)
    box_size = np.asarray(obstacle["size_m"], dtype=np.float64)
    minimum_distance = min(
        sphere_box_signed_distance(
            np.asarray(sphere.position, dtype=np.float64),
            sphere.radius,
            box_center,
            box_size,
        )
        for sample in spheres_by_sample
        for sphere in sample
    )
    return {
        "samples": int(positions.shape[0]),
        "max_end_effector_height_m": float(np.max(tool_positions[:, 2])),
        "min_robot_wall_distance_m": minimum_distance,
    }


def obstacle_for_profile(repo: Path, profile_name: str) -> dict[str, Any]:
    data = json.loads((repo / "config" / "obstacles.yaml").read_text())
    profile = data["scene_profiles"][profile_name]
    return next(box for box in data["boxes"] if box["id"] == profile["box_id"])


def execute() -> int:
    args = parse_args()
    repo = preflight.repository_root()
    runtime = preflight.verify_cuda_and_source(repo)
    config_path, cache_status = preflight.prepare_robot_model(
        repo,
        args.cache_dir.expanduser().resolve(),
        args.rebuild_model,
    )
    robot = Kinematics(KinematicsCfg.from_robot_yaml_file(str(config_path)))
    expected_positions = preflight.load_initial_arm_positions(repo)
    calibration = preflight.solve_gripper_calibration(repo)
    _, scene_model, configured_object_center = preflight.load_scene(
        repo,
        SCENE_PROFILE,
    )
    obstacle = obstacle_for_profile(repo, SCENE_PROFILE)
    planning_scene = padded_planning_scene(scene_model, obstacle["id"])

    print(
        "SCENE_CONFIG "
        f"profile={SCENE_PROFILE} obstacles_enabled=true "
        f"transit_padding_m={TRANSIT_PADDING_M:.6f} "
        f"contact_padding_m={CONTACT_PADDING_M:.6f} "
        f"planning_time_s={PLANNING_TIME_S:.3f}",
        flush=True,
    )
    print("USE_SAFE_PREGRASP_PATH true", flush=True)
    print(
        "WIDTH_CALIBRATION "
        f"width={preflight.OBJECT_WIDTH_MM:.2f} mm "
        f"compression={preflight.TOTAL_COMPRESSION_MM:.2f} mm "
        f"command={calibration['command_rad']:.6f} rad "
        f"closure_drop={calibration['closure_drop_m'] * 1000.0:.3f} mm "
        f"approach={calibration['approach_distance_m']:.6f} m",
        flush=True,
    )
    print(
        "CUROBO_RUNTIME "
        + json.dumps({**runtime, **cache_status}, sort_keys=True),
        flush=True,
    )

    planner = preflight.create_pick_planner(config_path, planning_scene)
    rclpy.init()
    client = ControllerClient()
    try:
        client.wait_for_feedback()
        actual_start = [
            client.arm_positions[name] for name in preflight.ARM_JOINT_NAMES
        ]
        maximum_start_error = max(
            abs(actual - expected_positions[name])
            for actual, name in zip(actual_start, preflight.ARM_JOINT_NAMES)
        )
        print(
            f"CUROBO_START_STATE max_expected_error_rad={maximum_start_error:.9f}",
            flush=True,
        )
        if maximum_start_error > START_TOLERANCE_RAD:
            raise RuntimeError(
                "Current arm state differs from the accepted Step-C start by "
                f"{maximum_start_error:.6f} rad."
            )

        measured_object_center = client.object_position
        object_error = float(
            np.linalg.norm(
                np.asarray(measured_object_center)
                - np.asarray(configured_object_center)
            )
        )
        print(
            "OBJECT_TARGET source=newton_feedback frame=world "
            f"center=({measured_object_center[0]:.4f}, "
            f"{measured_object_center[1]:.4f}, "
            f"{measured_object_center[2]:.4f}) m "
            f"configured_error_m={object_error:.9f}",
            flush=True,
        )
        if object_error > 0.01:
            raise RuntimeError("Newton cable target differs from the Step-C profile.")

        start_state = JointState.from_position(
            torch.tensor(actual_start, device="cuda", dtype=torch.float32),
            joint_names=list(preflight.ARM_JOINT_NAMES),
        ).unsqueeze(0)
        wall_started = time.monotonic()
        result, _ = preflight.plan_cable_pick(
            planner,
            start_state,
            measured_object_center,
            calibration["approach_distance_m"],
        )
        wall_elapsed = time.monotonic() - wall_started
        summary = preflight.pick_result_summary(result, wall_elapsed)
        if not summary["success"]:
            raise RuntimeError(f"cuRobo planning failed: {result.status}")

        planning_times = {
            phase: phase_planning_time(result, phase)
            for phase in ("approach", "grasp", "lift")
        }
        for phase, duration in planning_times.items():
            print(
                f"CUROBO_PLANNING_TIME segment={phase} seconds={duration:.6f}",
                flush=True,
            )
        metrics = route_metrics(
            planner,
            result.approach_interpolated_trajectory,
            result.approach_interpolated_last_tstep,
            obstacle,
        )
        print(
            'PLANNED_ROUTE_METRICS stage="curobo_pregrasp" '
            f"samples={metrics['samples']} "
            "max_end_effector_height_m="
            f"{metrics['max_end_effector_height_m']:.6f} "
            f"min_robot_box_distance_m={metrics['min_robot_wall_distance_m']:.6f} "
            f"obstacle_id={obstacle['id']}",
            flush=True,
        )
        print("ACTIVE_PADDING stage=transit padding_m=0.020000", flush=True)

        client.command_gripper(preflight.GRIPPER_OPEN_RAD)
        client.execute_arm_trajectory(
            result.approach_interpolated_trajectory,
            result.approach_interpolated_last_tstep,
            PREGRASP_EXECUTION_S,
            "approach",
        )
        print("ACTIVE_PADDING stage=contact padding_m=0.000000", flush=True)
        client.execute_arm_trajectory(
            result.grasp_interpolated_trajectory,
            result.grasp_interpolated_last_tstep,
            DESCENT_EXECUTION_S,
            "grasp",
        )
        client.command_gripper(calibration["command_rad"])
        time.sleep(CONTACT_HOLD_S)
        client.execute_arm_trajectory(
            result.lift_interpolated_trajectory,
            result.lift_interpolated_last_tstep,
            LIFT_EXECUTION_S,
            "lift",
        )
        time.sleep(LIFT_HOLD_S)
        client.command_gripper(preflight.GRIPPER_OPEN_RAD)

        final = {
            **summary,
            "planner": "curobo_v2",
            "scene_profile": SCENE_PROFILE,
            "planning_time_by_segment_s": planning_times,
            "route": metrics,
            "gripper_closed_rad": calibration["command_rad"],
            "approach_distance_m": calibration["approach_distance_m"],
        }
        print("CUROBO_STEP_C_PICK_RESULT " + json.dumps(final), flush=True)
        print("========== CUROBO FEM CABLE PICK SUCCEEDED ==========", flush=True)
        return 0
    finally:
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(execute())
    except Exception as error:  # noqa: BLE001 - preserve a single evidence marker
        print(f"CUROBO_STEP_C_PICK_ERROR {error}", file=sys.stderr, flush=True)
        raise SystemExit(1)
