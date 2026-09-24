#!/usr/bin/env python3
"""Measure Robotiq fingertip-link motion versus gripper command.

This is a kinematic calibration.  It measures link-frame separation and
midpoint motion; it does not claim that link-frame separation is the physical
gap between the inner pad surfaces.
"""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


LEFT_TIP = "robotiq_85_left_finger_tip_link"
RIGHT_TIP = "robotiq_85_right_finger_tip_link"


def xyz(transform):
    value = transform.transform.translation
    return (value.x, value.y, value.z)


def midpoint(a, b):
    return tuple((x + y) / 2.0 for x, y in zip(a, b))


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class Calibrator(Node):
    def __init__(self):
        super().__init__("gripper_kinematics_calibration")
        self.client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def transform_xyz(self, reference, child):
        deadline = time.monotonic() + 5.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = self.buffer.lookup_transform(
                    reference, child, rclpy.time.Time(), timeout=Duration(seconds=0.2)
                )
                return xyz(transform)
            except TransformException as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"TF unavailable: {reference} -> {child}: {last_error}")

    def command(self, position, effort):
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Gripper action server is unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Gripper command {position:.3f} was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        result = result_future.result()
        if result is None:
            raise RuntimeError(f"Gripper command {position:.3f} timed out")
        return result.result.position

    def spin_for(self, seconds):
        """Keep receiving joint-state/TF callbacks while mechanisms settle."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--positions", nargs="+", type=float, default=[0.0, 0.1, 0.2, 0.3, 0.35]
    )
    parser.add_argument("--max-effort", type=float, default=20.0)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--min-tool-z", type=float, default=0.20)
    parser.add_argument("--output", type=Path, default=Path("/tmp/gripper_calibration.csv"))
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.positions or args.positions[0] != 0.0:
        raise SystemExit("The first calibration position must be 0.0 (open baseline)")

    rclpy.init()
    node = Calibrator()
    rows = []
    try:
        # Spin briefly so the TF listener can receive the current tree.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        tool_world = node.transform_xyz("world", "tool0")
        if tool_world[2] < args.min_tool_z:
            raise RuntimeError(
                f"Safety stop: tool0 z={tool_world[2]:.3f} m is below "
                f"the {args.min_tool_z:.3f} m calibration limit. Move to free space first."
            )

        baseline_midpoint_world = None
        baseline_midpoint_tool = None
        for requested in args.positions:
            reached = node.command(requested, args.max_effort)
            node.spin_for(args.settle_seconds)

            left_world = node.transform_xyz("world", LEFT_TIP)
            right_world = node.transform_xyz("world", RIGHT_TIP)
            left_tool = node.transform_xyz("tool0", LEFT_TIP)
            right_tool = node.transform_xyz("tool0", RIGHT_TIP)
            midpoint_world = midpoint(left_world, right_world)
            midpoint_tool = midpoint(left_tool, right_tool)
            if baseline_midpoint_world is None:
                baseline_midpoint_world = midpoint_world
                baseline_midpoint_tool = midpoint_tool

            row = {
                "command_rad": requested,
                "reached_rad": reached,
                "link_frame_separation_m": distance(left_world, right_world),
                "world_midpoint_x_m": midpoint_world[0],
                "world_midpoint_y_m": midpoint_world[1],
                "world_midpoint_z_m": midpoint_world[2],
                "world_midpoint_z_shift_m": midpoint_world[2] - baseline_midpoint_world[2],
                "tool_midpoint_dx_m": midpoint_tool[0] - baseline_midpoint_tool[0],
                "tool_midpoint_dy_m": midpoint_tool[1] - baseline_midpoint_tool[1],
                "tool_midpoint_dz_m": midpoint_tool[2] - baseline_midpoint_tool[2],
            }
            rows.append(row)
            print("CALIBRATION", json.dumps(row, separators=(",", ":")))

        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"CALIBRATION_SAVED {args.output}")
    except Exception as error:
        print(f"CALIBRATION_FAILED {error}", file=sys.stderr)
        return 1
    finally:
        # Leave the gripper in a known open state whenever the action server remains available.
        try:
            node.command(0.0, args.max_effort)
        except Exception as error:
            print(f"WARNING could not restore open gripper: {error}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
