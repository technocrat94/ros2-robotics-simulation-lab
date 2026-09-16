#!/usr/bin/env python3
"""Compare the live ROS arm state with Newton's current demo start state."""

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .trajectory_contract import ARM_JOINT_NAMES, validate_start_state


NEWTON_INITIAL_POSITIONS = (
    0.0,
    -math.pi / 2.0,
    math.pi / 2.0,
    -math.pi / 2.0,
    -math.pi / 2.0,
    0.0,
)


class StartStateGuard(Node):
    def __init__(self):
        super().__init__("newton_start_state_guard")
        self.declare_parameter("threshold_rad", 0.2)
        self.result = None
        self.create_subscription(JointState, "/joint_states", self.receive, 10)
        self.get_logger().info("Waiting for /joint_states")

    def receive(self, message):
        if self.result is not None:
            return
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in ARM_JOINT_NAMES):
            return
        self.result = validate_start_state(
            positions,
            NEWTON_INITIAL_POSITIONS,
            float(self.get_parameter("threshold_rad").value),
        )


def main(args=None):
    rclpy.init(args=args)
    node = StartStateGuard()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.5)
        if node.result is not None:
            compact = {
                "pass": node.result["pass"],
                "threshold_rad": node.result["threshold_rad"],
                "maximum_error_rad": node.result["maximum_error_rad"],
                "mismatches": node.result["mismatches"],
            }
            print("START_STATE_GUARD " + json.dumps(compact, separators=(",", ":")))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

