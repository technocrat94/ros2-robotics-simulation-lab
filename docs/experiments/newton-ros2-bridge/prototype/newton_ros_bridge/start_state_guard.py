#!/usr/bin/env python3
"""Compare the live ROS arm state with Newton's current demo start state."""

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .trajectory_contract import ARM_JOINT_NAMES, validate_start_state


class StartStateGuard(Node):
    def __init__(self):
        super().__init__("newton_start_state_guard")
        self.declare_parameter("threshold_rad", 0.2)
        self.result = None
        self.ros_positions = None
        self.newton_positions = None
        self.create_subscription(JointState, "/joint_states", self.receive_ros, 10)
        self.create_subscription(
            JointState, "/newton/joint_states", self.receive_newton, 10)
        self.get_logger().info("Waiting for /joint_states and /newton/joint_states")

    @staticmethod
    def arm_positions(message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in ARM_JOINT_NAMES):
            return None
        return {name: float(positions[name]) for name in ARM_JOINT_NAMES}

    def receive_ros(self, message):
        self.ros_positions = self.arm_positions(message)
        self.compare()

    def receive_newton(self, message):
        by_name = self.arm_positions(message)
        if by_name is not None:
            self.newton_positions = [by_name[name] for name in ARM_JOINT_NAMES]
        self.compare()

    def compare(self):
        if self.result is not None or self.ros_positions is None or self.newton_positions is None:
            return
        self.result = validate_start_state(
            self.ros_positions,
            self.newton_positions,
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
