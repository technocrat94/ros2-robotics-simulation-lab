#!/usr/bin/env python3
"""ROS 2 Humble process: translate loopback UDP state and commands."""
import json
import socket
import time
import uuid

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger

STATE_ADDRESS = ("127.0.0.1", 15100)
COMMAND_ADDRESS = ("127.0.0.1", 15101)
PROTOCOL = 1
ARM_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
GRIPPER_LEADER = "robotiq_85_left_knuckle_joint"


class NewtonRosBridge(Node):
    def __init__(self):
        super().__init__("newton_ros_bridge")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(STATE_ADDRESS)
        self.socket.setblocking(False)
        self.pose_pub = self.create_publisher(PoseStamped, "/newton/object_pose", 10)
        self.time_pub = self.create_publisher(Float64, "/newton/sim_time", 10)
        self.running_pub = self.create_publisher(Bool, "/newton/running", 10)
        self.status_pub = self.create_publisher(String, "/newton/bridge_status", 10)
        self.joint_pub = self.create_publisher(JointState, "/newton/joint_states", 10)
        self.phase_pub = self.create_publisher(String, "/newton/demo_phase", 10)
        self.mimic_error_pub = self.create_publisher(
            Float64, "/newton/mimic_max_error", 10)
        self.shadow_error_pub = self.create_publisher(
            Float64, "/newton/trajectory_shadow_error", 10)
        self.create_service(SetBool, "/newton/set_running", self.set_running)
        self.create_service(Trigger, "/newton/reset", self.reset)
        self.create_service(Trigger, "/newton/sync_robot_state", self.sync_robot_state)
        self.create_service(
            SetBool, "/newton/set_trajectory_shadow", self.set_trajectory_shadow)
        self.create_subscription(
            JointState, "/joint_states", self.receive_ros_joint_state, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/state",
            self.receive_controller_state,
            10,
        )
        self.create_timer(0.01, self.receive)
        self.create_timer(0.5, self.publish_status)
        self.last_received = None
        self.last_sequence = None
        self.latest_ros_positions = {}
        self.latest_desired_arm = None
        self.latest_newton_arm = None
        self.latest_gripper = 0.0
        self.shadow_enabled = False
        self.last_shadow_send = 0.0
        self.get_logger().info("ROS_ADAPTER_READY protocol=1 state=15100 command=15101")

    def send_command(self, command, value=None):
        command_id = str(uuid.uuid4())
        message = {"protocol": PROTOCOL, "id": command_id, "command": command}
        if value is not None:
            message["value"] = value
        self.socket.sendto(json.dumps(message).encode("utf-8"), COMMAND_ADDRESS)
        return command_id

    def set_running(self, request, response):
        command_id = self.send_command("set_running", request.data)
        response.success = True
        response.message = "command sent; verify /newton/running; id=" + command_id
        return response

    def reset(self, _request, response):
        command_id = self.send_command("reset")
        response.success = True
        response.message = "command sent; verify returned state reset; id=" + command_id
        return response

    def receive_ros_joint_state(self, message):
        self.latest_ros_positions.update(
            {str(name): float(value) for name, value in zip(message.name, message.position)}
        )
        if GRIPPER_LEADER in self.latest_ros_positions:
            self.latest_gripper = self.latest_ros_positions[GRIPPER_LEADER]
        self.maybe_send_shadow()

    def receive_controller_state(self, message):
        if len(message.joint_names) != len(message.desired.positions):
            return
        by_name = dict(zip(message.joint_names, message.desired.positions))
        if all(name in by_name for name in ARM_JOINT_NAMES):
            self.latest_desired_arm = [float(by_name[name]) for name in ARM_JOINT_NAMES]
            self.maybe_send_shadow()

    def robot_state_value(self, arm_positions):
        return {
            "joint_names": ARM_JOINT_NAMES,
            "joint_positions": [float(value) for value in arm_positions],
            "gripper_position": float(self.latest_gripper),
        }

    def sync_robot_state(self, _request, response):
        if not all(name in self.latest_ros_positions for name in ARM_JOINT_NAMES):
            response.success = False
            response.message = "no complete ROS arm state available"
            return response
        arm = [self.latest_ros_positions[name] for name in ARM_JOINT_NAMES]
        command_id = self.send_command("sync_robot_state", self.robot_state_value(arm))
        response.success = True
        response.message = "synchronization sent; verify start_state_guard; id=" + command_id
        return response

    def set_trajectory_shadow(self, request, response):
        if request.data and self.latest_desired_arm is None:
            response.success = False
            response.message = "no controller desired state available"
            return response
        self.shadow_enabled = bool(request.data)
        response.success = True
        response.message = "trajectory shadow enabled" if self.shadow_enabled else "trajectory shadow disabled"
        return response

    def maybe_send_shadow(self):
        if not self.shadow_enabled or self.latest_desired_arm is None:
            return
        now = time.monotonic()
        if now - self.last_shadow_send < 0.02:
            return
        self.send_command(
            "set_robot_positions",
            self.robot_state_value(self.latest_desired_arm),
        )
        self.last_shadow_send = now

    def receive(self):
        while True:
            try:
                payload, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                data = json.loads(payload.decode("utf-8"))
                if data.get("type") != "state" or data.get("protocol") != PROTOCOL:
                    continue
                stamp = self.get_clock().now().to_msg()
                if "position_m" in data:
                    position = data["position_m"]
                    orientation = data["orientation_xyzw"]
                    pose = PoseStamped()
                    pose.header.stamp = stamp
                    pose.header.frame_id = data["frame_id"]
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
                    (pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w) = orientation
                    self.pose_pub.publish(pose)
                if "joint_names" in data:
                    joints = JointState()
                    joints.header.stamp = stamp
                    joints.name = [str(name) for name in data["joint_names"]]
                    joints.position = [float(value) for value in data["joint_positions"]]
                    self.joint_pub.publish(joints)
                    self.phase_pub.publish(String(data=str(data["demo_phase"])))
                    self.mimic_error_pub.publish(
                        Float64(data=float(data["mimic_max_error_rad"])))
                    by_name = dict(zip(joints.name, joints.position))
                    if all(name in by_name for name in ARM_JOINT_NAMES):
                        self.latest_newton_arm = [by_name[name] for name in ARM_JOINT_NAMES]
                    if self.latest_desired_arm is not None and self.latest_newton_arm is not None:
                        maximum_error = max(
                            abs(desired - actual)
                            for desired, actual in zip(
                                self.latest_desired_arm, self.latest_newton_arm)
                        )
                        self.shadow_error_pub.publish(Float64(data=maximum_error))
                self.time_pub.publish(Float64(data=float(data["sim_time_s"])))
                self.running_pub.publish(Bool(data=bool(data["running"])))
                self.last_received = time.monotonic()
                self.last_sequence = int(data["sequence"])
            except Exception as error:
                self.get_logger().warning("Rejected state packet: %s" % error)

    def publish_status(self):
        status = String()
        if self.last_received is None:
            status.data = "WAITING: no Newton state received"
        else:
            age = time.monotonic() - self.last_received
            state = "OK" if age < 0.25 else "STALE"
            status.data = "%s: age=%.3fs sequence=%s" % (
                state, age, self.last_sequence)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = NewtonRosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
