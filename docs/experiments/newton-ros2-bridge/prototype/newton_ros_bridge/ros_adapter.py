#!/usr/bin/env python3
"""ROS 2 process: translate loopback UDP state and commands."""
import json
import os
import socket
import time
import uuid

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger

STATE_PORT = int(os.environ.get("NEWTON_STATE_PORT", "15100"))
COMMAND_PORT = int(os.environ.get("NEWTON_COMMAND_PORT", "15101"))
STATE_ADDRESS = ("127.0.0.1", STATE_PORT)
COMMAND_ADDRESS = ("127.0.0.1", COMMAND_PORT)
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
        self.obstacle_pose_pub = self.create_publisher(
            PoseArray, "/newton/obstacle_poses", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
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
            "/joint_trajectory_controller/controller_state",
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
        self.endpoint_pending_commands = 0
        self.shadow_command_in_flight = False
        self.last_sent_shadow = None
        self.get_logger().info(
            "ROS_ADAPTER_READY protocol=1 state=%d command=%d"
            % (STATE_PORT, COMMAND_PORT)
        )

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
        reference = getattr(message, "reference", None)
        if reference is None:
            reference = getattr(message, "desired", None)
        if reference is None or len(message.joint_names) != len(reference.positions):
            return
        by_name = dict(zip(message.joint_names, reference.positions))
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
        if not self.shadow_enabled:
            self.shadow_command_in_flight = False
            self.last_sent_shadow = None
        else:
            self.maybe_send_shadow()
        response.success = True
        response.message = "trajectory shadow enabled" if self.shadow_enabled else "trajectory shadow disabled"
        return response

    def maybe_send_shadow(self):
        if not self.shadow_enabled or self.latest_desired_arm is None:
            return
        target = tuple(self.latest_desired_arm) + (float(self.latest_gripper),)
        if target == self.last_sent_shadow:
            return
        # Pace ROS commands from Newton's simulated-time consumption, not wall
        # time.  On a slow CPU, a 20 ms wall-time sender can otherwise build
        # seconds of stale trajectory backlog.
        if self.endpoint_pending_commands > 0 or self.shadow_command_in_flight:
            return
        self.send_command(
            "set_robot_positions",
            self.robot_state_value(self.latest_desired_arm),
        )
        self.shadow_command_in_flight = True
        self.last_sent_shadow = target

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
                self.endpoint_pending_commands = int(
                    data.get("pending_robot_commands", 0))
                # This state packet was emitted after the endpoint processed
                # all UDP commands received before its current physics step.
                self.shadow_command_in_flight = False
                sim_time = float(data["sim_time_s"])
                seconds = int(sim_time)
                nanoseconds = int(round((sim_time - seconds) * 1.0e9))
                if nanoseconds >= 1_000_000_000:
                    seconds += 1
                    nanoseconds -= 1_000_000_000
                clock = Clock()
                clock.clock.sec = seconds
                clock.clock.nanosec = nanoseconds
                stamp = clock.clock
                self.clock_pub.publish(clock)
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
                if "obstacle_poses" in data:
                    obstacle_poses = PoseArray()
                    obstacle_poses.header.stamp = stamp
                    obstacle_poses.header.frame_id = data["frame_id"]
                    for obstacle in data["obstacle_poses"]:
                        position = obstacle["position_m"]
                        orientation = obstacle["orientation_xyzw"]
                        pose = Pose()
                        pose.position.x, pose.position.y, pose.position.z = position
                        (pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w) = orientation
                        obstacle_poses.poses.append(pose)
                    self.obstacle_pose_pub.publish(obstacle_poses)
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
                self.time_pub.publish(Float64(data=sim_time))
                self.running_pub.publish(Bool(data=bool(data["running"])))
                self.last_received = time.monotonic()
                self.last_sequence = int(data["sequence"])
                self.maybe_send_shadow()
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
