#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


LENGTH_M = 0.40
WIDTH_M = 0.05
THICKNESS_M = 0.02


def rotate_vector(vector, quaternion):
    """Rotate a local vector by a normalized xyzw quaternion."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class ObjectDebugMarker(Node):
    def __init__(self):
        super().__init__("object_debug_marker")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/newton/object_markers", qos
        )
        self.subscription = self.create_subscription(
            PoseStamped, "/newton/object_pose", self.pose_callback, 10
        )
        self.reported = False
        self.get_logger().info(
            "Waiting for /newton/object_pose; this node only visualizes and sends no robot command"
        )

    @staticmethod
    def set_color(marker, red, green, blue, alpha=1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    @staticmethod
    def axis_marker(header, marker_id, start, direction, length, color):
        marker = Marker()
        marker.header = header
        marker.ns = "newton_strip_axes"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [
            Point(x=start[0], y=start[1], z=start[2]),
            Point(
                x=start[0] + direction[0] * length,
                y=start[1] + direction[1] * length,
                z=start[2] + direction[2] * length,
            ),
        ]
        marker.scale.x = 0.008
        marker.scale.y = 0.016
        marker.scale.z = 0.025
        ObjectDebugMarker.set_color(marker, *color)
        return marker

    def pose_callback(self, message):
        pose = message.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Rejected non-finite /newton/object_pose")
            return

        q = values[3:]
        norm = math.sqrt(sum(value * value for value in q))
        if norm < 1.0e-9:
            self.get_logger().error("Rejected invalid zero-length object quaternion")
            return
        q = tuple(value / norm for value in q)
        center = values[:3]

        box = Marker()
        box.header = message.header
        box.ns = "newton_strip"
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose = pose
        box.scale.x = LENGTH_M
        box.scale.y = WIDTH_M
        box.scale.z = THICKNESS_M
        self.set_color(box, 0.10, 0.75, 0.95, 0.55)

        local_axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        colors = ((1.0, 0.1, 0.1), (0.1, 1.0, 0.1), (0.1, 0.3, 1.0))
        lengths = (0.25, 0.12, 0.12)
        axes = [
            self.axis_marker(
                message.header,
                index + 1,
                center,
                rotate_vector(axis, q),
                length,
                color,
            )
            for index, (axis, length, color) in enumerate(
                zip(local_axes, lengths, colors)
            )
        ]

        label = Marker()
        label.header = message.header
        label.ns = "newton_strip"
        label.id = 4
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = center[0]
        label.pose.position.y = center[1]
        label.pose.position.z = center[2] + 0.10
        label.pose.orientation.w = 1.0
        label.scale.z = 0.035
        self.set_color(label, 1.0, 1.0, 1.0)
        label.text = "strip 0.40 x 0.05 x 0.02 m\nX=length, Y=width"

        self.publisher.publish(MarkerArray(markers=[box, *axes, label]))
        if not self.reported:
            self.get_logger().info(
                "OBJECT_MARKERS_READY count=5 center=(%.4f, %.4f, %.4f) "
                "dimensions=(0.40, 0.05, 0.02) m"
                % center
            )
            self.reported = True


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDebugMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
