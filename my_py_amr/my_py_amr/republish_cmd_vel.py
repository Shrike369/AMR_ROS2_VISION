"""Simple republisher: forwards /cmd_vel_nav -> /cmd_vel (or configurable topics).

This helps when Nav2 publishes velocities to a non-default topic and the robot
expects commands on `/cmd_vel`.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RepublishCmdVel(Node):
    def __init__(self):
        super().__init__('republish_cmd_vel')
        self.declare_parameter('in_topic', '/cmd_vel_nav')
        self.declare_parameter('out_topic', '/cmd_vel')
        in_t = self.get_parameter('in_topic').get_parameter_value().string_value
        out_t = self.get_parameter('out_topic').get_parameter_value().string_value
        self._in_topic = in_t
        self._out_topic = out_t
        self.get_logger().info(f"Republishing: {self._in_topic} -> {self._out_topic}")

        self._pub = self.create_publisher(Twist, self._out_topic, 10)
        self._sub = self.create_subscription(Twist, self._in_topic, self._cb, 10)

    def _cb(self, msg: Twist):
        # Forward message unchanged
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
