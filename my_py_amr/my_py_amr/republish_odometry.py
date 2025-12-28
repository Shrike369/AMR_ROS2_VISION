"""Simple republisher: forwards Gazebo model odometry -> /odom.

This is useful when the Gazebo odom topic is namespaced under
`/model/<name>/odometry` (the simulator side) and other ROS packages expect
`/odom`.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class RepublishOdom(Node):
    def __init__(self):
        super().__init__('republish_odometry')
        self.declare_parameter('in_topic', '/model/amr_with_markers/odometry')
        self.declare_parameter('out_topic', '/odom')
        in_t = self.get_parameter('in_topic').get_parameter_value().string_value
        out_t = self.get_parameter('out_topic').get_parameter_value().string_value
        self._in_topic = in_t
        self._out_topic = out_t
        self.get_logger().info(f"Republishing: {self._in_topic} -> {self._out_topic}")

        self._pub = self.create_publisher(Odometry, self._out_topic, 10)
        self._sub = self.create_subscription(Odometry, self._in_topic, self._cb, 10)

    def _cb(self, msg: Odometry):
        # Forward message unchanged
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
