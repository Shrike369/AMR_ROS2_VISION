#!/usr/bin/env python3
"""Republish joint states by integrating wheel velocities from odometry."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.time import Time


class RepublishJointStates(Node):
    def __init__(self):
        super().__init__('republish_joint_states')

        self.declare_parameter('in_topic', '/odom')
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('wheel_radius', 0.09)

        self.in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        self.left_joint = self.get_parameter('left_joint').get_parameter_value().string_value
        self.right_joint = self.get_parameter('right_joint').get_parameter_value().string_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Integrated wheel angles (radians)
        self.theta_l = 0.0
        self.theta_r = 0.0

        self.last_time = None

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Odometry, self.in_topic, self.odom_cb, 10)

        self.get_logger().info(f"Republishing joint states from '{self.in_topic}' -> /joint_states")

    def odom_cb(self, msg: Odometry) -> None:
        # Use odometry linear x and angular z (assumes planar differential drive)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            dt = 0.0
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

        # Compute wheel angular velocities (rad/s)
        # v = linear velocity of robot center; omega = angular velocity around z
        # wheel angular velocity w_r = (2*v + omega*L) / (2*r)
        L = self.wheel_separation
        r = self.wheel_radius
        w_r = (2.0 * v + omega * L) / (2.0 * r)
        w_l = (2.0 * v - omega * L) / (2.0 * r)

        # Integrate angles
        if dt > 0.0:
            self.theta_l += w_l * dt
            self.theta_r += w_r * dt

        js = JointState()
        js.header.stamp = msg.header.stamp
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.theta_l, self.theta_r]
        js.velocity = [w_l, w_r]

        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
