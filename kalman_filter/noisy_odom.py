#!/usr/bin/env python3
"""
Noisy Odometry Publisher

This node simulates a robot moving in a circle and publishes noisy odometry.
Use this for testing the Kalman filter without Gazebo.

Topics:
  Published:
    /odom (nav_msgs/Odometry) - Noisy odometry
    /ground_truth (nav_msgs/Odometry) - True position (for comparison)

Run with:
    ros2 run kalman_filter noisy_odom
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import numpy as np
import math


class NoisyOdomPublisher(Node):
    """
    Simulates a robot moving in a circle with noisy odometry.
    """

    def __init__(self):
        super().__init__('noisy_odom_publisher')

        # Parameters
        self.declare_parameter('noise_std', 0.3)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('angular_velocity', 0.2)

        self.noise_std = self.get_parameter('noise_std').value
        publish_rate = self.get_parameter('publish_rate').value
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('angular_velocity').value

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth', 10)

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # State
        self.t = 0.0
        self.dt = 1.0 / publish_rate

        self.get_logger().info(
            f'Noisy Odometry Publisher started!\n'
            f'  Noise std: {self.noise_std} m\n'
            f'  Radius: {self.radius} m\n'
            f'  Angular velocity: {self.omega} rad/s\n'
            f'Publishing to /odom (noisy) and /ground_truth'
        )

    def timer_callback(self):
        """Publish noisy and ground truth odometry."""
        self.t += self.dt

        # True circular motion
        true_x = self.radius * math.cos(self.omega * self.t)
        true_y = self.radius * math.sin(self.omega * self.t)

        # Velocity (tangent to circle)
        true_vx = -self.radius * self.omega * math.sin(self.omega * self.t)
        true_vy = self.radius * self.omega * math.cos(self.omega * self.t)

        # Heading angle
        yaw = math.atan2(true_vy, true_vx)

        # Add noise for odometry
        noisy_x = true_x + np.random.normal(0, self.noise_std)
        noisy_y = true_y + np.random.normal(0, self.noise_std)

        now = self.get_clock().now().to_msg()

        # Publish noisy odometry
        odom_msg = self.create_odom_msg(noisy_x, noisy_y, true_vx, true_vy, yaw, now)
        self.odom_pub.publish(odom_msg)

        # Publish ground truth
        truth_msg = self.create_odom_msg(true_x, true_y, true_vx, true_vy, yaw, now)
        truth_msg.header.frame_id = 'odom'
        truth_msg.child_frame_id = 'base_link_truth'
        self.truth_pub.publish(truth_msg)

    def create_odom_msg(self, x, y, vx, vy, yaw, stamp):
        """Create an Odometry message."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Orientation (from yaw)
        msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)

        # Velocity
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = self.omega

        return msg

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
