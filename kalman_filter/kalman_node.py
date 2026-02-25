#!/usr/bin/env python3
"""
ROS2 Kalman Filter Node

This node demonstrates Kalman filtering for robot localization.
It subscribes to noisy odometry and publishes filtered pose estimates
with uncertainty visualization.

Topics:
  Subscribed:
    /odom (nav_msgs/Odometry) - Noisy odometry from robot

  Published:
    /kalman/pose (geometry_msgs/PoseWithCovarianceStamped) - Filtered pose
    /kalman/uncertainty (visualization_msgs/Marker) - Uncertainty ellipse

Run with:
    ros2 run kalman_filter kalman_node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import math

from kalman_filter.linear_kf import LinearKalmanFilter


class KalmanFilterNode(Node):
    """
    ROS2 node that applies Kalman filtering to odometry data.

    This demonstrates how to:
    1. Subscribe to sensor data (odometry)
    2. Apply a Kalman filter for state estimation
    3. Publish filtered estimates with uncertainty
    4. Visualize uncertainty as an ellipse marker
    """

    def __init__(self):
        super().__init__('kalman_filter_node')

        # Declare parameters
        self.declare_parameter('process_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.5)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameters
        process_noise = self.get_parameter('process_noise').value
        measurement_noise = self.get_parameter('measurement_noise').value
        publish_rate = self.get_parameter('publish_rate').value

        # Initialize Kalman filter
        dt = 1.0 / publish_rate
        self.kf = LinearKalmanFilter(
            dt=dt,
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )
        self.initialized = False

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kalman/pose',
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            '/kalman/uncertainty',
            10
        )

        # Timer for prediction step (runs even without measurements)
        self.predict_timer = self.create_timer(dt, self.predict_callback)

        # Statistics
        self.measurement_count = 0

        self.get_logger().info(
            f'Kalman Filter Node started!\n'
            f'  Process noise: {process_noise}\n'
            f'  Measurement noise: {measurement_noise}\n'
            f'  Publish rate: {publish_rate} Hz\n'
            f'Subscribing to /odom, publishing to /kalman/pose'
        )

    def odom_callback(self, msg: Odometry):
        """
        Handle incoming odometry measurements.

        This is where we apply the Kalman filter update step.
        """
        # Extract position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Initialize on first measurement
        if not self.initialized:
            self.kf.initialize(x=x, y=y, vx=0.0, vy=0.0)
            self.initialized = True
            self.get_logger().info(f'Kalman filter initialized at ({x:.2f}, {y:.2f})')
            return

        # Kalman filter update step
        self.kf.update(x, y)
        self.measurement_count += 1

        # Publish filtered pose
        self.publish_pose()
        self.publish_uncertainty_marker()

    def predict_callback(self):
        """
        Timer callback for prediction step.

        This runs at a fixed rate to propagate the state forward,
        even when measurements are delayed or missing.
        """
        if not self.initialized:
            return

        # Kalman filter predict step
        self.kf.predict()

    def publish_pose(self):
        """
        Publish the filtered pose estimate with covariance.
        """
        x, y, vx, vy = self.kf.get_state()
        cov = self.kf.get_covariance()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Orientation (compute from velocity direction)
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            yaw = math.atan2(vy, vx)
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            msg.pose.pose.orientation.w = 1.0

        # Covariance (6x6 for ROS, we fill position part)
        # Row-major order: [xx, xy, xz, xr, xp, xy, ...]
        msg.pose.covariance[0] = cov[0, 0]   # x-x
        msg.pose.covariance[1] = cov[0, 1]   # x-y
        msg.pose.covariance[6] = cov[1, 0]   # y-x
        msg.pose.covariance[7] = cov[1, 1]   # y-y

        self.pose_pub.publish(msg)

    def publish_uncertainty_marker(self):
        """
        Publish a marker showing the uncertainty ellipse.

        The ellipse size represents the 95% confidence region.
        """
        x, y, _, _ = self.kf.get_state()
        cov = self.kf.get_covariance()

        # Extract 2x2 position covariance
        pos_cov = cov[:2, :2]

        # Compute eigenvalues for ellipse axes
        eigenvalues, eigenvectors = np.linalg.eig(pos_cov)

        # 95% confidence ellipse (chi-squared with 2 DOF)
        chi2_95 = 5.991
        scale = np.sqrt(chi2_95)

        # Ellipse dimensions
        width = 2 * scale * np.sqrt(abs(eigenvalues[0]))
        height = 2 * scale * np.sqrt(abs(eigenvalues[1]))

        # Ellipse orientation
        angle = math.atan2(eigenvectors[1, 0], eigenvectors[0, 0])

        # Create marker
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'kalman_uncertainty'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position (slightly above ground for visibility)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05

        # Orientation
        marker.pose.orientation.z = math.sin(angle / 2)
        marker.pose.orientation.w = math.cos(angle / 2)

        # Scale (ellipse dimensions)
        marker.scale.x = max(width, 0.1)
        marker.scale.y = max(height, 0.1)
        marker.scale.z = 0.02  # Thin disk

        # Color (semi-transparent blue)
        marker.color = ColorRGBA(r=0.2, g=0.4, b=0.8, a=0.5)

        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100000000  # 100ms

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = KalmanFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
