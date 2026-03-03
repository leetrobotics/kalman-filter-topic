#!/usr/bin/env python3
"""
ROS2 Kalman Filter Node

Fuses odometry (fast, drifty) with GPS (slow, noisy but absolute)
using a Linear Kalman Filter for robot localization.

Architecture:
  - PREDICT on every /odom callback (~30 Hz) using velocity
  - UPDATE on every /gps/fix callback (~1 Hz) using position

Topics:
  Subscribed:
    /odom (nav_msgs/Odometry) - Odometry from diff-drive (predict step)
    /gps/fix (sensor_msgs/NavSatFix) - GPS fix from NavSat sensor (update step)

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
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import math

from kalman_filter.linear_kf import LinearKalmanFilter


class KalmanFilterNode(Node):
    """
    ROS2 node that fuses odometry + GPS using a Kalman Filter.

    Predict step: Uses odometry velocity to propagate state forward.
      - Fast (~30 Hz), smooth, but drifts over time.

    Update step: Uses GPS position to correct the estimate.
      - Slow (~1 Hz), noisy, but gives absolute position.

    The Kalman filter optimally balances these two sources.
    """

    def __init__(self):
        super().__init__('kalman_filter_node')

        # Parameters
        self.declare_parameter('process_noise', 0.1)
        self.declare_parameter('gps_noise', 2.0)
        self.declare_parameter('origin_lat', 38.161479)
        self.declare_parameter('origin_lon', -122.454630)

        process_noise = self.get_parameter('process_noise').value
        gps_noise = self.get_parameter('gps_noise').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value

        # Initialize Kalman filter
        # dt=0.033 for ~30Hz odom, but predict uses actual dt
        self.kf = LinearKalmanFilter(
            dt=0.033,
            process_noise=process_noise,
            measurement_noise=gps_noise
        )
        self.initialized = False
        self.last_odom_time = None

        # Meters per degree at origin latitude
        self.meters_per_deg_lat = 111132.92
        self.meters_per_deg_lon = 111132.92 * math.cos(math.radians(self.origin_lat))

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, sensor_qos
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/kalman/pose', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/kalman/uncertainty', 10
        )

        # Stats
        self.predict_count = 0
        self.update_count = 0

        self.get_logger().info(
            f'Kalman Filter Node started!\n'
            f'  Process noise: {process_noise}\n'
            f'  GPS noise: {gps_noise}\n'
            f'  GPS origin: ({self.origin_lat}, {self.origin_lon})\n'
            f'  PREDICT on /odom, UPDATE on /gps/fix\n'
            f'  Publishing to /kalman/pose and /kalman/uncertainty'
        )

    def gps_to_local(self, lat: float, lon: float):
        """
        Convert GPS (lat, lon) to local (x, y) in meters.

        Uses a simple flat-Earth approximation relative to the origin.
        Good enough for small areas like a racetrack.
        """
        x = (lon - self.origin_lon) * self.meters_per_deg_lon
        y = (lat - self.origin_lat) * self.meters_per_deg_lat
        return x, y

    def odom_callback(self, msg: Odometry):
        """
        PREDICT step — called on every odometry message (~30 Hz).

        Uses the robot's velocity to propagate the state forward.
        This is fast and smooth but accumulates drift.
        """
        if not self.initialized:
            return

        # Compute actual dt from timestamps
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom_time is not None:
            dt = current_time - self.last_odom_time
            if dt > 0 and dt < 1.0:  # Sanity check
                # Update the filter's dt for this step
                self.kf.dt = dt
                self.kf.F[0, 2] = dt
                self.kf.F[1, 3] = dt

                # Extract velocity from odometry
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y

                # Get heading from quaternion to transform body-frame velocity
                q = msg.pose.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)

                # Transform velocity from body frame to odom frame
                vx_world = vx * math.cos(yaw) - vy * math.sin(yaw)
                vy_world = vx * math.sin(yaw) + vy * math.cos(yaw)

                # Set velocity in state before predict
                self.kf.x[2, 0] = vx_world
                self.kf.x[3, 0] = vy_world

                # Predict!
                self.kf.predict()
                self.predict_count += 1

                # Publish filtered pose after every predict
                self.publish_pose()
                self.publish_uncertainty_marker()

        self.last_odom_time = current_time

    def gps_callback(self, msg: NavSatFix):
        """
        UPDATE step — called on every GPS fix (~1 Hz).

        Converts lat/lon to local XY and corrects the Kalman estimate.
        This is slow and noisy but gives absolute position (no drift).
        """
        # Convert GPS to local coordinates
        x, y = self.gps_to_local(msg.latitude, msg.longitude)

        # Initialize on first GPS fix
        if not self.initialized:
            self.kf.initialize(x=x, y=y, vx=0.0, vy=0.0)
            self.initialized = True
            self.get_logger().info(
                f'Kalman filter initialized from GPS!\n'
                f'  GPS: ({msg.latitude:.6f}, {msg.longitude:.6f})\n'
                f'  Local: ({x:.2f}, {y:.2f})'
            )
            return

        # Update!
        self.kf.update(x, y)
        self.update_count += 1

        if self.update_count % 10 == 0:
            est_x, est_y, _, _ = self.kf.get_state()
            self.get_logger().info(
                f'GPS updates: {self.update_count}, '
                f'Predicts: {self.predict_count}, '
                f'Est: ({est_x:.2f}, {est_y:.2f}), '
                f'GPS: ({x:.2f}, {y:.2f})'
            )

    def publish_pose(self):
        """Publish the filtered pose estimate with covariance."""
        x, y, vx, vy = self.kf.get_state()
        cov = self.kf.get_covariance()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Orientation from velocity direction
        if abs(vx) > 0.01 or abs(vy) > 0.01:
            yaw = math.atan2(vy, vx)
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            msg.pose.pose.orientation.w = 1.0

        # Covariance (6x6 for ROS, we fill position part)
        msg.pose.covariance[0] = cov[0, 0]   # x-x
        msg.pose.covariance[1] = cov[0, 1]   # x-y
        msg.pose.covariance[6] = cov[1, 0]   # y-x
        msg.pose.covariance[7] = cov[1, 1]   # y-y

        self.pose_pub.publish(msg)

    def publish_uncertainty_marker(self):
        """Publish uncertainty ellipse marker for visualization."""
        x, y, _, _ = self.kf.get_state()
        cov = self.kf.get_covariance()

        # Extract 2x2 position covariance
        pos_cov = cov[:2, :2]

        # Compute eigenvalues for ellipse axes
        eigenvalues, eigenvectors = np.linalg.eig(pos_cov)

        # 95% confidence ellipse (chi-squared with 2 DOF)
        chi2_95 = 5.991
        scale = np.sqrt(chi2_95)

        width = 2 * scale * np.sqrt(abs(eigenvalues[0]))
        height = 2 * scale * np.sqrt(abs(eigenvalues[1]))
        angle = math.atan2(eigenvectors[1, 0], eigenvectors[0, 0])

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'kalman_uncertainty'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05

        marker.pose.orientation.z = math.sin(angle / 2)
        marker.pose.orientation.w = math.cos(angle / 2)

        marker.scale.x = max(width, 0.1)
        marker.scale.y = max(height, 0.1)
        marker.scale.z = 0.02

        marker.color = ColorRGBA(r=0.2, g=0.4, b=0.8, a=0.5)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 100000000

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
