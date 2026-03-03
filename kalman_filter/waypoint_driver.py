#!/usr/bin/env python3
"""
Waypoint Driver Node

Drives the TurtleBot3 along a sequence of waypoints using simple
proportional control. This gives the Kalman filter node continuous
motion data to filter.

Topics:
  Subscribed:
    /odom (nav_msgs/Odometry) - Robot odometry for heading

  Published:
    /cmd_vel (geometry_msgs/TwistStamped) - Velocity commands

Run with:
    ros2 run kalman_filter waypoint_driver
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

import math


class WaypointDriver(Node):
    """
    Simple waypoint follower using proportional control.

    Drives the robot to each waypoint in sequence, then loops.
    Uses odometry for heading — not GPS — so the student's kalman
    filter doesn't interfere with driving.
    """

    def __init__(self):
        super().__init__('waypoint_driver')

        # Waypoints in odom frame (x, y) — a loop around the start area
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('waypoint_threshold', 2.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.wp_threshold = self.get_parameter('waypoint_threshold').value

        # Sonoma Raceway centerline from OpenStreetMap (way 505453048)
        # Converted to odom frame: GPS-local coords offset by spawn position.
        # Robot spawns at odom (0,0) = GPS-local (2.26, -3.80).
        # Section: ~200m along the track from spawn through Turn 7 and back.
        # At 0.5 m/s → ~7 min one way, loops continuously.
        #
        # Sonoma Raceway centerline from OpenStreetMap (way 505453048)
        # The Gazebo model is scaled to ~44% of real-world size.
        # GPS-local coords must be scaled by 0.437 relative to spawn.
        #
        # GPS-local → odom: subtract spawn, then scale by 0.437
        ox, oy = 2.26, -3.80  # spawn in GPS-local frame
        scale = 0.437  # Gazebo model scale vs real world
        track_gps_local = [
            (   2.26,   -3.80),  # spawn / start-finish line
            (  -1.67,   -0.71),
            (  -5.59,    2.39),
            (  -9.48,    5.54),
            ( -13.35,    8.69),
            ( -17.23,   11.85),
            ( -21.11,   15.01),
            ( -24.99,   18.17),
            ( -28.86,   21.32),
            ( -32.74,   24.48),
            ( -36.62,   27.64),
            ( -40.49,   30.79),
            ( -44.37,   33.95),
            ( -48.25,   37.11),
            ( -52.13,   40.27),
            ( -56.00,   43.42),
            ( -59.90,   46.55),
            ( -63.85,   49.62),
            ( -67.80,   52.69),
            ( -71.75,   55.76),  # entering Turn 7
            ( -76.16,   58.10),
            ( -80.61,   60.36),
            ( -85.08,   62.62),
            ( -89.80,   64.26),
            ( -94.54,   65.85),
            ( -99.31,   67.35),
            (-104.23,   68.21),
            (-109.17,   68.94),
            (-114.15,   69.46),  # exit Turn 7, back straight
            (-119.14,   69.69),
            (-124.14,   69.77),
            (-129.13,   69.84),
            (-134.13,   69.85),
            (-139.13,   69.82),  # end of back straight — turn around
        ]
        self.waypoints = [((x - ox) * scale, (y - oy) * scale)
                          for x, y in track_gps_local]
        self.current_wp = 0

        # Current robot state from odom
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        # Odom offset: waypoints are relative to spawn, but odom may not
        # start at (0,0) if robot was respawned. Capture first odom as offset.
        self.odom_offset_x = None
        self.odom_offset_y = None

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, sensor_qos
        )

        # Publishers — TwistStamped for nav2 bridge
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Waypoint Driver started!\n'
            f'  {len(self.waypoints)} waypoints in loop\n'
            f'  Speed: {self.linear_speed} m/s\n'
            f'  Driving to first waypoint: {self.waypoints[0]}'
        )

    def odom_callback(self, msg: Odometry):
        """Extract robot pose from odometry, relative to spawn."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        # Capture offset on first message so waypoints work even after respawn
        if self.odom_offset_x is None:
            self.odom_offset_x = raw_x
            self.odom_offset_y = raw_y
            self.get_logger().info(
                f'Odom offset: ({self.odom_offset_x:.2f}, {self.odom_offset_y:.2f})'
            )

        self.robot_x = raw_x - self.odom_offset_x
        self.robot_y = raw_y - self.odom_offset_y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def get_segment_heading(self, wp_idx):
        """Get the heading of the segment FROM wp_idx TO the next waypoint."""
        next_idx = (wp_idx + 1) % len(self.waypoints)
        dx = self.waypoints[next_idx][0] - self.waypoints[wp_idx][0]
        dy = self.waypoints[next_idx][1] - self.waypoints[wp_idx][1]
        return math.atan2(dy, dx)

    def control_loop(self):
        """
        Follow the track by driving along the heading of each segment.

        At each step:
        1. Compute the desired heading = direction from current WP to next WP
        2. Steer to match that heading (stay on the segment line)
        3. When close enough to the next WP, advance
        """
        if self.current_wp >= len(self.waypoints):
            self.current_wp = 0

        next_wp = (self.current_wp + 1) % len(self.waypoints)
        target_x, target_y = self.waypoints[next_wp]

        # Distance to next waypoint
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if we reached the next waypoint
        if dist < self.wp_threshold:
            self.current_wp = next_wp
            next_next = (self.current_wp + 1) % len(self.waypoints)
            self.get_logger().info(
                f'Reached waypoint {self.current_wp}! Next: {self.waypoints[next_next]}'
            )
            return

        # Desired heading = direction of current segment (current WP → next WP)
        segment_yaw = self.get_segment_heading(self.current_wp)

        # Also compute angle to the next waypoint for cross-track correction
        angle_to_wp = math.atan2(dy, dx)

        # Blend: mostly follow segment heading, but correct toward waypoint
        # to prevent drifting off the line
        blend = 0.7  # 0.7 = mostly segment heading, 0.3 = correct toward WP
        desired_yaw = segment_yaw * blend + angle_to_wp * (1.0 - blend)

        yaw_error = desired_yaw - self.robot_yaw

        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi

        # Proportional control
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Slow down when turning sharply
        if abs(yaw_error) > 0.3:
            cmd.twist.linear.x = self.linear_speed * 0.3
        else:
            cmd.twist.linear.x = self.linear_speed

        cmd.twist.angular.z = self.angular_gain * yaw_error

        # Clamp angular velocity
        max_angular = 2.0
        cmd.twist.angular.z = max(-max_angular, min(max_angular, cmd.twist.angular.z))

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot
        stop_cmd = TwistStamped()
        node.cmd_pub.publish(stop_cmd)
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
