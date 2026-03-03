#!/usr/bin/env python3
"""
Teleop Bridge Node

Converts geometry_msgs/Twist (from Foxglove's teleop panel or teleop_twist_keyboard)
to geometry_msgs/TwistStamped (what the ros_gz_bridge expects for /cmd_vel).

The Foxglove Studio teleop panel publishes Twist on /cmd_vel_teleop.
This node adds a timestamp header and republishes as TwistStamped on /cmd_vel.

Topics:
  Subscribed:
    /cmd_vel_teleop (geometry_msgs/Twist) - Raw teleop commands

  Published:
    /cmd_vel (geometry_msgs/TwistStamped) - Stamped velocity commands for robot

Run with:
    ros2 run kalman_filter teleop_bridge
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TeleopBridge(Node):
    """
    Converts Twist → TwistStamped for the ros_gz_bridge.

    The TurtleBot3 GPS bridge config expects TwistStamped on /cmd_vel.
    Foxglove's teleop panel publishes plain Twist. This node bridges
    the gap.
    """

    def __init__(self):
        super().__init__('teleop_bridge')

        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.twist_callback, 10
        )

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.get_logger().info(
            'Teleop Bridge started!\n'
            '  Subscribing to /cmd_vel_teleop (Twist)\n'
            '  Publishing to /cmd_vel (TwistStamped)\n'
            '  Use Foxglove teleop panel → publish to /cmd_vel_teleop'
        )

    def twist_callback(self, msg: Twist):
        """Add timestamp header and republish."""
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.cmd_pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
