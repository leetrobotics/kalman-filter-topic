#!/usr/bin/env python3
"""
Ground Truth Bridge Node

Bridges the Gazebo ground truth pose (from dynamic_pose/info via ros_gz_bridge)
to a ROS2 Odometry message on /ground_truth.

The ros_gz_bridge converts gz.msgs.Pose_V → tf2_msgs/TFMessage.
This node extracts the robot's pose from that TFMessage and publishes
it as nav_msgs/Odometry so it's easy to compare with /odom and /kalman/pose
in Foxglove.

Topics:
  Subscribed:
    /world/default/dynamic_pose/info (tf2_msgs/TFMessage) - All model poses from Gazebo

  Published:
    /ground_truth (nav_msgs/Odometry) - Robot's true pose

Run with:
    ros2 run kalman_filter ground_truth
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class GroundTruthBridge(Node):
    """
    Extracts robot ground truth pose from Gazebo's dynamic_pose topic.

    Gazebo publishes all model poses as gz.msgs.Pose_V on
    /world/default/dynamic_pose/info. The ros_gz_bridge converts this
    to tf2_msgs/TFMessage. This node finds the robot's transform
    and re-publishes it as Odometry on /ground_truth.
    """

    def __init__(self):
        super().__init__('ground_truth_bridge')

        self.declare_parameter('robot_name', 'turtlebot3_waffle_gps')
        self.robot_name = self.get_parameter('robot_name').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/default/dynamic_pose/info',
            self.pose_callback,
            sensor_qos
        )

        self.gt_pub = self.create_publisher(Odometry, '/ground_truth', 10)

        self.msg_count = 0
        self.get_logger().info(
            f'Ground Truth Bridge started!\n'
            f'  Looking for model: {self.robot_name}\n'
            f'  Publishing to /ground_truth'
        )

    def pose_callback(self, msg: TFMessage):
        """Extract robot pose from TFMessage and publish as Odometry."""
        for transform in msg.transforms:
            if transform.child_frame_id == self.robot_name:
                odom = Odometry()
                odom.header.stamp = transform.header.stamp
                odom.header.frame_id = 'world'
                odom.child_frame_id = self.robot_name

                t = transform.transform
                odom.pose.pose.position.x = t.translation.x
                odom.pose.pose.position.y = t.translation.y
                odom.pose.pose.position.z = t.translation.z
                odom.pose.pose.orientation = t.rotation

                self.gt_pub.publish(odom)

                self.msg_count += 1
                if self.msg_count == 1:
                    self.get_logger().info(
                        f'First ground truth: '
                        f'({t.translation.x:.2f}, {t.translation.y:.2f})'
                    )
                break


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
