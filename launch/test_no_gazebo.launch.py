#!/usr/bin/env python3
"""
Kalman Filter Test Launch (No Gazebo)

This launch file runs the Kalman filter with simulated noisy odometry.
Use this for quick testing without needing Gazebo.

Launches:
1. Noisy odometry publisher (simulates robot moving in circle)
2. Kalman filter node

Usage:
    ros2 launch kalman_filter test_no_gazebo.launch.py

View results:
    ros2 topic echo /kalman/pose
    ros2 topic echo /ground_truth
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    noise_std = LaunchConfiguration('noise_std', default='0.3')
    process_noise = LaunchConfiguration('process_noise', default='0.1')

    declare_noise_std = DeclareLaunchArgument(
        'noise_std',
        default_value='0.3',
        description='Standard deviation of odometry noise (meters)'
    )

    declare_process_noise = DeclareLaunchArgument(
        'process_noise',
        default_value='0.1',
        description='Process noise for Kalman filter'
    )

    # Noisy Odometry Publisher (simulates robot)
    noisy_odom = Node(
        package='kalman_filter',
        executable='noisy_odom',
        name='noisy_odom_publisher',
        output='screen',
        parameters=[{
            'noise_std': noise_std,
            'publish_rate': 10.0,
            'radius': 2.0,
            'angular_velocity': 0.2,
        }],
    )

    # Kalman Filter Node
    kalman_node = Node(
        package='kalman_filter',
        executable='kalman_node',
        name='kalman_filter',
        output='screen',
        parameters=[{
            'process_noise': process_noise,
            'measurement_noise': noise_std,
            'publish_rate': 10.0,
        }],
    )

    return LaunchDescription([
        declare_noise_std,
        declare_process_noise,
        noisy_odom,
        kalman_node,
    ])
