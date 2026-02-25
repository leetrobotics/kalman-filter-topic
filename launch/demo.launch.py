#!/usr/bin/env python3
"""
Kalman Filter Demo Launch File

Launches:
1. Gazebo with simple world and diff-drive robot
2. ROS-Gazebo bridge for odometry and IMU topics
3. Kalman filter node

Usage:
    ros2 launch kalman_filter demo.launch.py

To control the robot:
    ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.25}}"
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_kalman = get_package_share_directory('kalman_filter')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_kalman, 'worlds', 'simple_world.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-s -r {world_file}',
        }.items()
    )

    # ROS-Gazebo Bridge
    # Bridge odometry and IMU topics from Gazebo to ROS2
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Odometry: Gazebo -> ROS2
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Noisy odometry: Gazebo -> ROS2
            '/odom_noisy@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # IMU: Gazebo -> ROS2
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Cmd vel: ROS2 -> Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    # Kalman Filter Node
    kalman_node = Node(
        package='kalman_filter',
        executable='kalman_node',
        name='kalman_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'process_noise': 0.1,
            'measurement_noise': 0.3,
            'publish_rate': 10.0,
        }],
        remappings=[
            # Use noisy odometry as input
            ('/odom', '/odom_noisy'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        gz_sim,
        ros_gz_bridge,
        kalman_node,
    ])
