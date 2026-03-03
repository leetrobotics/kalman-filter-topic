#!/usr/bin/env python3
"""
Kalman Filter Demo Launch File

Launches everything needed for the Kalman Filter tutorial:
1. Gazebo simulation server (headless) with Sonoma Raceway world
2. TurtleBot3 with GPS spawned into the world
3. Ground truth bridge (publishes /ground_truth from Gazebo)
4. Teleop bridge (converts Twist → TwistStamped for /cmd_vel)
5. Kalman filter node (fuses odom + GPS)
6. A ros_gz_bridge for the dynamic_pose topic

The user drives the robot manually using Foxglove's teleop panel
and watches the Kalman filter fuse noisy sensors in real time.

Usage:
    ros2 launch kalman_filter demo.launch.py

To visualize:
    Open Foxglove and load the kalman_layout.json
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('kalman_filter')
    world_file = os.path.join(pkg_dir, 'worlds', 'simple_world.sdf')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    process_noise = LaunchConfiguration('process_noise', default='0.1')
    gps_noise = LaunchConfiguration('gps_noise', default='2.0')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    declare_process_noise = DeclareLaunchArgument(
        'process_noise', default_value='0.1',
        description='Process noise for Kalman filter'
    )
    declare_gps_noise = DeclareLaunchArgument(
        'gps_noise', default_value='2.0',
        description='GPS measurement noise (meters)'
    )

    # --- Gazebo simulation server (headless) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -s {world_file}',
            'gz_version': '8',
        }.items(),
    )

    # --- Spawn TurtleBot3 with GPS into the simulation ---
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_minimal_tb3_sim'),
                'launch', 'spawn_tb3_gps.launch.py'
            )
        ),
    )

    # Bridge: Gazebo ground truth poses → ROS2
    # Bridges /world/default/dynamic_pose/info (gz.msgs.Pose_V → tf2_msgs/TFMessage)
    ground_truth_bridge_gz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_gz_bridge',
        output='screen',
        arguments=[
            '/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Ground Truth Node — extracts robot pose from bridged Pose_V
    ground_truth_node = Node(
        package='kalman_filter',
        executable='ground_truth',
        name='ground_truth_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': 'turtlebot3_waffle_gps',
        }],
    )

    # Teleop Bridge — converts Twist (Foxglove panel) → TwistStamped (robot)
    teleop_bridge = Node(
        package='kalman_filter',
        executable='teleop_bridge',
        name='teleop_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Kalman Filter Node — fuses odom (predict) + GPS (update)
    kalman_node = Node(
        package='kalman_filter',
        executable='kalman_node',
        name='kalman_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'process_noise': process_noise,
            'gps_noise': gps_noise,
            'origin_lat': 38.161479,
            'origin_lon': -122.454630,
        }],
    )

    # --- Gazebo WebSocket server (for browser-based sim viewer) ---
    # Delayed start to ensure gz sim is up before the websocket plugin connects
    gz_websocket = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'launch', '/usr/share/gz/gz-launch7/configs/websocket.gzlaunch'],
                name='gz_websocket',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_process_noise,
        declare_gps_noise,
        gz_sim,
        spawn_tb3,
        ground_truth_bridge_gz,
        ground_truth_node,
        teleop_bridge,
        kalman_node,
        gz_websocket,
    ])
