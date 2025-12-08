# SPDX-License-Identifier: Apache-2.0
"""
Robot Localization Launch File
Fuses VSLAM odometry with IMU data using EKF
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('your_robot_bringup'),
            'config', 'ekf.yaml'
        ]),
        description='Path to EKF configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # EKF node for fusing visual odometry and IMU
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Visual SLAM odometry as input
            ('odometry/filtered', 'odometry/local'),
            ('/set_pose', '/initialpose'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        ekf_node,
    ])
