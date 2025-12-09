# SPDX-License-Identifier: Apache-2.0
"""
Master Bringup Launch File
Launches complete Isaac ROS navigation stack:
- RealSense Camera
- Visual SLAM
- Robot Localization (EKF)
- Nvblox (3D Reconstruction)
- Depth to LaserScan
- Nav2
- RViz2

CRITICAL FIXES:
1. ✅ Static TF published FIRST (before any nodes)
2. ✅ Proper timing for all nodes
3. ✅ RViz disabled by default (headless Docker)
4. ✅ All frames properly configured
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'your_robot_bringup'
    
    # ===== Launch Arguments =====
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera namespace'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',  # ✅ CHANGED: false for headless Docker
        description='Launch RViz2'
    )
    
    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Launch Nav2 stack'
    )
    
    enable_robot_localization_arg = DeclareLaunchArgument(
        'enable_robot_localization',
        default_value='false',  # ✅ CHANGED: false (VSLAM publishes TF directly)
        description='Enable robot_localization EKF (not needed if VSLAM publishes TF)'
    )
    
    enable_ground_constraint_arg = DeclareLaunchArgument(
        'enable_ground_constraint',
        default_value='true',
        description='Enable ground constraint for 2D navigation'
    )
    
    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'config', 'nav2_params.yaml'
        ]),
        description='Nav2 parameters file'
    )

    # ===== CRITICAL: Static TF (MUST BE FIRST!) =====
    # base_link → camera_link static transform
    # ⚠️ ADJUST THESE VALUES based on your camera mount position!
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.05', '0.0', '0.1',  # x=5cm forward, y=0, z=10cm up
            '0.0', '0.0', '0.0',   # roll=0, pitch=0, yaw=0 (HORIZONTAL mount)
            'base_link', 'camera_link'
        ],
        output='screen'
    )

    # ===== Sensor Layer =====
    # 1. RealSense Camera (starts immediately after static TF)
    realsense_launch = TimerAction(
        period=0.5,  # Small delay to ensure TF is published
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'sensors', 'realsense.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': LaunchConfiguration('camera_name'),
                    'enable_depth': 'true',
                    'enable_color': 'true',
                }.items()
            )
        ]
    )

    # ===== Perception Layer =====
    # 2. Visual SLAM (2초 지연 - 카메라 준비 대기)
    vslam_launch = TimerAction(
        period=2.0,  # ✅ INCREASED: Wait for camera to fully initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'perception', 'vslam.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': LaunchConfiguration('camera_name'),
                    'enable_imu_fusion': 'true',
                    'enable_ground_constraint': LaunchConfiguration('enable_ground_constraint'),
                }.items()
            )
        ]
    )
    
    # 3. Robot Localization (OPTIONAL - 4초 지연)
    # ⚠️ NOTE: Not needed if VSLAM publishes TF directly!
    robot_localization_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'localization', 'robot_localization.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_robot_localization'))
            )
        ]
    )

    # 4. Nvblox 3D Reconstruction (3초 지연 - VSLAM 초기화 대기)
    nvblox_launch = TimerAction(
        period=3.0,  # ✅ CHANGED: After VSLAM starts
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'perception', 'nvblox.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': LaunchConfiguration('camera_name'),
                    'global_frame': 'odom',  # ✅ Use odom (published by VSLAM)
                }.items()
            )
        ]
    )

    # 5. Depth Image to LaserScan (2.5초 지연)
    depthimage_to_laserscan_launch = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'sensors', 'depthimage_to_laserscan.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': LaunchConfiguration('camera_name'),
                    'scan_height': '10',
                    'range_min': '0.3',
                    'range_max': '10.0',
                }.items()
            )
        ]
    )

    # ===== Navigation Layer =====
    # 6. Nav2 (5초 지연 - 모든 센서 준비 대기)
    nav2_launch = TimerAction(
        period=5.0,  # ✅ INCREASED: Wait for VSLAM and Nvblox
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'navigation', 'nav2.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('nav2_params_file'),
                    'autostart': 'true',
                    'use_rviz': 'false',
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_nav2'))
            )
        ]
    )

    # ===== Visualization Layer =====
    # 7. RViz2 (6초 지연 - 모든 노드 준비 후)
    rviz_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare(pkg_name),
                        'launch', 'visualization', 'rviz2.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_rviz'))
            )
        ]
    )

    # ===== Build Launch Description =====
    return LaunchDescription([
        # Arguments
        camera_name_arg,
        use_sim_time_arg,
        enable_rviz_arg,
        enable_nav2_arg,
        enable_robot_localization_arg,
        enable_ground_constraint_arg,
        nav2_params_file_arg,
        
        # ✅ CRITICAL: Static TF FIRST (no delay!)
        base_to_camera_tf,
        
        # Launch sequence (with proper timing)
        realsense_launch,              # 0.5초: 카메라
        vslam_launch,                  # 2초: VSLAM
        depthimage_to_laserscan_launch,# 2.5초: Depth to Scan
        nvblox_launch,                 # 3초: Nvblox
        robot_localization_launch,     # 4초: EKF (optional)
        nav2_launch,                   # 5초: Nav2
        rviz_launch,                   # 6초: RViz2
    ])