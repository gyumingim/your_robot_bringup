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


def generate_launch_description():
    # Package name (변경 필요)
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
        default_value='true',
        description='Launch RViz2'
    )
    
    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Launch Nav2 stack'
    )
    
    enable_robot_localization_arg = DeclareLaunchArgument(
        'enable_robot_localization',
        default_value='true',
        description='Enable robot_localization EKF'
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

    # ===== Sensor Layer =====
    # 1. RealSense Camera (가장 먼저 실행)
    realsense_launch = IncludeLaunchDescription(
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

    # ===== Perception Layer =====
    # 2. Visual SLAM (1초 지연 후 실행)
    vslam_launch = TimerAction(
        period=1.0,
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
    
    # 3. Robot Localization (2초 지연)
    robot_localization_launch = TimerAction(
        period=2.0,
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

    # 4. Nvblox 3D Reconstruction (2초 지연)
    nvblox_launch = TimerAction(
        period=2.0,
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
                    'global_frame': 'odom',
                }.items()
            )
        ]
    )

    # 5. Depth Image to LaserScan (1.5초 지연)
    depthimage_to_laserscan_launch = TimerAction(
        period=1.5,
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
    # 6. Nav2 (3초 지연)
    nav2_launch = TimerAction(
        period=3.0,
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
                    'use_rviz': 'false',  # RViz는 별도로 실행
                }.items(),
                condition=IfCondition(LaunchConfiguration('enable_nav2'))
            )
        ]
    )

    # ===== Visualization Layer =====
    # 7. RViz2 (4초 지연 - 모든 노드가 준비된 후)
    rviz_launch = TimerAction(
        period=4.0,
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
        
        # Launch sequence
        realsense_launch,              # 0초: 카메라
        vslam_launch,                  # 1초: VSLAM
        depthimage_to_laserscan_launch,# 1.5초: Depth to Scan
        robot_localization_launch,     # 2초: EKF
        nvblox_launch,                 # 2초: Nvblox
        nav2_launch,                   # 3초: Nav2
        rviz_launch,                   # 4초: RViz2
    ])
