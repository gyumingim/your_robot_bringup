# SPDX-License-Identifier: Apache-2.0
"""
Master Bringup Launch File
Launches complete Isaac ROS navigation stack for indoor/outdoor SLAM

Components:
- RealSense D435i Camera (stereo IR + depth + IMU)
- Isaac ROS Visual SLAM (cuVSLAM)
- Isaac ROS Nvblox (3D reconstruction + 2D costmap)
- Depth to LaserScan (for local costmap)
- Nav2 (navigation stack)
- RViz2 (visualization)

✅ Updated for Isaac ROS 3.2
✅ Compatible with Docker environment (/workspaces/isaac_ros-dev)
✅ Scout Mini support prepared (commented out)

References:
- https://nvidia-isaac-ros.github.io/
- https://navigation.ros.org/
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'your_robot_bringup'

    # ===================================================================
    # LAUNCH ARGUMENTS
    # ===================================================================

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
        default_value='false',
        description='Launch RViz2 (set true for visualization)'
    )

    enable_nav2_arg = DeclareLaunchArgument(
        'enable_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack'
    )

    enable_robot_localization_arg = DeclareLaunchArgument(
        'enable_robot_localization',
        default_value='false',
        description='Enable robot_localization EKF (not needed - VSLAM publishes TF)'
    )

    enable_ground_constraint_arg = DeclareLaunchArgument(
        'enable_ground_constraint',
        default_value='true',
        description='Enable ground constraint for 2D navigation (constrains to XY plane)'
    )

    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable full SLAM mode (true) or odometry-only mode (false)'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'config', 'nav2_params.yaml'
        ]),
        description='Nav2 parameters file'
    )

    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='Nvblox voxel size in meters (smaller = more detail, more memory)'
    )

    # ===================================================================
    # STATIC TF - MUST BE FIRST!
    # ===================================================================
    # base_link → camera_link static transform
    # ⚠️ ADJUST THESE VALUES based on your camera mount position!
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.05',   # 5cm forward from base_link
            '--y', '0.0',    # Centered
            '--z', '0.1',    # 10cm above base_link
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen'
    )

    # ===================================================================
    # SENSOR LAYER
    # ===================================================================

    # 1. RealSense Camera (0.5초 지연 - TF 안정화 대기)
    realsense_launch = TimerAction(
        period=0.5,
        actions=[
            LogInfo(msg='[1/6] Starting RealSense camera...'),
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

    # ===================================================================
    # PERCEPTION LAYER
    # ===================================================================

    # 2. Visual SLAM (2초 지연 - 카메라 초기화 대기)
    vslam_launch = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[2/6] Starting Visual SLAM...'),
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
                    'enable_localization_n_mapping': LaunchConfiguration('enable_slam'),
                }.items()
            )
        ]
    )

    # 3. Depth to LaserScan (2.5초 지연)
    depthimage_to_laserscan_launch = TimerAction(
        period=2.5,
        actions=[
            LogInfo(msg='[3/6] Starting Depth to LaserScan...'),
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

    # 4. Nvblox 3D Reconstruction (3초 지연 - VSLAM odometry 대기)
    nvblox_launch = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[4/6] Starting Nvblox 3D reconstruction...'),
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
                    'voxel_size': LaunchConfiguration('voxel_size'),
                }.items()
            )
        ]
    )

    # ===================================================================
    # LOCALIZATION LAYER (OPTIONAL)
    # ===================================================================

    # 5. Robot Localization EKF (4초 지연) - 보통 필요 없음
    # VSLAM이 직접 TF를 publish하므로 EKF는 필요 없음
    # Scout Mini IMU 추가 시 활성화 가능
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

    # ===================================================================
    # NAVIGATION LAYER
    # ===================================================================

    # 6. Nav2 (5초 지연 - 모든 센서/맵 준비 대기)
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[5/6] Starting Nav2 navigation stack...'),
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

    # ===================================================================
    # VISUALIZATION LAYER
    # ===================================================================

    # 7. RViz2 (6초 지연 - 모든 노드 준비 후)
    rviz_launch = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg='[6/6] Starting RViz2 visualization...'),
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

    # ===================================================================
    # SCOUT MINI SUPPORT (주석 처리 - 나중에 활성화)
    # ===================================================================
    # Scout Mini와 연동 시 아래 코드 활성화:
    #
    # scout_mini_odom_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='scout_odom_tf',
    #     arguments=[
    #         '--x', '0.0',
    #         '--y', '0.0',
    #         '--z', '0.0',
    #         '--roll', '0.0',
    #         '--pitch', '0.0',
    #         '--yaw', '0.0',
    #         '--frame-id', 'odom',
    #         '--child-frame-id', 'base_link',
    #     ],
    #     output='screen'
    # )
    #
    # Scout Mini IMU 사용 시:
    # 1. enable_robot_localization:=true 로 설정
    # 2. config/ekf.yaml에 Scout Mini IMU 토픽 추가
    # 3. VSLAM의 publish_odom_to_base_tf를 false로 변경 (EKF가 대신 publish)

    # ===================================================================
    # BUILD LAUNCH DESCRIPTION
    # ===================================================================

    return LaunchDescription([
        # Arguments
        camera_name_arg,
        use_sim_time_arg,
        enable_rviz_arg,
        enable_nav2_arg,
        enable_robot_localization_arg,
        enable_ground_constraint_arg,
        enable_slam_arg,
        nav2_params_file_arg,
        voxel_size_arg,

        # Startup message
        LogInfo(msg='=== Isaac ROS Navigation Stack Starting ==='),
        LogInfo(msg='Workspace: /workspaces/isaac_ros-dev'),

        # ✅ CRITICAL: Static TF FIRST (no delay!)
        base_to_camera_tf,

        # Launch sequence (timed)
        realsense_launch,              # 0.5초: RealSense camera
        vslam_launch,                  # 2.0초: Visual SLAM
        depthimage_to_laserscan_launch,# 2.5초: Depth to LaserScan
        nvblox_launch,                 # 3.0초: Nvblox 3D reconstruction
        robot_localization_launch,     # 4.0초: EKF (optional)
        nav2_launch,                   # 5.0초: Nav2 navigation
        rviz_launch,                   # 6.0초: RViz2 visualization
    ])
