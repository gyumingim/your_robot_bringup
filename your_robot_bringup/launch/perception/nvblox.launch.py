# SPDX-License-Identifier: Apache-2.0
"""
Nvblox Launch File
3D reconstruction and costmap generation for Nav2
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera namespace'
    )
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='odom',
        description='Global frame for nvblox (odom or map)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nvblox_ros'),
            'config', 'nvblox.yaml'
        ]),
        description='Path to nvblox configuration file'
    )

    camera_name = LaunchConfiguration('camera_name')

    # Nvblox node
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'global_frame': LaunchConfiguration('global_frame'),
                
                # Mapping parameters
                'voxel_size': 0.05,
                'esdf_2d_min_height': 0.0,
                'esdf_2d_max_height': 1.0,
                'esdf_slice_height': 0.5,
                
                # Integration
                'use_depth': True,
                'use_lidar': False,
                'use_color': True,
                
                # Performance
                'max_integration_time_ms': 10.0,
                'mesh_bandwidth_limit_mbps': 10.0,
                
                # Costmap
                'map_clearing_radius_m': 5.0,
                'occupancy_publication_rate_hz': 10.0,
            }
        ],
        remappings=[
            ('depth/image', [camera_name, '/depth/image_rect_raw']),
            ('depth/camera_info', [camera_name, '/depth/camera_info']),
            ('color/image', [camera_name, '/color/image_raw']),
            ('color/camera_info', [camera_name, '/color/camera_info']),
            ('pointcloud', [camera_name, '/depth/color/points']),
            # TF remapping 제거 - 기본 /tf 사용
            ('pose', '/visual_slam/tracking/odometry'),
        ],
    )

    return LaunchDescription([
        camera_name_arg,
        global_frame_arg,
        config_file_arg,
        nvblox_node,
    ])
