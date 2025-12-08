# SPDX-License-Identifier: Apache-2.0
"""
RealSense Camera Launch File
Launches RealSense D435i with depth, color, and IMU
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera namespace'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth stream'
    )
    
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream'
    )

    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[{
            # Infrared streams for VSLAM
            'enable_infra1': True,
            'enable_infra2': True,
            
            # Depth stream
            'enable_depth': LaunchConfiguration('enable_depth'),
            'depth_module.profile': '640x480x30',
            'align_depth.enable': True,
            
            # Color stream
            'enable_color': LaunchConfiguration('enable_color'),
            'rgb_camera.profile': '640x480x30',
            
            # IMU
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 200,
            'unite_imu_method': 2,  # 2 = copy mode
            
            # Point cloud
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,  # 2 = color
            
            # Emitter (for better depth in dark environments)
            'depth_module.emitter_enabled': 1,
            
            # Frame rates alignment
            'depth_module.profile': '640x480x30',
        }],
        output='screen'
    )

    return LaunchDescription([
        camera_name_arg,
        enable_depth_arg,
        enable_color_arg,
        realsense_node,
    ])
