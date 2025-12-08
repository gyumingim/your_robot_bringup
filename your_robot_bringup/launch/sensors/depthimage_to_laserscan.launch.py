# SPDX-License-Identifier: Apache-2.0
"""
Depth Image to LaserScan Launch File
Converts depth image to 2D laser scan for Nav2
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
    
    scan_height_arg = DeclareLaunchArgument(
        'scan_height',
        default_value='10',
        description='Number of pixel rows to use for scan'
    )
    
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.3',
        description='Minimum range value [m]'
    )
    
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='10.0',
        description='Maximum range value [m]'
    )

    camera_name = LaunchConfiguration('camera_name')

    # Depth image to laserscan converter
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': LaunchConfiguration('scan_height'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'output_frame': 'camera_depth_optical_frame',
        }],
        remappings=[
            ('depth', [camera_name, '/depth/image_rect_raw']),
            ('depth_camera_info', [camera_name, '/depth/camera_info']),
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        camera_name_arg,
        scan_height_arg,
        range_min_arg,
        range_max_arg,
        depthimage_to_laserscan_node,
    ])
