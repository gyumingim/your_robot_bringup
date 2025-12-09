# SPDX-License-Identifier: Apache-2.0
"""
Depth Image to LaserScan Launch File
Converts depth image to 2D laser scan for Nav2

⚠️ IMPORTANT: If depth appears rotated 90 degrees:
   1. Camera must be mounted HORIZONTALLY (landscape orientation)
   2. If vertically mounted, add image_rotate node (see comments below)
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

    # ===== OPTIONAL: Image Rotation Node (if camera is vertical) =====
    # Uncomment this if your depth image is rotated 90 degrees
    # 
    # depth_rotate_node = Node(
    #     package='image_rotate',
    #     executable='image_rotate_node',
    #     name='depth_rotate',
    #     parameters=[{
    #         'target_frame_id': 'camera_depth_optical_frame',
    #         'target_x': 0.0,
    #         'target_y': 0.0,
    #         'target_z': 1.5708,  # 90 degrees in radians
    #     }],
    #     remappings=[
    #         ('image', [camera_name, '/depth/image_rect_raw']),
    #         ('rotated/image', [camera_name, '/depth/image_rect_raw_rotated']),
    #         ('camera_info', [camera_name, '/depth/camera_info']),
    #         ('rotated/camera_info', [camera_name, '/depth/camera_info_rotated']),
    #     ],
    # )

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
            # ✅ Use original depth image (change if using rotation node)
            ('depth', [camera_name, '/depth/image_rect_raw']),
            ('depth_camera_info', [camera_name, '/depth/camera_info']),
            
            # ✅ If using rotation node, change to:
            # ('depth', [camera_name, '/depth/image_rect_raw_rotated']),
            # ('depth_camera_info', [camera_name, '/depth/camera_info_rotated']),
            
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        camera_name_arg,
        scan_height_arg,
        range_min_arg,
        range_max_arg,
        # depth_rotate_node,  # Uncomment if needed
        depthimage_to_laserscan_node,
    ])