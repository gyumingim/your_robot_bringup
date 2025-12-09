# SPDX-License-Identifier: Apache-2.0
"""
Nvblox Launch File
3D reconstruction and costmap generation for Nav2

CRITICAL FIXES:
1. ✅ NO TF remapping (uses default /tf)
2. ✅ Proper parameters for Nav2 integration
3. ✅ Visualization enabled
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
        description='Global frame for nvblox (use odom from VSLAM)'
    )
    
    # Note: nvblox_ros may not have a default config file
    # We override all parameters directly

    camera_name = LaunchConfiguration('camera_name')

    # Nvblox node
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[{
            # ===== Frame Configuration =====
            'global_frame': LaunchConfiguration('global_frame'),
            
            # ===== Mapping Parameters =====
            'voxel_size': 0.05,  # 5cm voxels
            'esdf_2d_min_height': 0.0,  # Min height for 2D slice (m)
            'esdf_2d_max_height': 1.0,  # Max height for 2D slice (m)
            'esdf_slice_height': 0.5,   # Height of 2D slice (m)
            
            # ===== Input Configuration =====
            'use_depth': True,
            'use_lidar': False,
            'use_color': True,
            
            # ===== Integration Parameters =====
            'max_integration_time_ms': 10.0,
            'tsdf_integrator_max_integration_distance_m': 10.0,
            'mesh_integrator_min_weight': 1e-4,
            'mesh_integrator_weld_vertices': True,
            
            # ===== Performance =====
            'max_poll_rate_hz': 100.0,
            'mesh_update_rate_hz': 5.0,
            'esdf_update_rate_hz': 5.0,
            
            # ===== Mesh Bandwidth (for visualization) =====
            'mesh_bandwidth_limit_mbps': 10.0,  # Increase if mesh updates are slow
            
            # ===== Costmap Configuration (for Nav2) =====
            'map_clearing_radius_m': 5.0,
            'occupancy_publication_rate_hz': 10.0,
            
            # ===== Visualization =====
            'publish_esdf_distance_slice': True,
            'publish_occupancy_distance_slice': True,
            'esdf_and_gradients_unobserved_value': 1000.0,
            
            # ===== Mapper Type =====
            'mapper_type': 'static',  # 'static' or 'dynamic'
        }],
        remappings=[
            # ✅ CRITICAL: Correct topic names
            ('depth/image', [camera_name, '/depth/image_rect_raw']),
            ('depth/camera_info', [camera_name, '/depth/camera_info']),
            ('color/image', [camera_name, '/color/image_raw']),
            ('color/camera_info', [camera_name, '/color/camera_info']),
            ('pointcloud', [camera_name, '/depth/color/points']),
            
            # ✅ CRITICAL: NO TF remapping! Use default /tf
            # ('transform', '/tf'),  # ❌ REMOVED - causes crash!
            
            # Pose from VSLAM
            ('pose', '/visual_slam/tracking/odometry'),
        ],
        respawn=False,  # Don't auto-restart (for debugging)
    )

    return LaunchDescription([
        camera_name_arg,
        global_frame_arg,
        nvblox_node,
    ])