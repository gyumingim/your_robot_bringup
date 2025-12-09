# SPDX-License-Identifier: Apache-2.0
"""
Nvblox Launch File
3D reconstruction and costmap generation for Nav2

✅ Updated for Isaac ROS 3.2
✅ Proper parameters for Nav2 integration
✅ 2D ESDF mode for costmap generation

References:
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ===== Launch Arguments =====
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

    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='Voxel size in meters (default: 5cm)'
    )

    camera_name = LaunchConfiguration('camera_name')

    # ===== Nvblox Node (Isaac ROS 3.2 Compatible) =====
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[{
            # ===== Core Settings =====
            'global_frame': LaunchConfiguration('global_frame'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'use_tf_transforms': True,
            'num_cameras': 1,

            # ===== Mapping Type =====
            # 'static_tsdf': For static environments (indoor/outdoor mapping)
            # 'dynamic': For environments with moving objects
            'mapping_type': 'static_tsdf',

            # ===== Processing Rates (Hz) - Isaac ROS 3.2 =====
            'tick_period_ms': 10,
            'integrate_depth_rate_hz': 30.0,
            'integrate_color_rate_hz': 5.0,
            'update_mesh_rate_hz': 5.0,
            'update_esdf_rate_hz': 10.0,
            'publish_layer_rate_hz': 5.0,
            'publish_debug_vis_rate_hz': 2.0,
            'decay_tsdf_rate_hz': 5.0,
            'clear_map_outside_radius_rate_hz': 1.0,

            # ===== Input Configuration =====
            'use_depth': True,
            'use_color': True,
            'use_lidar': False,

            # ===== ESDF Configuration (for Nav2 costmap) =====
            # '2d' mode generates 2D costmap slice for navigation
            'esdf_mode': '2d',
            'publish_esdf_distance_slice': True,

            # ===== 2D ESDF Slice Parameters =====
            # Height range for 2D projection (obstacles within this range are projected)
            'esdf_2d_min_height': 0.1,   # Min height (m) - ignore ground
            'esdf_2d_max_height': 1.5,   # Max height (m) - robot height

            # ===== Integrator Parameters =====
            'projective_integrator_max_integration_distance_m': 7.0,
            'projective_integrator_truncation_distance_vox': 4.0,
            'projective_integrator_weighting_mode': 'inverse_square_tsdf_distance_penalty',
            'projective_integrator_max_weight': 5.0,

            # ===== Map Management =====
            'map_clearing_radius_m': 7.0,
            'map_clearing_frame_id': 'base_link',
            'maximum_input_queue_length': 10,

            # ===== Mesh Visualization =====
            'mesh_integrator_min_weight': 0.1,
            'mesh_integrator_weld_vertices': True,
            'layer_streamer_bandwidth_limit_mbps': 30.0,

            # ===== Workspace Bounds (optional) =====
            'workspace_bounds_type': 'unbounded',
            'workspace_bounds_min_height_m': -0.5,
            'workspace_bounds_max_height_m': 2.0,

            # ===== TSDF Decay (for dynamic environments) =====
            'tsdf_decay_factor': 0.95,
            'exclude_last_view_from_decay': True,

            # ===== Visualization Settings =====
            'layer_visualization_min_tsdf_weight': 0.1,
            'layer_visualization_exclusion_height_m': 2.0,
            'layer_visualization_exclusion_radius_m': 7.0,
            'max_back_projection_distance': 7.0,

            # ===== Debug Output (disable for production) =====
            'print_rates_to_console': False,
            'print_timings_to_console': False,
            'print_delays_to_console': False,
            'print_queue_drops_to_console': False,
        }],
        remappings=[
            # Depth input from RealSense
            ('depth/image', [camera_name, '/depth/image_rect_raw']),
            ('depth/camera_info', [camera_name, '/depth/camera_info']),
            # Color input (optional, for mesh coloring)
            ('color/image', [camera_name, '/color/image_raw']),
            ('color/camera_info', [camera_name, '/color/camera_info']),
            # Pose from VSLAM odometry
            ('pose', '/visual_slam/tracking/odometry'),
        ],
        respawn=False,
    )

    return LaunchDescription([
        camera_name_arg,
        global_frame_arg,
        voxel_size_arg,
        nvblox_node,
    ])