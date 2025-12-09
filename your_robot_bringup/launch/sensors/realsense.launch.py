# SPDX-License-Identifier: Apache-2.0
"""
RealSense Camera Launch File
Launches RealSense D435i with depth, color, and IMU

CRITICAL FIXES:
1. ✅ Depth emitter DISABLED (interferes with VSLAM)
2. ✅ Proper IMU configuration
3. ⚠️ NOTE: Camera must be mounted HORIZONTALLY (landscape)
   If mounted vertically, depth will appear rotated 90 degrees
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
            # ===== Device initialization (CRITICAL for USB stability!) =====
            'wait_for_device_timeout': 60.0,  # Wait up to 60s for device (default: 10s)
            'reconnect_timeout': 10.0,  # Auto-reconnect timeout
            'initial_reset': True,  # Force USB reset on startup to clear bad state

            # ===== Infrared streams for VSLAM (CRITICAL!) =====
            'enable_infra1': True,
            'enable_infra2': True,
            'infra_rgb': False,  # ✅ Keep as grayscale

            # ===== Depth stream =====
            'enable_depth': LaunchConfiguration('enable_depth'),
            'align_depth.enable': False,  # ✅ DISABLED - reduces USB bandwidth

            # ===== Color stream =====
            'enable_color': LaunchConfiguration('enable_color'),

            # ===== IMU (CRITICAL for VSLAM!) =====
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,  # 200Hz
            'accel_fps': 200,  # 200Hz
            'unite_imu_method': 2,  # 2 = copy mode (publish to single /imu topic)

            # ===== Point cloud - DISABLED to reduce USB load =====
            'pointcloud.enable': False,  # ✅ DISABLED - reduces USB bandwidth significantly

            # ===== CRITICAL: Depth emitter MUST BE OFF for VSLAM! =====
            # Emitter interferes with stereo vision
            'depth_module.emitter_enabled': 0,  # ✅ OFF for VSLAM
            # Set to 1 only if you need depth in dark environments WITHOUT VSLAM

            # ===== Frame rates - REDUCED to 15fps for stability =====
            # Lower frame rate = less USB bandwidth = more stable
            'depth_module.profile': '640x480x15',  # ✅ Reduced from 30fps to 15fps
            'rgb_camera.profile': '640x480x15',    # ✅ Reduced from 30fps to 15fps
            'infra_profile': '640x480x15',         # ✅ Match depth rate for sync

            # ===== Filters DISABLED =====
            'enable_depth_to_disparity_filter': False,
            'enable_spatial_filter': False,
            'enable_temporal_filter': False,
            'enable_hole_filling_filter': False,

            # ===== Additional USB stability settings =====
            'decimation_filter.enable': False,  # No decimation
            'enable_sync': True,  # Sync streams to reduce timing issues
        }],
        output='screen',
        respawn=True,  # ✅ Auto-restart if crashes
        respawn_delay=5.0,  # ✅ Increased from 2s to 5s for more recovery time
    )

    return LaunchDescription([
        camera_name_arg,
        enable_depth_arg,
        enable_color_arg,
        realsense_node,
    ])