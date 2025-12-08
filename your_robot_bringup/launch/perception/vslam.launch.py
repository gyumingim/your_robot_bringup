# SPDX-License-Identifier: Apache-2.0
"""
Visual SLAM Launch File
Launches Isaac ROS Visual SLAM with RealSense camera
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera namespace'
    )
    
    enable_imu_fusion_arg = DeclareLaunchArgument(
        'enable_imu_fusion',
        default_value='true',
        description='Enable IMU fusion in VSLAM'
    )
    
    enable_ground_constraint_arg = DeclareLaunchArgument(
        'enable_ground_constraint',
        default_value='false',
        description='Enable ground constraint for 2D navigation'
    )

    camera_name = LaunchConfiguration('camera_name')
    
    # Visual SLAM node as ComposableNode
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # Image processing
            'enable_image_denoising': False,
            'rectified_images': True,
            
            # IMU fusion
            'enable_imu_fusion': LaunchConfiguration('enable_imu_fusion'),
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            
            # Timing
            'image_jitter_threshold_ms': 35.0,
            
            # Frames
            'base_frame': 'camera_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'imu_frame': 'camera_gyro_optical_frame',
            
            # Camera frames
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
            
            # Ground constraint for 2D navigation
            'enable_ground_constraint_in_odometry': LaunchConfiguration('enable_ground_constraint'),
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            
            # Map management
            'enable_localization_n_mapping': True,
            'path_max_size': 1024,
        }],
        remappings=[
            ('visual_slam/image_0', [camera_name, '/infra1/image_rect_raw']),
            ('visual_slam/camera_info_0', [camera_name, '/infra1/camera_info']),
            ('visual_slam/image_1', [camera_name, '/infra2/image_rect_raw']),
            ('visual_slam/camera_info_1', [camera_name, '/infra2/camera_info']),
            ('visual_slam/imu', [camera_name, '/imu']),
        ],
    )

    # Container for visual slam
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([
        camera_name_arg,
        enable_imu_fusion_arg,
        enable_ground_constraint_arg,
        visual_slam_container,
    ])
