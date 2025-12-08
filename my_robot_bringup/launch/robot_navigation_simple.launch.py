import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # RealSense 카메라
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_gyro': True,
            'enable_accel': True,
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'unite_imu_method': 2,
            'enable_sync': True,
        }],
        output='screen'
    )

    # Static TF: base_link -> camera_link
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.0', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # VSLAM 노드 (일반 노드로 실행)
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        parameters=[{
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_debug_mode': False,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'enable_localization_n_mapping': True,
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
        }],
        remappings=[
            ('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
            ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
            ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
            ('stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
            ('visual_slam/imu', '/camera/imu'),
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(realsense_camera_node)
    ld.add_action(base_to_camera_tf)
    ld.add_action(visual_slam_node)
    
    return ld
