from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Static TF
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # RealSense - remapping 추가!
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        remappings=[
            ('/infra1/image_rect_raw', '/visual_slam/image_0'),
            ('/infra1/camera_info', '/visual_slam/camera_info_0'),
            ('/infra2/image_rect_raw', '/visual_slam/image_1'),
            ('/infra2/camera_info', '/visual_slam/camera_info_1'),
        ]
    )

    # VSLAM - 기본 토픽 사용
    vslam = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        output='screen',
        parameters=[{
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )

    return LaunchDescription([
        map_to_odom,
        odom_to_base,
        base_to_camera,
        realsense,
        vslam,
    ])
