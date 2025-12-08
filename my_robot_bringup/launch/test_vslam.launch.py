from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # 1. Static TF만
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.0', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 2. RealSense (namespace 없음)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': True,
            'enable_color': True,
            'enable_gyro': True,
            'enable_accel': True,
        }],
        output='screen'
    )

    # 3. VSLAM (기본 토픽 사용)
    vslam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        parameters=[{
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        base_to_camera_tf,
        realsense_node,
        vslam_node,
    ])
