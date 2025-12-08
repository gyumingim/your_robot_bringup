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

    # Static TF 체인 생성
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    odom_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.0', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
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

    # VSLAM
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        parameters=[{
            'enable_rectified_pose': True,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'enable_localization_n_mapping': True,
            'publish_odom_to_base_tf': False,  # static TF 사용하므로 False
            'publish_map_to_odom_tf': False,   # static TF 사용하므로 False
        }],
        remappings=[
            ('stereo_camera/left/image', '/infra1/image_rect_raw'),
            ('stereo_camera/left/camera_info', '/infra1/camera_info'),
            ('stereo_camera/right/image', '/infra2/image_rect_raw'),
            ('stereo_camera/right/camera_info', '/infra2/camera_info'),
            ('visual_slam/imu', '/imu'),
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        map_to_odom_tf,
        odom_to_base_tf,
        base_to_camera_tf,
        realsense_camera_node,
        visual_slam_node,
    ])
