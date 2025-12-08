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
    
    # RealSense는 자체적으로 TF를 발행하므로 camera_link 연결만 하면 됨
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # RealSense - 완전히 최적화된 설정
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[{
            # 해상도 및 프레임레이트
            'depth_module.profile': '640x480x30',  # 30fps로 증가 (VSLAM에 더 좋음)
            
            # 활성화할 스트림
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_depth': False,
            'enable_color': False,
            'enable_gyro': False,
            'enable_accel': False,
            'enable_sync': True,
            
            # IR Emitter 끄기 (Stereo에 방해됨!)
            'enable_infra_emitter': False,  # 중요!
            
            # 노출 및 게인 설정 (조명 최적화)
            'infra_exposure': 8500.0,      # 자동 노출
            'infra_gain': 16.0,             # 게인 증가 (밝게)
            
            # 자동 노출 활성화
            'enable_auto_exposure': True,
        }],
        remappings=[
            ('/infra1/image_rect_raw', '/visual_slam/image_0'),
            ('/infra1/camera_info', '/visual_slam/camera_info_0'),
            ('/infra2/image_rect_raw', '/visual_slam/image_1'),
            ('/infra2/camera_info', '/visual_slam/camera_info_1'),
        ]
    )

    # VSLAM - 최적화된 설정
    vslam = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        output='screen',
        parameters=[{
            # 시각화
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            
            # 프레임 설정
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'left_camera_frame': 'camera_infra1_optical_frame',   # 추가!
            'right_camera_frame': 'camera_infra2_optical_frame',  # 추가!
            
            # 이미지 처리
            'denoise_input_images': True,   # 노이즈 제거 활성화
            'rectified_images': True,
            'enable_debug_mode': False,
            'image_jitter_threshold_ms': 10000.0,
            
            # 특징점 검출 파라미터 (추가!)
            'enable_localization_n_mapping': True,
            'enable_observations_view': True,
            'enable_slam_visualization': True,
            'num_cameras': 2,
        }]
    )

    return LaunchDescription([
        map_to_odom,
        odom_to_base,
        base_to_camera,
        realsense,
        vslam,
    ])
