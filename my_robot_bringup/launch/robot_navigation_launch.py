import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_nvblox_costmap = LaunchConfiguration('enable_nvblox_costmap', default='true')
    localization_mode = LaunchConfiguration('localization_mode', default='false')
    
    # 맵 파일 경로
    vslam_map_folder = LaunchConfiguration('vslam_map_folder', default='/path/to/vslam_map')
    nvblox_map_path = LaunchConfiguration('nvblox_map_path', default='/path/to/nvblox_map.nvblx')
    
    # 카메라 관련 설정
    camera_width = 640
    camera_height = 480
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_localization_mode_cmd = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='Enable localization mode (load existing maps)'
    )
    
    declare_vslam_map_folder_cmd = DeclareLaunchArgument(
        'vslam_map_folder',
        default_value='/path/to/vslam_map',
        description='Path to VSLAM map folder'
    )
    
    declare_nvblox_map_path_cmd = DeclareLaunchArgument(
        'nvblox_map_path',
        default_value='/path/to/nvblox_map.nvblx',
        description='Path to NVBlox map file'
    )

    # =============================================================================
    # 1. RealSense D435i 카메라 노드
    # =============================================================================
    realsense_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_gyro': True,
            'enable_accel': True,
            'depth_module.profile': f'{camera_width}x{camera_height}x30',
            'rgb_camera.profile': f'{camera_width}x{camera_height}x30',
            'unite_imu_method': 2,  # linear interpolation
            'enable_sync': True,
        }],
        remappings=[
            ('/camera/camera/depth/image_rect_raw', '/camera/depth/image'),
            ('/camera/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/camera/infra1/image_rect_raw', '/camera/infra1/image_rect_raw'),
            ('/camera/camera/infra2/image_rect_raw', '/camera/infra2/image_rect_raw'),
        ],
        output='screen'
    )

    # =============================================================================
    # 2. Isaac ROS Visual SLAM
    # =============================================================================
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_debug_mode': False,
            'debug_dump_path': '/tmp/cuvslam',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'input_imu_frame': 'camera_gyro_optical_frame',
            'enable_localization_n_mapping': True,
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'invert_odom_to_base_tf': False,
            'image_jitter_threshold_ms': 33.33,
            # 로컬라이제이션 모드 파라미터
            'load_map_folder_path': vslam_map_folder,
            'enable_reading_slam_internals': localization_mode,
            'enable_slam_visualization': True,
        }],
        remappings=[
            ('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
            ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
            ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
            ('stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
            ('visual_slam/imu', '/camera/imu'),
        ]
    )

    # =============================================================================
    # 3. NVBlox 노드
    # =============================================================================
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[{
            'global_frame': 'map',
            'use_sim_time': use_sim_time,
            # 맵핑 파라미터
            'voxel_size': 0.05,
            'esdf_2d': True,
            'esdf_2d_min_height': 0.0,
            'esdf_2d_max_height': 1.0,
            'distance_slice_height': 0.5,
            'mesh_bandwidth_limit_mbps': 100.0,
            # 입력 큐 크기
            'maximum_input_queue_length': 40,
            # 맵 저장/로드
            'map_save_path': nvblox_map_path,
            'load_map_on_start': localization_mode,
            # 레이어 설정
            'esdf_mode': 1,  # 0: 3D, 1: 2D
            'projective_layer_type': 0,  # 0: TSDF
        }],
        remappings=[
            ('depth/image', '/camera/depth/image'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('color/image', '/camera/color/image_raw'),
            ('color/camera_info', '/camera/color/camera_info'),
            ('pose', '/visual_slam/tracking/odometry'),
            ('transform', '/visual_slam/tracking/slam_path'),
        ]
    )

    # =============================================================================
    # 4. NVBlox을 위한 컨테이너
    # =============================================================================
    nvblox_container = Node(
        name='nvblox_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen'
    )
    
    load_composable_nodes = LoadComposableNodes(
        target_container='nvblox_container',
        composable_node_descriptions=[visual_slam_node, nvblox_node]
    )

    # =============================================================================
    # 5. 휠 오도메트리 노드 (사용자 정의)
    # =============================================================================
    #wheel_odometry_node = Node(
    #    package='my_robot_package',  # 실제 패키지 이름으로 변경
    #    executable='wheel_odometry_node',
    #    name='wheel_odometry',
    #    parameters=[{
    #        'wheel_separation': 0.3,  # 바퀴 간격 (미터)
    #        'wheel_radius': 0.05,     # 바퀴 반지름 (미터)
    #        'publish_tf': False,       # VSLAM이 TF를 발행하므로 False
    #    }],
    #    remappings=[
    #        ('odom', '/wheel/odometry'),
    #    ],
    #    output='screen'
    #)
	
    # =============================================================================
    # 6. Robot Localization (센서 퓨전)
    # =============================================================================
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frequency': 30.0,
            'two_d_mode': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            
            # 센서 설정
            'odom0': '/visual_slam/tracking/odometry',
            'odom0_config': [False, False, False,  # x, y, z (사용 안함)
                            False, False, False,  # roll, pitch, yaw (사용 안함)
                            True, True, False,    # vx, vy, vz (x,y 속도만)
                            False, False, True,   # vroll, vpitch, vyaw (회전 속도만)
                            False, False, False], # ax, ay, az (사용 안함)
            'odom0_differential': False,
            'odom0_relative': False,
            
            'odom1': '/wheel/odometry',
            'odom1_config': [True, True, False,   # x, y 위치
                            False, False, True,   # yaw 각도
                            False, False, False,
                            False, False, False,
                            False, False, False],
            'odom1_differential': True,
        }],
        output='screen'
    )

    # =============================================================================
    # 7. Static Transform Publisher (base_link -> camera_link)
    # =============================================================================
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
    )

    # =============================================================================
    # 8. NAV2 (Navigation2)
    # =============================================================================
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('your_robot_package'),  # 실제 패키지명으로 변경
                'config',
                'nav2_params.yaml'
            ]),
        }.items()
    )

    # =============================================================================
    # Launch Description
    # =============================================================================
    ld = LaunchDescription()
    
    # 인자 추가
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_localization_mode_cmd)
    ld.add_action(declare_vslam_map_folder_cmd)
    ld.add_action(declare_nvblox_map_path_cmd)
    
    # 노드 추가 (RPLidar 제거됨)
    ld.add_action(realsense_camera_node)
    ld.add_action(nvblox_container)
    ld.add_action(load_composable_nodes)
    #ld.add_action(wheel_odometry_node)
    ld.add_action(robot_localization_node)
    ld.add_action(base_to_camera_tf)
    ld.add_action(nav2_launch)
    
    return ld
