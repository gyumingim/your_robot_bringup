from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # 1. Isaac ROS Visual SLAM with RealSense
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_visual_slam'),
                'launch',
                'isaac_ros_visual_slam_realsense.launch.py'
            ])
        ])
    )
    
    # 2. Nvblox for mapping and collision avoidance
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nvblox_examples_bringup'),
                'launch',
                'realsense_example.launch.py'
            ])
        ])
    )
    
    # 3. Nav2 for autonomous navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )
    
    return LaunchDescription([
        vslam_launch,
        nvblox_launch,
        nav2_launch,
    ])
