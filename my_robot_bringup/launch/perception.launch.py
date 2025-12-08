from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Visual SLAM
    vslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_visual_slam'),
                'launch',
                'isaac_ros_visual_slam_realsense.launch.py'
            ])
        ])
    )
    
    # Nvblox
    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nvblox_examples_bringup'),
                'launch',
                'realsense_example.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        vslam,
        nvblox,
    ])
