from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('robot_monitor_mapping'),
                'launch',
                'mapping_launch.py'
            ])
        ])
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('robot_monitor_navigation'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )
    
    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('robot_monitor_ui'),
                'launch',
                'ui_launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        mapping_launch,
        navigation_launch,
        ui_launch,
    ])
