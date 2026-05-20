from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_monitor_mapping')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mapping.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        
        Node(
            package='robot_monitor_mapping',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        
        Node(
            package='robot_monitor_mapping',
            executable='map_manager',
            name='map_manager',
            output='screen',
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])
