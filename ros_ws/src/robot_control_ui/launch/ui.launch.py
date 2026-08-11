from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace_path",
            default_value="/home/yy/ros2_ws",
            description="ROS 2 workspace path on the robot",
        ),
        DeclareLaunchArgument(
            "remote_user",
            default_value="yy",
            description="SSH user for the robot",
        ),
        DeclareLaunchArgument(
            "remote_host",
            default_value="192.168.43.31",
            description="SSH host for the robot",
        ),
        DeclareLaunchArgument(
            "ros_setup_path",
            default_value="/opt/ros/jazzy/setup.bash",
            description="ROS setup script path on the robot",
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="/home/yy/ros2_ws/map_name.yaml",
            description="Default map yaml path for navigation",
        ),
        Node(
            package="robot_control_ui",
            executable="robot_control_ui",
            name="robot_control_ui",
            output="screen",
            parameters=[{
                "workspace_path": LaunchConfiguration("workspace_path"),
                "remote_user": LaunchConfiguration("remote_user"),
                "remote_host": LaunchConfiguration("remote_host"),
                "ros_setup_path": LaunchConfiguration("ros_setup_path"),
                "map_path": LaunchConfiguration("map_path"),
            }],
        ),
    ])
