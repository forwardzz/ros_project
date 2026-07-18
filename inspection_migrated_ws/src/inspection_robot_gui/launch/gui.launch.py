from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # These paths are consumed by the SSH launch manager on the Raspberry Pi,
    # not by a local source checkout running the desktop GUI.
    default_workspace = "/home/yy/inspection_migrated_ws"
    default_map = "/home/yy/inspection_migrated_ws/maps/inspection_map.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "workspace",
            default_value=default_workspace,
            description="Workspace used by the GUI launch controls",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Default map YAML used by the navigation controls",
        ),
        DeclareLaunchArgument(
            "ros_setup",
            default_value="/opt/ros/jazzy/setup.bash",
            description="ROS setup.bash sourced by GUI launch controls",
        ),
        DeclareLaunchArgument("remote_user", default_value="yy"),
        DeclareLaunchArgument("remote_host", default_value="192.168.43.24"),
        Node(
            package="inspection_robot_gui",
            executable="inspection_robot_gui",
            name="inspection_robot_gui",
            output="screen",
            parameters=[{
                "workspace_path": LaunchConfiguration("workspace"),
                "map_path": LaunchConfiguration("map"),
                "ros_setup_path": LaunchConfiguration("ros_setup"),
                "remote_user": LaunchConfiguration("remote_user"),
                "remote_host": LaunchConfiguration("remote_host"),
            }],
        ),
    ])
