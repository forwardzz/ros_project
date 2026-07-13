from launch import LaunchDescription
import fcntl
import os

from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIDAR_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"
BRINGUP_LOCK_PATH = "/tmp/inspection_robot_bringup.lock"
_BRINGUP_LOCK = None


def _acquire_bringup_lock(_context):
    """Allow only one hardware bringup (mapping or navigation) per host."""
    global _BRINGUP_LOCK
    try:
        lock = open(BRINGUP_LOCK_PATH, "w", encoding="utf-8")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock.write("%d navigation\\n" % os.getpid())
        lock.flush()
        _BRINGUP_LOCK = lock
        return []
    except OSError as exc:
        try:
            lock.close()
        except (UnboundLocalError, OSError):
            pass
        return [
            LogInfo(msg="Hardware bringup is already running; refusing duplicate navigation launch: %s" % exc),
            EmitEvent(event=Shutdown(reason="duplicate hardware bringup")),
        ]


def generate_launch_description():
    pkg = FindPackageShare("inspection_robot_bringup")
    sllidar_launch = PathJoinSubstitution([FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"])
    nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    ekf_params = PathJoinSubstitution([pkg, "config", "ekf.yaml"])
    nav_to_pose_bt = PathJoinSubstitution([pkg, "config", "navigate_to_pose_no_spin.xml"])
    nav_through_poses_bt = PathJoinSubstitution([pkg, "config", "navigate_through_poses_no_spin.xml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "sllidar_ros2.rviz"])
    lifecycle_nodes = [
        "map_server", "amcl", "planner_server", "controller_server",
        "bt_navigator", "behavior_server", "smoother_server", "velocity_smoother",
    ]

    return LaunchDescription([
        OpaqueFunction(function=_acquire_bringup_lock),
        SetEnvironmentVariable(
            "CYCLONEDDS_URI",
            "<CycloneDDS xmlns='https://cdds.io/config'><Domain Id='any'>"
            "<Discovery><ParticipantIndex>none</ParticipantIndex>"
            "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
            "</Discovery></Domain></CycloneDDS>",
        ),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("serial_port", default_value=LIDAR_PORT),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("lidar_frame_id", default_value="laser"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="3.1415926"),
        DeclareLaunchArgument("imu_serial_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("imu_z", default_value="0.05"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": LaunchConfiguration("serial_baudrate"),
                "frame_id": LaunchConfiguration("lidar_frame_id"),
            }.items(),
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="base_to_laser",
            arguments=["--x", "0", "--y", "0", "--z", LaunchConfiguration("lidar_z"),
                       "--roll", "0", "--pitch", "0", "--yaw", LaunchConfiguration("lidar_yaw"),
                       "--frame-id", "base_link", "--child-frame-id", LaunchConfiguration("lidar_frame_id")],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="base_to_imu",
            arguments=["--x", "0", "--y", "0", "--z", LaunchConfiguration("imu_z"),
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link", "--child-frame-id", LaunchConfiguration("imu_frame_id")],
        ),
        Node(package="imu_ros2_device", executable="ybimu_driver", name="ybimu_node", output="screen",
             parameters=[{"serial_port": LaunchConfiguration("imu_serial_port")}]),
        Node(
            package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node", name="rf2o_laser_odometry",
            output="screen", arguments=["--ros-args", "--log-level", "error"],
            parameters=[{"laser_scan_topic": "/scan", "odom_topic": "/laser_odom", "publish_tf": False,
                         "base_frame_id": "base_link", "odom_frame_id": "odom", "init_pose_from_topic": "", "freq": 20.0}],
        ),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[ekf_params], remappings=[("odometry/filtered", "/odom")]),
        Node(package="inspection_robot_hardware", executable="tracked_motor_driver", name="motor_driver", output="screen"),
        Node(package="inspection_robot_mission", executable="mission_manager", name="mission_manager", output="screen"),
        Node(package="inspection_robot_mission", executable="actual_path_recorder", name="actual_path_recorder", output="screen"),
        Node(package="inspection_robot_safety", executable="safety_monitor", name="safety_monitor", output="screen",
             parameters=[{"fault_on_undervoltage_seen": False}]),
        Node(package="inspection_robot_safety", executable="velocity_safety_gate", name="velocity_safety_gate", output="screen"),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[nav2_params, {"yaml_filename": LaunchConfiguration("map")}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen", parameters=[nav2_params]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen",
             parameters=[nav2_params], remappings=[("/cmd_vel", "/cmd_vel_nav")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[nav2_params, {"default_nav_to_pose_bt_xml": nav_to_pose_bt,
                                       "default_nav_through_poses_bt_xml": nav_through_poses_bt}]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother", output="screen",
             parameters=[nav2_params], remappings=[("/cmd_vel", "/cmd_vel_nav"), ("/cmd_vel_smoothed", "/cmd_vel_auto")]),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen", arguments=["-d", rviz_config],
             condition=IfCondition(LaunchConfiguration("use_rviz"))),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True, "node_names": lifecycle_nodes}]),
    ])
