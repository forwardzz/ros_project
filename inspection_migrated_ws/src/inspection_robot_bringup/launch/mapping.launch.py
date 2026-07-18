import fcntl
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIDAR_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BRINGUP_LOCK_PATH = "/tmp/inspection_robot_bringup.lock"
_BRINGUP_LOCK = None


def _acquire_bringup_lock(_context):
    """Allow only one hardware bringup (mapping or navigation) per host."""
    global _BRINGUP_LOCK
    try:
        lock = open(BRINGUP_LOCK_PATH, "w", encoding="utf-8")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        lock.write("%d mapping\\n" % os.getpid())
        lock.flush()
        _BRINGUP_LOCK = lock
        return []
    except OSError as exc:
        try:
            lock.close()
        except (UnboundLocalError, OSError):
            pass
        return [
            LogInfo(msg="Hardware bringup is already running; refusing duplicate mapping launch: %s" % exc),
            EmitEvent(event=Shutdown(reason="duplicate hardware bringup")),
        ]


def generate_launch_description():
    sllidar_launch = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    slam_params = PathJoinSubstitution(
        [FindPackageShare("inspection_robot_bringup"), "config", "slam.yaml"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("inspection_robot_bringup"), "config", "ekf.yaml"]
    )

    return LaunchDescription([
        OpaqueFunction(function=_acquire_bringup_lock),
        SetEnvironmentVariable(
            "CYCLONEDDS_URI",
            "<CycloneDDS xmlns='https://cdds.io/config'><Domain Id='any'>"
            "<Discovery><ParticipantIndex>none</ParticipantIndex>"
            "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
            "</Discovery></Domain></CycloneDDS>",
        ),
        DeclareLaunchArgument("serial_port", default_value=LIDAR_PORT),
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("lidar_frame_id", default_value="laser"),
        DeclareLaunchArgument("lidar_z", default_value="0.15"),
        DeclareLaunchArgument("lidar_yaw", default_value="3.1415926"),
        DeclareLaunchArgument("imu_serial_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("imu_z", default_value="0.05"),
        DeclareLaunchArgument("motor_pair", default_value="disabled"),
        DeclareLaunchArgument("left_inverted", default_value="false"),
        DeclareLaunchArgument("right_inverted", default_value="false"),
        DeclareLaunchArgument("actuation_enabled", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": LaunchConfiguration("serial_baudrate"),
                "frame_id": LaunchConfiguration("lidar_frame_id"),
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser",
            arguments=[
                "--x", "0", "--y", "0", "--z", LaunchConfiguration("lidar_z"),
                "--roll", "0", "--pitch", "0", "--yaw", LaunchConfiguration("lidar_yaw"),
                "--frame-id", "base_link", "--child-frame-id", LaunchConfiguration("lidar_frame_id"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu",
            arguments=[
                "--x", "0", "--y", "0", "--z", LaunchConfiguration("imu_z"),
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", LaunchConfiguration("imu_frame_id"),
            ],
        ),
        Node(
            package="imu_ros2_device",
            executable="ybimu_driver",
            name="ybimu_node",
            output="screen",
            parameters=[{"serial_port": LaunchConfiguration("imu_serial_port")}],
        ),
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[{
                "laser_scan_topic": "/scan",
                "odom_topic": "/laser_odom",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            }],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params],
            remappings=[("odometry/filtered", "/odom")],
        ),
        Node(
            package="inspection_robot_hardware",
            executable="tracked_motor_driver",
            name="motor_driver",
            output="screen",
            parameters=[{
                "motor_pair": LaunchConfiguration("motor_pair"),
                "left_inverted": LaunchConfiguration("left_inverted"),
                "right_inverted": LaunchConfiguration("right_inverted"),
                "actuation_enabled": LaunchConfiguration("actuation_enabled"),
            }],
        ),
        Node(
            package="inspection_robot_safety",
            executable="safety_monitor",
            name="safety_monitor",
            output="screen",
            parameters=[{"fault_on_undervoltage_seen": False}],
        ),
        Node(
            package="inspection_robot_safety",
            executable="velocity_safety_gate",
            name="velocity_safety_gate",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"use_sim_time": "false", "slam_params_file": slam_params}.items(),
        ),
    ])
