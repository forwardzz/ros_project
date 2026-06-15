from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIDAR_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"


def generate_launch_description():
    cyclone_uri = (
        "<CycloneDDS xmlns='https://cdds.io/config'>"
        "<Domain Id='any'>"
        "<Discovery>"
        "<ParticipantIndex>none</ParticipantIndex>"
        "<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>"
        "</Discovery>"
        "</Domain>"
        "</CycloneDDS>"
    )
    sllidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"]
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare("mapping_bringup"), "config", "slam.yaml"]
    )

    return LaunchDescription([
        SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_uri),
        DeclareLaunchArgument(
            name="serial_port",
            default_value=LIDAR_PORT,
            description="RPLIDAR serial port",
        ),
        DeclareLaunchArgument(
            name="serial_baudrate",
            default_value="115200",
            description="RPLIDAR baudrate",
        ),
        DeclareLaunchArgument(
            name="lidar_frame_id",
            default_value="laser",
            description="Lidar frame id",
        ),
        DeclareLaunchArgument(
            name="lidar_z",
            default_value="0.15",
            description="Lidar mounting height (m)",
        ),
        DeclareLaunchArgument(
            name="lidar_yaw",
            default_value="3.1415926",
            description="Lidar yaw relative to base_link (rad)",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
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
                "--frame-id", "base_link",
                "--child-frame-id", LaunchConfiguration("lidar_frame_id"),
            ],
        ),

        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            arguments=["--ros-args", "--log-level", "error"],
            parameters=[{
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom",
                "publish_tf": True,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 20.0,
            }],
        ),

        Node(
            package="mapping_bringup",
            executable="tracked_motor_driver",
            name="motor_driver",
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": slam_config_path,
            }.items(),
        ),
    ])
