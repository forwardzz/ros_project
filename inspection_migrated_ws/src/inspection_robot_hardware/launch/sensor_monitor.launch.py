from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


GAS_PORT = "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0"


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

    return LaunchDescription([
        SetEnvironmentVariable("CYCLONEDDS_URI", cyclone_uri),
        DeclareLaunchArgument(
            name="gas_serial_port",
            default_value=GAS_PORT,
            description="Gas sensor serial port",
        ),
        DeclareLaunchArgument(
            name="use_thermal",
            default_value="true",
            description="Start the MLX90640 thermal camera",
        ),
        DeclareLaunchArgument(
            name="use_gas",
            default_value="false",
            description="Start the optional gas sensor",
        ),
        Node(
            package="inspection_robot_hardware",
            executable="thermal_camera_node",
            name="thermal_camera_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_thermal")),
        ),
        Node(
            package="inspection_robot_hardware",
            executable="gas_sensor_node",
            name="gas_sensor_node",
            output="screen",
            parameters=[{"serial_port": LaunchConfiguration("gas_serial_port")}],
            condition=IfCondition(LaunchConfiguration("use_gas")),
        ),
    ])
