from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LIDAR_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"
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
        Node(
            package="mapping_bringup",
            executable="thermal_camera_node",
            name="thermal_camera_node",
            output="screen",
        ),
        Node(
            package="mapping_bringup",
            executable="gas_sensor_node",
            name="gas_sensor_node",
            output="screen",
            parameters=[{"serial_port": LaunchConfiguration("gas_serial_port")}],
        ),
    ])
