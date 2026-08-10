from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LIDAR_PORT = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0"


def generate_launch_description():
    sllidar_launch_path = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"]
    )
    nav2_param_path = PathJoinSubstitution(
        [FindPackageShare("mapping_bringup"), "config", "nav2_params.yaml"]
    )
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("mapping_bringup"), "config", "ekf.yaml"]
    )
    nav_to_pose_bt_path = PathJoinSubstitution(
        [FindPackageShare("mapping_bringup"), "config", "navigate_to_pose_no_spin.xml"]
    )
    nav_through_poses_bt_path = PathJoinSubstitution(
        [FindPackageShare("mapping_bringup"), "config", "navigate_through_poses_no_spin.xml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name="map",
            default_value="",
            description="Path to saved map YAML file (required)",
        ),
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
        DeclareLaunchArgument(
            name="use_rviz",
            default_value="false",
            description="Launch RViz on the robot",
        ),
        DeclareLaunchArgument(
            name="imu_serial_port",
            default_value="/dev/ttyAMA0",
            description="YB IMU serial port",
        ),
        DeclareLaunchArgument(
            name="imu_frame_id",
            default_value="imu_link",
            description="IMU frame id",
        ),
        DeclareLaunchArgument(
            name="imu_z",
            default_value="0.05",
            description="IMU mounting height (m)",
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
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu",
            arguments=[
                "--x", "0", "--y", "0", "--z", LaunchConfiguration("imu_z"),
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link",
                "--child-frame-id", LaunchConfiguration("imu_frame_id"),
            ],
        ),

        Node(
            package="imu_ros2_device",
            executable="ybimu_driver",
            name="ybimu_node",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("imu_serial_port"),
            }],
        ),

        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            arguments=["--ros-args", "--log-level", "error"],
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
            parameters=[ekf_config_path],
            remappings=[
                ("odometry/filtered", "/odom"),
            ],
        ),

        Node(
            package="mapping_bringup",
            executable="tracked_motor_driver",
            name="motor_driver",
            output="screen",
        ),

        Node(
            package="mapping_bringup",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[{"use_tsp": True}],
        ),

        Node(
            package="mapping_bringup",
            executable="actual_path_recorder",
            name="actual_path_recorder",
            output="screen",
        ),

        Node(
            package="mapping_bringup",
            executable="safety_monitor",
            name="safety_monitor",
            output="screen",
            parameters=[{
                "fault_on_undervoltage_seen": False,
            }],
        ),

        Node(
            package="mapping_bringup",
            executable="velocity_safety_gate",
            name="velocity_safety_gate",
            output="screen",
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[nav2_param_path,
                        {"yaml_filename": LaunchConfiguration("map")}],
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_param_path],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_param_path],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_param_path],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_param_path,
                {
                    "default_nav_to_pose_bt_xml": nav_to_pose_bt_path,
                    "default_nav_through_poses_bt_xml": nav_through_poses_bt_path,
                },
            ],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_param_path],
            remappings=[("/cmd_vel", "/cmd_vel_nav")],
        ),

        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_param_path],
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_param_path],
            remappings=[
                ("/cmd_vel", "/cmd_vel_nav"),
                ("/cmd_vel_smoothed", "/cmd_vel_auto"),
            ],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                    "smoother_server",
                    "velocity_smoother",
                ],
            }],
        ),
    ])
