import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # imu 驱动
    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        output='screen'
    )

    # TF 广播：IMU -> base_link
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_tf',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0.1', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )


    # EKF 融合节点
    ekf_config_path = os.path.join(
        get_package_share_directory('imu_ros2_device'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )


    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom',
            'publish_tf' : False,             
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0
        }],
        remappings=[
            ('/tf', '/tf_dummy'),
            ('/tf_static', '/tf_static_dummy')
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('imu_ros2_device'), 'rviz', 'ybimu.rviz')]
    )

    return LaunchDescription([
        device_node,
        tf_node,
        ekf_node,
        rviz_node,
        rf2o_node,
    ])