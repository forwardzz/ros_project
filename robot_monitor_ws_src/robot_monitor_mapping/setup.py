from setuptools import setup

package_name = 'robot_monitor_mapping'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapping_launch.py']),
        ('share/' + package_name + '/config', ['config/mapping_params.yaml']),
        ('share/' + package_name + '/config', ['config/mapping.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Mapping node for robot monitor system using RPLIDAR A1 and IMU',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = robot_monitor_mapping.mapping_node:main',
            'map_manager = robot_monitor_mapping.map_manager:main',
        ],
    },
)
