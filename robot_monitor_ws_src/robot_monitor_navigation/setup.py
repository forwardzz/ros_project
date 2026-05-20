from setuptools import setup

package_name = 'robot_monitor_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
        ('share/' + package_name + '/config', ['config/navigation_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Navigation node with TSP+A* algorithm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = robot_monitor_navigation.navigation_node:main',
            'tsp_planner = robot_monitor_navigation.tsp_planner:main',
        ],
    },
)
