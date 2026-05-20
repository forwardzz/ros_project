from setuptools import setup

package_name = 'robot_monitor_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ui_launch.py']),
        ('share/' + package_name + '/config', ['config/ui_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='UI node for robot monitor system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_ui = robot_monitor_ui.monitor_ui:main',
        ],
    },
)
