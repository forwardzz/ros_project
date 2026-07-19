from setuptools import setup
import os
from glob import glob

package_name = "mapping_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yy",
    maintainer_email="yy@example.com",
    description="Mapping bringup with RPLIDAR, IMU, rf2o, EKF, and slam_toolbox",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tracked_motor_driver = mapping_bringup.tracked_motor_driver:main",
            "mission_manager = mapping_bringup.mission_manager:main",
            "thermal_camera_node = mapping_bringup.thermal_camera_node:main",
            "gas_sensor_node = mapping_bringup.gas_sensor_node:main",
            "safety_monitor = mapping_bringup.safety_monitor:main",
            "velocity_safety_gate = mapping_bringup.velocity_safety_gate:main",
            "actual_path_recorder = mapping_bringup.actual_path_recorder:main",
        ],
    },
)
