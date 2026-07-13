from glob import glob
import os

from setuptools import setup


package_name = "inspection_robot_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zjy",
    maintainer_email="zjy@example.com",
    description="Real sensor and GPIO/PWM adapters",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tracked_motor_driver = inspection_robot_hardware.tracked_motor_driver:main",
            "gas_sensor_node = inspection_robot_hardware.gas_sensor_node:main",
            "thermal_camera_node = inspection_robot_hardware.thermal_camera_node:main",
        ],
    },
)
