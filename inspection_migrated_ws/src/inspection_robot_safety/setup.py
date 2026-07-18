from glob import glob
import os

from setuptools import setup


package_name = "inspection_robot_safety"

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
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="zjy",
    maintainer_email="zjy@example.com",
    description="Safety supervisor and velocity arbitration",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "safety_monitor = inspection_robot_safety.safety_monitor:main",
            "velocity_safety_gate = inspection_robot_safety.velocity_safety_gate:main",
        ],
    },
)
