from glob import glob
import os

from setuptools import setup

package_name = "robot_control_ui"

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
    maintainer="yy",
    maintainer_email="yy@example.com",
    description="Desktop control UI for the tracked robot",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_control_ui = robot_control_ui.robot_control_ui:main",
        ],
    },
)
