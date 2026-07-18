from glob import glob
import os

from setuptools import setup


package_name = "inspection_robot_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "assets"), glob("assets/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="zjy",
    maintainer_email="zjy@example.com",
    description="PyQt5 control panel for the migrated real inspection robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspection_robot_gui = inspection_robot_gui.main:main",
        ],
    },
)
