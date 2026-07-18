from setuptools import setup


package_name = "inspection_robot_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zjy",
    maintainer_email="zjy@example.com",
    description="Inspection mission and region execution for the migrated robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager = inspection_robot_mission.mission_manager:main",
            "actual_path_recorder = inspection_robot_mission.actual_path_recorder:main",
        ],
    },
)
