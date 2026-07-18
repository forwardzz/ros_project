cmake_minimum_required(VERSION 3.8)
project(inspection_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config behavior_trees rviz maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py)
  ament_add_pytest_test(test_navigation_recovery_config test/test_navigation_recovery_config.py)
endif()

ament_package()
