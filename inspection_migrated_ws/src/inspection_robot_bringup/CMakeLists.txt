cmake_minimum_required(VERSION 3.8)
project(inspection_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config behavior_trees rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
