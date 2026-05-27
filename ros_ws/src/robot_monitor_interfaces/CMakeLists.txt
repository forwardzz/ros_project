cmake_minimum_required(VERSION 3.8)
project(robot_monitor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GasData.msg"
  "msg/InspectionPoint.msg"
  "msg/RobotSafetyStatus.msg"
  "srv/LoadMap.srv"
  "srv/ConfirmInspectionPoints.srv"
  "srv/Localize.srv"
  "srv/StartNavigation.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
