# cmake-format: off
 #############################################################################
 #                            _     _     _     _                            #
 #                           / \   / \   / \   / \                           #
 #                          ( D ) ( Y ) ( N ) ( O )                          #
 #                           \_/   \_/   \_/   \_/                           #
 #                                                                           #
 #              DYNO: Ground Vehicle Dynamics Validation Toolkit             #
 #############################################################################
# cmake-format: on

cmake_minimum_required(VERSION 3.8)

project(dyno_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  # Messages
  msg/Obstacle.msg
  msg/ObstacleArray.msg
  msg/VehicleState.msg
  msg/Status.msg
  DEPENDENCIES
  geometry_msgs
  std_msgs)

ament_package()
