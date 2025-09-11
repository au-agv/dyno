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

project(dyno_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# TODO: Remove vsgImGui and vsgXchange find_package directives.
# vsgImGui/vsgXchange bug fix
# ---------------------------
#find_package(vsgImGui PATHS /opt/vsg/lib/cmake)
#find_package(vsgXchange PATHS /opt/vsg/lib/cmake)
# ---------------------------

add_subdirectory(src/native)

# Keyboard teleoperation node
# ---------------------------

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/teleop_dbw_keyboard.py DESTINATION lib/${PROJECT_NAME})

# ---------------------------

install(DIRECTORY data launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
