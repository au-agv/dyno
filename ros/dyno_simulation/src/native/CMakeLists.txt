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

# NATIVE SIMULATOR INTERFACE NODE
# -------------------------------
message(STATUS "Building native simulator interface node ...")

# > Find external packages
# ------------------------
find_package(ackermann_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dyno REQUIRED)
find_package(dyno_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_native_simulator_interface_node
               executor.cpp node.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}_native_simulator_interface_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
ament_target_dependencies(
  ${PROJECT_NAME}_native_simulator_interface_node
  ackermann_msgs
  cv_bridge
  dyno_interfaces
  nav_msgs
  OpenCV
  rclcpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_ros)

target_link_libraries(${PROJECT_NAME}_native_simulator_interface_node dyno)

# > Define install rules
# ----------------------
install(TARGETS ${PROJECT_NAME}_native_simulator_interface_node
        DESTINATION lib/${PROJECT_NAME})
