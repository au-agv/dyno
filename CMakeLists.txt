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

project(dyno)

set(VERSION 1.0.0)

#option(VSG_PATH "VulkanSceneGraph" /opt/vsg/lib/cmake)

# Find package dependencies.
find_package(
  Chrono REQUIRED QUIET
  COMPONENTS Vehicle Irrlicht
  OPTIONAL_COMPONENTS Sensor VSG CONFIG)
if(NOT CHRONO_VSG_FOUND)
  message(
    WARNING
      "Could not find the Chrono::VSG module! The VulkanSceneGraph visualization engine wrapper will not be built."
  )
else()

endif() # if(NOT CHRONO_VSG_FOUND)

if(NOT CHRONO_SENSOR_FOUND)
  message(
    WARNING
      "Could not find the Chrono::Sensor module. The autonomous navigation functionality and the sensor templates will not be built."
  )
endif() # if(NOT CHRONO_SENSOR_FOUND)

# > Configuration and simulation I/O
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(HighFive REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ZeroMQ QUIET)

# > Testing
find_package(GTest REQUIRED)

# Data directories
# ----------------

include(GNUInstallDirs)

if(TRUE)
set(DYNO_DATA_DIR
    ${CMAKE_SOURCE_DIR}/data/)
set(DYNO_VEHICLE_DATA_DIR
    ${CMAKE_SOURCE_DIR}/data/)
else()
set(DYNO_DATA_DIR
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/)
set(DYNO_VEHICLE_DATA_DIR
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/)
endif()

set(CHRONO_DATA_DIRS
    CHRONO_DEFAULT_DATA_DIR=\"${CHRONO_DATA_DIR}\"
    CHRONO_DEFAULT_VEHICLE_DATA_DIR=\"/usr/local/share/chrono/data/vehicle/\"
    DYNO_DATA_DIR=\"${DYNO_DATA_DIR}\"
    DYNO_VEHICLE_DATA_DIR=\"${DYNO_VEHICLE_DATA_DIR}\")
# ----------------

# Core library
# ------------

# > Define source files
# ---------------------
set(DYNO_SRC
    src/drivers/acceleration_based_driver.cpp
    src/drivers/autonomous_driver.cpp
    src/drivers/velocity_based_driver.cpp
    src/environments/asymmetric_friction_functor.cpp
    src/environments/asymmetric_soil_parameters_callback.cpp
    src/environments/double_lane_change.cpp
    src/environments/heightmap_processor.cpp
    src/environments/terrain.cpp
    src/interfaces/json_configuration.cpp
    src/interfaces/json_terrain_output.cpp
    src/interfaces/zmq_socket.cpp
    src/interfaces/json_utilities.cpp
    src/serialization/json_serializer.cpp
    src/math/comparisons.cpp
    src/math/poisson_disk_sampler.cpp
    src/math/poisson_disk_sampler_1d.cpp
    src/math/poisson_disk_sampler_2d.cpp
    src/math/poisson_point_1d.cpp
    src/math/poisson_point_2d.cpp
    src/models/aerodynamic_properties.cpp
    src/models/controller_tuning.cpp
    src/models/olav.cpp
    src/models/speed_controller_tuning.cpp
    src/models/steering_controller_tuning.cpp
    src/models/vehicle.cpp
    src/models/tracked_vehicle.cpp
    src/models/wheeled_vehicle.cpp
    src/serialization/hdf5_serializer.cpp
    src/serialization/serializer.cpp
    src/simulation/collision_checker.cpp
    src/simulation/double_lane_change.cpp
    src/simulation/gear_downshift_latch.cpp
    src/simulation/grade_climbing.cpp
    src/simulation/mobility_map_interpolator.cpp
    src/simulation/obstacle.cpp
    src/simulation/path.cpp
    src/simulation/path_tracker.cpp
    src/simulation/sideslope_stability.cpp
    src/simulation/straight_line_acceleration.cpp
    src/simulation/split_surface.cpp
    src/simulation/terrain_mobility.cpp
    src/simulation/sinusoidal_steering.cpp
    src/simulation/straight_line_braking.cpp
    src/simulation/vehicle_failure_detector.cpp
    src/simulation/vehicle_simulation.cpp
    src/simulation/vehicle_steady_state_detector.cpp
    src/simulation/wall_to_wall.cpp
    src/visualization/irrlicht.cpp
    src/visualization/wrapper.cpp)

# Conditionally append VulkanSceneGraph wrapper to library sources.
if(CHRONO_VSG_FOUND)
  set(DYNO_SRC ${DYNO_SRC} src/visualization/vulkan_scene_graph.cpp)
endif() # if(CHRONO_VSG_FOUND)

# Conditionally append sensor implementations to library sources.
if(CHRONO_SENSOR_FOUND)
  set(DYNO_SRC
      ${DYNO_SRC}
      src/simulation/autonomous_vehicle_simulation.cpp
      src/simulation/autonomous_navigation.cpp
      src/sensors/rgba_camera.cpp
      src/sensors/imu.cpp
      src/sensors/lidar_xyzi.cpp
      src/interfaces/zmq_interface.cpp)
endif() # if(CHRONO_SENSOR_FOUND)

# Add the library target.
add_library(${PROJECT_NAME} SHARED ${DYNO_SRC})

# > Configure compiler
# --------------------
target_compile_definitions(${PROJECT_NAME} PUBLIC ${CHRONO_DATA_DIRS})

if(CHRONO_SENSOR_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DDYNO_HAS_SENSORS_SUPPORT)
endif()

if(CHRONO_VSG_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DDYNO_HAS_VSG_SUPPORT)
endif() # if(CHRONO_VSG_FOUND)

target_compile_options(${PROJECT_NAME} PUBLIC ${CHRONO_CXX_FLAGS})

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
         $<INSTALL_INTERFACE:${INCLUDE_PATH}>
         $<INSTALL_INTERFACE:${INSTALL_PATH}>
         ${CHRONO_INCLUDE_DIRS})

# > Link external dependencies
# ----------------------------
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${Boost_LIBRARIES}
         ${CHRONO_TARGETS}
         ${OpenCV_LIBS}
         HighFive
         nlohmann_json
         spdlog::spdlog_header_only
         stdc++fs
         zmq)

# > Build scenarios
# -----------------
add_subdirectory(src/executors ${CMAKE_BINARY_DIR}/executors)


# > Testing
# ---------

enable_testing()
add_subdirectory(tests)


# > Define install rules
# ----------------------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME})

add_custom_target(uninstall COMMAND xargs rm -vf <install_manifest.txt)

# > Set up CMake package configuration
# ------------------------------------
include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_SET_AND_CHECK_MACRO NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
  ${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        DESTINATION lib/cmake/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})

install(
  DIRECTORY data/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}Targets
       FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake)

