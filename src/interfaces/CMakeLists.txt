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

message(STATUS "Configuring UDP communication module...")

message(STATUS "Building the UDP server ...")

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_udp udp_socket.cpp)

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
target_link_libraries(${PROJECT_NAME}_udp ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}_udp PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                     ${CMAKE_BINARY_DIR}/bin)

# > Define install rules
# ----------------------
install(
  TARGETS ${PROJECT_NAME}_udp
  DESTINATION bin
  RENAME ${PROJECT_NAME})