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

message(STATUS "Configuring scenarios module...")

# > Add targets
# -------------
add_executable(${PROJECT_NAME}_cli dyno.cpp)
set_property(TARGET ${PROJECT_NAME}_cli PROPERTY OUTPUT_NAME ${PROJECT_NAME})

# > Include directories
# ---------------------
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

# > Link external dependencies
# ----------------------------
target_link_libraries(${PROJECT_NAME}_cli ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME}_cli PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                     ${CMAKE_BINARY_DIR}/bin)

# > Define install rules
# ----------------------
install(
  TARGETS ${PROJECT_NAME}_cli
  DESTINATION bin
  RENAME ${PROJECT_NAME})
