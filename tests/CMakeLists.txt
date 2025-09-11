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

include(GoogleTest)

add_executable(${PROJECT_NAME}_test_parsers
               test_parsers.cpp)
target_link_libraries(${PROJECT_NAME}_test_parsers
                        ${PROJECT_NAME}
                      GTest::gtest_main)
gtest_discover_tests(${PROJECT_NAME}_test_parsers)

add_executable(${PROJECT_NAME}_test_scenarios
               test_scenarios.cpp)
target_link_libraries(${PROJECT_NAME}_test_scenarios
                        ${PROJECT_NAME}
                      GTest::gtest_main)
if(CHRONO_SENSOR_FOUND)
  target_compile_definitions(${PROJECT_NAME}_test_scenarios PUBLIC -DDYNO_HAS_SENSORS_SUPPORT)
endif()
gtest_discover_tests(${PROJECT_NAME}_test_scenarios)

add_executable(${PROJECT_NAME}_test_cosimulation
               test_cosimulation.cpp)
target_link_libraries(${PROJECT_NAME}_test_cosimulation
                        ${PROJECT_NAME}
                      GTest::gtest_main)
gtest_discover_tests(${PROJECT_NAME}_test_cosimulation)

add_executable(${PROJECT_NAME}_test_functors
               test_functors.cpp)
target_link_libraries(${PROJECT_NAME}_test_functors
                        ${PROJECT_NAME}
                      GTest::gtest_main)
gtest_discover_tests(${PROJECT_NAME}_test_functors)


add_executable(${PROJECT_NAME}_test_generators
               test_generators.cpp)
target_link_libraries(${PROJECT_NAME}_test_generators
                        ${PROJECT_NAME}
                      GTest::gtest_main)
gtest_discover_tests(${PROJECT_NAME}_test_generators)
