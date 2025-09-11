/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( D ) ( Y ) ( N ) ( O )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +              DYNO: Ground Vehicle Dynamics Validation Toolkit             +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifdef DYNO_HAS_SENSORS_SUPPORT
#include <dyno/simulation/autonomous_navigation.hpp>
#endif
#include <dyno/simulation/double_lane_change.hpp>
#include <dyno/simulation/grade_climbing.hpp>
#include <dyno/simulation/sinusoidal_steering.hpp>
#include <dyno/simulation/split_surface.hpp>
#include <dyno/simulation/straight_line_braking.hpp>
#include <dyno/simulation/terrain_mobility.hpp>
#include <dyno/simulation/wall_to_wall.hpp>

#include <gtest/gtest.h>

#ifdef DYNO_HAS_SENSORS_SUPPORT
TEST(Scenarios, AutonomousNavigationObstacleField) {
    // clang-format on
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario",
         {{"path",
           {{"waypoints",
             nlohmann::json::array({nlohmann::json::array({0.0, 0.0, 0.0}),
                                    nlohmann::json::array({0.0, 0.0, 0.0})})}}},
          {"obstacles",
           {{"enabled", true},
            {"mode", "field"},
            {"positions",
             nlohmann::json::array({nlohmann::json::array({0.0, 0.0, 0.0})})},
            {"sizes", nlohmann::json::array(
                          {nlohmann::json::array({1.0, 1.0, 1.0})})}}}}}};
    // clang-format on

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::AutonomousNavigation>(configuration);
    scenario->Initialize();
    scenario->Step();
}

/*
TEST(Scenarios, AutonomousNavigationPathGates) {
    // clang-format on
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario",
         {{"path",
           {{"waypoints", nlohmann::json::array(
                              {nlohmann::json::array({0.0, 0.0, 0.0}),
                               nlohmann::json::array({10.0, 0.0, 0.0}),
                               nlohmann::json::array({10.0, 10.0, 0.0}),
                               nlohmann::json::array({20.0, 10.0, 0.0})})}}},
          {"obstacles",
           {{"enabled", true},
            {"mode", "path_gates"},
            {"positions",
             nlohmann::json::array({nlohmann::json::array({0.0, 0.0, 0.0})})},
            {"sizes", nlohmann::json::array(
                          {nlohmann::json::array({1.0, 1.0, 1.0})})}}}}}};
    // clang-format on

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::AutonomousNavigation>(configuration);
    scenario->Initialize();
    scenario->Step();
}
*/
#endif

TEST(Scenarios, DoubleLaneChange) {
    nlohmann::json configuration_json = {{"vehicle", {{"name", "olav"}}},
                                         {"scenario", {{"targetSpeed", 10.0}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::DoubleLaneChange>(configuration);
    scenario->Initialize();
    scenario->Step();
}

TEST(Scenarios, GradeClimbing) {
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario", {{"minimumTimeToInitialSpeed", 10.0}, {"timeToMaxGrade", 10.0}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::GradeClimbing>(configuration);
    scenario->Initialize();
    scenario->Step();
}

TEST(Scenarios, SinusoidalSteering) {
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario",
         {{"steering", {{"amplitude", 1.0}, {"frequency", 5.0}}},
          {"targetSpeed", 10.0}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::SinusoidalSteering>(configuration);
    scenario->Initialize();
    scenario->Step();
}

TEST(Scenarios, SplitSurface) {
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario", {{"targetSpeed", 10.0}, {"brakeStart", 10.0}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::SplitSurface>(configuration);
    scenario->Initialize();
    scenario->Step();
}

TEST(Scenarios, StraightLineBraking) {
    nlohmann::json configuration_json = {{"vehicle", {{"name", "olav"}}},
                                         {"scenario", {{"targetSpeed", 10.0}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::StraightLineBraking>(configuration);
    scenario->Initialize();
    scenario->Step();
}

TEST(Scenarios, TerrainMobility) {
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario",
         {{"targetSpeed", 10.0},
          {"waypoints",
           std::string(DYNO_DATA_DIR) + "paths/terrain_mobility_example.json"},
          {"terrain",
           {{"meta",
             std::string(DYNO_DATA_DIR) + "terrains/slope_test/metadata.json"},
            {"path", std::string(DYNO_DATA_DIR) +
                         "terrains/slope_test/elevation.bmp"}}}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::TerrainMobility>(configuration);
    scenario->Initialize();
    scenario->Step();
}

TEST(Scenarios, WallToWall) {
    nlohmann::json configuration_json = {
        {"vehicle", {{"name", "olav"}}},
        {"scenario", {{"targetSteering", 10.0}, {"targetSpeed", 10.0}}}};

    auto configuration = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        configuration_json);
    auto scenario =
        std::make_shared<DYNO::Simulation::WallToWall>(configuration);
    scenario->Initialize();
    scenario->Step();
}
