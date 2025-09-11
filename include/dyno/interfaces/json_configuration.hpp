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

#pragma once

#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <spdlog/spdlog.h>
#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include <dyno/interfaces/json_utilities.hpp>

namespace DYNO {
namespace Interfaces {

class JSONConfiguration {
   public:
    JSONConfiguration();

    JSONConfiguration(const nlohmann::json& dictionary);

    /**
     * @brief Construct a new JSON configuration object.
     */
    JSONConfiguration(const std::string& path);

    void ParseString(const std::string& options);

    void Parse(int argc, char* argv[]);

    void Parse(const boost::program_options::parsed_options& options);

    template <class T>
    T GetValue(const std::string& keys) const {
        return JSON::GetValue<T>(configuration_, keys);
    }

    template <class T>
    T GetValue(const std::string& keys, T default_value) const {
        return JSON::GetValue<T>(configuration_, keys, default_value);
    }

    const std::string& GetScenario() const;

    void ReadConfiguration();

   private:
    /**
     * @brief Shared pointer to the Boost::Program_options descriptions.
     */
    std::shared_ptr<boost::program_options::options_description> description_;

    void AddDescription();

    /**
     * @brief Shared pointer to the Boost::Program_options variables.
     */
    std::shared_ptr<boost::program_options::variables_map> variables_;

    /**
     * @brief Fully-qualified path to the JSON configuration file.
     */
    std::string path_;

    std::string scenario_;

    const std::string templates_path_ =
        std::string(DYNO_DATA_DIR) + "/scenarios/";

    std::map<std::string, std::string> default_configurations_ = {
        {"doubleLaneChange",
         templates_path_ + "double_lane_change_default.json"},
        {"terrainMobility", templates_path_ + "mobility_mapping_default.json"},
        {"autonomousNavigation",
         templates_path_ + "single_obstacle_avoidance_default.json"},
        {"sinusoidalSteering",
         templates_path_ + "sinusoidal_steering_default.json"},
        {"splitSurface", templates_path_ + "split_surface_default.json"},
        {"straightLineAcceleration",
         templates_path_ + "straight_line_acceleration_default.json"},
        {"straightLineBraking",
         templates_path_ + "straight_line_braking_default.json"},
        {"wallToWallTurn", templates_path_ + "wall_to_wall_turn_default.json"}};

    /**
     * @brief Parsed JSON configuration dictionary
     */
    nlohmann::json configuration_;

    void Initialize();
};

}  // namespace Interfaces
}  // namespace DYNO
