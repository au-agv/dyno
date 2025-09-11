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

#include <dyno/interfaces/json_configuration.hpp>

namespace DYNO {
namespace Interfaces {

JSONConfiguration::JSONConfiguration() : path_("") {
    Initialize();
}

JSONConfiguration::JSONConfiguration(const nlohmann::json& dictionary)
    : path_(""), configuration_(dictionary) {
    Initialize();
}

JSONConfiguration::JSONConfiguration(const std::string& path) : path_(path) {
    Initialize();
}

void JSONConfiguration::Initialize() {
    description_ =
        std::make_shared<boost::program_options::options_description>();
    variables_ = std::make_shared<boost::program_options::variables_map>();
    AddDescription();
}

void JSONConfiguration::AddDescription() {
    description_ =
        std::make_shared<boost::program_options::options_description>(
            "Allowed options");
    description_->add_options()("help", "Print help message")(
        "scenario", boost::program_options::value<std::string>(),
        "Scenario to be simulated.")(
        "options", boost::program_options::value<std::string>(),
        "Path to JSON options file.")(
        "log", boost::program_options::value<std::string>(), "Log level.");
}

void JSONConfiguration::ParseString(const std::string& options) {
    Parse(boost::program_options::command_line_parser(
              boost::program_options::split_unix(options))
              .options(*description_)
              .run());
}

void JSONConfiguration::Parse(
    const boost::program_options::parsed_options& options) {
    boost::program_options::store(options, *variables_);
    boost::program_options::notify(*variables_);

    // Show the help message and quit if option "--help" is passed to the
    // executable.
    if (variables_->count("help")) {
        std::cout << "Usage: " << " [options]\n";
        std::cout << description_;
        exit(EXIT_SUCCESS);
    }

    // Turn off logging or set it to the default verbosity value.
    if (variables_->count("silent")) {
        spdlog::set_level(spdlog::level::off);
    } else {
        spdlog::set_level(spdlog::level::warn);
    }

    // Parse options from the JSON file passed to the executable via the
    // "--options" flag.
    if (variables_->count("scenario")) {
        scenario_ = (*variables_)["scenario"].as<std::string>();
        SPDLOG_INFO("Loading scenario \"{}\"", path_);
    } else {
        SPDLOG_ERROR("No scenario specified!");
        throw std::invalid_argument("No scenario specified!");
    }
}

void JSONConfiguration::Parse(int argc, char* argv[]) {
    Parse(
        boost::program_options::parse_command_line(argc, argv, *description_));
}

void JSONConfiguration::ReadConfiguration() {
    // Parse options from the JSON file passed to the executable via the
    // "--options" flag.
    if (path_ != "") {
        SPDLOG_INFO("Loading options file at \"{}\"", path_);
    } else if (path_ == "" && variables_->count("options")) {
        path_ = (*variables_)["options"].as<std::string>();
        SPDLOG_INFO("Loading options file at \"{}\"", path_);
    } else {
        SPDLOG_WARN("No configuration file provided, using default ...");
        path_ = default_configurations_[scenario_];
    }

    configuration_ =
        nlohmann::json::parse(std::ifstream(path_), nullptr, true, true);
}

const std::string& JSONConfiguration::GetScenario() const {
    return scenario_;
}

}  // namespace Interfaces
}  // namespace DYNO
