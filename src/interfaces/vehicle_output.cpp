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

#include <dyno/interfaces/vehicle_output.hpp>

namespace DYNO {
namespace Interfaces {

VehicleOutput::VehicleOutput(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    std::shared_ptr<chrono::vehicle::ChDriver> driver,
    std::shared_ptr<chrono::vehicle::ChTerrain> terrain)
    : vehicle_(vehicle),
      driver_(driver),
      terrain_(terrain) {}

bool VehicleOutput::UseNameGenerator() { return use_name_generator_; }

void VehicleOutput::UseNameGenerator(bool use_name_generator) {
    use_name_generator_ = use_name_generator;
}

std::string VehicleOutput::GetFilename() { return filename_; }

void VehicleOutput::SetFilename(std::string filename) {
    SPDLOG_INFO("Setting output filename to {}", filename.c_str());

    filename_ = filename;
}

bool VehicleOutput::AddTimestamp() { return add_timestamp_; }

void VehicleOutput::AddTimestamp(bool add_timestamp) {
    add_timestamp_ = add_timestamp;
}

void VehicleOutput::InitializePath() {
    // Check if the path includes a trailing slash.
    if(!path_.empty() && *path_.rbegin() != '/') path_ += '/';
}

void VehicleOutput::Initialize() {
    InitializePath();
    InitializeTimestamp();
    InitializeFriendlyName();

    std::string subfolder_path;
    if(add_timestamp_) {
        if(use_name_generator_) {
            subfolder_path = filename_ + "-" + date_ + "-" + time_ + "-" +
                adjectives_[random_adjective_index_] + "-" +
                names_[random_name_index_];
        } else {
            subfolder_path = filename_ + "-" + date_;
        }
    } else {
        if(use_name_generator_) {
            subfolder_path = filename_ + "-" +
                adjectives_[random_adjective_index_] + "-" +
                names_[random_name_index_];
        } else {
            subfolder_path = filename_;
        }
    }
    std::string full_path(path_ + subfolder_path);

    if(create_subfolder_) {
        std::experimental::filesystem::create_directory(full_path);
        file_path_ = full_path + "/" + subfolder_path;
    } else {
        file_path_ = subfolder_path;
    }
}

void VehicleOutput::InitializeTimestamp() {
    // Get the current date and time.
    auto current_system_clock = std::chrono::system_clock::now();
    std::time_t current_time =
        std::chrono::system_clock::to_time_t(current_system_clock);
    std::tm current_datetime = *std::localtime(&current_time);

    std::stringstream formatted_date;
    formatted_date << std::put_time(&current_datetime, "%Y-%m-%d");
    date_ = formatted_date.str();

    std::stringstream formatted_time;
    formatted_time << std::put_time(&current_datetime, "%H-%M-%S");
    time_ = formatted_time.str();
}

void VehicleOutput::InitializeFriendlyName() {
    // Initialize a random number generator.
    std::random_device random_device;
    std::mt19937 random_number_generator(random_device());

    // Pick a random name based on an integer index sampled from the random
    // distribution.
    std::uniform_int_distribution<std::mt19937::result_type> names_distribution(
        0,
        names_.size() - 1);
    random_name_index_ = names_distribution(random_number_generator);

    // Pick a random adjective based on an integer index sampled from the random
    // distribution.
    std::uniform_int_distribution<std::mt19937::result_type>
        adjectives_distribution(0, adjectives_.size() - 1);
    random_adjective_index_ = adjectives_distribution(random_number_generator);
}

} // namespace Interfaces
} // namespace DYNO