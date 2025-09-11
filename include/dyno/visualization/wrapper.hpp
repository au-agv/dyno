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

#include <chrono_vehicle/ChVehicle.h>
#include <spdlog/spdlog.h>

#include <dyno/interfaces/json_configuration.hpp>

namespace DYNO {
namespace Visualization {

class Wrapper {
   public:
    Wrapper(std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
            std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    virtual void Initialize() = 0;

    virtual void Advance(const double& time) = 0;

    virtual void Synchronize(
        const double& time,
        const chrono::vehicle::DriverInputs& driver_inputs) = 0;

   protected:
    // Properties
    // ----------

    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;

    /** @brief Visualization options JSON document. */
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration_;

    // Options
    // -------

    void ParseOptions();

    /** @brief Whether or not the visualization is enabled. */
    bool is_enabled_ = false;

    // Time
    // ----
    /** @brief Frames per second for the visualization (measured with respect to
     * system clock). */
    double frame_rate_ = 30.0;

    /** @brief Current simulation time. */
    double simulation_time_ = 0.0;

    bool IsTimeToRender();

    /** @brief System clock at the latest visualization step. */
    double last_visualization_time_;

    int frame_index_ = 0;

    bool save_output_ = false;

    std::string output_frames_path_ = "./";

    std::string GetPaddedFrameIndex() const;

    bool use_system_clock_ = false;

    double current_time_;

    double GetTimeInSeconds() {
        auto current_time = std::chrono::system_clock::now();
        auto duration_in_seconds =
            std::chrono::duration<double>(current_time.time_since_epoch());

        double num_seconds = duration_in_seconds.count();
        return num_seconds;
    }
};

}  // namespace Visualization
}  // namespace DYNO
