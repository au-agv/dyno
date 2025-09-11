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

#include <chrono/functions/ChFunctionSine.h>
#include <chrono/utils/ChOpenMP.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

class SinusoidalSteering : public VehicleSimulation {
  public:
    SinusoidalSteering(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

  protected:
    void InitializeDriver() override;
    void InitializeTerrain() override;
    void WriteMetadata() override;
    void PreSynchronizationHook() override;
    void PostSynchronizationHook() override;

  private:
    // Slope
    // --------------------------------------------------------------------- //
    double current_slope_ = 0.0;
    double target_slope_;
    bool slope_reached_ = false;
    bool use_slope_ramp_ = true;
    double amplitude_;
    double frequency_;
    double time_to_max_slope_;
    std::shared_ptr<chrono::ChFunctionSineStep> slope_ramp_;

    double steering_time_counter_ = 0.0;
    double target_speed_;
    // --------------------------------------------------------------------- //

    // Throttle
    // --------------------------------------------------------------------- //
    std::shared_ptr<chrono::ChFunctionSine> steering_input_;
    // --------------------------------------------------------------------- //

    // Steady state speed
    // --------------------------------------------------------------------- //
    double minimum_time_;
    double mean_speed_ = 0.0;
    double averaging_window_;
    double steady_time_ = 0.0;
    double negative_time_ = 0.0;
    bool is_successful_ = true;

    bool is_steering_ = false;
    // --------------------------------------------------------------------- //

    // Terrain type
    // --------------------------------------------------------------------- //
    std::string terrain_type_;
    // --------------------------------------------------------------------- //
};

} // namespace Simulation
} // namespace DYNO