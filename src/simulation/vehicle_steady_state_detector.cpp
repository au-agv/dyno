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

#include <dyno/simulation/vehicle_steady_state_detector.hpp>

namespace DYNO {
namespace Simulation {

VehicleSteadyStateDetector::VehicleSteadyStateDetector(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    double min_speed,    // m/s
    double tolerance,    // m/s
    double persistence)  // seconds
    : vehicle_(vehicle),
      speed_min_(min_speed),
      tolerance_(tolerance),
      persistence_(persistence) {}

// ---- Set target speed dynamically ----
void VehicleSteadyStateDetector::SetTargetSpeed(double speed) {
    speed_target_ = speed;
    Reset();
}

// ---- Set minimum allowed speed ----
void VehicleSteadyStateDetector::SetMinSpeed(double speed) {
    speed_min_ = speed;
    Reset();
}

// ---- Reset internal buffers ----
void VehicleSteadyStateDetector::Reset() {
    speed_history_.clear();
    timer_ = 0.0;
    steady_state_ = false;
}

// ---- Call every simulation step ----
bool VehicleSteadyStateDetector::CheckSteadyState(double time_step) {
    if (!vehicle_)
        return false;

    double speed = vehicle_->GetChassis()
                       ->GetBody()
                       ->GetPosDt()
                       .x();  // longitudinal speed
    speed_history_.push_back(speed);

    // Keep history only for the duration of the persistence window
    max_history_size_ = static_cast<size_t>(persistence_ / time_step) + 1;
    if (speed_history_.size() > max_history_size_)
        speed_history_.pop_front();

    // Check if speed is above min speed
    if (speed < speed_min_) {
        timer_ = 0.0;
        steady_state_ = false;
        return false;
    }

    // Check if speed has stopped oscillating beyond tolerance
    double max_speed =
        *std::max_element(speed_history_.begin(), speed_history_.end());
    double speed_min_history =
        *std::min_element(speed_history_.begin(), speed_history_.end());

    if ((max_speed - speed_min_history) < tolerance_) {
        timer_ += time_step;
        if (timer_ >= persistence_) {
            steady_state_ = true;
        }
    } else {
        timer_ = 0.0;
        steady_state_ = false;
    }

    return steady_state_;
}

double VehicleSteadyStateDetector::GetCurrentSpeed() const {
    return speed_history_.empty() ? 0.0 : speed_history_.back();
}

bool VehicleSteadyStateDetector::IsSteady() const {
    return steady_state_;
}

}  // namespace Simulation
}  // namespace DYNO
