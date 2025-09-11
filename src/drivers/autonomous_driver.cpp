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

#include <dyno/drivers/autonomous_driver.hpp>

namespace DYNO {
namespace Drivers {

AutonomousDriver::AutonomousDriver(chrono::vehicle::ChVehicle& vehicle,
                                   double steering_angle_max)
    : chrono::vehicle::ChDriver(vehicle),
      steering_angle_max_(steering_angle_max) {
    speed_controller_.Reset(vehicle.GetRefFrame());
}

void AutonomousDriver::SetSpeedControllerGains(double proportional_gain,
                                               double integral_gain,
                                               double derivative_gain) {
    speed_controller_.SetGains(proportional_gain, integral_gain,
                               derivative_gain);
}

void AutonomousDriver::SetSpeedControllerGains(
    const DYNO::Models::SpeedControllerTuning& tuning) {
    speed_controller_.SetGains(tuning.GetProportionalGain(),
                               tuning.GetIntegralGain(),
                               tuning.GetDerivativeGain());
}

void AutonomousDriver::SetBrakeTransitionEffort(double effort) {
    brake_transition_effort_ = effort;
}

void AutonomousDriver::AdvanceSpeedController(double step) {
    speed_controller_output_ = speed_controller_.Advance(
        m_vehicle.GetRefFrame(), target_speed_, m_vehicle.GetChTime(), step);
    chrono::ChClampValue(speed_controller_output_, -1.0, 1.0);
}

void AutonomousDriver::ToThrottleBrakeEffort(double output,
                                             double& throttle_effort,
                                             double& brake_effort) {
    // Parse the output of the speed controller into control efforts.
    if (speed_controller_output_ > 0.0) {
        // Apply throttle to increase speed.
        m_braking = 0.0;
        m_throttle = speed_controller_output_;
    } else if (m_throttle > brake_transition_effort_) {
        // Reduce throttle to regulate speed.
        m_braking = 0.0;
        m_throttle = 1.0 + speed_controller_output_;
    } else {
        // Apply brakes to further regulate speed.
        m_braking = -speed_controller_output_;
        m_throttle = 0.0;
    }
}

void AutonomousDriver::ToSteeringEffort(double steering_angle,
                                        double& steering_effort) {
    steering_effort =
        boost::algorithm::clamp(steering_angle, -steering_angle_max_,
                                steering_angle_max_) /
        steering_angle_max_;
}

void AutonomousDriver::Synchronize(double time) {
    ToThrottleBrakeEffort(speed_controller_output_, m_throttle, m_braking);
    ToSteeringEffort(target_steering_angle_, m_steering);
}

}  // namespace Drivers
}  // namespace DYNO
