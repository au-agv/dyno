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

#include <dyno/drivers/acceleration_based_driver.hpp>

namespace DYNO {
namespace Drivers {

AccelerationBasedDriver::AccelerationBasedDriver(
    chrono::vehicle::ChVehicle& vehicle, double steering_angle_max)
    : AutonomousDriver(vehicle, steering_angle_max) {
    speed_controller_.Reset(vehicle.GetRefFrame());
}

void AccelerationBasedDriver::Advance(double step) {
    target_speed_ = last_speed_ + target_acceleration_ * step;
    AdvanceSpeedController(step);

    target_steering_angle_ = boost::algorithm::clamp(
        last_steering_angle_ + target_steering_rate_ * step,
        -steering_angle_max_, steering_angle_max_);

    // Store the current speed and steering angle for the calculation of the
    // acceleration and steering rate at the next step.
    last_speed_ = target_speed_;
    last_steering_angle_ = target_steering_angle_;
}

void AccelerationBasedDriver::Synchronize(double time) {
    AutonomousDriver::Synchronize(time);
}

void AccelerationBasedDriver::SetAcceleration(double acceleration) {
    target_acceleration_ = acceleration;
}

void AccelerationBasedDriver::SetSteeringRate(double steering_rate) {
    target_steering_rate_ = steering_rate;
}

void AccelerationBasedDriver::Reset(double speed, double steering_angle) {
    last_speed_ = speed;
    last_steering_angle_ = steering_angle;
}

}  // namespace Drivers
}  // namespace DYNO
