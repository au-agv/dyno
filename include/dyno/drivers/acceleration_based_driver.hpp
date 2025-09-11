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

#include <dyno/drivers/autonomous_driver.hpp>

namespace DYNO {
namespace Drivers {

class AccelerationBasedDriver : public AutonomousDriver {
   public:
    AccelerationBasedDriver(chrono::vehicle::ChVehicle& vehicle,
                            double steering_angle_max);

    void Advance(double step) override;

    void Synchronize(double time) override;

    void SetControls(double acceleration, double steering_rate);

    void SetAcceleration(double acceleration);

    void SetSteeringRate(double steering_rate);

    void Reset(double speed, double steering_angle);

   private:
    double target_acceleration_ = 0.0;

    double last_speed_ = 0.0;

    /** @brief The target steering rate to be maintained. This value is only
     * used when the controller mode is set to "steering rate". */
    double target_steering_rate_ = 0.0;

    /** @brief The steering angle in degrees at the last driver synchronization
     * step. This value is used to compute the rate of change between steps. */
    double last_steering_angle_ = 0.0;
};

}  // namespace Drivers
}  // namespace DYNO
