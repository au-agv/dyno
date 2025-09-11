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

#include <chrono/utils/ChUtils.h>
#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/utils/ChSpeedController.h>
#include <boost/algorithm/clamp.hpp>

#include <dyno/models/speed_controller_tuning.hpp>

namespace DYNO {
namespace Drivers {

class AutonomousDriver : public chrono::vehicle::ChDriver {
   public:
    AutonomousDriver(chrono::vehicle::ChVehicle& vehicle,
                     double steering_angle_max);

    virtual void Advance(double step) = 0;

    virtual void Synchronize(double time);

    void SetSpeedControllerGains(double proportional_gain, double integral_gain,
                                 double derivative_gain);

    void SetSpeedControllerGains(
        const DYNO::Models::SpeedControllerTuning& tuning);

    void SetBrakeTransitionEffort(double effort);

   protected:
    void AdvanceSpeedController(double step);

    void ToThrottleBrakeEffort(double output, double& throttle_effort,
                               double& brake_effort);

    void ToSteeringEffort(double steering_angle, double& steering_effort);

    void SynchronizeEfforts();

    /** @brief PID controller for controlling the speed of the vehicle through
     * throttle/brake actuation. */
    chrono::vehicle::ChSpeedController speed_controller_;

    double target_speed_;

    /** @brief The output effort from the latest PID speed controller tick. */
    double speed_controller_output_ = 0.0;

    /** @brief The PID speed controller output effort below which efforts are
     * piped to brake instead of throttle. */
    double brake_transition_effort_ = 0.2;

    /** @brief The maximum steering angle in degrees. This value is used to
     * remap control inputs to efforts in the compact [-1.0, 1.0]. */
    double steering_angle_max_;

    /** @brief The target steering angle to be set at the next driver
     * synchronization step. */
    double target_steering_angle_ = 0.0;
};

}  // namespace Drivers
}  // namespace DYNO
