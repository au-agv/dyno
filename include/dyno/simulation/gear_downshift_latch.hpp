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

#include <memory>

#include <chrono_vehicle/ChVehicle.h>

namespace DYNO {
namespace Simulation {

enum class GearLatchState { NORMAL, LATCHED };

enum class LatchReason {
        NONE,
        DOWNSHIFT,
        STEADY_DECEL
    };

class GearDownshiftLatch {
   public:
    explicit GearDownshiftLatch(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle);

    void Enforce(double step_size);

    void Reset();

    GearLatchState GetState() const;

    LatchReason GetLatchReason() const { return latch_reason_; }

   private:
    GearLatchState state_ = GearLatchState::NORMAL;

    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;

    int GetCurrentGear() const;

    void SetGear(int gear) const;

    int previous_gear_ = -1;

    std::vector<double> speed_window_;
    size_t window_size_ = 1;
    size_t window_index_ = 0;
    double window_total_time_ = 0.0;
    double avg_window_ = 5.0;  // seconds

    // Speed-based latch tracking
    double previous_speed_ = -1.0;
    double steady_decel_time_ = 0.0;
    double max_decel_ = 0.1;
    double min_duration_ = 3.0;
    double GetSpeed() const;
    LatchReason latch_reason_ = LatchReason::NONE;

    int latched_gear_ = -1;



};

}  // namespace Simulation
}  // namespace DYNO
