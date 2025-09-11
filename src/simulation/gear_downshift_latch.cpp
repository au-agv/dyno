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

#include <dyno/simulation/gear_downshift_latch.hpp>

namespace DYNO {
namespace Simulation {

GearDownshiftLatch::GearDownshiftLatch(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle)
    : vehicle_(std::move(vehicle)) {

    // Window size is number of steps (at least 1)
    window_size_ =
        std::max(size_t(1), size_t(avg_window_ / 1e-3));  // default step ~1ms
    speed_window_.resize(window_size_, -1.0);
}

int GearDownshiftLatch::GetCurrentGear() const {
    return vehicle_->GetTransmission()->GetCurrentGear();
}

void GearDownshiftLatch::Enforce(double step_size) {
    const int current_gear = GetCurrentGear();
    const double speed = GetSpeed();

    // --- Initialization ---
    if (previous_gear_ < 0) {
        previous_gear_ = current_gear;
        std::fill(speed_window_.begin(), speed_window_.end(), speed);
        return;
    }

    const bool upshift = current_gear > previous_gear_;
    const bool downshift = current_gear < previous_gear_;

    bool latch_this_step = false;
    LatchReason reason_this_step = LatchReason::NONE;

    // --- Gear-based latch ---
    if (downshift && state_ == GearLatchState::NORMAL) {
        latch_this_step = true;
        reason_this_step = LatchReason::DOWNSHIFT;
    }

    // --- Update speed window ---
    speed_window_[window_index_] = speed;
    window_index_ = (window_index_ + 1) % window_size_;
    window_total_time_ = window_size_ * step_size;

    // --- Speed-based latch ---
    if (state_ == GearLatchState::NORMAL) {
        double oldest_speed = speed_window_[window_index_];
        double avg_decel = (oldest_speed - speed) / window_total_time_;

        const bool steady_small_decel =
            avg_decel > 0.0 && avg_decel <= max_decel_;

        if (steady_small_decel) {
            steady_decel_time_ += step_size;
            if (steady_decel_time_ >= min_duration_) {
                latch_this_step = true;
                reason_this_step = LatchReason::STEADY_DECEL;
            }
        } else {
            steady_decel_time_ = 0.0;
        }
    }

    // --- Apply latch ---
    if (latch_this_step) {
        state_ = GearLatchState::LATCHED;
        latch_reason_ = reason_this_step;

        // Speed-based latch: lock one gear lower
        if (reason_this_step == LatchReason::STEADY_DECEL) {
            latched_gear_ = std::max(1, current_gear - 1);
        } else {
            latched_gear_ = current_gear;
        }
    }

    // --- Enforce gear lock if latched ---
    if (state_ == GearLatchState::LATCHED) {
        if (current_gear > latched_gear_) {  // attempted upshift
            SetGear(latched_gear_);
        } else {
            previous_gear_ = current_gear;   // allow downshift
        }
    } else {
        previous_gear_ = current_gear;
    }
}

double GearDownshiftLatch::GetSpeed() const {
    return vehicle_->GetSpeed();
}

void GearDownshiftLatch::Reset() {
    previous_gear_ = -1;
    latched_gear_ = -1;
    std::fill(speed_window_.begin(), speed_window_.end(), -1.0);
    window_index_ = 0;
    window_total_time_ = 0.0;
    steady_decel_time_ = 0.0;
    state_ = GearLatchState::NORMAL;
    latch_reason_ = LatchReason::NONE;
}

void GearDownshiftLatch::SetGear(int gear) const {
    vehicle_->GetTransmission()->asAutomatic()->SetShiftMode(
        chrono::vehicle::ChAutomaticTransmission::ShiftMode::MANUAL);
    vehicle_->GetTransmission()->SetGear(gear);
}

GearLatchState GearDownshiftLatch::GetState() const {
    return state_;
}

}  // namespace Simulation
}  // namespace DYNO
