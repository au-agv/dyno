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

#include <dyno/simulation/vehicle_failure_detector.hpp>

namespace DYNO {
namespace Simulation {

VehicleFailureDetector::VehicleFailureDetector(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle)
    : vehicle_(vehicle) {}

VehicleFailureReport VehicleFailureDetector::Check(double time_step) {
    if (!vehicle_)
        return {};

    const auto& body = vehicle_->GetChassis()->GetBody();

    const auto rotation = body->GetRot().GetCardanAnglesXYZ();
    const auto roll = rotation.x();
    const auto pitch = rotation.y();
    const auto yaw = rotation.z();

    chrono::ChVector3d angular_velocity = body->GetAngVelLocal();

    if (auto result = CheckYawAngle(yaw, time_step))
        return *result;
    if (auto result = CheckYawRate(angular_velocity.z(), time_step))
        return *result;
    if (auto result = CheckRollAngle(roll, time_step))
        return *result;
    if (auto result = CheckRollRate(angular_velocity.x(), time_step))
        return *result;
    if (auto result = CheckPitchAngle(pitch, time_step))
        return *result;
    if (auto result = CheckPitchRate(angular_velocity.y(), time_step))
        return *result;

    return {};
}

void VehicleFailureDetector::Reset() {
    timers_ = {};
}

// ---- Set limits ----
void VehicleFailureDetector::SetYawLimit(double value, AngleUnit unit) {
    yaw_limit_ =
        (unit == AngleUnit::Degrees) ? ConvertDegreesToRadians(value) : value;
}

void VehicleFailureDetector::SetYawRateLimit(double value, RateUnit unit) {
    yaw_rate_limit_ = (unit == RateUnit::DegreesPerSecond)
                          ? ConvertDegreesToRadians(value)
                          : value;
}

void VehicleFailureDetector::SetRollLimit(double value, AngleUnit unit) {
    roll_limit_ =
        (unit == AngleUnit::Degrees) ? ConvertDegreesToRadians(value) : value;
}

void VehicleFailureDetector::SetRollRateLimit(double value, RateUnit unit) {
    roll_rate_limit_ = (unit == RateUnit::DegreesPerSecond)
                           ? ConvertDegreesToRadians(value)
                           : value;
}

void VehicleFailureDetector::SetPitchLimit(double value, AngleUnit unit) {
    pitch_limit_ =
        (unit == AngleUnit::Degrees) ? ConvertDegreesToRadians(value) : value;
}

void VehicleFailureDetector::SetPitchRateLimit(double value, RateUnit unit) {
    pitch_rate_limit_ = (unit == RateUnit::DegreesPerSecond)
                            ? ConvertDegreesToRadians(value)
                            : value;
}

// ---- Set persistence (seconds) ----
void VehicleFailureDetector::SetYawPersistence(double time) {
    persist_.yaw_angle = time;
}

void VehicleFailureDetector::SetYawRatePersistence(double time) {
    persist_.yaw_rate = time;
}

void VehicleFailureDetector::SetRollPersistence(double time) {
    persist_.roll_angle = time;
}

void VehicleFailureDetector::SetRollRatePersistence(double time) {
    persist_.roll_rate = time;
}

void VehicleFailureDetector::SetPitchPersistence(double time) {
    persist_.pitch_angle = time;
}

void VehicleFailureDetector::SetPitchRatePersistence(double time) {
    persist_.pitch_rate = time;
}

double VehicleFailureDetector::ConvertDegreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

bool VehicleFailureDetector::Accumulate(double& timer, double threshold,
                                        double time_step) {
    timer += time_step;
    return timer >= threshold;
}

void VehicleFailureDetector::ResetTimer(double& timer) {
    timer = 0.0;
}

std::optional<VehicleFailureReport> VehicleFailureDetector::CheckYawAngle(
    double yaw, double time_step) {
    if (std::abs(yaw) > yaw_limit_) {
        if (Accumulate(timers_.yaw_angle, persist_.yaw_angle, time_step))
            return VehicleFailureReport{VehicleFailureType::YawAngleExceeded,
                                        yaw, yaw_limit_};
    } else {
        ResetTimer(timers_.yaw_angle);
    }
    return std::nullopt;
}

std::optional<VehicleFailureReport> VehicleFailureDetector::CheckYawRate(
    double rate, double time_step) {
    if (std::abs(rate) > yaw_rate_limit_) {
        if (Accumulate(timers_.yaw_rate, persist_.yaw_rate, time_step))
            return VehicleFailureReport{VehicleFailureType::YawRateExceeded,
                                        rate, yaw_rate_limit_};
    } else {
        ResetTimer(timers_.yaw_rate);
    }
    return std::nullopt;
}

std::optional<VehicleFailureReport> VehicleFailureDetector::CheckRollAngle(
    double roll, double time_step) {
    if (std::abs(roll) > roll_limit_) {
        if (Accumulate(timers_.roll_angle, persist_.roll_angle, time_step))
            return VehicleFailureReport{VehicleFailureType::RollAngleExceeded,
                                        roll, roll_limit_};
    } else {
        ResetTimer(timers_.roll_angle);
    }
    return std::nullopt;
}

std::optional<VehicleFailureReport> VehicleFailureDetector::CheckRollRate(
    double rate, double time_step) {
    if (std::abs(rate) > roll_rate_limit_) {
        if (Accumulate(timers_.roll_rate, persist_.roll_rate, time_step))
            return VehicleFailureReport{VehicleFailureType::RollRateExceeded,
                                        rate, roll_rate_limit_};
    } else {
        ResetTimer(timers_.roll_rate);
    }
    return std::nullopt;
}

std::optional<VehicleFailureReport> VehicleFailureDetector::CheckPitchAngle(
    double pitch, double time_step) {
    if (std::abs(pitch) > pitch_limit_) {
        if (Accumulate(timers_.pitch_angle, persist_.pitch_angle, time_step))
            return VehicleFailureReport{VehicleFailureType::PitchAngleExceeded,
                                        pitch, pitch_limit_};
    } else {
        ResetTimer(timers_.pitch_angle);
    }
    return std::nullopt;
}

std::optional<VehicleFailureReport> VehicleFailureDetector::CheckPitchRate(
    double rate, double time_step) {
    if (std::abs(rate) > pitch_rate_limit_) {
        if (Accumulate(timers_.pitch_rate, persist_.pitch_rate, time_step))
            return VehicleFailureReport{VehicleFailureType::PitchRateExceeded,
                                        rate, pitch_rate_limit_};
    } else {
        ResetTimer(timers_.pitch_rate);
    }
    return std::nullopt;
}

}  // namespace Simulation
}  // namespace DYNO
