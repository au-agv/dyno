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

#include <cmath>
#include <optional>

#include <chrono/core/ChQuaternion.h>
#include <chrono/physics/ChBody.h>
#include <chrono_vehicle/ChVehicle.h>

namespace DYNO {
namespace Simulation {

/**
 * @brief Enumerates the possible units for angles.
 */
enum class AngleUnit { Radians, Degrees };

/**
 * @brief Enumerates the possible units for angular rates.
 */
enum class RateUnit { RadiansPerSecond, DegreesPerSecond };

/**
 * @brief Enumerates the possible types of vehicle failure conditions.
 */
enum class VehicleFailureType {
    None = 0,
    YawAngleExceeded,
    YawRateExceeded,
    RollAngleExceeded,
    RollRateExceeded,
    PitchAngleExceeded,
    PitchRateExceeded
};

/**
 * @brief Struct containing information about a detected vehicle failure.
 */
struct VehicleFailureReport {
    VehicleFailureType type = VehicleFailureType::None;
    double value = 0.0;  // internal unit (radians / rad/s)
    double limit = 0.0;  // internal unit
};

/**
 * @brief Struct containing persistence values for vehicle failure detection.
 */
struct VehicleFailurePersistence {
    double yaw_angle = 0.0;
    double yaw_rate = 0.0;
    double roll_angle = 0.0;
    double roll_rate = 0.0;
    double pitch_angle = 0.0;
    double pitch_rate = 0.0;
};

/**
 * @brief Class for detecting vehicle failure conditions.
 */
class VehicleFailureDetector {
   public:
    /**
     * @brief Constructs a vehicle failure detecotr for a given vehicle.
     *
     * @param vehicle A shared pointer to the vehicle object.
     */
    explicit VehicleFailureDetector(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle);

    /**
     * @brief Sets the yaw angle limit for failure detection.
     *
     * @param value The limit value.
     * @param unit The unit of the limit value (radians or degrees).
     */
    void SetYawLimit(double value, AngleUnit unit = AngleUnit::Radians);

    /**
     * @brief Sets the yaw rate limit for failure detection.
     *
     * @param value The limit value.
     * @param unit The unit of the limit value (radians per second or
     *              degrees per second).
     */
    void SetYawRateLimit(double value,
                         RateUnit unit = RateUnit::RadiansPerSecond);

    /**
     * @brief Sets the roll angle limit for failure detection.
     *
     * @param value The limit value.
     * @param unit The unit of the limit value (radians or degrees).
     */
    void SetRollLimit(double value, AngleUnit unit = AngleUnit::Radians);

    /**
     * @brief Sets the roll rate limit for failure detection.
     *
     * @param value The limit value.
     * @param unit The unit of the limit value (radians per second or
     *             degrees per second).
     */
    void SetRollRateLimit(double value,
                          RateUnit unit = RateUnit::RadiansPerSecond);

    /**
     * @brief Sets the pitch angle limit for failure detection.
     *
     * @param value The limit value.
     * @param unit The unit of the limit value (radians or degrees).
     */
    void SetPitchLimit(double value, AngleUnit unit = AngleUnit::Radians);

    /**
     * @brief Sets the pitch rate limit for failure detection.
     *
     * @param value The limit value.
     * @param unit The unit of the limit value (radians per second or
     *             degrees per second).
     */
    void SetPitchRateLimit(double value,
                           RateUnit unit = RateUnit::RadiansPerSecond);

    /**
     * @brief Sets the persistence time for yaw angle failure detection.
     *
     * @param time The persistence time.
     */
    void SetYawPersistence(double time);

    /**
     * @brief Sets the persistence time for yaw rate failure detection.
     *
     * @param time The persistence time.
     */
    void SetYawRatePersistence(double time);

    /**
     * @brief Sets the persistence time for roll angle failure detection.
     *
     * @param time The persistence time.
     */
    void SetRollPersistence(double time);

    /**
     * @brief Sets the persistence time for roll rate failure detection.
     *
     * @param time The persistence time.
     */
    void SetRollRatePersistence(double time);

    /**
     * @brief Sets the persistence time for pitch angle failure detection.
     *
     * @param time The persistence time.
     */
    void SetPitchPersistence(double time);

    /**
     * @brief Sets the persistence time for pitch rate failure detection.
     *
     * @param time The persistence time.
     */
    void SetPitchRatePersistence(double time);

    /**
     * @brief Resets all failure detection timers and states.
     */
    void Reset();

    /**
     * @brief Checks for any vehicle failure conditions based on the current
     *        state.
     *
     * @param time_step The time step for the simulation.
     *
     * @return A VehicleFailureReport object indicating any detected failures.
     */
    VehicleFailureReport Check(double time_step);

   private:
    /**
     * @brief Converts degrees to radians.
     *
     * @param degrees The value in degrees.
     *
     * @return The value in radians.
     */
    double ConvertDegreesToRadians(double degrees);

    /**
     * @brief Accumulates the failure timer for a given threshold.
     *
     * @param timer The timer value.
     * @param threshold The threshold value.
     * @param time_step The time step for the simulation.
     *
     * @return True if the threshold is exceeded, false otherwise.
     */
    bool Accumulate(double& timer, double threshold, double time_step);

    /**
     * @brief Resets the specified timer.
     *
     * @param timer The timer value to reset.
     */
    void ResetTimer(double& timer);

    /**
     * @brief Checks for yaw angle failure.
     *
     * @param yaw The current yaw angle.
     * @param time_step The time step for the simulation.
     *
     * @return An optional VehicleFailureReport if a failure is detected.
     */
    std::optional<VehicleFailureReport> CheckYawAngle(double yaw,
                                                      double time_step);

    /**
     * @brief Checks for yaw rate failure.
     *
     * @param rate The current yaw rate.
     * @param time_step The time step for the simulation.
     *
     * @return An optional VehicleFailureReport if a failure is detected.
     */
    std::optional<VehicleFailureReport> CheckYawRate(double rate,
                                                     double time_step);

    /**
     * @brief Checks for roll angle failure.
     *
     * @param roll The current roll angle.
     * @param time_step The time step for the simulation.
     *
     * @return An optional VehicleFailureReport if a failure is detected.
     */
    std::optional<VehicleFailureReport> CheckRollAngle(double roll,
                                                       double time_step);

    /**
     * @brief Checks for roll rate failure.
     *
     * @param rate The current roll rate.
     * @param time_step The time step for the simulation.
     *
     * @return An optional<VehicleFailureReport> if a failure is detected.
     */
    std::optional<VehicleFailureReport> CheckRollRate(double rate,
                                                      double time_step);

    /**
     * @brief Checks for pitch angle failure.
     *
     * @param pitch The current pitch angle.
     * @param time_step The time step for the simulation.
     *
     * @return An optional<VehicleFailureReport> if a failure is detected.
     */
    std::optional<VehicleFailureReport> CheckPitchAngle(double pitch,
                                                        double time_step);

    /**
     * @brief Checks for pitch rate failure.
     *
     * @param rate The current pitch rate.
     * @param time_step The time step for the simulation.
     *
     * @return An optional<VehicleFailureReport> if a failure is detected.
     */
    std::optional<VehicleFailureReport> CheckPitchRate(double rate,
                                                       double time_step);

    /** @brief A shared pointer to the vehicle object being monitored. */
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;

    /** @brief The yaw angle limit for failure detection. */
    double yaw_limit_ = M_PI;

    /** @brief The yaw rate limit for failure detection. */
    double yaw_rate_limit_ = 1.0e6;

    /** @brief The roll angle limit for failure detection. */
    double roll_limit_ = M_PI / 2;

    /** @brief The roll rate limit for failure detection. */
    double roll_rate_limit_ = 1.0e6;

    /** @brief The pitch angle limit for failure detection. */
    double pitch_limit_ = M_PI / 2;

    /** @brief The pitch rate limit for failure detection. */
    double pitch_rate_limit_ = 1.0e6;

    /** @brief Persistence values for vehicle failure detection. */
    VehicleFailurePersistence persist_;

    /** @brief Timers for failure detection. */
    VehicleFailurePersistence timers_;
};

}  // namespace Simulation
}  // namespace DYNO
