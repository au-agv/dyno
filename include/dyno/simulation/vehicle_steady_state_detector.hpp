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

#include <chrono_vehicle/ChVehicle.h>

#include <algorithm>
#include <deque>
#include <memory>

namespace DYNO {
namespace Simulation {

/**
 * @brief Class for detecting steady-state conditions of a vehicle based on its
 *        speed.
 */
class VehicleSteadyStateDetector {
   public:
    /**
     * @brief Constructs a vehicle steady state detector for a given vehicle.
     *
     * @param vehicle A shared pointer to the vehicle object.
     * @param speed_min The minimum allowed speed (in m/s).
     * @param tolerance The speed tolerance for steady-state detection (in m/s).
     * @param persistence The time duration for which the steady-state condition
     *                    must be maintained (in seconds).
     */
    explicit VehicleSteadyStateDetector(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
        double speed_min = 0.1, double tolerance = 0.05,
        double persistence = 1.0);

    /**
     * @brief Sets the target speed for the vehicle.
     *
     * @param speed The target speed (in m/s).
     */
    void SetTargetSpeed(double speed);

    /**
     * @brief Sets the minimum allowed speed for the vehicle.
     *
     * @param speed The minimum speed (in m/s).
     */
    void SetMinSpeed(double speed);

    /**
     * @brief Resets the internal buffers and state of the detector.
     */
    void Reset();

    /**
     * @brief Checks if the vehicle is in a steady-state condition.
     *
     * @param time_step The time step for the simulation.
     *
     * @return True if the vehicle is in steady-state, false otherwise.
     */
    bool CheckSteadyState(double time_step);

    /**
     * @brief Retrieves the current speed of the vehicle.
     *
     * @return The current speed (in m/s).
     */
    double GetCurrentSpeed() const;

    /**
     * @brief Checks if the vehicle is currently in a steady-state condition.
     *
     * @return True if the vehicle is in steady-state, false otherwise.
     */
    bool IsSteady() const;

   private:
    /** @brief A shared pointer to the vehicle being monitored. */
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;

    /** @brief The target speed for the vehicle. */
    double speed_target_ = 0.0;

    /** @brief The minimum allowed speed for the vehicle. */
    double speed_min_ = 0.1;

    /** @brief The speed tolerance for steady-state detection. */
    double tolerance_ = 0.01;  // m/s

    /**
     * @brief The time duration for which the steady-state condition must be
     *        maintained.
     */
    double persistence_ = 5.0;  // seconds

    /** @brief A deque storing the history of vehicle speeds. */
    std::deque<double> speed_history_;

    /** @brief The maximum size of the speed history buffer. */
    size_t max_history_size_ = 0;

    /**
     * @brief A timer used to track the duration of the steady-state condition.
     */
    double timer_ = 0.0;

    /**
     * @brief A flag indicating whether the vehicle is in a steady-state
     *        condition.
     */
    bool steady_state_ = false;
};

}  // namespace Simulation
}  // namespace DYNO
