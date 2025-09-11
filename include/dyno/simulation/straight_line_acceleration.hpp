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

#include <chrono/functions/ChFunctionSineStep.h>
#include <chrono/utils/ChOpenMP.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

class StraightLineAcceleration : public VehicleSimulation {
   public:
    /**
     * @brief Construct a new straight line acceleration scenario.
     *
     * @param configuration Shared pointer to the scenario configuration object.
     */
    StraightLineAcceleration(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

   protected:
    /**
     * @brief Initialize the driver for the straight line acceleration scenario.
     */
    void InitializeDriver() override;

    /**
     * @brief Initialize the terrain for the straight line acceleration
     *        scenario.
     */
    void InitializeTerrain() override;

    /**
     * @brief Write the metadata for the straight line acceleration scenario to
     *        the output file.
     */
    void WriteMetadata() override;

    /**
     * @brief Run the pre-synchronization hook for the straight line
     *        acceleration scenario.
     */
    void PreSynchronizationHook() override;

    /**
     * @brief Run the post-synchronization hook for the straight line
     *        acceleration scenario.
     */
    void PostSynchronizationHook() override;

    void GetConfiguration() override;

    void Instantiate() override;

   private:
    /** @brief The current slope, in the direction perpendicular to the
     *         longitudinal axis of the terrain, as a percentage. */
    double current_slope_ = 0.0;

    /** @brief The target slope, in the direction perpendicular to the
     *         longitudinal axis of the terrain, as a percentage. */
    double target_slope_;

    /** @brief The target sideslope, in the direction parallel to the
     *         longitudinal axis of the terrain, as a percentage. */
    double target_sideslope_;

    /** @brief Indicates whether the target slope has been reached when using
     *         slope ramping. */
    bool slope_reached_ = false;

    /** @brief Shared pointer to the sinusoidal step function for slope ramping.
     * */
    std::shared_ptr<chrono::ChFunctionSineStep> slope_ramp_;

    /** @brief Indicated whether the slope will be gradually increased or
     *         suddenly stepped to the target value. */
    bool use_slope_ramp_ = false;

    /** @brief The time over which the slope will be increased from null to the
     *         target value, in seconds. */
    double time_to_max_slope_;

    /** @brief The current throttle, as a percentage effort. */
    double current_throttle_ = 0.0;

    /** @brief The target throttle for the straight line acceleration manoeuvre,
     *         as a percentage effort.
     *
     *         This value is usually wide-open-throttle (WOT) for a standard
     *         straight line acceleration test.
     */
    const double target_throttle_ = 1.0;

    /** @brief Shared pointer to the sinusoidal step function for throttle
     * ramping. */
    std::shared_ptr<chrono::ChFunctionSineStep> throttle_ramp_;

    /** @brief The time over which the throttle effort will be increased from
     *         null to target value, in seconds. */
    double time_to_max_throttle_;

    /** @brief The minimum amount of simulation time before end conditions are
     *         checked.
     *
     *         This value should be higher than null to avoid null initial
     *         speeds counting as steady state speeds for the straight line
     *         acceleration test.
     */
    double minimum_time_;

    /** @brief Shared pointer to the moving average filter for steady state
     *         speed detection. */
    std::shared_ptr<chrono::utils::ChRunningAverage> speed_filter_;

    /** @brief The current mean speed of the vehicle as computed by the moving
     *         average filter. */
    double mean_speed_ = 0.0;

    /** @brief The time window over which the moving average is computed, in
     *         seconds. */
    double averaging_window_;

    /** @brief The absolute tolerance between the mean speed measured at two
     *         subsequent steps before the simulation is terminated. */
    double speed_tolerance_ = 0.1;

    /** @brief The time window over which a consistent steady speed has been
     *         detected. */
    double steady_time_ = 0.0;

    /** @brief The time window over which a consistent negative speed has been
     *         detected.
     *
     *         This value is used to detect failed runs where the vehicle is
     *         sliding down the grade.
     */
    double negative_time_ = 0.0;

    /** @brief The terrain type identifier used to initialize the terrain model
     *         for the straight line acceleration simulation. */
    std::string terrain_type_;

    double friction_coefficient_;
};

}  // namespace Simulation
}  // namespace DYNO
