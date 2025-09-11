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

#include <chrono/functions/ChFunctionSineStep.h>
#include <chrono/utils/ChOpenMP.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/math/comparisons.hpp>
#include <dyno/simulation/gear_downshift_latch.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>
#include <dyno/simulation/vehicle_steady_state_detector.hpp>

namespace DYNO {
namespace Simulation {

class GradeClimbing : public VehicleSimulation {
   public:
    GradeClimbing(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

   protected:
    void GetConfiguration() override;

    void Instantiate() override;

    void InitializeDriver() override;

    void InitializeTerrain() override;

    void WriteMetadata() override;

    void PostInitializationHook() override;

    void PreSynchronizationHook() override;

    void PostSynchronizationHook() override;

    void PostStepHook() override;

   private:
    // ---------------------------------------------------------------------- //
    // Initial speed handling
    // ---------------------------------------------------------------------- //
    /** @brief The target speed for the grade climbing manoeuvre,
     *         as a percentage effort.
     */
    double target_initial_speed_;

    /** @brief Indicates whether the target speed has been reached. */
    bool speed_reached_ = false;

    /** @brief Shared pointer to the sinusoidal step function for throttle
     * ramping. */
    std::shared_ptr<chrono::ChFunctionSineStep> speed_ramp_;

    /** @brief The time over which speed will be increased from null to target
     *         value, in seconds. */
    double time_to_initial_speed_;

    double time_speed_initial_start_;

    double time_speed_initial_end_;

    double experiment_speed_;

    double initial_speed_tolerance_;

    // ---------------------------------------------------------------------- //
    // Grade handling
    // ---------------------------------------------------------------------- //
    void ApplyGrade();

    void InitializeGradeRamp(double initial_time);

    /** @brief The current grade, in the direction perpendicular to the
     *         longitudinal axis of the terrain, as a percentage. */
    double current_grade_ = 0.0;

    /** @brief The target grade, in the direction perpendicular to the
     *         longitudinal axis of the terrain, as a percentage. */
    double target_grade_;

    /** @brief Indicates whether the target grade has been reached when using
     *         grade ramping. */
    bool grade_reached_ = false;

    /** @brief Shared pointer to the sinusoidal step function for grade ramping.
     * */
    std::shared_ptr<chrono::ChFunctionSineStep> grade_ramp_;

    /** @brief Indicated whether the grade will be gradually increased or
     *         suddenly stepped to the target value. */
    bool use_grade_ramp_ = false;

    bool grade_ramp_initialized_ = false;

    /** @brief The time over which the grade will be increased from null to the
     *         target value, in seconds. */
    double time_to_target_grade_;

    double time_grade_end_;

    double time_grade_start_;

    // ---------------------------------------------------------------------- //
    // Steady state detection
    // ---------------------------------------------------------------------- //
    /** @brief Shared pointer to the moving average filter for steady state
     *         speed detection. */
    std::shared_ptr<chrono::utils::ChRunningAverage> speed_filter_;

    double start_time_ = std::numeric_limits<double>::infinity();

    /** @brief The current mean speed of the vehicle as computed by the moving
     *         average filter. */
    double current_speed_ = 0.0;

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

    // ---------------------------------------------------------------------- //
    // Steady state detection
    // ---------------------------------------------------------------------- //

    /** @brief The terrain type identifier used to initialize the terrain model
     *         for the straight line acceleration simulation. */
    std::string terrain_type_;

    double friction_coefficient_;

    std::shared_ptr<VehicleSteadyStateDetector> steady_state_detector_;

    std::shared_ptr<GearDownshiftLatch> downshift_latch_;

    bool has_latched_;
};

}  // namespace Simulation
}  // namespace DYNO
