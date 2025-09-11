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

#include <chrono/functions/ChFunctionSine.h>
#include <chrono/utils/ChOpenMP.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

class SideslopeStability : public VehicleSimulation {
  public:
    /**
     * @brief Construct a new serpentine steering scenario.
     *
     * @param configuration Shared pointer to the parsed scenario JSON
     *                      configuration.
     */
    SideslopeStability(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

  protected:
    /**
     * @brief Initialize the driver for the serpentine steering scenario.
     */
    void InitializeDriver() override;

    /**
     * @brief Initialize the terrain for the serpentine steering scenario.
     */
    void InitializeTerrain() override;

    /**
     * @brief Run the pre-synchronization hook for the serpentine steering
     *        scenario.
     */
    void PreSynchronizationHook() override;

    /**
     * @brief Run the post-synchronization hook for the serpentine steering
     *        scenario.
     */
    void PostSynchronizationHook() override;

    /**
     * @brief Write the scenario metadata for the serpentine steering scenario.
     */
    void WriteMetadata() override;

  private:
    // Grade climbing
    // --------------------------------------------------------------------- //
    /** @brief Current grade, in percentage. */
    double current_grade_ = 0.0;

    /** @brief Target grade, in percentage. */
    double target_grade_;

    /** @brief Whether or not to use a sinusoidal step function to ease-in the
     *         grade. */
    bool use_grade_ramp_ = true;

    /** @brief Grade ease-in sinusoidal step function. */
    std::shared_ptr<chrono::ChFunctionSineStep> slope_ramp_;

    /** @brief Period of the grade ease-in sinusoidal step function. */
    double time_to_max_grade_;

    /** @brief Generate the Bezier curve for the serpentine path mimicking
     *         obstacle avoidance. */
    std::shared_ptr<chrono::ChBezierCurve> GeneratePath();

    /** @brief Target speed, in meters per second. */
    double target_speed_;
    // --------------------------------------------------------------------- //

    // Sideslope
    // --------------------------------------------------------------------- //
    // --------------------------------------------------------------------- //

    // Open-loop steering
    // --------------------------------------------------------------------- //
    /** @brief Open-loop steering input function. */
    std::shared_ptr<chrono::ChFunctionSine> steering_input_;

    /** @brief Amplitude of the open-loop steering input sinusoidal. */
    double steering_amplitude_;

    /** @brief Frequency of the open-loop steering input sinusoidal. */
    double steering_frequency_;

    // TODO: Fill this documentation entry.
    /** @brief  */
    double steering_time_counter_ = 0.0;
    // --------------------------------------------------------------------- //

    // Scenario state
    // --------------------------------------------------------------------- //
    /** @brief Whether or not the target grade has been reached. */
    bool grade_reached_ = false;

    /** @brief Whether or not the scenario has been completed successfully. */
    bool is_successful_ = true;

    /** @brief Whether or not the vehicle has initiated the obstacle avoidance
     *         maneuver. */
    bool is_steering_ = false;
    // --------------------------------------------------------------------- //

    // Obstacle avoidance
    // --------------------------------------------------------------------- //
    /** @brief Length of the acceleration section, in meters. */
    double acceleration_length_;

    double deceleration_length_;

    /** @brief Obstacle position (the Z component is ignored), in meters. */
    chrono::ChVector3d obstacle_position_;

    /** @brief Obstacle size (the Z component is ignored), in meters. */
    chrono::ChVector3d obstacle_size_;

    /** @brief Serpentine steering  path offset measured from the center of the
     *         obstacle, in meters. */
    chrono::ChVector3d obstacle_offset_;

    /** @brief Serpentine steering Bezier curve path. */
    std::shared_ptr<chrono::ChBezierCurve> path_;
    // --------------------------------------------------------------------- //

    // Terrain type
    // --------------------------------------------------------------------- //
    std::string terrain_type_;
    // --------------------------------------------------------------------- //
};

} // namespace Simulation
} // namespace DYNO