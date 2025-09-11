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

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

class TerrainMobility : public VehicleSimulation {
   public:
    TerrainMobility(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    void PrintTerrainHeights(chrono::vehicle::RigidTerrain& terrain,
                             double spacing = 1.0);

   protected:
    void OverrideInitialPose() override;

    /**
     * @brief Initialize the driver for the mobility mapping scenario.
     */
    void InitializeDriver() override;

    /**
     * @brief Initialize the terrain for the mobility mapping scenario.
     */
    void InitializeTerrain() override;

    /**
     * @brief Run the terrain post-initialization hook for the mobility mapping
     * scenario.
     */
    void PostInitializeTerrainHook() override;

    /**
     * @brief Synchronize the driver for the mobility mapping scenario.
     */
    void SynchronizeDriver() override;

    /**
     * @brief Run the pre-synchronization hook for the mobility mapping
     * scenario.
     *
     */
    void PreSynchronizationHook() override;

    /**
     * @brief Run the post-synchronization hook for the mobility mapping
     * scenario.
     */
    void PostSynchronizationHook() override;

    /**
     * @brief Run the post-initialization hook for the mobility mapping
     * scenario.
     */
    void PostInitializationHook() override;

   private:
    // Waypoints
    // --------------------------------------------------------------------- //
    /** @brief Vector of target waypoints (expressed as three-dimensional
     * positions). */
    std::vector<chrono::ChVector3d> points_;

    /** @brief Load the waypoints vector from a specified JSON configuration
     * file on disk. */
    void LoadWaypoints();

    /** @brief Vector of target headings. */
    std::vector<double> headings_;

    /** @brief Vector of waypoints speeds. */
    std::vector<double> speeds_;

    /** @brief Current target waypoint index. */
    unsigned int waypoint_index_ = 0;

    /** @brief Tolerance radius for waypoint completion, in meters. */
    double endpoint_tolerance_ = 3.0;

    std::shared_ptr<chrono::vehicle::SCMTerrain> scm_terrain_;

    double initial_height_;
};

}  // namespace Simulation
}  // namespace DYNO
