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

#include <chrono/geometry/ChLineBezier.h>

#include <dyno/interfaces/json_utilities.hpp>
#include <dyno/math/poisson_disk_sampler_1d.hpp>
#include <dyno/math/poisson_disk_sampler_2d.hpp>
#include <dyno/simulation/autonomous_vehicle_simulation.hpp>
#include <dyno/simulation/collision_checker.hpp>
#include <dyno/simulation/obstacle.hpp>
#include <dyno/simulation/path.hpp>
#include <dyno/simulation/path_tracker.hpp>

namespace DYNO {
namespace Simulation {

enum class ObstacleGeneratorMode {
    STAGGERED = 0,
    FIELD = 1,
    PATH_GATES = 2,
    MANUAL = 3
};

class AutonomousNavigation : public AutonomousVehicleSimulation {
   public:
    AutonomousNavigation(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    const chrono::ChVector3d& GetTargetWaypoint() const;

    static std::string ToString(const ObstacleGeneratorMode& mode);

    static ObstacleGeneratorMode FromString(const std::string& string);

    const std::vector<chrono::ChVector3d>& GetNavigationWaypoints() const;

    const std::vector<chrono::ChVector3d>& GetNavigationPath() const;

    /**
     * @brief Retrieves the list of obstacles.
     *
     * This method returns a constant reference to the vector containing all
     * obstacles.
     *
     * @return A constant reference to the vector of Obstacle objects.
     */
    const std::vector<Obstacle>& GetObstacles() const;

    std::string GetCompletionMessage() const;

   protected:
    /**
     * @brief Initializes the terrain.
     *
     * This method is responsible for setting up and initializing the terrain.
     */
    void InitializeTerrain() override;

    /**
     * @brief Initializes the assets.
     *
     * This method is responsible for setting up and initializing the assets.
     */
    void InitializeAssets() override;

    void LoadWaypoints();

    void AddObstacles();

    void CheckProgress();

    void CheckCollision();

    void GenerateObstaclesAlongPath();

    void InitializePath();

    void AddObstacle(const chrono::ChVector3d& position,
                     const chrono::ChVector3d& size);

    void DensifyWaypoints();

    void Instantiate() override;

    void GetConfiguration() override;

    void PostInitializeTerrainHook() override;

    void PreInitializeSystemHook() override;

    void PostInitializationHook() override;

    void PostAdvanceHook() override;

    void WriteMetadata() override;

    void GenerateStaggeredObstacles();

    void GenerateObstacleField();

    void GeneratePathGatesObstacles();

    void GenerateManualObstacles();

   private:
    chrono::ChFramed GetFrenetFrame(
        std::shared_ptr<chrono::ChBezierCurve> curve,
        chrono::ChVector3d position, unsigned int interval, double parameter,
        double tolerance = 1.0e-6);

    void AddPathVisualizationAsset();

    std::unique_ptr<Path> path_;

    std::vector<double> coordinates_;

    double curve_dense_step_;

    double curve_integration_step_;

    std::vector<chrono::ChVector3d> waypoints_;

    std::vector<chrono::ChVector3d> dense_waypoints_;

    void AddObstaclesToPath(std::vector<DYNO::Math::PoissonPoint1D>& samples,
                            chrono::ChFramed& frenet_frame);

    void AddObstacles(std::vector<DYNO::Math::PoissonPoint2D>& samples);

    /** @brief The target waypoint passed to the autonomous navigation system
     * upstream. */
    chrono::ChVector3d target_waypoint_;

    /** @brief The length of the obstacle field. */
    double field_length_;

    /** @brief The width of the obstacle field. */
    double field_width_;

    /** @brief The minimum distance between generated obstacles, measured
     * relative to the outer perimeter of their footprints. */
    double obstacles_minimum_distance_;

    /** @brief The obstacle generation modality. See the matching enumeration
     * class for the available options. */
    ObstacleGeneratorMode obstacle_generator_mode_ =
        ObstacleGeneratorMode::FIELD;

    /** @brief Indicates whether obstacles will be generated. */
    bool enable_obstacles_;

    /** @brief The vector of generated obstacles. */
    std::vector<Obstacle> obstacles_;

    /** @brief The radial distance from the vehicle centre of gravity below
     * which a collision with an obstacle is detected. */
    double collision_threshold_;

    /** @brief The height of the generated obstacles. */
    double obstacles_height_;

    /** @brief The width of the obstacles generation line along each gate. */
    double obstacles_gates_width_;

    /** @brief Indicates wheter the simulation will be halted when a collision
     * with an obstacle is detected. */
    bool halt_on_collision_;

    /** @brief The lower bound for the radius of the generated obstacles. */
    double obstacle_radius_min_;

    /** @brief The upper bound for the radius of the generated obstacles. */
    double obstacle_radius_max_;

    bool densify_waypoints_;

    double obstacles_field_origin_;

    chrono::ChVector3d final_waypoint_;

    double waypoint_tolerance_;

    std::string completion_message_;

    bool is_successful_ = false;

    double obstacle_longitudinal_spacing_;

    double obstacle_lateral_spacing_;

    double initial_obstacle_position_;

    double friction_coefficient_;

    std::shared_ptr<SimplifiedCollisionChecker> collision_checker_;

    std::shared_ptr<PathTracker> path_tracker_;
};

}  // namespace Simulation
}  // namespace DYNO
