#pragma once

#include <chrono/geometry/ChLineBezier.h>

#include <dyno/interfaces/json_utilities.hpp>
#include <dyno/math/poisson_disk_sampler_2d.hpp>
#include <dyno/simulation/autonomous_vehicle_simulation.hpp>
#include <dyno/simulation/obstacle.hpp>
#include <dyno/simulation/path.hpp>

namespace DYNO {
namespace Simulation {

enum class ObstacleGeneratorMode {
    SINGLE = 0,
    FIELD = 1,
    PATH_GATES = 2,
    PATH_DISK = 3
};

class AutonomousNavigation : public AutonomousVehicleSimulation {
   public:
    AutonomousNavigation(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    const chrono::ChVector3d& GetTargetWaypoint() const;

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

    void FindPointsAlongPath();

   protected:
    void PostAdvanceHook() override;

   private:
    void Instantiate();

    chrono::ChFramed GetFrenetFrame(
        std::shared_ptr<chrono::ChBezierCurve> curve,
        chrono::ChVector3d position, unsigned int interval, double parameter,
        double tolerance = 1.0e-6);

    void AddPathVisualizationAsset();

    std::unique_ptr<Path> path_;

    std::vector<double> coordinates_;

    double waypoints_arc_distance_;

    double waypoints_integration_step_;

    std::vector<chrono::ChVector3d> path_nodes_;

    void AddObstaclesToPath(std::vector<DYNO::Math::PoissonPoint2D>& samples,
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

    /** @brief Indicates wheter the simulation will be halted when a collision
     * with an obstacle is detected. */
    bool halt_on_collision_;

    /** @brief The lower bound for the radius of the generated obstacles. */
    double obstacle_radius_min_;

    /** @brief The upper bound for the radius of the generated obstacles. */
    double obstacle_radius_max_;
};

}  // namespace Simulation
}  // namespace DYNO
