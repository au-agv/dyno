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

#include <dyno/simulation/autonomous_navigation.hpp>

namespace DYNO {
namespace Simulation {

AutonomousNavigation::AutonomousNavigation(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : AutonomousVehicleSimulation(configuration) {}

void AutonomousNavigation::GetConfiguration() {
    SPDLOG_INFO(
        "Retrieving configuration parameters for the autonomous navigation "
        "scenario ...");

    // ---------------------------------------------------------------------- //
    // > Instantiate the terrain parameters.
    // ---------------------------------------------------------------------- //
    friction_coefficient_ =
        configuration_->GetValue("scenario/terrain/frictionCoefficient", 0.85);

    // ---------------------------------------------------------------------- //
    // > Instantiate the waypoints parameters.
    // ---------------------------------------------------------------------- //
    waypoints_ = configuration_->GetValue<std::vector<chrono::ChVector3d>>(
        "scenario/path/waypoints");
    densify_waypoints_ =
        configuration_->GetValue("scenario/path/densifyWaypoints", false);
    curve_integration_step_ = configuration_->GetValue(
        "scenario/path/waypointsIntegrationStep", 1.0e-3);
    curve_dense_step_ =
        configuration_->GetValue("scenario/path/waypointsArcDistance", 5.0);
    waypoint_tolerance_ =
        configuration_->GetValue("scenario/path/waypointTolerance", 3.0);

    // ---------------------------------------------------------------------- //
    // > Instantiate the collision parameters.
    // ---------------------------------------------------------------------- //
    halt_on_collision_ =
        configuration_->GetValue("scenario/haltOnCollision", true);
    collision_threshold_ =
        configuration_->GetValue("scenario/obstacle/collisionThreshold", 0.5);

    // ---------------------------------------------------------------------- //
    // > Instantiate the staggered obstacles parameters.
    // ---------------------------------------------------------------------- //
    initial_obstacle_position_ =
        configuration_->GetValue("scenario/obstacles/initialPosition", 50.0);
    obstacle_longitudinal_spacing_ = configuration_->GetValue(
        "scenario/obstacles/spacing/longitudinal", 50.0);
    obstacle_lateral_spacing_ =
        configuration_->GetValue("scenario/obstacles/spacing/lateral", 25.0);

    // ---------------------------------------------------------------------- //
    // > Instantiate the obstacle field parameters.
    // ---------------------------------------------------------------------- //
    field_width_ = configuration_->GetValue("scenario/terrain/width", 100.0);
    field_length_ = configuration_->GetValue("scenario/terrain/length", 100.0);

    // ---------------------------------------------------------------------- //
    // > Instantiate the obstacles parameters.
    // ---------------------------------------------------------------------- //
    enable_obstacles_ =
        configuration_->GetValue("scenario/obstacles/enabled", true);
    obstacles_minimum_distance_ =
        configuration_->GetValue("scenario/obstacles/minimumDistance", 0.2);
    obstacle_radius_min_ =
        configuration_->GetValue("scenario/obstacles/radius/minimum", 1.0);
    obstacle_radius_max_ =
        configuration_->GetValue("scenario/obstacles/radius/maximum", 3.0);
    obstacles_height_ =
        configuration_->GetValue("scenario/obstacles/height", 1.0);
    obstacle_generator_mode_ = FromString(configuration_->GetValue<std::string>(
        "scenario/obstacles/mode", "staggered"));
    obstacles_gates_width_ =
        configuration_->GetValue("scenario/obstacles/gatesWidth", 5.0);

    // ---------------------------------------------------------------------- //
    // > Instantiate the obstacles field parameters.
    // ---------------------------------------------------------------------- //
    obstacles_field_origin_ = configuration_->GetValue(
        "scenario/obstacles/fieldOrigin", acceleration_length_ + 10.0);
}

void AutonomousNavigation::Instantiate() {}

void AutonomousNavigation::PreInitializeSystemHook() {
    InitializePath();

    if (densify_waypoints_ ||
        obstacle_generator_mode_ == ObstacleGeneratorMode::PATH_GATES) {
        DensifyWaypoints();
    } else {
        dense_waypoints_ = waypoints_;
    }
}

void AutonomousNavigation::InitializeTerrain() {
    SPDLOG_INFO("Initializing autonomous navigation scenario terrain ...");

    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    terrain_->InitializeRigidTerrainSinglePatch(
        friction_coefficient_, chrono::ChVector2d(0.0, 0.0),
        chrono::ChVector2d(2000.0, 2000.0),
        std::filesystem::path(std::string(DYNO_DATA_DIR) +
                              "textures/terrain/checker_white.png"));
}

void AutonomousNavigation::PostInitializeTerrainHook() {
    GenerateObstaclesAlongPath();
}

void AutonomousNavigation::PostInitializationHook() {
    failure_detector_ =
        std::make_shared<VehicleFailureDetector>(vehicle_->GetVehicle());
    failure_detector_->SetYawLimit(10.0, AngleUnit::Degrees);
    failure_detector_->SetYawRateLimit(10.0, RateUnit::DegreesPerSecond);
    failure_detector_->SetRollLimit(10.0, AngleUnit::Degrees);
    failure_detector_->SetRollRateLimit(10.0, RateUnit::DegreesPerSecond);

    VehicleSimulation::PostInitializationHook();
}

void AutonomousNavigation::PostAdvanceHook() {
    CheckCollision();
    CheckProgress();

    if ((vehicle_->GetPosition() - final_waypoint_).Length() <
        waypoint_tolerance_) {
        is_completed_ = true;
        is_successful_ = true;
        completion_message_ = "Final waypoint reached.";
    }

    if (time_ > end_time_) {
        SPDLOG_WARN("Simulation timed out, ending ...");
        is_completed_ = true;
        is_successful_ = false;
        completion_message_ = "Simulation timed out.";
    }
}

void AutonomousNavigation::DensifyWaypoints() {
    SPDLOG_INFO("Densifying waypoints alongside the Bezier path ...");

    // Add the initial point of the Bezier curve to the list of dense
    // waypoints.
    std::vector<chrono::ChVector3d> locations;
    double total_arc_length = 0.0;
    auto location = path_->GetCurve()->Eval(0.0);
    auto next_location = location;
    locations.push_back(location);

    double parameter = 0.0;
    while (parameter < 1.0) {
        // Integrate along the curve until a prescribed arc length is reached.
        double arc_length = 0.0;
        while (arc_length < curve_dense_step_) {

            // Advance the stepper.
            parameter += curve_integration_step_;
            next_location = path_->GetCurve()->Eval(parameter);

            // Calculate the new arc length.
            arc_length += (next_location - location).Length();
            total_arc_length += arc_length;
            location = next_location;
        }

        dense_waypoints_.push_back(location);
        coordinates_.push_back(parameter);
    }

    SPDLOG_INFO("Densified path length :{:0.2f} meters over {} nodes.",
                total_arc_length, dense_waypoints_.size());
}

void AutonomousNavigation::AddPathVisualizationAsset() {
    SPDLOG_INFO("Adding navigation path visualization asset ...");

    auto path_dummy_body = std::make_shared<chrono::ChBody>();
    path_dummy_body->SetFixed(true);
    system_->AddBody(path_dummy_body);

    auto path_visual_shape = std::make_shared<chrono::ChVisualShapeLine>();
    path_visual_shape->SetLineGeometry(
        std::make_shared<chrono::ChLineBezier>(path_->GetCurve()));
    path_visual_shape->SetName("Navigation path");
    path_visual_shape->SetNumRenderPoints(
        std::max<unsigned int>(2 * dense_waypoints_.size(), 400));

    path_dummy_body->AddVisualShape(path_visual_shape);
    if (system_->GetVisualSystem()) {
        system_->GetVisualSystem()->BindItem(path_dummy_body);
    }
}

chrono::ChFramed AutonomousNavigation::GetFrenetFrame(
    std::shared_ptr<chrono::ChBezierCurve> curve, chrono::ChVector3d position,
    unsigned int interval, double parameter, double tolerance) {
    // Compute the first and second derivative of the Bezier curve and their
    // cross product.
    chrono::ChVector3d first_derivative = curve->EvalDer(interval, parameter);
    chrono::ChVector3d second_derivative = curve->EvalDer2(interval, parameter);
    chrono::ChVector3d derivative_product =
        chrono::Vcross(first_derivative, second_derivative);

    // Calculate the tangent, normal and binormal vectors.
    chrono::ChVector3d tangent = first_derivative / first_derivative.Length();
    chrono::ChVector3d normal;
    chrono::ChVector3d binormal;
    if (std::abs(derivative_product.Length()) > tolerance) {
        normal = chrono::Vcross(derivative_product, first_derivative) /
                 (first_derivative.Length() * derivative_product.Length());
        binormal = derivative_product / derivative_product.Length();
    } else {  // Handle the zero curvature case separately.
        binormal = chrono::ChVector3d(0.0, 0.0, 1.0);
        normal = chrono::Vcross(binormal, tangent);
        binormal = chrono::Vcross(tangent, normal);
    }

    // Instantiate a Frenet frame from the specified position and the
    // calculated rotation matrix.
    chrono::ChMatrix33<> rotation(tangent, normal, binormal);
    chrono::ChFramed frenet_frame;
    frenet_frame.SetRot(rotation);
    frenet_frame.SetPos(position);

    return frenet_frame;
}

void AutonomousNavigation::InitializePath() {
    SPDLOG_INFO("Initializing path ...");

    if (waypoints_.size() < 2) {
        throw DYNO::Exceptions::InvalidConfigurationValue();
    }

    path_ = std::make_unique<Path>(waypoints_);

    SPDLOG_INFO("Loaded {} waypoints.", waypoints_.size());
}

void AutonomousNavigation::InitializeAssets() {
    SPDLOG_INFO("Enabling assets ...");

    AddPathVisualizationAsset();
}

void AutonomousNavigation::CheckProgress() {
    if (obstacle_generator_mode_ == ObstacleGeneratorMode::FIELD) {
        if (vehicle_->GetPosition().x() > final_waypoint_.x() + 30.0) {
            is_completed_ = true;
            is_successful_ = false;
            completion_message_ = "Final waypoint missed.";
        }
    }
}

void AutonomousNavigation::CheckCollision() {
    for (const auto& obstacle : obstacles_) {
        if ((vehicle_->GetPosition() - obstacle.GetPosition()).Length() <
            (obstacle.GetMaxSize() + collision_threshold_)) {
            SPDLOG_DEBUG(
                "Collision detected with obstacle at position [X: {:0.2f} "
                "m, "
                "Y: {:0.2f} m]");
            if (halt_on_collision_) {
                SPDLOG_WARN(
                    "Stopping simulation due to obstacle collision ...");
                is_completed_ = true;
                is_successful_ = false;
                completion_message_ = "Obstacle collision.";
            }
        }
    }
}

const std::vector<chrono::ChVector3d>&
AutonomousNavigation::GetNavigationWaypoints() const {
    return dense_waypoints_;
}

const std::vector<chrono::ChVector3d>& AutonomousNavigation::GetNavigationPath()
    const {
    return dense_waypoints_;
}

const chrono::ChVector3d& AutonomousNavigation::GetTargetWaypoint() const {
    return target_waypoint_;
}

const std::vector<Obstacle>& AutonomousNavigation::GetObstacles() const {
    return obstacles_;
}

void AutonomousNavigation::WriteMetadata() {
    output_->AddMetadata("reason", completion_message_);
    output_->AddMetadata("success", is_successful_);
}

std::string AutonomousNavigation::GetCompletionMessage() const {
    return completion_message_;
}

void AutonomousNavigation::AddObstacle(const chrono::ChVector3d& position,
                                       const chrono::ChVector3d& size) {
    obstacles_.emplace_back(Obstacle(position, size));

    auto obstacle_body = std::make_shared<chrono::ChBodyEasyCylinder>(
        chrono::ChAxis::Z, std::max(size.x(), size.y()), size.z(), 1000.0, true,
        true, std::make_shared<chrono::ChContactMaterialNSC>());

    if (configuration_->GetValue("scenario/obstacles/adjustHeight", false)) {
        // Get the terrain height at the centroid of the obstacle.
        auto terrain_height = terrain_->GetTerrain()->GetHeight(
            chrono::ChVector3<double>(position.x(), position.y(),
                                      std::numeric_limits<double>::max()));

        // Set the position of the obstacle bottom face above the
        // terrain height.
        obstacle_body->SetPos(chrono::ChVector3<double>(
            position.x(), position.y(), terrain_height + size.z() / 2.0));

        SPDLOG_DEBUG(
            "Adjusted height for obstacle at position [X: {:0.2f} m Y: "
            "{:0.2f} "
            "m], "
            "new height: {} m",
            position.x(), position.y(), terrain_height);

    } else {
        obstacle_body->SetPos(position);
    }

    double radius_range = obstacle_radius_max_ - obstacle_radius_min_;
    double colormap = (size.x() - obstacle_radius_min_) / radius_range;

    obstacle_body->GetVisualShape(0)->SetColor(
        chrono::ChColor(1.0 - colormap, 1.0 - colormap, 1.0 - colormap));
    obstacle_body->SetFixed(true);
    obstacle_body->EnableCollision(false);
    system_->AddBody(obstacle_body);
}

void AutonomousNavigation::AddObstacles(
    std::vector<DYNO::Math::PoissonPoint2D>& samples) {
    for (const auto& sample : samples) {
        auto position = chrono::ChVector3d(sample.GetX(), sample.GetY(), 0.0);
        AddObstacle(position,
                    chrono::ChVector3d(sample.GetRadius(), sample.GetRadius(),
                                       obstacles_height_));

        SPDLOG_DEBUG(
            "Added obstacle at location [X: {:0.2f} m Y:{:0.2f} m] "
            "with "
            "radius r "
            "= {:0.2f} m",
            sample.GetX(), sample.GetY(), sample.GetRadius());
    }
}

void AutonomousNavigation::AddObstaclesToPath(
    std::vector<DYNO::Math::PoissonPoint1D>& samples,
    chrono::ChFramed& frenet_frame) {
    for (const auto& sample : samples) {
        auto transformed_sample =
            frenet_frame * chrono::ChVector3d(0.0, sample.GetX(), 0.0);
        AddObstacle(transformed_sample,
                    chrono::ChVector3d(sample.GetRadius(), sample.GetRadius(),
                                       obstacles_height_));

        SPDLOG_INFO("Adding obstacle in position {:0.2f} {:0.2f} {:0.2f}",
                    transformed_sample.x(), transformed_sample.y(),
                    transformed_sample.z());
    }
}

void AutonomousNavigation::GenerateObstaclesAlongPath() {
    if (!enable_obstacles_) {
        SPDLOG_INFO("Skipping obstacles generation ...");
        return;
    }

    switch (obstacle_generator_mode_) {
        SPDLOG_INFO("Generating staggered obstacles ...");
        case ObstacleGeneratorMode::STAGGERED: {
            GenerateStaggeredObstacles();
            break;
        }
        case ObstacleGeneratorMode::PATH_GATES: {
            GeneratePathGatesObstacles();
            break;
        }
        case ObstacleGeneratorMode::FIELD: {
            GenerateObstacleField();
            break;
        }
        case ObstacleGeneratorMode::MANUAL: {
            GenerateManualObstacles();
            break;
        }
    }
}

void AutonomousNavigation::GeneratePathGatesObstacles() {
    SPDLOG_INFO("Generating obstacles at the specified path gates ...");

    target_waypoint_ = path_->GetCurve()->Eval(1.0);
    final_waypoint_ = target_waypoint_;

    auto sampler = std::make_unique<DYNO::Math::PoissonDiskSampler1D>(
        obstacles_gates_width_, obstacles_minimum_distance_);
    sampler->SetRadiusRange(obstacle_radius_min_, obstacle_radius_max_);

    for (size_t i = 0; i < dense_waypoints_.size(); ++i) {
        sampler->Reset();
        sampler->Generate();
        auto samples = sampler->GetSamples();

        SPDLOG_INFO("Generated {} samples at gate #{}.", samples.size(), i);

        auto segment_index =
            size_t(path_->GetCurve()->GetNumSegments() * coordinates_[i]);

        SPDLOG_INFO("Segment index: {}", segment_index);

        SPDLOG_INFO("Current waypoint: {:0.2f} {:0.2f} {:0.2f}",
                    dense_waypoints_[i].x(), dense_waypoints_[i].y(),
                    dense_waypoints_[i].z());

        auto frenet_frame =
            GetFrenetFrame(path_->GetCurve(), dense_waypoints_[i],
                           segment_index, coordinates_[i]);

        SPDLOG_INFO("Frenet frame position: {:0.2f} {:0.2f} {:0.2f}",
                    frenet_frame.GetPos().x(), frenet_frame.GetPos().y(),
                    frenet_frame.GetPos().z());

        AddObstaclesToPath(samples, frenet_frame);
    }
}

void AutonomousNavigation::GenerateStaggeredObstacles() {
    SPDLOG_INFO(
        "Generating staggered obstacles of radius {:0.2f} m at origin "
        "[{:0.2f}, 0.0, 0.0] m with spacing "
        "[{:0.2f} x {:0.2f}] m ...",
        initial_obstacle_position_, obstacle_longitudinal_spacing_,
        obstacle_lateral_spacing_, obstacle_lateral_spacing_);

    // Add the single obstacle in the first row.
    AddObstacle(chrono::ChVector3d(initial_obstacle_position_, 0.0, 0.0),
                chrono::ChVector3d(obstacle_radius_max_, obstacle_radius_max_,
                                   obstacles_height_));

    // Add the two obstacles in the second row.
    AddObstacle(chrono::ChVector3d(
                    initial_obstacle_position_ + obstacle_longitudinal_spacing_,
                    obstacle_lateral_spacing_ / 2.0, 0.0),
                chrono::ChVector3d(obstacle_radius_max_, obstacle_radius_max_,
                                   obstacles_height_));
    AddObstacle(chrono::ChVector3d(
                    initial_obstacle_position_ + obstacle_longitudinal_spacing_,
                    -obstacle_lateral_spacing_ / 2.0, 0.0),
                chrono::ChVector3d(obstacle_radius_max_, obstacle_radius_max_,
                                   obstacles_height_));

    // Configure the waypoints for the staggered obstacles case.
    target_waypoint_ = waypoints_.back();
    final_waypoint_ = target_waypoint_;
}

void AutonomousNavigation::GenerateObstacleField() {
    SPDLOG_INFO("Generating obstacle field ...");

    target_waypoint_ = waypoints_.back();
    final_waypoint_ = target_waypoint_;

    auto sampler = std::make_unique<DYNO::Math::PoissonDiskSampler2D>(
        field_length_, field_width_, obstacles_minimum_distance_);
    sampler->SetRadiusRange(obstacle_radius_min_, obstacle_radius_max_);
    sampler->SetOrigin(obstacles_field_origin_, -field_width_ / 2.0, 0.0);
    sampler->Generate();
    auto samples = sampler->GetSamples();

    AddObstacles(samples);
}

void AutonomousNavigation::GenerateManualObstacles() {
    SPDLOG_INFO("Genearting obstacles at the specified locations ...");

    auto positions = configuration_->GetValue<std::vector<chrono::ChVector3d>>(
        "scenario/obstacles/positions");
    auto sizes = configuration_->GetValue<std::vector<chrono::ChVector3d>>(
        "scenario/obstacles/sizes");

    if (positions.size() != sizes.size()) {
        throw DYNO::Exceptions::InvalidConfigurationValue();
    }

    for (size_t i = 0; i < positions.size(); ++i) {
        AddObstacle(positions[i], sizes[i]);
    }
}

ObstacleGeneratorMode AutonomousNavigation::FromString(
    const std::string& string) {
    if (string == "staggered") {
        return ObstacleGeneratorMode::STAGGERED;
    }
    if (string == "field") {
        return ObstacleGeneratorMode::FIELD;
    }
    if (string == "path_gates") {
        return ObstacleGeneratorMode::PATH_GATES;
    }
    if (string == "manual") {
        return ObstacleGeneratorMode::MANUAL;
    }
    throw DYNO::Exceptions::InvalidConfigurationValue();
}

std::string AutonomousNavigation::ToString(const ObstacleGeneratorMode& mode) {
    switch (mode) {
        case ObstacleGeneratorMode::STAGGERED:
            return "staggered";
        case ObstacleGeneratorMode::FIELD:
            return "field";
        case ObstacleGeneratorMode::PATH_GATES:
            return "path_gates";
        case ObstacleGeneratorMode::MANUAL:
            return "manual";
        default:
            throw DYNO::Exceptions::InvalidConfigurationValue();
    };
}

}  // namespace Simulation
}  // namespace DYNO
