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
    : AutonomousVehicleSimulation(configuration) {
    Instantiate();
    InitializePath();
}

void AutonomousNavigation::Instantiate() {
    enable_obstacles_ =
        configuration_->GetValue("scenario/obstacle/enabled", false);
    waypoints_arc_distance_ =
        configuration_->GetValue("scenario/path/waypointsArcDistance", 5.0);
    waypoints_integration_step_ = configuration_->GetValue(
        "scenario/path/waypointsIntegrationStep", 1.0e-4);

    field_width_ = configuration_->GetValue("scenario/terrain/width", 100.0);
    field_length_ = configuration_->GetValue("scenario/terrain/length", 100.0);

    collision_threshold_ =
        configuration_->GetValue("scenario/obstacle/collisionThreshold", 0.5);

    obstacles_height_ =
        configuration_->GetValue("scenario/obstacles/height", 1.0);

    obstacles_minimum_distance_ =
        configuration_->GetValue("scenario/obstacles/minimumDistance", 0.2);

    halt_on_collision_ =
        configuration_->GetValue("scenario/haltOnCollision", true);

    obstacle_radius_min_ =
        configuration_->GetValue("scenario/obstacles/radius/minimum", 1.0);
    obstacle_radius_max_ =
        configuration_->GetValue("scenario/obstacles/radius/maximum", 3.0);
}

void AutonomousNavigation::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);
    const auto friction_coefficient = configuration_->GetValue(
        "scenario/terrain/rigid/frictionCoefficient", 0.85);
    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());
    rigid_terrain
        ->AddPatch(
            GetContactMaterial(friction_coefficient),
            chrono::ChCoordsys(chrono::ChVector3(0.0, 0.0, 0.0), chrono::QUNIT),
            field_length_ + 2.0 * acceleration_length_, 1.25 * field_width_,
            0.25)
        ->SetTexture(
            std::string(DYNO_DATA_DIR) + "textures/terrain/checker_pink.png",
            100.0, 100.0);
    rigid_terrain->Initialize();
    terrain_->InitializeFrom(rigid_terrain);
}

void AutonomousNavigation::FindPointsAlongPath() {

    std::vector<chrono::ChVector3d> positions;
    auto current_position = path_->GetCurve()->Eval(0.0);
    positions.push_back(current_position);

    double current_step = 0.0;

    while (current_step < 1.0) {
        double cumulative_length = 0.0;
        while (cumulative_length < waypoints_arc_distance_ &&
               current_step < 1.0) {
            auto integrated_position = path_->GetCurve()->Eval(current_step);
            cumulative_length +=
                (integrated_position - current_position).Length();
            current_position = integrated_position;
            current_step += waypoints_integration_step_;
        }
        positions.push_back(current_position);
        coordinates_.push_back(current_step);

        SPDLOG_DEBUG(
            "Adding point {:0.2f} {:0.2f} {:0.2f} at coordinate t = "
            "{:0.2f}",
            current_position.x(), current_position.y(), current_position.z(),
            current_step);
    }

    path_nodes_ = positions;
}

void AutonomousNavigation::GenerateObstaclesAlongPath() {
    FindPointsAlongPath();

    switch (obstacle_generator_mode_) {
        case ObstacleGeneratorMode::PATH_GATES: {
            SPDLOG_INFO("Generating obstacles at the specified path gates ...");

            auto sampler = std::make_unique<DYNO::Math::PoissonDiskSampler2D>(
                5.0, 5.0, obstacles_minimum_distance_);

            auto previous_position = chrono::ChVector3d(0.0, 0.0, 0.0);
            for (size_t i = 0; i < path_nodes_.size(); ++i) {

                sampler->Generate();
                auto samples = sampler->GetSamples();

                SPDLOG_INFO("Generated {} samples.", samples.size());

                auto segment_index = size_t(
                    path_->GetCurve()->GetNumSegments() * coordinates_[i]);
                auto frenet_frame =
                    GetFrenetFrame(path_->GetCurve(), path_nodes_[i],
                                   segment_index, coordinates_[i]);

                AddObstaclesToPath(samples, frenet_frame);
            }
            break;
        }

        case ObstacleGeneratorMode::FIELD: {
            SPDLOG_INFO("Generating obstacle field ...");

            auto sampler = std::make_unique<DYNO::Math::PoissonDiskSampler2D>(
                field_length_, field_width_, obstacles_minimum_distance_);
            sampler->SetRadiusRange(obstacle_radius_min_, obstacle_radius_max_);
            sampler->SetOrigin(-field_length_ / 2.0, -field_width_ / 2.0, 0.0);
            sampler->Generate();
            auto samples = sampler->GetSamples();
            AddObstacles(samples);

            break;
        }
    }
}

void AutonomousNavigation::AddObstacles(
    std::vector<DYNO::Math::PoissonPoint2D>& samples) {
    for (const auto& sample : samples) {
        auto position = chrono::ChVector3d(sample.GetX(), sample.GetY(), 0.0);
        AddObstacle(position,
                    chrono::ChVector3d(sample.GetRadius(), sample.GetRadius(),
                                       obstacles_height_));

        SPDLOG_DEBUG(
            "Added obstacle at location [X: {:0.2f} m Y:{:0.2f} m] with "
            "radius r "
            "= {:0.2f} m",
            sample.GetX(), sample.GetY(), sample.GetRadius());
    }
}

void AutonomousNavigation::AddObstaclesToPath(
    std::vector<DYNO::Math::PoissonPoint2D>& samples,
    chrono::ChFramed& frenet_frame) {
    for (const auto& sample : samples) {
        auto transformed_sample =
            frenet_frame *
            chrono::ChVector3d(sample.GetX(), sample.GetY(), 0.0);
        AddObstacle(transformed_sample, chrono::ChVector3d(0.1, 0.1, 10.0));

        SPDLOG_INFO("Adding obstacle in position {} {} {}",
                    transformed_sample.x(), transformed_sample.y(),
                    transformed_sample.z());
    }
}

void AutonomousNavigation::AddPathVisualizationAsset() {
    SPDLOG_INFO("Adding navigation path visualization asset ...");

    auto path_dummy_body = std::make_shared<chrono::ChBody>();
    path_dummy_body->SetFixed(true);
    system_->AddBody(path_dummy_body);

    auto path_visual_shape = std::make_shared<chrono::ChVisualShapeLine>();
    path_visual_shape->SetLineGeometry(
        std::make_shared<chrono::ChLineBezier>(path_->GetCurve()));
    // path_visual_shape->SetColor(chrono::ChColor(0.27, 0.1, 0.1));
    path_visual_shape->SetName("Navigation path");
    path_visual_shape->SetNumRenderPoints(
        std::max<unsigned int>(2 * path_nodes_.size(), 400));

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

    // Instantiate a Frenet frame from the specified position and the calculated
    // rotation matrix.
    chrono::ChMatrix33<> rotation(tangent, normal, binormal);
    chrono::ChFramed frenet_frame;
    frenet_frame.SetRot(rotation);
    frenet_frame.SetPos(position);

    return frenet_frame;
}

void AutonomousNavigation::InitializePath() {
    SPDLOG_INFO("Initializing path ...");

    auto waypoints = configuration_->GetValue<std::vector<chrono::ChVector3d>>(
        "scenario/path/waypoints");

    if (waypoints.size() < 2) {
        throw DYNO::Exceptions::InvalidConfigurationValue();
    }

    path_ = std::make_unique<Path>(waypoints);

    SPDLOG_INFO("Loaded %i waypoints.", waypoints.size());

    target_waypoint_ = path_->GetCurve()->Eval(1.0);

    auto initial_pose_path = path_->GetCurve()->Eval(0.0);
    path_initial_pose_ =
        chrono::ChVector3d(initial_pose_path.x() - 20.0, initial_pose_path.y(),
                           initial_pose_path.z());
}

void AutonomousNavigation::InitializeAssets() {
    SPDLOG_INFO("Enabling assets ...");

    enable_obstacles_ =
        configuration_->GetValue("scenario/obstacles/enabled", false);

    if (enable_obstacles_) {
        SPDLOG_INFO("Initializing obstacles ...");

        auto positions =
            configuration_->GetValue<std::vector<chrono::ChVector3d>>(
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

    GenerateObstaclesAlongPath();
    AddPathVisualizationAsset();
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
            "Adjusted height for obstacle at position [X: {:0.2f} m Y: {:0.2f} "
            "m], "
            "new height: {} m",
            position.x(), position.y(), terrain_height);

    } else {
        obstacle_body->SetPos(position);
    }

    double radius_range = obstacle_radius_max_ - obstacle_radius_min_;
    double colormap = (size.x() - obstacle_radius_min_) / radius_range;

    obstacle_body->GetVisualShape(0)->SetColor(
        chrono::ChColor(colormap, 0.0, 0.0));
    obstacle_body->SetFixed(true);
    obstacle_body->EnableCollision(false);
    system_->AddBody(obstacle_body);
}

void AutonomousNavigation::CheckProgress() {}

void AutonomousNavigation::PostAdvanceHook() {
    CheckCollision();
    CheckProgress();
}

void AutonomousNavigation::CheckCollision() {
    for (const auto& obstacle : obstacles_) {
        if ((vehicle_->GetPosition() - obstacle.GetPosition()).Length() <
            (obstacle.GetMaxSize() + collision_threshold_)) {
            SPDLOG_DEBUG(
                "Collision detected with obstacle at position [X: {:0.2f} m, "
                "Y: {:0.2f} m]");
            if (halt_on_collision_) {
                SPDLOG_ERROR(
                    "Stopping simulation due to obstacle collision ...");
                is_completed_ = true;
            }
        }
    }
}

const std::vector<chrono::ChVector3d>& AutonomousNavigation::GetNavigationPath()
    const {
    return path_nodes_;
}

const chrono::ChVector3d& AutonomousNavigation::GetTargetWaypoint() const {
    return target_waypoint_;
}

const std::vector<Obstacle>& AutonomousNavigation::GetObstacles() const {
    return obstacles_;
}

}  // namespace Simulation
}  // namespace DYNO
