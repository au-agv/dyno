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

#include <dyno/simulation/sideslope_stability.hpp>

namespace DYNO {
namespace Simulation {

SideslopeStability::SideslopeStability(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    SPDLOG_INFO("Instantiating a sideslope stability scenario ...");

    // Configure the grade.
    target_grade_ = std::atan(
        configuration_->GetValue<double>("scenario/gradePercentage", 0.0) /
        100.0);

    // Configure the grade ease-in function.
    use_grade_ramp_ = configuration_->GetValue("scenario/useGradeRamp", false);
    if (use_grade_ramp_) {
        time_to_max_grade_ =
            configuration_->GetValue<double>("scenario/timeToMaxGrade");

        slope_ramp_ = std::make_shared<chrono::ChFunctionSineStep>(
            chrono::ChVector2(warmup_time_, 0.0),
            chrono::ChVector2(
                warmup_time_ + time_to_max_grade_,
                configuration_->GetValue<double>("scenario/gradePercentage") /
                    100.0));
    }

    // Configure the obstacle avoidance section.

    acceleration_length_ =
        configuration_->GetValue<double>("scenario/accelerationLength");
    deceleration_length_ = configuration_->GetValue<double>(
        "scenario/decelerationLength", acceleration_length_);
    target_speed_ = configuration_->GetValue<double>("scenario/targetSpeed");
    obstacle_position_ = chrono::ChVector3d(
        configuration_->GetValue<double>("scenario/obstacle/position/x"),
        configuration_->GetValue<double>("scenario/obstacle/position/y"), 0.0);
    obstacle_position_ = chrono::ChVector3d(
        configuration_->GetValue<double>("scenario/obstacle/size/x"),
        configuration_->GetValue<double>("scenario/obstacle/size/y"), 0.0);
    obstacle_offset_ = chrono::ChVector3d(
        configuration_->GetValue<double>("scenario/obstacle/offset/x"),
        configuration_->GetValue<double>("scenario/obstacle/offset/y"), 0.0);
}

void SideslopeStability::InitializeDriver() {
    SPDLOG_INFO(
        "Initializing path follower driver for the sideslope stability "
        "scenario ...");
    path_ = GeneratePath();

    auto path_driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(), path_, "path", target_speed_);

    InitializeSpeedController(path_driver->GetSpeedController());
    InitializeSteeringController(path_driver->GetSteeringController());

    path_driver->Initialize();
    driver_ = path_driver;

    VehicleSimulation::InitializeDriver();
}

std::shared_ptr<chrono::ChBezierCurve> SideslopeStability::GeneratePath() {
    SPDLOG_INFO(
        "Generating ideal path for the sideslope stability scenario ...");
    // Define a Bezier curve with fixed offsets for the control vertices.
    std::vector<chrono::ChVector3d> control_vertices_in;
    std::vector<chrono::ChVector3d> control_vertices_out;
    std::vector<chrono::ChVector3d> centerline;

    double current_position_x = 0.0;

    // ------------------------------------------------------------------------
    // / Acceleration section
    // ------------------------------------------------------------------------
    // /
    centerline.emplace_back(chrono::ChVector3(0.0, 0.0, 0.0));
    current_position_x += acceleration_length_;
    centerline.emplace_back(chrono::ChVector3(current_position_x, 0.0, 0.0));

    // ------------------------------------------------------------------------
    // / Obstacle avoidance section
    // ------------------------------------------------------------------------
    // /
    current_position_x += 20.0;
    centerline.emplace_back(chrono::ChVector3(current_position_x, 0.0, 0.0));
    current_position_x += 15.0;
    centerline.emplace_back(chrono::ChVector3(current_position_x, 5.0, 0.0));

    // ------------------------------------------------------------------------
    // / Rebound section
    // ------------------------------------------------------------------------
    // /
    current_position_x += 30.0;
    centerline.emplace_back(chrono::ChVector3(current_position_x, 0.0, 0.0));
    current_position_x += 50.0;
    centerline.emplace_back(chrono::ChVector3(current_position_x, 0.0, 0.0));
    current_position_x += 1000.0;
    centerline.emplace_back(chrono::ChVector3(current_position_x, 0.0, 0.0));

    // Apply the offsets to the control vertices.
    for (chrono::ChVector3<double>& node : centerline) {
        control_vertices_in.push_back(node - obstacle_offset_);
        control_vertices_out.push_back(node + obstacle_offset_);
    }

    return std::make_shared<chrono::ChBezierCurve>(
        centerline, control_vertices_in, control_vertices_out);
}

void SideslopeStability::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    terrain_type_ =
        configuration_->GetValue<std::string>("scenario/terrain/type", "rigid");

    const double patch_length =
        configuration_->GetValue("scenario/terrain/length", 1.0e4);
    const double patch_width =
        configuration_->GetValue("scenario/terrain/width", 20.0);
    const chrono::ChVector3d patch_position(patch_length / 2.0 - 10.0, 0.0,
                                            0.0);

    if (terrain_type_ == "rigid") {
        InitializeRigidTerrain(patch_position, patch_length, patch_width);
    } else if (terrain_type_ == "scm") {
        InitializeSCMTerrain(patch_position, patch_length, patch_width);
    }
}

void SideslopeStability::PreSynchronizationHook() {}

void SideslopeStability::PostSynchronizationHook() {
    if (time_ > warmup_time_ && current_grade_ < target_grade_) {
        current_grade_ = use_grade_ramp_ ? std::atan(slope_ramp_->GetVal(time_))
                                         : target_grade_;
        SPDLOG_INFO("Current slope: {:0.2f} deg",
                    current_grade_ * 180.0 / M_PI);

        system_->SetGravitationalAcceleration(
            chrono::QuatFromRodrigues(
                chrono::ChVector3(target_grade_, 0.0, 0.0))
                .Rotate(chrono::ChVector3d(0.0, 0.0, -9.81)));
    } else if (current_grade_ >= target_grade_ && !grade_reached_) {
        grade_reached_ = true;
        SPDLOG_INFO("Final slope reached: {:0.0f} deg",
                    current_grade_ * 180.0 / M_PI);
    }

    double current_speed = GetVehicle()->GetVehicle()->GetSpeed();

    if (!ValidateVehicleRoll(45.0) || !ValidateVehicleYaw(90.0)) {
        is_successful_ = false;
        is_completed_ = true;
    }

    if (vehicle_->GetPosition().x() > acceleration_length_ + 20.0 + 15.0 +
                                          30.0 + 50.0 +
                                          deceleration_length_ / 2.0) {
        is_successful_ = true;
        is_completed_ = true;
    }
}

void SideslopeStability::WriteMetadata() {
    output_->AddMetadata("targetSlope", target_grade_);
    output_->AddMetadata("timeToMaxGrade", time_to_max_grade_);
    output_->AddMetadata("success", is_successful_);

    if (terrain_type_ == "rigid") {
        output_->AddMetadata("frictionCoefficient",
                             configuration_->GetValue<double>(
                                 "scenario/terrain/rigid/frictionCoefficient"));
    }
}

}  // namespace Simulation
}  // namespace DYNO
