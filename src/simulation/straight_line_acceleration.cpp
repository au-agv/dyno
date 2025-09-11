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

#include <dyno/simulation/straight_line_acceleration.hpp>

namespace DYNO {
namespace Simulation {

StraightLineAcceleration::StraightLineAcceleration(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {}

void StraightLineAcceleration::GetConfiguration() {
    averaging_window_ =
        configuration_->GetValue<double>("scenario/averagingWindow", 3.0);
    speed_filter_ = std::make_shared<chrono::utils::ChRunningAverage>(
        int(averaging_window_ /
            configuration_->GetValue<double>("simulation/timeStep", 1.0e-3)));
    speed_tolerance_ =
        configuration_->GetValue<double>("scenario/speedTolerance", 0.2);
    time_to_max_throttle_ =
        configuration_->GetValue<double>("scenario/timeToMaxThrottle");
    throttle_ramp_ = std::make_shared<chrono::ChFunctionSineStep>(
        chrono::ChVector2(warmup_time_, 0.0),
        chrono::ChVector2(warmup_time_ + time_to_max_throttle_, 1.0));
    use_slope_ramp_ = configuration_->GetValue("scenario/useGradeRamp", false);
    target_slope_ = std::atan(
        configuration_->GetValue<double>("scenario/gradePercentage", 0.0) /
        100.0);
    target_sideslope_ = std::atan(
        configuration_->GetValue<double>("scenario/sideSlopePercentage", 0.0) /
        100.0);
    time_to_max_slope_ =
        configuration_->GetValue<double>("scenario/timeToMaxGrade");
    friction_coefficient_ = configuration_->GetValue(
        "scenario/terrain/rigid/frictionCoefficient", 0.85);
}

void StraightLineAcceleration::Instantiate() {

    minimum_time_ = warmup_time_ + time_to_max_throttle_ + time_to_max_slope_;

    if (use_slope_ramp_) {
        slope_ramp_ = std::make_shared<chrono::ChFunctionSineStep>(
            chrono::ChVector2(warmup_time_ + time_to_max_throttle_, 0.0),
            chrono::ChVector2(
                warmup_time_ + time_to_max_throttle_ + time_to_max_slope_,
                target_slope_));
    }
}

void StraightLineAcceleration::InitializeDriver() {
    auto path_driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(),
        chrono::vehicle::StraightLinePath(
            chrono::ChVector3<double>(0.0, 0.0, 0.0),
            chrono::ChVector3<double>(1.0e4, 0.0, 0.0), 100),
        "path", 0.0);

    InitializeSteeringController(path_driver->GetSteeringController());
    InitializeSpeedController(path_driver->GetSpeedController());

    path_driver->Initialize();
    driver_ = path_driver;

    VehicleSimulation::InitializeDriver();
}

void StraightLineAcceleration::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    terrain_type_ =
        configuration_->GetValue<std::string>("scenario/terrain/type", "rigid");

    const double patch_length =
        configuration_->GetValue("scenario/terrain/length", 1.0e4);
    const double patch_width =
        configuration_->GetValue("scenario/terrain/width", 5.0);
    const double patch_position_x = patch_length / 2.0 - 10.0;

    chrono::ChVector3d patch_position(patch_position_x, 0.0, 0.0);

    if (terrain_type_ == "rigid") {
        chrono::ChContactMaterialData minfo;
        minfo.mu = friction_coefficient_;
        minfo.cr = 0.75f;
        minfo.Y = 2e7f;
        auto material = minfo.CreateMaterial(system_->GetContactMethod());

        auto rigid_terrain =
            std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());
        rigid_terrain
            ->AddPatch(material,
                       chrono::ChCoordsys(
                           chrono::ChVector3(patch_position_x, 0.0, 0.0),
                           chrono::QUNIT),
                       patch_length, patch_width,
                       0.25  // Patch thickness
                       )
            ->SetTexture(std::string(DYNO_DATA_DIR) +
                             "textures/terrain/checker_white.png",
                         patch_length, patch_width);

        rigid_terrain->Initialize();

        terrain_->InitializeFrom(rigid_terrain);
    } else if (terrain_type_ == "scm") {
        InitializeSCMTerrain(patch_position, patch_length, patch_width);
    }
}

void StraightLineAcceleration::PreSynchronizationHook() {
    if (time_ > warmup_time_ && current_throttle_ < target_throttle_) {
        current_throttle_ = throttle_ramp_->GetVal(time_);
        SPDLOG_DEBUG("Current throttle: {:0.2f}", current_throttle_);
        OverrideControlsSpeed(current_throttle_, 0.0);
    } else if (time_ > warmup_time_ + time_to_max_throttle_) {
        OverrideControlsSpeed(target_throttle_, 0.0);
    } else if (time_ < warmup_time_) {
        OverrideControlsSpeed(0.0, 1.0);
    }
}

void StraightLineAcceleration::PostSynchronizationHook() {
    if (time_ > warmup_time_ + time_to_max_throttle_ &&
        current_slope_ < target_slope_) {
        current_slope_ = use_slope_ramp_ ? std::atan(slope_ramp_->GetVal(time_))
                                         : target_slope_;
        SPDLOG_DEBUG("Current slope: {:0.2f} deg",
                     current_slope_ * 180.0 / M_PI);

        system_->SetGravitationalAcceleration(
            chrono::QuatFromRodrigues(
                chrono::ChVector3(target_sideslope_, current_slope_, 0.0))
                .Rotate(chrono::ChVector3d(0.0, 0.0, -9.81)));
    } else if (current_slope_ >= target_slope_ && !slope_reached_) {
        slope_reached_ = true;
        SPDLOG_INFO("Final slope reached: {:0.2f} deg",
                    current_slope_ * 180.0 / M_PI);
    }

    double current_speed = GetVehicle()->GetVehicle()->GetSpeed();
    mean_speed_ = speed_filter_->Add(current_speed);

    SPDLOG_DEBUG("Mean speed: {:0.2f}, latest residual: {:0.2f}", mean_speed_,
                 std::abs(current_speed - mean_speed_));

    if (time_ > minimum_time_ && mean_speed_ < 0.0) {
        negative_time_ += time_step_;
    } else {
        negative_time_ = 0.0;
    }

    if (time_ > minimum_time_ &&
        std::abs(current_speed - mean_speed_) < speed_tolerance_) {
        steady_time_ += time_step_;
    } else {
        steady_time_ = 0.0;
    }

    if (negative_time_ > averaging_window_) {
        SPDLOG_ERROR("Failed!");
        is_successful_ = false;
        is_completed_ = true;
    }

    if (steady_time_ > averaging_window_) {
        is_completed_ = true;
    }
}

void StraightLineAcceleration::WriteMetadata() {
    output_->AddMetadata("targetSlope", target_slope_);
    output_->AddMetadata("targetSideslope", target_sideslope_);
    output_->AddMetadata("timeToMaxGrade", time_to_max_slope_);
    output_->AddMetadata("targetThrottle", target_throttle_);
    output_->AddMetadata("timeToMaxThrottle", time_to_max_throttle_);
    output_->AddMetadata("success", is_successful_);

    if (terrain_type_ == "rigid") {
        output_->AddMetadata("frictionCoefficient",
                             configuration_->GetValue<double>(
                                 "scenario/terrain/rigid/frictionCoefficient"));
    }
}

}  // namespace Simulation
}  // namespace DYNO
