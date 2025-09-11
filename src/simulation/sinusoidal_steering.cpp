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

#include <dyno/simulation/sinusoidal_steering.hpp>

namespace DYNO {
namespace Simulation {

SinusoidalSteering::SinusoidalSteering(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    averaging_window_ =
        configuration_->GetValue<double>("scenario/averagingWindow", 3.0);

    amplitude_ =
        configuration_->GetValue<double>("scenario/steering/amplitude");
    frequency_ =
        configuration_->GetValue<double>("scenario/steering/frequency");

    steering_input_ =
        std::make_shared<chrono::ChFunctionSine>(amplitude_, frequency_, 0.0);

    target_slope_ = std::atan(
        configuration_->GetValue<double>("scenario/gradePercentage", 0.0) /
        100.0);

    target_speed_ = configuration_->GetValue<double>("scenario/targetSpeed");

    use_slope_ramp_ = configuration_->GetValue("scenario/useGradeRamp", false);

    if (use_slope_ramp_) {
        time_to_max_slope_ =
            configuration_->GetValue<double>("scenario/timeToMaxGrade", 3.0);

        slope_ramp_ = std::make_shared<chrono::ChFunctionSineStep>(
            chrono::ChVector2(warmup_time_, 0.0),
            chrono::ChVector2(
                warmup_time_ + time_to_max_slope_,
                configuration_->GetValue<double>("scenario/gradePercentage") /
                    100.0));
    }

    minimum_time_ = warmup_time_ + time_to_max_slope_;
}

void SinusoidalSteering::InitializeDriver() {
    auto path_driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(),
        chrono::vehicle::StraightLinePath(
            chrono::ChVector3<double>(0.0, 0.0, 0.0),
            chrono::ChVector3<double>(1.0e4, 0.0, 0.0), 100),
        "path", target_speed_);

    InitializeSpeedController(path_driver->GetSpeedController());
    InitializeSteeringController(path_driver->GetSteeringController());

    path_driver->Initialize();
    driver_ = path_driver;

    VehicleSimulation::InitializeDriver();
}

void SinusoidalSteering::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    terrain_type_ =
        configuration_->GetValue<std::string>("scenario/terrain/type", "rigid");

    const double patch_length =
        configuration_->GetValue("scenario/terrain/length", 1.0e4);
    const double patch_width =
        configuration_->GetValue("scenario/terrain/width", 5.0);
    const double patch_position_x = patch_length / 2.0 - 10.0;
    const chrono::ChVector3d patch_position(patch_position_x, 0.0, 0.0);

    if (terrain_type_ == "rigid") {
        chrono::ChContactMaterialData minfo;
        minfo.mu = configuration_->GetValue(
            "scenario/terrain/rigid/frictionCoefficient", 0.85);
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
                             "textures/terrain/checker_yellow.png",
                         patch_length, patch_width);

        rigid_terrain->Initialize();

        terrain_->InitializeFrom(rigid_terrain);
    } else if (terrain_type_ == "scm") {
        InitializeSCMTerrain(patch_position, patch_length, patch_width);
    }
}

void SinusoidalSteering::PreSynchronizationHook() {
    if (time_ > warmup_time_ + 5.0 && vehicle_->GetSpeed() >= target_speed_ &&
        slope_reached_) {
        if (!is_steering_) {
            SPDLOG_INFO("Starting turn ...");
            is_steering_ = true;
        }
        steering_time_counter_ += time_step_;
        if (steering_time_counter_ <= 1.0 / frequency_) {
            OverrideControlsSteering(steering_input_->GetVal(time_));

        } else {
            if (steering_time_counter_ > 1.0 / frequency_ + 3.0) {
                is_completed_ = true;
                is_successful_ = true;
            }
        }
    }
}

void SinusoidalSteering::PostSynchronizationHook() {
    if (time_ > warmup_time_ && current_slope_ < target_slope_ &&
        vehicle_->GetSpeed() > target_speed_) {
        current_slope_ = use_slope_ramp_ ? std::atan(slope_ramp_->GetVal(time_))
                                         : target_slope_;
        SPDLOG_INFO("Current slope: {:0.2f} deg",
                    current_slope_ * 180.0 / M_PI);

        system_->SetGravitationalAcceleration(
            chrono::QuatFromRodrigues(
                chrono::ChVector3(target_slope_, 0.0, 0.0))
                .Rotate(chrono::ChVector3d(0.0, 0.0, -9.81)));
    } else if (current_slope_ >= target_slope_ && !slope_reached_) {
        slope_reached_ = true;
        SPDLOG_INFO("Final slope reached: {:0.2f} deg",
                    current_slope_ * 180.0 / M_PI);
    }

    double current_speed = GetVehicle()->GetVehicle()->GetSpeed();

    if (vehicle_->GetVehicle()->GetRoll() * 180.0 / M_PI > 45.0) {
        SPDLOG_ERROR("ROLLOVER");
        is_successful_ = false;
        is_completed_ = true;
    }
}

void SinusoidalSteering::WriteMetadata() {
    output_->AddMetadata("steeringAmplitude", amplitude_);
    output_->AddMetadata("steeringFrequency", frequency_);
    output_->AddMetadata("targetSlope", target_slope_);
    output_->AddMetadata("timeToMaxGrade", time_to_max_slope_);
    output_->AddMetadata("success", is_successful_);

    if (terrain_type_ == "rigid") {
        output_->AddMetadata(
            "frictionCoefficient",
            configuration_->GetValue<double>(
                "scenario/terrain/rigid/frictionCoefficient", 0.85));
    }
}

}  // namespace Simulation
}  // namespace DYNO
