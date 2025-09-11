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

#include <dyno/simulation/split_surface.hpp>

namespace DYNO {
namespace Simulation {

SplitSurface::SplitSurface(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    SPDLOG_INFO("Instantiating the split-mu simulation ...");

    acceleration_length_ =
        configuration_->GetValue<double>("scenario/accelerationLength", 100.0);
    target_brake_effort_ =
        configuration_->GetValue<double>("scenario/brakeEffort", 1.0);
    brake_trigger_position_ =
        configuration_->GetValue<double>("scenario/brakeStart");
    left_side_friction_coefficient_ =
        configuration_->GetValue<double>("scenario/terrain/friction/left", 0.9);

    right_side_friction_coefficient_ = configuration_->GetValue<double>(
        "scenario/terrain/friction/right", 0.1);
}

void SplitSurface::OverrideInitialPose() {
    vehicle_->OverrideInitialPose(chrono::ChCoordsysd(
        chrono::ChVector3d(-acceleration_length_, 0.0, 0.5), chrono::QUNIT));
}

void SplitSurface::InitializeDriver() {
    InitializeStraightLineDriver(
        configuration_->GetValue<double>("scenario/targetSpeed"));
}

void SplitSurface::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    const double patch_length =
        configuration_->GetValue("scenario/terrain/length", 100.0);
    const double patch_width =
        configuration_->GetValue("scenario/terrain/width", 5.0);
    const double patch_position_x = patch_length / 2.0 - 10.0;

    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());

    // Left patch
    rigid_terrain
        ->AddPatch(
            GetContactMaterial(left_side_friction_coefficient_),
            chrono::ChCoordsys(
                chrono::ChVector3(brake_trigger_position_ + patch_position_x,
                                  -patch_width / 2.0, 0.0),
                chrono::QUNIT),
            patch_length, patch_width,
            0.25  // Patch thickness
            )
        ->SetTexture(std::string(DYNO_DATA_DIR) +
                         "textures/terrain/checker_cyan.png",
                     patch_length, patch_width);

    // Right patch
    rigid_terrain
        ->AddPatch(
            GetContactMaterial(right_side_friction_coefficient_),
            chrono::ChCoordsys(
                chrono::ChVector3(brake_trigger_position_ + patch_position_x,
                                  patch_width / 2.0, 0.0),
                chrono::QUNIT),
            patch_length, patch_width,
            0.25  // Patch thickness
            )
        ->SetTexture(std::string(DYNO_DATA_DIR) +
                         "textures/terrain/checker_cyan.png",
                     patch_length, patch_width);

    rigid_terrain->Initialize();

    terrain_->InitializeFrom(rigid_terrain);
}

void SplitSurface::PreSynchronizationHook() {
    if (vehicle_->GetPositionX() > brake_trigger_position_) {
        OverrideControlsSpeed(0.0, target_brake_effort_);

        if (!is_braking_) {
            SPDLOG_INFO("Applying brake ...");
            is_braking_ = true;
        }
    }
}

void SplitSurface::PostStepHook() {
    if (is_braking_ && vehicle_->GetSpeed() < 0.05) {
        SPDLOG_INFO("Vehicle came to a halt - ending simulation ...");
        is_completed_ = true;
    }

    VehicleSimulation::PostStepHook();
}

void SplitSurface::WriteMetadata() {}

}  // namespace Simulation
}  // namespace DYNO
