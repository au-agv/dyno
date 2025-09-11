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

#include <dyno/simulation/straight_line_braking.hpp>

namespace DYNO {
namespace Simulation {

StraightLineBraking::StraightLineBraking(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    SPDLOG_INFO("Instantiating the straight line acceleration simulation ...");
}

void StraightLineBraking::Initialize() {
    target_speed_ = configuration_->GetValue<double>("scenario/targetSpeed");

    minimum_speed_ =
        configuration_->GetValue<double>("scenario/minimumSpeed", 0.01);

    brake_effort_ =
        configuration_->GetValue<double>("scenario/brakeEffort", 1.0);

    final_time_ = configuration_->GetValue<double>("scenario/finalTime", 3.0);

    VehicleSimulation::Initialize();
}

void StraightLineBraking::InitializeDriver() {
    InitializeStraightLineDriver(
        configuration_->GetValue<double>("scenario/targetSpeed"));

    VehicleSimulation::InitializeDriver();
}

void StraightLineBraking::InitializeTerrain() {
    SPDLOG_INFO(
        "Initializing the default terrain for the straight line braking "
        "scenario ...");

    double patch_length = 1.0e3;

    // Instantiate a new terrain through the terrain wrapper.
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    auto material = std::make_shared<chrono::ChContactMaterialNSC>();
    material->SetFriction(
        configuration_->GetValue("scenario/frictionCoefficient", 0.85));

    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());

    rigid_terrain
        ->AddPatch(material,
                   chrono::ChCoordsysd(chrono::ChVector3<double>(0.0, 0.0, 0.0),
                                       chrono::QUNIT),
                   patch_length,  // Patch length
                   5.0,           // Patch width
                   0.25           // Patch thickness
                   )
        ->SetTexture(std::string(DYNO_DATA_DIR) + "textures/terrain/checker_blue.png",
                     patch_length);

    rigid_terrain->Initialize();
    terrain_->InitializeFrom(rigid_terrain);
}

void StraightLineBraking::PostAdvanceHook() {
    double current_time = time_;
    if (!is_braking_ && vehicle_->GetSpeed() > target_speed_) {
        is_braking_ = true;
        SPDLOG_INFO("Target speed reached - applying brake ...");
    }

    if (is_braking_) {
        if (vehicle_->GetSpeed() < minimum_speed_) {
            final_timer_ += time_step_;
            if (final_timer_ >= final_time_) {
                is_completed_ = true;
            }
        } else {
            final_timer_ = 0.0;
        }

        OverrideControlsSpeed(0.0, brake_effort_);
    }
}

}  // namespace Simulation
}  // namespace DYNO
