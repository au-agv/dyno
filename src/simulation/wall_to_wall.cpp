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

#include <dyno/simulation/wall_to_wall.hpp>

namespace DYNO {
namespace Simulation {

WallToWall::WallToWall(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    SPDLOG_INFO("Instantiating the wall-to-wall simulation ...");
}

void WallToWall::GetConfiguration() {
    target_speed_ = configuration_->GetValue<double>("scenario/targetSpeed");
    heading_tolerance_ =
        configuration_->GetValue<double>("scenario/headingTolerance", 1.0e-2);
    steering_trigger_position_ =
        configuration_->GetValue<double>("scenario/triggerPosition", 50.0);
    turn_direction_ = configuration_->GetValue<bool>("scenario/leftTurn", false)
                          ? TurnDirection::LEFT
                          : TurnDirection::RIGHT;
    target_steering_ =
        boost::math::sign(turn_direction_) *
        configuration_->GetValue<double>("scenario/targetSteering");
}

void WallToWall::OverrideInitialPose() {
    vehicle_->OverrideInitialPose(
        chrono::ChCoordsysd(chrono::ChVector3d(0.0, 0.0, 0.5), chrono::QUNIT));
}

void WallToWall::InitializeDriver() {
    InitializeStraightLineDriver(
        configuration_->GetValue<double>("scenario/targetSpeed"));
}

void WallToWall::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    const double patch_length = 100.0;
    const double patch_width = 100.0;

    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());

    rigid_terrain
        ->AddPatch(GetContactMaterial(1.0),
                   chrono::ChCoordsys(chrono::VNULL, chrono::QUNIT),
                   patch_length, patch_width,
                   0.25  // Patch thickness
                   )
        ->SetTexture(
            std::string(DYNO_DATA_DIR) + "textures/terrain/checker_lime.png",
            patch_length, patch_width);
    rigid_terrain->Initialize();
    terrain_->InitializeFrom(rigid_terrain);
}

void WallToWall::PreSynchronizationHook() {
    if (is_turning_ || vehicle_->GetPositionX() > steering_trigger_position_) {
        OverrideControlsSteering(target_steering_);

        if (!is_turning_) {
            SPDLOG_INFO("Applying steering input ...");
            is_turning_ = true;
        }
    }
}

void WallToWall::PostStepHook() {
    const double angle =
        vehicle_->GetVehicle()->GetRot().GetCardanAnglesZYX().z();

    SPDLOG_DEBUG("Current heading angle {:0.2f} deg", angle * 180.0 / M_PI);

    if (is_turning_ && angle > (M_PI - 0.01)) {
        SPDLOG_INFO("Vehicle completed the turn - ending simulation ...");
        is_completed_ = true;
    }

    VehicleSimulation::PostStepHook();
}

void WallToWall::WriteMetadata() {}

}  // namespace Simulation
}  // namespace DYNO
