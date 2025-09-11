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

#include <chrono/functions/ChFunctionSineStep.h>

#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

enum TurnDirection { LEFT = 1, RIGHT = -1 };

class WallToWall : public VehicleSimulation {
   public:
    WallToWall(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

   protected:
    void GetConfiguration() override;
    void OverrideInitialPose() override;
    void InitializeDriver() override;
    void InitializeTerrain() override;
    void WriteMetadata() override;
    void PreSynchronizationHook() override;
    void PostStepHook() override;

   private:
    double target_steering_;
    double target_speed_;
    double heading_tolerance_;
    double steering_trigger_position_;
    TurnDirection turn_direction_;

    bool is_turning_ = false;
};

}  // namespace Simulation
}  // namespace DYNO
