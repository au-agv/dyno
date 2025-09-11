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

#include <cctype>
#include <memory>

#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>

#include <chrono/assets/ChVisualSystem.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/models/vehicle.hpp>

namespace DYNO {
namespace Models {

class WheeledVehicle : public Vehicle {
   public:
    WheeledVehicle(std::shared_ptr<chrono::ChSystem> system);

    double GetSteeringAngleMax() const;

    void Setup(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    void Initialize();

    void OverrideInitialPose(const chrono::ChCoordsysd& initial_pose) {
        initial_pose_ = initial_pose;
        is_initial_pose_overridden_ = true;
    }

    std::shared_ptr<chrono::vehicle::ChVehicle> GetVehicle();

    double GetPositionX();

    const chrono::ChVector3d& GetPosition();

    double GetSteeringAngle();

    double GetSpeed();

    void Synchronize(const double& time,
                     const chrono::vehicle::DriverInputs driver_inputs,
                     const chrono::vehicle::ChTerrain& terrain) override;

    void AddActiveDomain(
        const std::shared_ptr<chrono::vehicle::SCMTerrain> terrain) override;

    const VehicleType& GetVehicleType();

   protected:
    std::shared_ptr<chrono::vehicle::ChWheeledVehicle> vehicle_;
    VehicleType type_ = VehicleType::WHEELED;
    double wheel_radius_ = 0.8;
};

}  // namespace Models
}  // namespace DYNO
