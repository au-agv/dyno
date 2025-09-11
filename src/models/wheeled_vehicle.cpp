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

#include <dyno/models/wheeled_vehicle.hpp>

namespace DYNO {
namespace Models {

using DYNO::Interfaces::JSON::GetPath;
using DYNO::Interfaces::JSON::GetValue;

WheeledVehicle::WheeledVehicle(std::shared_ptr<chrono::ChSystem> system)
    : Vehicle(system) {}

double WheeledVehicle::GetSteeringAngleMax() const {
    throw DYNO::Exceptions::NotImplemented();
}

void WheeledVehicle::Setup(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration) {
    // Create and initialize the vehicle. Here we suppress the "Loaded JSON"
    // output by temporarily turning off std::cout.
    chrono::vehicle::SetVehicleDataPath(DYNO_VEHICLE_DATA_DIR + base_path_);
    std::string parent_path = DYNO_VEHICLE_DATA_DIR + base_path_;

    SPDLOG_INFO("Initializing the vehicle system ...");
    std::cout.setstate(std::ios_base::failbit);

    vehicle_ = std::make_shared<chrono::vehicle::WheeledVehicle>(
        system_.get(),
        parent_path + configuration->GetValue<std::string>(
                          "vehicle/system/vehicle", "Vehicle.json"));

    std::cout.clear();

    // Unlock the chassis and move the vehicle system to the specified initial
    // pose.
    if (!is_initial_pose_overridden_) {
        initial_pose_ = chrono::ChCoordsys(
            chrono::ChVector3<double>(configuration->GetValue<double>(
                                          "vehicle/initialPosition/x", 0.0),
                                      configuration->GetValue<double>(
                                          "vehicle/initialPosition/y", 0.0),
                                      configuration->GetValue<double>(
                                          "vehicle/initialPosition/z", 0.1)),
            chrono::QUNIT);
    }

    vehicle_->Initialize(initial_pose_);
    vehicle_->GetChassis()->SetFixed(false);

    auto engine = chrono::vehicle::ReadEngineJSON(
        parent_path + configuration->GetValue<std::string>(
                          "vehicle/system/engine", "EngineSimpleMap.json"));

    const auto transmission_path = configuration->GetValue<std::string>(
        "vehicle/system/transmission",
        parent_path + "AutomaticTransmissionSimpleMap.json");

    auto transmission =
        chrono::vehicle::ReadTransmissionJSON(transmission_path);
    auto powertrain = std::make_shared<chrono::vehicle::ChPowertrainAssembly>(
        engine, transmission);
    vehicle_->InitializePowertrain(powertrain);

    // Create and initialize the tires.
    const auto tire_type =
        configuration->GetValue<std::string>("vehicle/system/tires", "tmeasy");
    std::string tire_label = "invalid";
    if (tire_type == "tmeasy") {
        tire_label = "_TMeasyTire.json";
    } else if (tire_type == "rigid") {
        tire_label = "_RigidTire.json";
    } else {
        throw std::invalid_argument("Invalid tire type.");
    }

    for (size_t axle_index = 0; axle_index < vehicle_->GetAxles().size();
         ++axle_index) {
        auto axle_labels = std::vector<std::string>{"Front", "Rear"};

        for (auto& wheel : vehicle_->GetAxles()[axle_index]->GetWheels()) {
            try {
                auto tire = chrono::vehicle::ReadTireJSON(
                    parent_path + configuration->GetValue<std::string>(
                                      "vehicle/system/" +
                                          boost::algorithm::to_lower_copy(
                                              axle_labels[axle_index]) +
                                          "Tires",
                                      axle_labels[axle_index] + tire_label));
                vehicle_->InitializeTire(tire, wheel,
                                         chrono::VisualizationType::MESH);

            } catch (const std::invalid_argument& exception) {
                throw;
            }
        }
    }

    vehicle_->SetChassisVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetSuspensionVisualizationType(
        chrono::VisualizationType::PRIMITIVES);
    vehicle_->SetSteeringVisualizationType(
        chrono::VisualizationType::PRIMITIVES);
    vehicle_->SetWheelVisualizationType(chrono::VisualizationType::MESH);

    vehicle_->LockAxleDifferential(
        0,  // Front differential
        configuration->GetValue<bool>("vehicle/differentialLock/front", false));

    vehicle_->LockAxleDifferential(
        1,  // Rear differential
        configuration->GetValue<bool>("vehicle/differentialLock/rear", false));

    vehicle_->LockCentralDifferential(
        0,  // Two front-most axles differential
        configuration->GetValue<bool>("vehicle/differentialLock/central",
                                      false));

    vehicle_->EnableBrakeLocking(
        configuration->GetValue<bool>("vehicle/brakeLock", false));

    // FIXME: This is a required hack to ensure Chrono::Vehicle uses the
    // correct paths, but it should probably be called from elsewhere.
    chrono::vehicle::SetVehicleDataPath(CHRONO_DEFAULT_VEHICLE_DATA_DIR);
}

void WheeledVehicle::Initialize() {}

std::shared_ptr<chrono::vehicle::ChVehicle> WheeledVehicle::GetVehicle() {
    return vehicle_;
}

void WheeledVehicle::Synchronize(
    const double& time, const chrono::vehicle::DriverInputs driver_inputs,
    const chrono::vehicle::ChTerrain& terrain) {
    vehicle_->Synchronize(time, driver_inputs, terrain);
}

double WheeledVehicle::GetPositionX() {
    return vehicle_->GetChassis()->GetBody()->GetPos().x();
}

const chrono::ChVector3d& WheeledVehicle::GetPosition() {
    return vehicle_->GetChassis()->GetBody()->GetPos();
}

double WheeledVehicle::GetSteeringAngle() {
    // Compute the current steering angle as the average steering angle for the
    // left and right wheels at the front axle.
    return (vehicle_->GetSteeringAngle(0, chrono::vehicle::VehicleSide::LEFT) +
            vehicle_->GetSteeringAngle(0,
                                       chrono::vehicle::VehicleSide::RIGHT)) /
           2.0;
}

double WheeledVehicle::GetSpeed() {
    return vehicle_->GetSpeed();
}

void WheeledVehicle::AddActiveDomain(
    const std::shared_ptr<chrono::vehicle::SCMTerrain> terrain) {
    for (int i = 0; i < 2; ++i) {

        terrain->AddActiveDomain(
            vehicle_->GetAxle(i)
                ->GetWheel(chrono::vehicle::VehicleSide::LEFT)
                ->GetSpindle(),
            chrono::VNULL,
            chrono::ChVector3d(0.5, 2.0 * wheel_radius_, 2.0 * wheel_radius_));

        terrain->AddActiveDomain(
            vehicle_->GetAxle(i)
                ->GetWheel(chrono::vehicle::VehicleSide::RIGHT)
                ->GetSpindle(),
            chrono::VNULL,
            chrono::ChVector3d(0.5, 2.0 * wheel_radius_, 2.0 * wheel_radius_));
    }
}

const VehicleType& WheeledVehicle::GetVehicleType() {
    return type_;
}

}  // namespace Models
}  // namespace DYNO
