#include <chrono/solver/ChSolverBB.h>
#include <dyno/models/tracked_vehicle.hpp>

namespace DYNO {
namespace Models {

TrackedVehicle::TrackedVehicle(std::shared_ptr<chrono::ChSystem> system)
    : Vehicle(system) {}

double TrackedVehicle::GetSteeringAngleMax() const {
    throw DYNO::Exceptions::NotImplemented();
}

void TrackedVehicle::Setup(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration) {
    chrono::vehicle::SetVehicleDataPath(CHRONO_DEFAULT_VEHICLE_DATA_DIR);

    vehicle_ = std::make_shared<chrono::vehicle::TrackedVehicle>(
        system_.get(), std::string(CHRONO_DEFAULT_VEHICLE_DATA_DIR) +
                           "M113/vehicle/M113_Vehicle_SinglePin.json");

    // Unlock the chassis and move the vehicle system to the specified initial
    // pose.
    if (!is_initial_pose_overridden_) {
        initial_pose_ = chrono::ChCoordsys(
            chrono::ChVector3<double>(configuration->GetValue<double>(
                                          "vehicle/initialPosition/x", 0.0),
                                      configuration->GetValue<double>(
                                          "vehicle/initialPosition/y", 0.0),
                                      configuration->GetValue<double>(
                                          "vehicle/initialPosition/z", 0.9)),
            chrono::QUNIT);
    }
    vehicle_->Initialize(initial_pose_);
    vehicle_->GetChassis()->SetFixed(false);
    auto engine = chrono::vehicle::ReadEngineJSON(
        std::string(CHRONO_DEFAULT_VEHICLE_DATA_DIR) +
        "M113/powertrain/M113_EngineShafts.json");
    auto transmission = chrono::vehicle::ReadTransmissionJSON(
        std::string(CHRONO_DEFAULT_VEHICLE_DATA_DIR) +
        "M113/powertrain/"
        "M113_AutomaticTransmissionShafts.json");
    auto powertrain = std::make_shared<chrono::vehicle::ChPowertrainAssembly>(
        engine, transmission);
    vehicle_->InitializePowertrain(powertrain);

    vehicle_->SetChassisVisualizationType(chrono::VisualizationType::NONE);
    vehicle_->SetSprocketVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetIdlerVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetSuspensionVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetIdlerWheelVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetRoadWheelVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetRollerVisualizationType(chrono::VisualizationType::MESH);
    vehicle_->SetTrackShoeVisualizationType(chrono::VisualizationType::MESH);

    system_->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

    system_->SetMaxPenetrationRecoverySpeed(1.5);
}

void TrackedVehicle::Initialize() {}

std::shared_ptr<chrono::vehicle::ChVehicle> TrackedVehicle::GetVehicle() {
    return vehicle_;
}

void TrackedVehicle::Synchronize(
    const double& time, const chrono::vehicle::DriverInputs driver_inputs,
    const chrono::vehicle::ChTerrain& terrain) {
    vehicle_->Synchronize(time, driver_inputs);
}

double TrackedVehicle::GetPositionX() {
    return vehicle_->GetChassis()->GetBody()->GetPos().x();
}

const chrono::ChVector3d& TrackedVehicle::GetPosition() {
    return vehicle_->GetChassis()->GetBody()->GetPos();
}

double TrackedVehicle::GetSteeringAngle() {
    throw std::invalid_argument(
        "Steering angle not implemented for tracked vehicles!");
}

void TrackedVehicle::AddActiveDomain(
    const std::shared_ptr<chrono::vehicle::SCMTerrain> terrain) {
    for (int side_index = 0; side_index < 2; ++side_index) {
        const auto side = static_cast<chrono::vehicle::VehicleSide>(side_index);
        for (unsigned int shoe_index = 0;
             shoe_index < vehicle_->GetNumTrackShoes(side); ++shoe_index) {
            const auto shoe = vehicle_->GetTrackShoe(side, shoe_index);
            const auto bounding_box =
                shoe->GetGroundContactGeometry().CalculateAABB();
            // TODO: Use AddActiveDomain from future Chrono release
            /*
            terrain->AddActiveDomain(shoe->GetShoeBody(),
                                 bounding_box.Center(),
                                 bounding_box.Size());
        */
        }
    }
}

double TrackedVehicle::GetSpeed() {
    return vehicle_->GetSpeed();
}

const VehicleType& TrackedVehicle::GetVehicleType() {
    return type_;
}

}  // namespace Models
}  // namespace DYNO
