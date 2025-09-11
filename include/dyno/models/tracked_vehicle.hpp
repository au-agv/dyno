#pragma once

#include <chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h>
#include <chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/models/vehicle.hpp>

namespace DYNO {
namespace Models {

class TrackedVehicle : public Vehicle {
  public:
    TrackedVehicle(std::shared_ptr<chrono::ChSystem> system);

    double GetSteeringAngleMax() const;

    void
    Setup(std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    void Initialize();

    void OverrideInitialPose(const chrono::ChCoordsysd& initial_pose) {
        initial_pose_ = initial_pose;
        is_initial_pose_overridden_ = true;
    }

    std::shared_ptr<chrono::vehicle::ChVehicle> GetVehicle();

    double GetPositionX();

    const chrono::ChVector3d& GetPosition();

    double GetSteeringAngle();

    void Synchronize(const double& time,
                     const chrono::vehicle::DriverInputs driver_inputs,
                     const chrono::vehicle::ChTerrain& terrain) override;

    std::shared_ptr<chrono::ChBody>
    GetActiveDomainBody(const int& axle,
                        chrono::vehicle::VehicleSide side) const;

    void AddActiveDomain(
        const std::shared_ptr<chrono::vehicle::SCMTerrain> terrain) override;

    const VehicleType& GetVehicleType();

    double GetSpeed();

    double GetVelocity();


  protected:
    std::shared_ptr<chrono::vehicle::ChTrackedVehicle> vehicle_;
    VehicleType type_ = VehicleType::TRACKED;
};

} // namespace Models
} // namespace DYNO