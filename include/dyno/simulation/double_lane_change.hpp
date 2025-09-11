#pragma once

#include <dyno/environments/asymmetric_friction_functor.hpp>
#include <dyno/environments/asymmetric_soil_parameters_callback.hpp>
#include <dyno/environments/double_lane_change.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

class DoubleLaneChange : public VehicleSimulation {
   public:
    DoubleLaneChange(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

   protected:
    void GetConfiguration() override;
    void Instantiate() override;
    void OverrideInitialPose() override;
    void InitializeDriver() override;
    void InitializeTerrain() override;
    void PostSynchronizationHook() override;
    // void WriteMetadata() override;

   private:
    std::shared_ptr<DYNO::Environments::DoubleLaneChange> dlc_;
    double acceleration_length_;
    double vehicle_length_ = 3.0;
    double vehicle_width_ = 3.0;
    double path_height_ = 0.5;
    bool left_turn_ = false;
    std::string terrain_type_;
    bool use_split_surface_ = false;
    double target_sideslope_;
};

}  // namespace Simulation
}  // namespace DYNO
