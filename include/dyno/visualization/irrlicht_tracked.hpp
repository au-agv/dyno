#pragma once

#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h>
#include <dyno/visualization/irrlicht.hpp>

namespace DYNO {
namespace Visualization {

class IrrlichtTracked : public Irrlicht {
  public:
    IrrlichtTracked(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
        : Irrlicht(vehicle, configuration) {
        ParseOptions();
    }

  protected:
    void ParseOptions() override {
        visualization_ = std::make_shared<
            chrono::vehicle::ChTrackedVehicleVisualSystemIrrlicht>();
    }
};

} // namespace Visualization
} // namespace DYNO