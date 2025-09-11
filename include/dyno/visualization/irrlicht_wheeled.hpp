#pragma once

#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h>
#include <dyno/visualization/irrlicht.hpp>

namespace DYNO {
namespace Visualization {

class IrrlichtWheeled : public Irrlicht {
  public:
    IrrlichtWheeled(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
        : Irrlicht(vehicle, configuration) {
        ParseOptions();
    }

  protected:
    void ParseOptions() override {
        visualization_ = std::make_shared<
            chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
    }
};

} // namespace Visualization
} // namespace DYNO