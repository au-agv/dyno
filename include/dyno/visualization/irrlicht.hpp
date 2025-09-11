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

#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h>
// TODO: Add this include from future Chrono release
//#include <chrono_vehicle/visualization/ChVehicleVisualSystemIrrlicht.h>

#include <spdlog/spdlog.h>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/visualization/wrapper.hpp>

namespace DYNO {
namespace Visualization {

class Irrlicht : public Wrapper {
  public:
    Irrlicht(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    void Initialize();

    void Advance(const double& time);

    void Synchronize(const double& time,
                     const chrono::vehicle::DriverInputs& driver_inputs);

  protected:
    virtual void ParseOptions() = 0;

    std::shared_ptr<chrono::vehicle::ChVehicleVisualSystemIrrlicht>
        visualization_;
};

} // namespace Visualization
} // namespace DYNO