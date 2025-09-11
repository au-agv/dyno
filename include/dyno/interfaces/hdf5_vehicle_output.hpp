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

#include <chrono>
#include <experimental/filesystem>
#include <memory>
#include <string>
#include <utility>

#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h>
#include <highfive/H5Easy.hpp>
#include <highfive/eigen.hpp>
#include <highfive/highfive.hpp>

#include <dyno/interfaces/vehicle_output.hpp>

namespace DYNO {
namespace Interfaces {

/**
 * @brief HDF5 file interface for vehicle simulation output.
 */
class HDF5VehicleOutput : public VehicleOutput {
  public:
    HDF5VehicleOutput(std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
                      std::shared_ptr<chrono::vehicle::ChDriver> driver,
                      std::shared_ptr<chrono::vehicle::ChTerrain> terrain);

    void Save(double time) override;

    void Dump() override;

    void Initialize() override;

    void AddResult(const std::string& key, const double& value);

    void AddMetadata(const std::string& key, const double& value);

  private:
    /** @brief HDF5 vehicle simulation output file handle. */
    std::shared_ptr<H5Easy::File> file_;

    HighFive::Group root_group_;

    HighFive::Group data_group_;

    HighFive::Group metadata_group_;

    HighFive::Group results_group_;

    void CreateDataset(HighFive::Group& group, const std::string& path);

    void CreateVectorDataset(HighFive::Group& group, const std::string& path);

    void SaveSimulationTime(const double& time);

    void SaveChassisState();

    void SaveWheelState();

    void SaveCommands();
};

} // namespace Interfaces
} // namespace DYNO