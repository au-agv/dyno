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

#include <dyno/interfaces/hdf5_vehicle_output.hpp>

namespace DYNO {
namespace Interfaces {

HDF5VehicleOutput::HDF5VehicleOutput(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    std::shared_ptr<chrono::vehicle::ChDriver> driver,
    std::shared_ptr<chrono::vehicle::ChTerrain> terrain)
    : VehicleOutput(vehicle, driver, terrain) {
    SPDLOG_INFO("Instantiating HDF5 vehicle simulation output ...");
}

void HDF5VehicleOutput::Initialize() {
    VehicleOutput::Initialize();

    // Create the HDF5 file.
    SPDLOG_INFO("Creating dataset under \"{}\" ...",
                (file_path_ + ".h5").c_str());
    file_ = std::make_shared<H5Easy::File>(file_path_ + ".h5",
                                           H5Easy::File::Overwrite);

    // Create the subgroups under the HDF5 root group.
    root_group_ = file_->getGroup("/");
    data_group_ = root_group_.createGroup("data");
    metadata_group_ = root_group_.createGroup("metadata");
    results_group_ = root_group_.createGroup("results");
}

void HDF5VehicleOutput::AddResult(const std::string& key, const double& value) {
    results_group_.createDataSet(key, value);
}

void HDF5VehicleOutput::AddMetadata(const std::string& key,
                                    const double& value) {
    metadata_group_.createDataSet(key, value);
}

void HDF5VehicleOutput::Save(double time) {
    SaveSimulationTime(time);
    SaveChassisState();
    // SaveWheelState();
    SaveCommands();
}

void HDF5VehicleOutput::Dump() {
    SPDLOG_INFO("Dumping vehicle simulation output to HDF5 file on disk ...");

    if(map_["time"].size() < 1) {
        SPDLOG_WARN(
            "Dumping empty data to H5 - did you reach the trigger position?");
    }

    CreateDataset(data_group_, "time");
    CreateVectorDataset(data_group_, "pose/position");
    CreateVectorDataset(data_group_, "pose/rotation");
    CreateVectorDataset(data_group_, "velocity/linear");
    CreateVectorDataset(data_group_, "velocity/angular");
    CreateVectorDataset(data_group_, "acceleration/linear");
    CreateVectorDataset(data_group_, "acceleration/angular");
    CreateDataset(data_group_, "commands/throttle");
    CreateDataset(data_group_, "commands/brake");
    CreateDataset(data_group_, "commands/steering");
    CreateDataset(data_group_, "commands/clutch");
}

void HDF5VehicleOutput::CreateDataset(HighFive::Group& group,
                                      const std::string& path) {
    group.createDataSet(path, map_[path]);
}

void HDF5VehicleOutput::CreateVectorDataset(HighFive::Group& group,
                                            const std::string& path) {
    Eigen::MatrixXd vector = Eigen::MatrixXd::Zero(map_[path + "/x"].size(), 3);

    std::vector<std::string> entries{"/x", "/y", "/z"};
    for(size_t index = 0; index < entries.size(); ++index) {
        vector.col(index) =
            Eigen::VectorXd::Map(map_[path + entries[index]].data(),
                                 map_[path + entries[index]].size());
    }

    group.createDataSet(path, vector);
}

void HDF5VehicleOutput::SaveSimulationTime(const double& time) {
    map_["time"].push_back(time);
}

void HDF5VehicleOutput::SaveChassisState() {
    auto body = vehicle_->GetChassisBody();

    auto position = vehicle_->GetPos();
    map_["pose/position/x"].push_back(position.x());
    map_["pose/position/y"].push_back(position.y());
    map_["pose/position/z"].push_back(position.z());

    auto rotation = vehicle_->GetRot().GetCardanAnglesXYZ();
    map_["pose/rotation/x"].push_back(rotation.x());
    map_["pose/rotation/y"].push_back(rotation.y());
    map_["pose/rotation/z"].push_back(rotation.z());

    auto linear_velocity = body->GetPosDt();
    auto rotation_matrix = body->GetRotMat();
    map_["velocity/linear/x"].push_back(
        Vdot(linear_velocity, rotation_matrix.GetAxisX()));
    map_["velocity/linear/y"].push_back(
        Vdot(linear_velocity, rotation_matrix.GetAxisY()));
    map_["velocity/linear/z"].push_back(
        Vdot(linear_velocity, rotation_matrix.GetAxisZ()));

    auto angular_velocity = body->GetAngVelLocal();
    map_["velocity/angular/x"].push_back(angular_velocity.x());
    map_["velocity/angular/y"].push_back(angular_velocity.y());
    map_["velocity/angular/z"].push_back(angular_velocity.z());

    auto linear_acceleration = vehicle_->GetChassisBody()->GetPosDt2();
    map_["acceleration/linear/x"].push_back(
        Vdot(linear_acceleration, rotation_matrix.GetAxisX()));
    map_["acceleration/linear/y"].push_back(
        Vdot(linear_acceleration, rotation_matrix.GetAxisY()));
    map_["acceleration/linear/z"].push_back(
        Vdot(linear_acceleration, rotation_matrix.GetAxisZ()));

    auto angular_acceleration = vehicle_->GetChassisBody()->GetAngAccLocal();
    map_["acceleration/angular/x"].push_back(angular_acceleration.x());
    map_["acceleration/angular/y"].push_back(angular_acceleration.y());
    map_["acceleration/angular/z"].push_back(angular_acceleration.z());
}

void HDF5VehicleOutput::SaveWheelState() {
    // TODO: Reintroduce wheel state
    /*
    int wheel_index = 0;
    for(int axle_index = 0; axle_index < 2; ++axle_index) {
        for(int side_index = 0; side_index < 2; ++side_index) {
            chrono::vehicle::VehicleSide side;
            if(side_index == 0) side = chrono::vehicle::VehicleSide::LEFT;
            else
                side = chrono::vehicle::VehicleSide::RIGHT;

            // Save wheel position components
            // ------------------------------
            auto wheel_position = vehicle_->GetAxle(axle_index)
                                      ->GetWheel(side)
                                      ->GetSpindle()
                                      ->GetPos();
            map_["wheels/" + std::to_string(wheel_index) + "/position/x"]
                .push_back(wheel_position.x());
            map_["wheels/" + std::to_string(wheel_index) + "/position/y"]
                .push_back(wheel_position.y());
            map_["wheels/" + std::to_string(wheel_index) + "/position/z"]
                .push_back(wheel_position.z());
            // ------------------------------

            // Save wheel rotation components
            // ------------------------------
            auto wheel_rotation = vehicle_->GetAxle(axle_index)
                                      ->GetWheel(side)
                                      ->GetSpindle()
                                      ->GetRot()
                                      .GetCardanAnglesXYZ();
            map_["wheels/" + std::to_string(wheel_index) + "/rotation/x"]
                .push_back(wheel_rotation.x());
            map_["wheels/" + std::to_string(wheel_index) + "/rotation/y"]
                .push_back(wheel_rotation.y());
            map_["wheels/" + std::to_string(wheel_index) + "/rotation/z"]
                .push_back(wheel_rotation.z());
            // ------------------------------

            // Save wheel rotational speed
            // ---------------------------
            auto wheel_speed = vehicle_->GetAxle(axle_index)
                                   ->GetWheel(side)
                                   ->GetSpindle()
                                   ->GetCoordsysDt()
                                   .pos.x();
            map_["wheels/" + std::to_string(wheel_index) + "/speed"].push_back(
                wheel_speed);
            // ---------------------------

            // Save tire forces and moments
            // ----------------------------
            auto wrench = vehicle_->GetAxle(axle_index)
                              ->GetWheel(side)
                              ->GetTire()
                              ->ReportTireForce(terrain_.get());

            map_["wheels/" + std::to_string(wheel_index) + "/force/x"]
                .push_back(wrench.force.x());
            map_["wheels/" + std::to_string(wheel_index) + "/force/y"]
                .push_back(wrench.force.y());
            map_["wheels/" + std::to_string(wheel_index) + "/force/z"]
                .push_back(wrench.force.z());

            map_["wheels/" + std::to_string(wheel_index) + "/moment/x"]
                .push_back(wrench.moment.x());
            map_["wheels/" + std::to_string(wheel_index) + "/moment/y"]
                .push_back(wrench.moment.y());
            map_["wheels/" + std::to_string(wheel_index) + "/moment/z"]
                .push_back(wrench.force.z());
            // ----------------------------

            wheel_index++;
        }
    }
        */
}

void HDF5VehicleOutput::SaveCommands() {
    map_["commands/throttle"].push_back(driver_->GetThrottle());
    map_["commands/brake"].push_back(driver_->GetBraking());
    map_["commands/steering"].push_back(driver_->GetSteering());
    map_["commands/clutch"].push_back(driver_->GetClutch());
}

} // namespace Interfaces
} // namespace DYNO