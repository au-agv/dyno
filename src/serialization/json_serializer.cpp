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

#include <dyno/serialization/json_serializer.hpp>

namespace DYNO {
namespace Serialization {

JSONSerializer::JSONSerializer(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    std::shared_ptr<chrono::vehicle::ChDriver> driver,
    std::shared_ptr<chrono::vehicle::ChTerrain> terrain)
    : Serializer(vehicle, driver, terrain) {}

void JSONSerializer::Dump() {
    SPDLOG_INFO("Saving results file {}", filename_);

    data_["metadata"]["type"] = "vehicle";

    std::string subfolder_path;
    if (add_timestamp_) {
        if (use_name_generator_) {
            subfolder_path = filename_ + "-" + date_ + "-" + time_ + "-" +
                             adjectives_[random_adjective_index_] + "-" +
                             names_[random_name_index_];
        } else {
            subfolder_path = filename_ + "-" + date_;
        }
    } else {
        if (use_name_generator_) {
            subfolder_path = filename_ + "-" +
                             adjectives_[random_adjective_index_] + "-" +
                             names_[random_name_index_];
        } else {
            subfolder_path = filename_;
        }
    }
    std::string full_path(path_ + subfolder_path);
    std::string file_path(full_path + "/" + subfolder_path + ".json");
    std::experimental::filesystem::create_directory(full_path);

    std::ofstream file(file_path.c_str());
    file << data_;
}

void JSONSerializer::Save(double time) {
    data_["data"]["time"].push_back(time_);

    auto body = vehicle_->GetChassisBody();

    auto position = vehicle_->GetPos();
    auto rotation = vehicle_->GetRot().GetCardanAnglesXYZ();
    data_["data"]["pose_position_x"].push_back(position.x());
    data_["data"]["pose_position_y"].push_back(position.y());
    data_["data"]["pose_position_z"].push_back(position.z());
    data_["data"]["pose_orientation_x"].push_back(rotation.x());
    data_["data"]["pose_orientation_y"].push_back(rotation.y());
    data_["data"]["pose_orientation_z"].push_back(rotation.z());

    auto linear_velocity = vehicle_->GetChassisBody()->GetPosDt();
    auto angular_velocity =
        vehicle_->GetChassisBody()->GetRotDt().GetCardanAnglesXYZ();
    data_["data"]["velocity_linear_x"].push_back(
        Vdot(body->GetPosDt(), body->GetRotMat().GetAxisX()));
    data_["data"]["velocity_linear_y"].push_back(
        Vdot(body->GetPosDt(), body->GetRotMat().GetAxisY()));
    data_["data"]["velocity_linear_z"].push_back(
        Vdot(body->GetPosDt(), body->GetRotMat().GetAxisZ()));
    data_["data"]["velocity_angular_x"].push_back(angular_velocity.x());
    data_["data"]["velocity_angular_y"].push_back(angular_velocity.y());
    data_["data"]["velocity_angular_z"].push_back(angular_velocity.z());

    auto linear_acceleration = vehicle_->GetChassisBody()->GetPosDt2();
    auto angular_acceleration =
        vehicle_->GetChassisBody()->GetRotDt2().GetCardanAnglesXYZ();
    data_["data"]["acceleration_linear_x"].push_back(
        Vdot(body->GetPosDt2(), body->GetRotMat().GetAxisX()));
    data_["data"]["acceleration_linear_y"].push_back(
        Vdot(body->GetPosDt2(), body->GetRotMat().GetAxisY()));
    data_["data"]["acceleration_linear_z"].push_back(
        Vdot(body->GetPosDt2(), body->GetRotMat().GetAxisZ()));
    data_["data"]["acceleration_angular_x"].push_back(angular_acceleration.x());
    data_["data"]["acceleration_angular_y"].push_back(angular_acceleration.y());
    data_["data"]["acceleration_angular_z"].push_back(angular_acceleration.z());

    // TODO: Reintroduce tire forces.
    /*
    int index = 0;
    for(int i = 0; i < 2; ++i) {
        for(int j = 0; j < 2; ++j) {
            chrono::vehicle::VehicleSide side;
            if(j == 0) side = chrono::vehicle::VehicleSide::LEFT;
            else
                side = chrono::vehicle::VehicleSide::RIGHT;
            auto wrench = vehicle_->GetAxle(i)
                              ->GetWheel(side)
                              ->GetTire()
                              ->ReportTireForce(terrain_.get());

            auto wheel_position =
                vehicle_->GetAxle(i)->GetWheel(side)->GetSpindle()->GetPos();
            data_["data"]["wheels"][index]["position"]["x"].push_back(
                wheel_position.x());
            data_["data"]["wheels"][index]["position"]["y"].push_back(
                wheel_position.y());
            data_["data"]["wheels"][index]["position"]["z"].push_back(
                wheel_position.z());

            auto wheel_rotation = vehicle_->GetAxle(i)
                                      ->GetWheel(side)
                                      ->GetSpindle()
                                      ->GetRot()
                                      .GetCardanAnglesXYZ();
            data_["data"]["wheels"][index]["rotation"]["x"].push_back(
                wheel_rotation.x());
            data_["data"]["wheels"][index]["rotation"]["y"].push_back(
                wheel_rotation.y());
            data_["data"]["wheels"][index]["rotation"]["z"].push_back(
                wheel_rotation.z());

            auto wheel_speed = vehicle_->GetAxle(i)
                                   ->GetWheel(side)
                                   ->GetSpindle()
                                   ->GetCoordsysDt()
                                   .pos.x();
            data_["data"]["wheels"][index]["speed"].push_back(wheel_speed);

            data_["data"]["wheels"][index]["force"]["x"].push_back(
                wrench.force.x());
            data_["data"]["wheels"][index]["force"]["y"].push_back(
                wrench.force.y());
            data_["data"]["wheels"][index]["force"]["z"].push_back(
                wrench.force.z());
            data_["data"]["wheels"][index]["moment"]["x"].push_back(
                wrench.moment.x());
            data_["data"]["wheels"][index]["moment"]["y"].push_back(
                wrench.moment.y());
            data_["data"]["wheels"][index]["moment"]["z"].push_back(
                wrench.force.z());

            index++;
        }
    }
    */

    data_["data"]["commands_throttle"].push_back(driver_->GetThrottle());
    data_["data"]["commands_brake"].push_back(driver_->GetBraking());
    data_["data"]["commands_steering"].push_back(driver_->GetSteering());
}

}  // namespace Serialization
}  // namespace DYNO
