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

#include <dyno/interfaces/zmq_interface.hpp>

namespace DYNO {
namespace Interfaces {

int ZMQInterface::is_interrupted_ = 0;

ZMQInterface::ZMQInterface(std::string protocol, std::string address, int port)
    : ZMQSocket(protocol, address, port) {}

void ZMQInterface::DeserializeDriveByWire(nlohmann::json& data,
                                          double& throttle, double& brake,
                                          double& steering) {
    throttle = data["throttle"].get<double>();
    brake = data["brake"].get<double>();
    steering = data["steering"].get<double>();
}

#ifdef DYNO_HAS_SENSORS_SUPPORT
void ZMQInterface::SerializeIMU(
    nlohmann::json& data, std::shared_ptr<DYNO::Sensors::IMU> imu,
    std::string frame_id,
    std::vector<double> linear_acceleration_covariance_diagonal,
    std::vector<double> angular_velocity_covariance_diagonal) {
    // Retrieve the latest sensor readings from the buffers.
    auto acc_buff = imu->GetAccelerometerBuffer();
    auto gyro_buff = imu->GetGyroscopeBuffer();

    if (acc_buff->Buffer && gyro_buff->Buffer) {
        data["header"]["frame_id"] = frame_id;

        data["linear_acceleration"]["x"] = acc_buff->Buffer[0].X;
        data["linear_acceleration"]["y"] = acc_buff->Buffer[0].Y;
        data["linear_acceleration"]["z"] = acc_buff->Buffer[0].Z;

        // Populate the linear acceleration covariance entry of the
        // sensor_msgs/Imu message.
        std::vector<double> linear_acceleration_covariance(9, 0.0);
        linear_acceleration_covariance[0] =
            linear_acceleration_covariance_diagonal[0];
        linear_acceleration_covariance[4] =
            linear_acceleration_covariance_diagonal[1];
        linear_acceleration_covariance[8] =
            linear_acceleration_covariance_diagonal[2];
        data["linear_acceleration_covariance"] = 0.0;

        data["angular_velocity"]["x"] = gyro_buff->Buffer[0].Roll;
        data["angular_velocity"]["y"] = gyro_buff->Buffer[0].Pitch;
        data["angular_velocity"]["z"] = gyro_buff->Buffer[0].Yaw;

        // Populate the angular velocity covariance entry of the sensor_msgs/Imu
        // message.
        std::vector<double> angular_velocity_covariance(9, 0.0);
        angular_velocity_covariance[0] =
            angular_velocity_covariance_diagonal[0];
        angular_velocity_covariance[4] =
            angular_velocity_covariance_diagonal[1];
        angular_velocity_covariance[8] =
            angular_velocity_covariance_diagonal[2];
        data["angular_velocity_covariance"] = angular_velocity_covariance;
    }
}
#endif

void ZMQInterface::SerializeIdealIMU(
    nlohmann::json& message, std::shared_ptr<chrono::ChBody> body,
    std::string frame_id, std::vector<double> orientation_covariance_diagonal,
    std::vector<double> linear_acceleration_covariance_diagonal,
    std::vector<double> angular_velocity_covariance_diagonal) {
    // Populate the sensor_msgs/Imu message header.
    message["header"]["frame_id"] = frame_id;

    // Populate the orientation entries of the sensor_msgs/Imu message.
    message["orientation"]["x"] = body->GetRot().e1();
    message["orientation"]["y"] = body->GetRot().e2();
    message["orientation"]["z"] = body->GetRot().e3();
    message["orientation"]["w"] = body->GetRot().e0();

    // Populate the orientation covariance entry of the sensor_msgs/Imu
    // message.
    std::vector<double> orientation_covariance(9, 0.0);
    orientation_covariance[0] = orientation_covariance_diagonal[0];
    orientation_covariance[4] = orientation_covariance_diagonal[1];
    orientation_covariance[8] = orientation_covariance_diagonal[2];
    message["orientation_covariance"] = orientation_covariance;

    // Populate the linear acceleration entries of the sensor_msgs/Imu message.
    message["linear_acceleration"]["x"] = body->GetPosDt2().x();
    message["linear_acceleration"]["y"] = body->GetPosDt2().y();
    message["linear_acceleration"]["z"] = body->GetPosDt2().z();

    // Populate the linear acceleration covariance entry of the sensor_msgs/Imu
    // message.
    std::vector<double> linear_acceleration_covariance(9, 0.0);
    linear_acceleration_covariance[0] =
        linear_acceleration_covariance_diagonal[0];
    linear_acceleration_covariance[4] =
        linear_acceleration_covariance_diagonal[1];
    linear_acceleration_covariance[8] =
        linear_acceleration_covariance_diagonal[2];
    message["linear_acceleration_covariance"] = linear_acceleration_covariance;

    // Populate the angular velocity entries of the sensor_msgs/Imu message.
    auto angular_velocity = body->GetRotDt().GetCardanAnglesXYZ();
    message["angular_velocity"]["x"] = angular_velocity.x();
    message["angular_velocity"]["y"] = angular_velocity.y();
    message["angular_velocity"]["z"] = angular_velocity.z();

    // Populate the angular velocity covariance entry of the sensor_msgs/Imu
    // message.
    std::vector<double> angular_velocity_covariance(9, 0.0);
    angular_velocity_covariance[0] = angular_velocity_covariance_diagonal[0];
    angular_velocity_covariance[4] = angular_velocity_covariance_diagonal[1];
    angular_velocity_covariance[8] = angular_velocity_covariance_diagonal[2];
    message["angular_velocity_covariance"] = angular_velocity_covariance;
}

void ZMQInterface::SerializeOdometry(
    nlohmann::json& message, std::shared_ptr<chrono::ChBody> body,
    std::string frame_id, std::string child_frame_id,
    std::vector<double> pose_covariance_diagonal,
    std::vector<double> twist_covariance_diagonal) {
    // Populate the odometry message header.
    message["header"]["frame_id"] = frame_id;
    message["child_frame_id"] = child_frame_id;

    // Populate the pose entries.
    message["pose"]["pose"]["position"]["x"] =
        body->GetFrameRefToAbs().GetPos().x();
    message["pose"]["pose"]["position"]["y"] =
        body->GetFrameRefToAbs().GetPos().y();
    message["pose"]["pose"]["position"]["z"] =
        body->GetFrameRefToAbs().GetPos().z();
    message["pose"]["pose"]["orientation"]["x"] =
        body->GetFrameRefToAbs().GetRot().e1();
    message["pose"]["pose"]["orientation"]["y"] =
        body->GetFrameRefToAbs().GetRot().e2();
    message["pose"]["pose"]["orientation"]["z"] =
        body->GetFrameRefToAbs().GetRot().e3();
    message["pose"]["pose"]["orientation"]["w"] =
        body->GetFrameRefToAbs().GetRot().e0();

    // Populate the flattened pose covariance matrix entry.
    std::vector<double> pose_covariance(36, 1.0e-3);
    pose_covariance[0] = pose_covariance_diagonal[0];
    pose_covariance[7] = pose_covariance_diagonal[1];
    pose_covariance[14] = pose_covariance_diagonal[2];
    pose_covariance[21] = pose_covariance_diagonal[3];
    pose_covariance[28] = pose_covariance_diagonal[4];
    pose_covariance[35] = pose_covariance_diagonal[5];
    message["pose"]["covariance"] = pose_covariance;

    // Populate the linear entries of the twist message.
    message["twist"]["twist"]["linear"]["x"] =
        Vdot(body->GetPosDt(), body->GetFrameRefToAbs().GetRotMat().GetAxisX());
    message["twist"]["twist"]["linear"]["y"] =
        Vdot(body->GetPosDt(), body->GetFrameRefToAbs().GetRotMat().GetAxisY());
    message["twist"]["twist"]["linear"]["z"] =
        Vdot(body->GetPosDt(), body->GetFrameRefToAbs().GetRotMat().GetAxisZ());

    // Populate the angular entries of the twist message.
    auto angular_velocity = body->GetRotDt().GetCardanAnglesXYZ();
    message["twist"]["twist"]["angular"]["x"] = angular_velocity.x();
    message["twist"]["twist"]["angular"]["y"] = angular_velocity.y();
    message["twist"]["twist"]["angular"]["z"] = angular_velocity.z();

    // Populate the flattened twist covariance matrix.
    std::vector<double> twist_covariance(36, 1.0e-3);
    twist_covariance[0] = twist_covariance_diagonal[0];
    twist_covariance[7] = twist_covariance_diagonal[1];
    twist_covariance[14] = twist_covariance_diagonal[2];
    twist_covariance[21] = twist_covariance_diagonal[3];
    twist_covariance[28] = twist_covariance_diagonal[4];
    twist_covariance[35] = twist_covariance_diagonal[5];
    message["twist"]["covariance"] = twist_covariance;
}

#ifdef DYNO_HAS_SENSORS_SUPPORT
void ZMQInterface::SerializePointCloud2(
    nlohmann::json& data, std::shared_ptr<DYNO::Sensors::LidarXYZI> lidar,
    std::string frame_id) {
    // Retrieve the latest LiDAR point cloud data to a position/intensity
    // buffer.
    chrono::sensor::UserXYZIBufferPtr pcl_buffer =
        lidar->GetSensor()
            ->GetMostRecentBuffer<chrono::sensor::UserXYZIBufferPtr>();

    // Define the geometry of the point cloud buffer.
    size_t pcl_buffer_size = pcl_buffer->Height * pcl_buffer->Width;
    size_t pcl_buffer_step = 16;  // 24

    // Initialise a vector to store the point cloud data for serialisation.
    std::vector<uint8_t> pcl_data(pcl_buffer_step * pcl_buffer_size);
    std::vector<float> ring(pcl_buffer_size, 0.0);

    if (pcl_buffer->Buffer) {
        // We need to traverse the entire buffer to convert the LiDAR
        // returns from floating point values to bytes as specified in the
        // ROS sensor_msgs/msg/PointField specificaition.
        for (size_t i = 0; i < pcl_buffer_size; ++i) {
            std::memcpy(&pcl_data[pcl_buffer_step * i],
                        &pcl_buffer->Buffer[i].x, sizeof(float));
            // In the following two commands I am using a funny notation to
            // write the offset by 4 bytes and 8 bytes respectively.
            std::memcpy(&pcl_data[pcl_buffer_step * i + 4],
                        &pcl_buffer->Buffer[i].y, sizeof(float));
            std::memcpy(&pcl_data[pcl_buffer_step * i + 8],
                        &pcl_buffer->Buffer[i].z, sizeof(float));
            std::memcpy(&pcl_data[pcl_buffer_step * i + 12],
                        &pcl_buffer->Buffer[i].intensity, sizeof(float));
        }
    } else {
        throw std::runtime_error(
            "LiDAR buffer not available during serialization.");
    }

    data["header"]["frame_id"] = frame_id;
    data["height"] = pcl_buffer->Height;
    data["width"] = pcl_buffer->Width;

    // Populate the point cloud fields.
    auto fields = std::vector<std::string>{"x", "y", "z", "intensity"};
    for (size_t i = 0; i < fields.size(); ++i) {
        nlohmann::json pointfield;
        pointfield["name"] = fields[i];
        pointfield["offset"] = 4 * i;
        pointfield["datatype"] = 7;
        pointfield["count"] = 1;
        data["fields"].push_back(pointfield);
    }

    data["is_bigendian"] = false;
    data["point_step"] = 16;  // 24
    data["row_step"] = 1;

    data["data"] = nlohmann::json::binary_t(pcl_data);

    data["is_dense"] = true;
}

void ZMQInterface::SerializeImage(
    nlohmann::json& data, std::shared_ptr<DYNO::Sensors::RGBACamera> camera,
    std::string frame_id) {
    // Access the camera RGBA 8-bit buffer.
    chrono::sensor::UserRGBA8BufferPtr rgba8_ptr =
        camera->GetSensor()
            ->GetMostRecentBuffer<chrono::sensor::UserRGBA8BufferPtr>();

    data["header"]["frame_id"] = frame_id;
    data["height"] = rgba8_ptr->Height;
    data["width"] = rgba8_ptr->Width;
    data["encoding"] = "rgba8";
    data["step"] = rgba8_ptr->Width * 4;
    data["is_bigendian"] = false;

    uint img_buffer_step = 4;
    uint img_buffer_size = rgba8_ptr->Height * rgba8_ptr->Width;
    std::vector<uint8_t> image_data(img_buffer_step * img_buffer_size);

    if (rgba8_ptr->Buffer) {
        for (size_t i = 0; i < img_buffer_size; ++i) {
            std::memcpy(&image_data[img_buffer_step * i],
                        &rgba8_ptr->Buffer[i].R, sizeof(uint8_t));
            // In the following commands I am using a funny notation to
            // write the offset by 4 bytes and 8 bytes respectively.
            std::memcpy(&image_data[img_buffer_step * i + 1],
                        &rgba8_ptr->Buffer[i].G, sizeof(uint8_t));
            std::memcpy(&image_data[img_buffer_step * i + 2],
                        &rgba8_ptr->Buffer[i].B, sizeof(uint8_t));
            std::memcpy(&image_data[img_buffer_step * i + 3],
                        &rgba8_ptr->Buffer[i].A, sizeof(uint8_t));
        }

        data["data"] = nlohmann::json::binary_t(image_data);
    }
}

void ZMQInterface::SerializeCameraInfo(
    nlohmann::json& data, std::shared_ptr<DYNO::Sensors::RGBACamera> camera) {
    data["height"] = camera->GetSensor()->GetHeight();
    data["width"] = camera->GetSensor()->GetWidth();
    data["distortion_model"] = "plumb bob";

    // Compute the camera focal lenses.
    double vfov = (0.5 * camera->GetSensor()->GetHeight()) /
                  ((0.5 * camera->GetSensor()->GetWidth()) /
                   (std::tan(0.5 * camera->GetSensor()->GetHFOV())));
    double f_x = camera->GetSensor()->GetWidth() /
                 tan(camera->GetSensor()->GetHFOV() / 2.0);
    double f_y = camera->GetSensor()->GetHeight() / tan(vfov / 2.0);
    double c_x = camera->GetSensor()->GetWidth() / 2.0;
    double c_y = camera->GetSensor()->GetHeight() / 2.0;

    // Compute the projection matrix.
    std::vector<double> K{f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0};
    data["K"] = K;

    std::vector<double> R{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    data["R"] = R;

    std::vector<double> P{f_x, 0.0, c_x, 0.0, 0.0, f_y,
                          c_y, 0.0, 0.0, 0.0, 1.0, 0.0};

    data["P"] = P;
    data["binning_x"] = 0.0;
    data["binning_y"] = 0.0;
    data["roi"]["x_offset"] = 0;
    data["roi"]["y_offset"] = 0;
    data["roi"]["height"] = 0;
    data["roi"]["do_rectify"] = false;
}
#endif

void ZMQInterface::SerializeTimeMessage(nlohmann::json& message) {
    message["sec"] = seconds_;
    message["nanosec"] = nanoseconds_;
}

void ZMQInterface::SerializeClock(nlohmann::json& message) {
    SerializeTimeMessage(message["clock"]);
}

void ZMQInterface::StampMessage(nlohmann::json& message) {
    SerializeTimeMessage(message["header"]["stamp"]);
}

void ZMQInterface::SerializeTransform(nlohmann::json& message,
                                      chrono::ChFrame<double> frame,
                                      std::string frame_id,
                                      std::string child_frame_id) {
    // Populate the header of the geometry_msgs/TransformStamped message.
    message["header"]["frame_id"] = frame_id;
    message["child_frame_id"] = child_frame_id;

    // Populate the transform entries of the geometry_msgs/TransformStamped
    // message.
    message["transform"]["translation"]["x"] = frame.GetPos().x();
    message["transform"]["translation"]["y"] = frame.GetPos().y();
    message["transform"]["translation"]["z"] = frame.GetPos().z();
    message["transform"]["rotation"]["x"] = frame.GetRot().e1();
    message["transform"]["rotation"]["y"] = frame.GetRot().e2();
    message["transform"]["rotation"]["z"] = frame.GetRot().e3();
    message["transform"]["rotation"]["w"] = frame.GetRot().e0();
}

std::pair<int32_t, uint32_t> ZMQInterface::GetSimulationTime() {
    return std::pair<int32_t, uint32_t>(seconds_, nanoseconds_);
}

void ZMQInterface::SetSimulationTime(double time) {
    double integer_part, fractional_part;
    fractional_part = std::modf(time, &integer_part);
    seconds_ = (int32_t)integer_part,
    nanoseconds_ = (uint32_t)std::round(fractional_part * 1000000000);
}

#ifdef DYNO_HAS_SENSORS_SUPPORT
void ZMQInterface::Serialize(
    nlohmann::json& data, std::shared_ptr<DYNO::Sensors::LidarXYZI> lidar,
    std::shared_ptr<DYNO::Sensors::RGBACamera> camera) {
    SerializeClock(data["clock"]);

    SerializeOdometry(data["odometry"], chassis_body_, "odom", "base_link",
                      std::vector<double>(9, 0.1), std::vector<double>(9, 0.1));
    StampMessage(data["odometry"]);

    // Serialize the static transforms.
    for (size_t i = 0; i < transforms_.size(); ++i) {
        nlohmann::json transform;
        SerializeTransform(transform, transforms_[i], transforms_frame_id_[i],
                           transforms_child_frame_id_[i]);
        StampMessage(transform);
        data["transforms"].push_back(transform);
    }

    // Serialize the odom -> base_link transform.
    nlohmann::json transform;
    SerializeTransform(
        transform,
        chrono::ChFrame<double>(chassis_body_->GetFrameRefToAbs().GetPos(),
                                chassis_body_->GetFrameRefToAbs().GetRot()),
        "odom", "base_link");
    StampMessage(transform);
    data["transforms"].push_back(transform);

    SerializeIdealIMU(
        data["imu"], chassis_body_, "imu", std::vector<double>{0.0, 0.0, 0.0},
        std::vector<double>{0.0, 0.0, 0.0}, std::vector<double>{0.0, 0.0, 0.0});
    StampMessage(data["imu"]);

    SerializePointCloud2(data["lidar"], lidar, "lidar");
    StampMessage(data["lidar"]);

    SerializeImage(data["camera"], camera, "camera");
    StampMessage(data["camera"]);
}
#endif

}  // namespace Interfaces
}  // namespace DYNO
