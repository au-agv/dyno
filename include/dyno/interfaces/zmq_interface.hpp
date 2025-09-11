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

#include <signal.h>

#include <chrono/core/ChFrame.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h>
#include <nlohmann/json.hpp>

#include <dyno/interfaces/zmq_socket.hpp>

#ifdef DYNO_HAS_SENSORS_SUPPORT
#include <dyno/sensors/imu.hpp>
#include <dyno/sensors/lidar_xyzi.hpp>
#include <dyno/sensors/rgba_camera.hpp>
#endif

namespace DYNO {
namespace Interfaces {

class ZMQInterface : public DYNO::Interfaces::ZMQSocket {
   public:
    /**
     * @brief Construct a new Project Chrono ROS interface.
     *
     * @param protocol Interface communication protocol
     * @param address Interface server address.
     * @param port Interface server port.
     */
    ZMQInterface(std::string protocol, std::string address, int port);

    /**
     * @brief Deserialize a ROS auton_msgs/msg/DriveByWire message in JSON
     * format.
     *
     * @param data auton_msgs/msg/DriveByWire JSON message.
     * @param throttle Normalised output throttle effort value in the range (0.0
     * - 1.0)
     * @param brake Normalised output brake effort value in the range (0.0
     * - 1.0)
     * @param steering Normalised output steering effort value in the range
     * (-1.0 - 1.0)
     */
    void DeserializeDriveByWire(nlohmann::json& data, double& throttle,
                                double& brake, double& steering);

#ifdef DYNO_HAS_SENSORS_SUPPORT
    /**
     * @brief Serialise an IMU sensor reading to a ROS sensor_msgs/msg/Imu
     * message in JSON format.
     *
     * @param data Reference to the JSON dictionary entry to populate.
     * @param imu Shared pointer to the IMU sensor.
     * @param frame_id Frame ID to use when populating the ROS message header.
     * @param linear_acceleration_covariance_diagonal Diagonal entries (as a
     * 3-by-1 vector) of the covariance matrix for linear acceleration of the
     * body.
     * @param angular_velocity_covariance_diagonal Diagonal entries (as a 3-by-1
     * vector) of the covariance matrix for the angular velocity of the body.
     */
    void SerializeIMU(
        nlohmann::json& data, std::shared_ptr<DYNO::Sensors::IMU> imu,
        std::string frame_id,
        std::vector<double> linear_acceleration_covariance_diagonal,
        std::vector<double> angular_velocity_covariance_diagonal);
#endif

    /**
     * @brief Serialize body accelerations to a ROS sensor_msgs/msg/Imu message
     * in JSON format.
     *
     * @param message Reference to a JSON dictionary to populate.
     * @param body Reference to the body from which accelerations are read.
     * @param frame_id
     * @param orientation_covariance_diagonal Diagonal entries (as a 3-by-1
     * vector) of the covariance matrix for the orientation of the body.
     * @param linear_acceleration_covariance_diagonal Diagonal entries (as a
     * 3-by-1 vector) of the covariance matrix for linear acceleration of the
     * body.
     * @param angular_velocity_covariance_diagonal Diagonal entries (as a 3-by-1
     * vector) of the covariance matrix for the angular velocity of the body.
     */
    void SerializeIdealIMU(
        nlohmann::json& message, std::shared_ptr<chrono::ChBody> body,
        std::string frame_id,
        std::vector<double> orientation_covariance_diagonal,
        std::vector<double> linear_acceleration_covariance_diagonal,
        std::vector<double> angular_velocity_covariance_diagonal);

    /**
     * @brief Serialize simulation clock to a ROS rosgraph_msgs/msg/Clock
     * message in JSON format.
     *
     * @param data Reference to the JSON dictionary entry to populate.
     */

    void SerializeClock(nlohmann::json& message);

    /**
     * @brief Serialize body odometry to a ROS nav_msgs/msg/Odometry message in
     * JSON format.
     *
     * @param message  Reference to the JSON dictionary to populate.
     * @param body Reference to the body from which the odometry is read.
     * @param frame_id Frame ID to use when populating the ROS message header.
     * @param child_frame_id Child frame ID to use when populating the ROS
     * message header.
     * @param pose_covariance Diagonal entries (as a 6-by-1 vector) of the
     * covariance matrix for the pose of the body.
     * @param twist_covariance Diagonal entries (as a 6-by-1 vector) of the
     * covariance matrix for the twist of the body.
     */
    void SerializeOdometry(nlohmann::json& message,
                           std::shared_ptr<chrono::ChBody> body,
                           std::string frame_id, std::string child_frame_id,
                           std::vector<double> pose_covariance_diagonal,
                           std::vector<double> twist_covariance_diagonal);

#ifdef DYNO_HAS_SENSORS_SUPPORT
    /**
     * @brief Serialize a LiDAR sensor reading to a ROS
     * sensor_msgs/msg/PointCloud2 message in JSON format.
     *
     * @param data Reference to the JSON dictionary to populate.
     * @param lidar Shared pointer to the LiDAR sensor.
     * @param frame_id Frame ID to use when populating the ROS message header.
     */
    void SerializePointCloud2(nlohmann::json& data,
                              std::shared_ptr<DYNO::Sensors::LidarXYZI> lidar,
                              std::string frame_id);

    void SerializeImage(nlohmann::json& data,
                        std::shared_ptr<DYNO::Sensors::RGBACamera> camera,
                        std::string frame_id);

    void SerializeCameraInfo(nlohmann::json& data,
                             std::shared_ptr<DYNO::Sensors::RGBACamera> camera);

#endif

    void SerializeTimeMessage(nlohmann::json& message);

    /**
     * @brief Serialize a the transform between two reference frames as a ROS
     * geometry_msgs/msg/PoseStamped message.
     *
     * @param message Reference to the JSON dictionary to populate.
     * @param frame Project Chrono frame expressing the transform.
     * @param frame_id Frame ID to use when populating the ROS message header.
     * @param child_frame_id Child frame ID to use when populating the ROS
     * message header.
     */
    void SerializeTransform(nlohmann::json& message,
                            chrono::ChFrame<double> frame, std::string frame_id,
                            std::string child_frame_id);

    /**
     * @brief Stamp a ROS std_msgs/msg/Header in JSON format with the provided
     * simulation time.
     *
     * @param data Reference to the JSON dictionary to populate.
     */
    void StampMessage(nlohmann::json& message);

    void AddTransform(chrono::ChFrame<double> transform, std::string frame_id,
                      std::string child_frame_id) {
        transforms_.push_back(transform);
        transforms_frame_id_.push_back(frame_id);
        transforms_child_frame_id_.push_back(child_frame_id);
    }

#ifdef DYNO_HAS_SENSORS_SUPPORT
    void Serialize(nlohmann::json& data,
                   std::shared_ptr<DYNO::Sensors::LidarXYZI> lidar,
                   std::shared_ptr<DYNO::Sensors::RGBACamera> camera);
#endif

    void SetSimulationTime(double time);

    std::pair<int32_t, uint32_t> GetSimulationTime();

    void SetChassisBody(std::shared_ptr<chrono::ChBodyAuxRef> body) {
        chassis_body_ = body;
    }

    static void HandleSignal(int signal_value) {
        (void)signal_value;
        is_interrupted_ = 1;
    }

    static void CatchSignals(void) {
        is_interrupted_ = 0;
        struct sigaction action;
        action.sa_handler = HandleSignal;
        action.sa_flags = 0;
        sigemptyset(&action.sa_mask);
        sigaction(SIGINT, &action, NULL);
        sigaction(SIGTERM, &action, NULL);
    }

    bool IsInterrupted() { return (bool)is_interrupted_; }

   private:
    /** @brief Shared pointer to the body used as reference for the published
     * transforms and odometry messages. */
    std::shared_ptr<chrono::ChBodyAuxRef> chassis_body_;

    /** @brief Simulation time in seconds as expressed in a ROS std_msgs/Time
     * message. */
    int32_t seconds_ = 0;

    /** @brief Nanoseconds since the last simulation time in seconds as
     * expressed in a ROS std_msgs/Time message. */
    uint32_t nanoseconds_ = 0;

    std::vector<chrono::ChFrame<double>> transforms_;

    std::vector<std::string> transforms_frame_id_;

    std::vector<std::string> transforms_child_frame_id_;

    std::string odometry_frame_id_ = "odom";

    std::string odometry_child_frame_id_ = "base_link";

    /** @brief Static variable to capture process signals. */
    static int is_interrupted_;
};

}  // namespace Interfaces
}  // namespace DYNO
