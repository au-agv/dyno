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

#include <dyno/models/wheeled_vehicle.hpp>

namespace DYNO {
namespace Models {

class Olav : public WheeledVehicle {
   public:
    Olav(std::shared_ptr<chrono::ChSystem> system);

    double GetSteeringAngleMax() const;

    /**
     * @brief Retrieves the chassis body.
     *
     * This method returns a shared pointer to the ChBodyAuxRef object
     * representing the chassis body.
     *
     * @return A shared pointer to the chrono::ChBodyAuxRef object representing
     * the chassis body.
     */
    // FIXME: Should this method perhaps be added to the parent class instead?
    std::shared_ptr<chrono::ChBodyAuxRef> GetChassisBody();

    /**
     * @brief Retrieves the speed controller parameters.
     *
     * This method returns a constant reference to the SpeedControllerTuning
     * object containing the parameters for the speed controller.
     *
     * @return A constant reference to the SpeedControllerTuning object.
     */
    const SpeedControllerTuning& GetSpeedControllerTuning() override;

    /**
     * @brief Retrieves the steering controller parameters.
     *
     * This method returns a constant reference to the SteeringControllerTuning
     * object containing the parameters for the steering controller.
     *
     * @return A constant reference to the SteeringControllerTuning object.
     */
    const SteeringControllerTuning& GetSteeringControllerTuning() override;

    /**
     * @brief Retrieves the aerodynamic properties.
     *
     * This method returns a constant reference to the AerodynamicProperties
     * object containing the properties related to aerodynamics.
     *
     * @return A constant reference to the AerodynamicProperties object.
     */
    const AerodynamicProperties& GetAerodynamicProperties() override;

#ifdef DYNO_HAS_SENSORS_SUPPORT
    /**
     * @brief Retrieves the reference frame of the IMU sensor.
     *
     * This method returns a constant reference to the ChFramed object
     * representing the reference frame of the IMU (Inertial Measurement Unit)
     * sensor.
     *
     * @return A constant reference to the chrono::ChFramed object.
     */
    const chrono::ChFramed& GetImuSensorReferenceFrame() override;

    /**
     * @brief Retrieves the parameters of the LiDAR sensor.
     *
     * This method returns a constant reference to the LidarParameters object
     * containing the parameters related to the LiDAR (Light Detection and
     * Ranging) sensor.
     *
     * @return A constant reference to the LidarParameters object.
     */
    const LidarParameters& GetLidarSensorParameters() override;

    /**
     * @brief Retrieves the reference frame of the LiDAR sensor.
     *
     * This method returns a constant reference to the ChFramed object
     * representing the reference frame of the LiDAR (Light Detection and
     * Ranging) sensor.
     *
     * @return A constant reference to the chrono::ChFramed object.
     */
    const chrono::ChFramed& GetLidarSensorReferenceFrame() override;

    /**
     * @brief Retrieves the parameters of the camera sensor.
     *
     * This method returns a constant reference to the CameraParameters object
     * containing the parameters related to the camera sensor.
     *
     * @return A constant reference to the CameraParameters object.
     */
    const CameraParameters& GetCameraSensorParameters() override;

    /**
     * @brief Retrieves the reference frame of the camera sensor.
     *
     * This method returns a constant reference to the ChFramed object
     * representing the reference frame of the camera sensor.
     *
     * @return A constant reference to the chrono::ChFramed object.
     */
    const chrono::ChFramed& GetCameraSensorReferenceFrame() override;
#endif

   private:
    /**
     * @brief The initial pose of an object.
     *
     * This member variable stores the initial position and orientation of an
     * object using a chrono::ChCoordsysd object.
     */
    chrono::ChCoordsysd initial_pose_;

    /**
     * @brief Parameters for the speed controller.
     *
     * This member variable contains the tuning parameters for the speed
     * controller, encapsulated in a SpeedControllerTuning object.
     */
    SpeedControllerTuning speed_controller_tuning_;

    /**
     * @brief Parameters for the steering controller.
     *
     * This member variable contains the tuning parameters for the steering
     * controller, encapsulated in a SteeringControllerTuning object.
     */
    SteeringControllerTuning steering_controller_tuning_;

    /**
     * @brief Aerodynamic properties of the vehicle.
     *
     * This member variable stores the aerodynamic properties of the vehicle,
     * encapsulated in an AerodynamicProperties object.
     */
    AerodynamicProperties aerodynamic_properties_;

#ifdef DYNO_HAS_SENSORS_SUPPORT
    /**
     * @brief The reference frame of the IMU (Inertial Measurement Unit) sensor.
     *
     * This member variable stores the reference frame of the IMU sensor using a
     * chrono::ChFramed object.
     */
    chrono::ChFramed imu_frame_;

    /**
     * @brief Parameters for the LiDAR (Light Detection and Ranging) sensor.
     *
     * This member variable contains the parameters for the LiDAR sensor,
     * encapsulated in a LidarParameters object.
     */
    LidarParameters lidar_parameters_;

    /**
     * @brief The reference frame of the LiDAR sensor.
     *
     * This member variable stores the reference frame of the LiDAR sensor using
     * a chrono::ChFramed object.
     */
    chrono::ChFramed lidar_frame_;

    /**
     * @brief Parameters for the camera sensor.
     *
     * This member variable contains the parameters for the camera sensor,
     * encapsulated in a CameraParameters object.
     */
    CameraParameters camera_parameters_;

    /**
     * @brief The reference frame of the camera sensor.
     *
     * This member variable stores the reference frame of the camera sensor
     * using a chrono::ChFramed object.
     */
    chrono::ChFramed camera_frame_;
#endif
};

}  // namespace Models
}  // namespace DYNO
