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

#include <cctype>
#include <memory>

#include <chrono/assets/ChVisualSystem.h>
#include <chrono_vehicle/ChDriver.h>
#include <chrono_vehicle/ChTerrain.h>
#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/ChVehicleDataPath.h>
#include <chrono_vehicle/terrain/SCMTerrain.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>
#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>

#include <dyno/exceptions/exceptions.hpp>
#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/models/aerodynamic_properties.hpp>

#include <dyno/models/speed_controller_tuning.hpp>
#include <dyno/models/steering_controller_tuning.hpp>

#ifdef DYNO_HAS_SENSORS_SUPPORT
#include <dyno/models/camera_parameters.hpp>
#include <dyno/models/imu_parameters.hpp>
#include <dyno/models/lidar_parameters.hpp>
#endif

namespace DYNO {
namespace Models {

enum VehicleType { WHEELED = 0, TRACKED = 1 };

class Vehicle {
   public:
    Vehicle(std::shared_ptr<chrono::ChSystem> system);

    virtual void Setup(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration) = 0;

    virtual void Initialize() = 0;

    virtual double GetSteeringAngleMax() const = 0;

    virtual std::shared_ptr<chrono::vehicle::ChVehicle> GetVehicle() = 0;

    /**
     * @brief Overrides the initial pose of the vehicle.
     *
     * This method sets the initial pose of the vehicle. It also sets a flag
     * indicating that the initial pose has been overridden.
     *
     * @param initial_pose A constant reference to the
     * chrono::ChCoordsysd object representing the new initial pose.
     */
    void OverrideInitialPose(const chrono::ChCoordsysd& initial_pose);

    /**
     * @brief Retrieves the current speed.
     *
     * This pure virtual method is responsible for returning the current speed.
     * Derived classes must implement this method to provide specific logic for
     * retrieving the speed.
     *
     * @return The current speed in meters per second (m/s).
     */
    virtual double GetSpeed() = 0;

    virtual double GetVelocity() = 0;

    /**
     * @brief Retrieves the current vehicle position.
     *
     * This pure virtual method is responsible for returning the current vehicle
     * position as a ChVector3d object.
     *
     * Derived classes must implement this method to provide specific logic for
     * retrieving the position.
     *
     * @return A constant reference to the chrono::ChVector3d object
     * representing the current vehicle position.
     */
    virtual const chrono::ChVector3d& GetPosition() = 0;

    /**
     * @brief Retrieves the X-coordinate of the current position of the vehicle.
     *
     * This pure virtual method is responsible for returning the X-coordinate of
     * the current position of the vehicle. Derived classes must implement this
     * method to provide specific logic for retrieving the X-coordinate.
     *
     * @return A double representing the X-coordinate of the current position of
     * the vehicle.
     */
    // TODO: This method should be deprecated in favour of the more general
    // chrono::ChVector3d& GetPosition() one.
    virtual double GetPositionX() = 0;

    /**
     * @brief Retrieves the current steering angle.
     *
     * This pure virtual method is responsible for returning the current
     * steering angle. Derived classes must implement this method to provide
     * specific logic for retrieving the steering angle.
     *
     * @return The current steering angle in radians.
     */
    virtual double GetSteeringAngle() = 0;

    virtual void Synchronize(const double& time,
                             const chrono::vehicle::DriverInputs driver_inputs,
                             const chrono::vehicle::ChTerrain& terrain) = 0;

    /**
     * @brief Adds an active domain to the simulation.
     *
     * This pure virtual method is responsible for adding an active domain to
     * the SCMTerrain object. Derived classes must implement this method to
     * provide specific logic for adding the active domains (e.g. wheels, track
     * shoes, skid plates, etc.).
     *
     * @param terrain A shared pointer to the chrono::vehicle::SCMTerrain object
     * representing the terrain to which the active domain will be added.
     */
    virtual void AddActiveDomain(
        const std::shared_ptr<chrono::vehicle::SCMTerrain> terrain) = 0;

    /**
     * @brief Retrieves the type of the vehicle.
     *
     * This pure virtual method is responsible for returning the type of the
     * vehicle. Derived classes must implement this method to provide specific
     * logic for retrieving the vehicle type.
     *
     * @return A constant reference to a VehicleType object representing the
     * type of the vehicle.
     */
    virtual const VehicleType& GetVehicleType() = 0;

#ifdef DYNO_HAS_SENSORS_SUPPORT
    /**
     * @brief Initializes all sensors.
     *
     * This method is responsible for setting up and initializing all sensors.
     * It should be overridden by derived classes to provide specific
     * initialization logic for different types of sensors.
     */
    virtual void InitializeSensors();

    /**
     * @brief Retrieves the reference frame of the IMU sensor.
     *
     * This method returns a constant reference to the ChFramed object
     * representing the reference frame of the IMU (Inertial Measurement Unit)
     * sensor.
     *
     * It should be overridden by derived classes to provide the specific
     * IMU reference frame for that vehicle.
     *
     * @return A constant reference to the chrono::ChFramed object.
     */
    virtual const chrono::ChFramed& GetImuSensorReferenceFrame();

    /**
     * @brief Retrieves the parameters of the LiDAR sensor.
     *
     * This method returns a constant reference to the LidarParameters object
     * containing the parameters related to the LiDAR (Light Detection and
     * Ranging) sensor.
     *
     * It should be overridden by derived classes to provide the specific
     * sensor parameters for the LiDAR mounted on that vehicle.
     *
     * @return A constant reference to the LidarParameters object.
     */
    virtual const LidarParameters& GetLidarSensorParameters();

    /**
     * @brief Retrieves the reference frame of the LiDAR sensor.
     *
     * This method returns a constant reference to the ChFramed object
     * representing the reference frame of the LiDAR (Light Detection and
     * Ranging) sensor.
     *
     * It should be overridden by derived classes to provide the specific
     * LiDAR reference frame for that vehicle.
     *
     * @return A constant reference to the chrono::ChFramed object.
     */
    virtual const chrono::ChFramed& GetLidarSensorReferenceFrame();

    /**
     * @brief Retrieves the parameters of the camera sensor.
     *
     * This method returns a constant reference to the CameraParameters object
     * containing the parameters related to the camera sensor.
     *
     * It should be overridden by derived classes to provide the specific
     * sensor parameters for the camera mounted on that vehicle.
     *
     * @return A constant reference to the CameraParameters object.
     */
    virtual const CameraParameters& GetCameraSensorParameters();

    /**
     * @brief Retrieves the reference frame of the camera sensor.
     *
     * This method returns a constant reference to the ChFramed object
     * representing the reference frame of the camera sensor.
     *
     * It should be overridden by derived classes to provide the specific
     * camera reference frame for that vehicle.
     *
     * @return A constant reference to the chrono::ChFramed object.
     */
    virtual const chrono::ChFramed& GetCameraSensorReferenceFrame();
#endif

    /**
     * @brief Retrieves the speed controller parameters.
     *
     * This method returns a constant reference to the SpeedControllerTuning
     * object containing the parameters for the speed controller.
     *
     * It should be overridden by derived classes to provide the specific
     * speed controller parameters for that vehicle.
     *
     * @return A constant reference to the SpeedControllerTuning object.
     */
    virtual const SpeedControllerTuning& GetSpeedControllerTuning();

    /**
     * @brief Retrieves the steering controller parameters.
     *
     * This method returns a constant reference to the SteeringControllerTuning
     * object containing the parameters for the steering controller.
     *
     * It should be overridden by derived classes to provide the specific
     * steering controller parameters for that vehicle.
     *
     * @return A constant reference to the SteeringControllerTuning object.
     */
    virtual const SteeringControllerTuning& GetSteeringControllerTuning();

    /**
     * @brief Retrieves the aerodynamic properties.
     *
     * This method returns a constant reference to the AerodynamicProperties
     * object containing the properties related to aerodynamics.
     *
     * @return A constant reference to the AerodynamicProperties object.
     */
    virtual const AerodynamicProperties& GetAerodynamicProperties();

   protected:
    std::shared_ptr<chrono::ChSystem> system_;

    std::string base_path_ = "vehicles/";

    bool is_initial_pose_overridden_ = false;

    // FIXME: This member is defined in both the vehicle and the simulation
    // classes, and should probably be represented in only one of the two
    // instead.
    chrono::ChCoordsysd initial_pose_;
};

}  // namespace Models
}  // namespace DYNO
