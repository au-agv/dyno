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

#include <chrono/utils/ChUtils.h>
#include <chrono_sensor/ChSensorManager.h>
#include <boost/algorithm/clamp.hpp>

#include <dyno/drivers/acceleration_based_driver.hpp>
#include <dyno/drivers/velocity_based_driver.hpp>
#include <dyno/exceptions/exceptions.hpp>
#include <dyno/models/camera_parameters.hpp>
#include <dyno/models/imu_parameters.hpp>
#include <dyno/models/lidar_parameters.hpp>
#include <dyno/sensors/imu.hpp>
#include <dyno/sensors/lidar_xyzi.hpp>
#include <dyno/sensors/rgba_camera.hpp>
#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

class AutonomousVehicleSimulation : public VehicleSimulation {
   public:
    AutonomousVehicleSimulation(
        std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    std::shared_ptr<chrono::ChFrame<double>> GetIMUFrame();

    std::shared_ptr<DYNO::Sensors::IMU> GetIMUSensor();

    std::shared_ptr<DYNO::Sensors::LidarXYZI> GetLiDARSensor();

    std::shared_ptr<chrono::ChFrame<double>> GetLiDARFrame();

    std::shared_ptr<DYNO::Sensors::RGBACamera> GetCameraSensor();

    std::shared_ptr<chrono::ChFrame<double>> GetCameraFrame();

    void SetControl(double acceleration, double steering_rate);

   protected:
    void InitializeAssets() override;

    void InitializeDriver() override;

    void PostInitializationHook() override;

    void PreSynchronizationHook() override;

    void SynchronizeDriver() override;

    bool CheckPose(const chrono::ChVector3d& pose) const;

    void OverrideInitialPose() override;

    /** @brief The length of the acceleration lane instantiated for the vehicle
     * to reach the target speed. */
    double acceleration_length_;

    std::shared_ptr<DYNO::Sensors::IMU> imu_;

    chrono::ChFrame<double> imu_frame_;

    std::shared_ptr<DYNO::Sensors::LidarXYZI> lidar_;

    chrono::ChFrame<double> lidar_frame_;

    std::shared_ptr<DYNO::Sensors::RGBACamera> camera_;

    chrono::ChFrame<double> camera_frame_;

    /** @brief The initial pose of the navigation path.
     *
     * This vector represents the starting position of the loaded navigation
     * path.
     */
    chrono::ChVector3d path_initial_pose_;

   private:
    /**
     * Initializes the obstacles in the simulation.
     *
     * This function parses the obstacles from the configuration and
     * instantiates them in the simulation.
     */
    void InitializeObstacles();

    /** @brief Shared pointer to the Chrono::Sensor sensor manager handling
     * sensor initialization and updating. */
    std::shared_ptr<chrono::sensor::ChSensorManager> sensor_manager_;

    /**
     * @brief Initialize the Chrono::Sensor sensor manager.
     *
     * This function performs the initial setup of the Chrono::Sensor sensor
     * manager and initializes the NVIDIA OptiX engine. It also specifies the
     * directory from which the sensor shaders will be loaded.
     *
     * This function is run as part of the initialization phase.
     */
    void InitializeSensorManager();

    /**
     * @brief Add a sensor to the sensor manager.
     *
     * @param sensor Shared pointer to the Chrono::Sensor sensor to be added to
     * the sensor manager.
     */
    void AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor);

    /**
     * @brief Initialize the vehicle sensors.
     *
     * This function initializes the location and configuration of the inertial
     * measurement unit (IMU), the LiDAR (Light Detection and Ranging) and the
     * camera sensors specified in the autonomous vehicle model.
     *
     * This function may be overridden by derived classes to specify different
     * sensor configurations or initialization procedures.
     *
     */
    void InitializeSensors() override;

    void PostAdvanceHook() override;

    /** @brief Shared pointer to the specialized driver for interfacing with
     * autonomous navigation system. */
    std::shared_ptr<DYNO::Drivers::AccelerationBasedDriver> autonomous_driver_;

    /** @brief Shared pointer to the specialized driver handling the controls
     * during the initial part of the scenario. */
    std::shared_ptr<DYNO::Drivers::VelocityBasedDriver> warmup_driver_;

    /** @brief The target speed to be reached by the warmup driver before
     * handing over controls to the autonomous navigation system. */
    double target_speed_;

    /** @brief The target acceleration set by the autonomous navigation system.
     */
    double target_acceleration_ = 0.0;

    /** @brief The target steering rate set by the autonomous navigation system.
     * */
    double target_steering_rate_ = 0.0;

    /** @brief Indicates whether the system is accepting controls from the
     * autonomous navigation stack. */
    bool is_accepting_controls_ = false;

    /** @brief Indicates whether the target speed has been reached at least once
     * since the beginning of the simulation. */
    bool target_speed_reached_ = false;

    /** @brief Indicates whether the initial pose in the navigation path has
     * been reached at least once since the beginning of the simulation. */
    bool initial_pose_reached_ = false;

    /** @brief The radius within with waypoints are considered reached. */
    double waypoint_radius_;
};

}  // namespace Simulation
}  // namespace DYNO
