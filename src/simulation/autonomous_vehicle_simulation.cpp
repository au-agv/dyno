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

#include <dyno/simulation/autonomous_vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

AutonomousVehicleSimulation::AutonomousVehicleSimulation(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {

    waypoint_radius_ =
        configuration_->GetValue("scenario/path/waypointRadius", 3.0);

    acceleration_length_ =
        configuration_->GetValue("scenario/path/accelerationLength", 100.0);

    target_speed_ = configuration_->GetValue("scenario/targetSpeed", 10.0);
}

void AutonomousVehicleSimulation::GetConfiguration() {}

void AutonomousVehicleSimulation::Instantiate() {}

void AutonomousVehicleSimulation::PreInitializeSystemHook() {}

void AutonomousVehicleSimulation::PostInitializeSystemHook() {}

void AutonomousVehicleSimulation::OverrideInitialPose() {
    vehicle_->OverrideInitialPose(chrono::ChCoordsysd(
        chrono::ChVector3d(-acceleration_length_, 0.0, 0.1), chrono::QUNIT));
}

void AutonomousVehicleSimulation::InitializeSensors() {
    SPDLOG_INFO("Initializing AGV simulation sensor manager...");

    // ------------------------------------------------------------------------
    // Inertial measurement unit (IMU)
    // ------------------------------------------------------------------------
    try {
        imu_frame_ = vehicle_->GetImuSensorReferenceFrame();
    } catch (DYNO::Exceptions::NoSensorReferenceFrame& exception) {
        imu_frame_ = chrono::ChFrame<double>(
            chrono::ChVector3<double>(
                configuration_->GetValue<double>("sensors/imu/frame/x"),
                configuration_->GetValue<double>("sensors/imu/frame/y"),
                configuration_->GetValue<double>("sensors/imu/frame/z")),
            chrono::QUNIT);
    }

    imu_ = std::make_shared<DYNO::Sensors::IMU>(
        vehicle_->GetVehicle()->GetChassisBody(), imu_frame_);

    // ------------------------------------------------------------------------
    // LiDAR
    // ------------------------------------------------------------------------

    try {
        lidar_frame_ = vehicle_->GetLidarSensorReferenceFrame();
    } catch (DYNO::Exceptions::NoSensorReferenceFrame& exception) {
        lidar_frame_ = chrono::ChFrame<double>(
            chrono::ChVector3<double>(
                configuration_->GetValue<double>("sensors/lidar/frame/x"),
                configuration_->GetValue<double>("sensors/lidar/frame/y"),
                configuration_->GetValue<double>("sensors/lidar/frame/z")),
            chrono::QUNIT);
    }

    try {
        lidar_ = std::make_shared<DYNO::Sensors::LidarXYZI>(
            vehicle_->GetVehicle()->GetChassisBody(), lidar_frame_,
            vehicle_->GetLidarSensorParameters());

    } catch (DYNO::Exceptions::NoSensorConfiguration& exception) {
        throw;
    }

    // ------------------------------------------------------------------------
    // RGBA camera
    // ------------------------------------------------------------------------

    try {
        camera_frame_ = vehicle_->GetCameraSensorReferenceFrame();
    } catch (DYNO::Exceptions::NoSensorReferenceFrame& exception) {
        camera_frame_ = chrono::ChFrame<double>(
            chrono::ChVector3<double>(
                configuration_->GetValue<double>("sensors/camera/frame/x"),
                configuration_->GetValue<double>("sensors/camera/frame/y"),
                configuration_->GetValue<double>("sensors/camera/frame/z")),
            chrono::QUNIT);
    }

    // Camera sensor
    camera_ = std::make_shared<DYNO::Sensors::RGBACamera>(
        vehicle_->GetVehicle()->GetChassisBody(), 33.3, camera_frame_, 1280,
        720, 2.0943951, 4.0);

    // Temporarily mute the standard output stream to suppress verbose output
    // from Chrono::Sensor.
    InitializeSensorManager();
    AddSensor(imu_->GetAccelerometerSensor());
    AddSensor(imu_->GetGyroscopeSensor());
    AddSensor(lidar_->GetSensor());
    AddSensor(camera_->GetSensor());
}

void AutonomousVehicleSimulation::InitializeSensorManager() {
    const auto shaders_path =
        std::string("/usr/local/share/chrono/sensor_shaders");
    chrono::sensor::SetSensorShaderDir(shaders_path);
    SPDLOG_INFO("Set NVIDIA OptiX shaders directory to \"{}\"", shaders_path);

    std::cout.setstate(std::ios_base::failbit);
    sensor_manager_ =
        std::make_shared<chrono::sensor::ChSensorManager>(system_.get());

    // TODO: The verbose output of the sensor manager is outrageously dense, but
    // it can be useful to debug a wrong initialization of the NVIDIA OptiX
    // engine - perhaps this could be exposed in the configuration file?

    /*
    sensor_manager_->SetVerbose(true);
    */

    std::cout.clear();
}

void AutonomousVehicleSimulation::AddSensor(
    std::shared_ptr<chrono::sensor::ChSensor> sensor) {
    std::cout.setstate(std::ios_base::failbit);
    sensor_manager_->AddSensor(sensor);
    std::cout.clear();
}

void AutonomousVehicleSimulation::InitializeDriver() {
    autonomous_driver_ =
        std::make_shared<DYNO::Drivers::AccelerationBasedDriver>(
            *vehicle_->GetVehicle(), vehicle_->GetSteeringAngleMax());
    autonomous_driver_->SetSpeedControllerGains(
        vehicle_->GetSpeedControllerTuning());
    autonomous_driver_->Initialize();

    warmup_driver_ = std::make_shared<DYNO::Drivers::VelocityBasedDriver>(
        *vehicle_->GetVehicle(), vehicle_->GetSteeringAngleMax());
    warmup_driver_->SetSpeedControllerGains(
        vehicle_->GetSpeedControllerTuning());
    warmup_driver_->Initialize();

    driver_ = warmup_driver_;

    VehicleSimulation::InitializeDriver();
}

std::shared_ptr<DYNO::Sensors::IMU>
AutonomousVehicleSimulation::GetIMUSensor() {
    return imu_;
}

std::shared_ptr<chrono::ChFrame<double>>
AutonomousVehicleSimulation::GetIMUFrame() {
    return std::make_shared<chrono::ChFrame<double>>(imu_frame_);
}

std::shared_ptr<DYNO::Sensors::LidarXYZI>
AutonomousVehicleSimulation::GetLiDARSensor() {
    return lidar_;
}

std::shared_ptr<chrono::ChFrame<double>>
AutonomousVehicleSimulation::GetLiDARFrame() {
    return std::make_shared<chrono::ChFrame<double>>(lidar_frame_);
}

std::shared_ptr<DYNO::Sensors::RGBACamera>
AutonomousVehicleSimulation::GetCameraSensor() {
    return camera_;
}

std::shared_ptr<chrono::ChFrame<double>>
AutonomousVehicleSimulation::GetCameraFrame() {
    return std::make_shared<chrono::ChFrame<double>>(camera_frame_);
}

void AutonomousVehicleSimulation::InitializeAssets() {}

void AutonomousVehicleSimulation::PreSynchronizationHook() {
    SPDLOG_DEBUG("Running pre-synchronization hooks");

    if (vehicle_->GetSpeed() > target_speed_) {
        target_speed_reached_ = true;
    }

    if (vehicle_->GetPosition().x() > 0.0 && target_speed_reached_ &&
        !initial_pose_reached_) {
        SPDLOG_INFO("Initial pose reached!");

        // Configure the autonomous driver to receive commands from the
        // navigation system upstream.
        is_accepting_controls_ = true;

        autonomous_driver_->Reset(target_speed_, 0.0);
        autonomous_driver_->SetAcceleration(target_acceleration_);
        autonomous_driver_->SetSteeringRate(target_steering_rate_);
        autonomous_driver_->Advance(time_step_);
        autonomous_driver_->Synchronize(time_);

        driver_ = autonomous_driver_;

        current_driver_inputs_ = autonomous_driver_->GetInputs();

        SPDLOG_INFO(
            "Now accepting controls from the autonomous navigation system.");

        // Mark the initial pose as successfully reached to avoid entering this
        // branch in future hook calls.
        initial_pose_reached_ = true;
    }

    sensor_manager_->Update();
}

void AutonomousVehicleSimulation::PostAdvanceHook() {}

bool AutonomousVehicleSimulation::CheckPose(
    const chrono::ChVector3d& pose) const {
    if ((vehicle_->GetPosition() - pose).Length() < waypoint_radius_) {
        return true;
    }
    return false;
}

void AutonomousVehicleSimulation::PostInitializationHook() {
    SPDLOG_DEBUG("Running post-synchronization hooks");
    InitializeSensors();
}

void AutonomousVehicleSimulation::SynchronizeDriver() {
    if (time_ < warmup_time_) {
        warmup_driver_->SetSpeed(0.0);
        warmup_driver_->SetSteeringAngle(0.0);
        warmup_driver_->Synchronize(time_);

        current_driver_inputs_ = warmup_driver_->GetInputs();

        driver_->Synchronize(time_);
        return;
    }

    if (!target_speed_reached_ || !initial_pose_reached_) {
        warmup_driver_->SetSpeed(target_speed_);
        warmup_driver_->SetSteeringAngle(0.0);
        warmup_driver_->Synchronize(time_);

        current_driver_inputs_ = warmup_driver_->GetInputs();

        driver_->Synchronize(time_);
        return;
    }

    if (time_ > warmup_time_) {
        autonomous_driver_->SetAcceleration(target_acceleration_);
        autonomous_driver_->SetSteeringRate(target_steering_rate_);
        autonomous_driver_->Synchronize(time_);

        current_driver_inputs_ = autonomous_driver_->GetInputs();
    }

    driver_->Synchronize(time_);
}

void AutonomousVehicleSimulation::SetControl(double acceleration,
                                             double steering_rate) {
    if (is_accepting_controls_) {
        SPDLOG_DEBUG(
            "Received new control from the autonomy stack: [{:0.2f} m/s^2, "
            "{:0.2f} deg/s]",
            acceleration, steering_rate);
        target_acceleration_ = acceleration;
        target_steering_rate_ = steering_rate;
    }
}

}  // namespace Simulation
}  // namespace DYNO
