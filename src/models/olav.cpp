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

#include <dyno/models/olav.hpp>

namespace DYNO {
namespace Models {

Olav::Olav(std::shared_ptr<chrono::ChSystem> system)
    : WheeledVehicle(system),
      speed_controller_tuning_(SpeedControllerTuning(2.68, 0.001, 0.0)),
      steering_controller_tuning_(
          SteeringControllerTuning(0.5, 0.05, 0.0, 2.0)),
      aerodynamic_properties_(AerodynamicProperties(0.69, 2.3))
#ifdef DYNO_HAS_SENSORS_SUPPORT
      ,
      imu_frame_(chrono::ChFramed(chrono::ChCoordsysd(
          chrono::ChVector3d(-2.1662, -0.0216, 1.1993), chrono::QUNIT))),
      lidar_frame_(chrono::ChFramed(chrono::ChCoordsysd(
          chrono::ChVector3d(-1.109991, -0.003951, 2.5386), chrono::QUNIT))),
      camera_frame_(chrono::ChFramed(chrono::ChCoordsysd(
          chrono::ChVector3d(0.27, 0.0, 1.1002), chrono::QUNIT))),
      lidar_parameters_(Ouster64())
#endif
{
    base_path_ += "olav/templates/";
}

double Olav::GetSteeringAngleMax() const {
    return 33.0;
}

std::shared_ptr<chrono::ChBodyAuxRef> Olav::GetChassisBody() {
    return vehicle_->GetChassisBody();
}

const SpeedControllerTuning& Olav::GetSpeedControllerTuning() {
    return speed_controller_tuning_;
}

const SteeringControllerTuning& Olav::GetSteeringControllerTuning() {
    return steering_controller_tuning_;
}

const AerodynamicProperties& Olav::GetAerodynamicProperties() {
    return aerodynamic_properties_;
}

#ifdef DYNO_HAS_SENSORS_SUPPORT
const chrono::ChFramed& Olav::GetImuSensorReferenceFrame() {
    return imu_frame_;
}

const LidarParameters& Olav::GetLidarSensorParameters() {
    return lidar_parameters_;
}

const chrono::ChFramed& Olav::GetLidarSensorReferenceFrame() {
    return lidar_frame_;
}

const CameraParameters& Olav::GetCameraSensorParameters() {
    return camera_parameters_;
}

const chrono::ChFramed& Olav::GetCameraSensorReferenceFrame() {
    return camera_frame_;
}
#endif

}  // namespace Models
}  // namespace DYNO
