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

#include <dyno/models/vehicle.hpp>

namespace DYNO {
namespace Models {

Vehicle::Vehicle(std::shared_ptr<chrono::ChSystem> system) : system_(system) {}

void Vehicle::OverrideInitialPose(const chrono::ChCoordsysd& initial_pose) {
    initial_pose_ = initial_pose;
    is_initial_pose_overridden_ = true;
}

#ifdef DYNO_HAS_SENSORS_SUPPORT
void Vehicle::InitializeSensors() {
    throw DYNO::Exceptions::NoSensorSuite();
}

const chrono::ChFramed& Vehicle::GetImuSensorReferenceFrame() {
    throw DYNO::Exceptions::NoSensorReferenceFrame();
}

const LidarParameters& Vehicle::GetLidarSensorParameters() {
    throw DYNO::Exceptions::NoSensorConfiguration();
}

const chrono::ChFramed& Vehicle::GetLidarSensorReferenceFrame() {
    throw DYNO::Exceptions::NoSensorReferenceFrame();
}

const CameraParameters& Vehicle::GetCameraSensorParameters() {
    throw DYNO::Exceptions::NoSensorConfiguration();
}

const chrono::ChFramed& Vehicle::GetCameraSensorReferenceFrame() {
    throw DYNO::Exceptions::NoSensorReferenceFrame();
}
#endif

const AerodynamicProperties& Vehicle::GetAerodynamicProperties() {
    throw DYNO::Exceptions::NoAerodynamicParameters();
}

const SpeedControllerTuning& Vehicle::GetSpeedControllerTuning() {
    throw DYNO::Exceptions::NoControllerPreset();
}

const SteeringControllerTuning& Vehicle::GetSteeringControllerTuning() {
    throw DYNO::Exceptions::NoControllerPreset();
}

}  // namespace Models
}  // namespace DYNO
