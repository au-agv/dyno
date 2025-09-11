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

#include <dyno/sensors/imu_gq7.hpp>

namespace DYNO {
namespace Sensors {

GQ7IMU::GQ7IMU(std::shared_ptr<chrono::ChBodyAuxRef> body,
               chrono::ChFrame<double>& offset)
    : IMU(body,
          offset,
          /* Rate */ 1000.0,
          /* Lag */ 0.0,
          /* Collection time */ 0.0,
          /* Accelerometer noise model type */ IMUNoiseModel::NORMAL_DRIFT,
          /* Gyroscope noise model type */ IMUNoiseModel::NORMAL_DRIFT,
          /* Accelerometer noise Gaussian mean */
          chrono::ChVector3<double>({0.0, 0.0, 0.0}),
          /* Accelerometer noise Gaussian standard deviation */
          chrono::ChVector3<double>({4.0e-9, 4.0e-9, 4.0e-9}),
          /* Accelerometer noise bias */ 0.0,
          /* Accelerometer noise time constant */ 0.0,
          /* Gyroscope noise Gaussian mean */
          chrono::ChVector3<double>({0.0, 0.0, 0.0}),
          /* Gyroscope noise Gaussian standard deviation */
          chrono::ChVector3<double>({0.0, 0.0, 0.0}),
          /* Gyroscope noise bias */ 0.0,
          /* Gyroscope noise time constant */ 0.0) {}

} // namespace Sensors
} // namespace DYNO