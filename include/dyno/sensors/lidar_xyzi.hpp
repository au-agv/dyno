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

#include <chrono/physics/ChBodyAuxRef.h>
#include <chrono_sensor/filters/ChFilterAccess.h>
#include <chrono_sensor/filters/ChFilterLidarNoise.h>
#include <chrono_sensor/filters/ChFilterPCfromDepth.h>
#include <chrono_sensor/sensors/ChLidarSensor.h>

#include <dyno/models/lidar_parameters.hpp>

namespace DYNO {
namespace Sensors {

/**
 * @brief Simplified LiDAR sensor interface.
 */
class LidarXYZI {
   public:
    LidarXYZI(const std::shared_ptr<chrono::ChBodyAuxRef>& body,
              const chrono::ChFrame<double>& offset,
              const Models::LidarParameters& parameters);

    /**
     * @brief Get a shared pointer to the underlying LiDAR sensor object.
     *
     * @return std::shared_ptr<ChLidarSensor> LiDAR sensor.
     */
    std::shared_ptr<chrono::sensor::ChLidarSensor> GetSensor();

   private:
    /** @brief Shared pointer to the LiDAR sensor. */
    std::shared_ptr<chrono::sensor::ChLidarSensor> sensor_;

    /** @brief Shared pointer to the body the LiDAR sensor transform is defined
     * relative from. */
    std::shared_ptr<chrono::ChBodyAuxRef> body_;

    /** @brief Transform between the reference frame of the parent body and the
     * sensor. */
    chrono::ChFrame<double> offset_;
};

}  // namespace Sensors
}  // namespace DYNO
