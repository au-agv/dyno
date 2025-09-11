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

#include <dyno/sensors/lidar_xyzi.hpp>

namespace DYNO {
namespace Sensors {

LidarXYZI::LidarXYZI(const std::shared_ptr<chrono::ChBodyAuxRef>& body,
                     const chrono::ChFrame<double>& offset,
                     const Models::LidarParameters& parameters)
    : body_(body), offset_(offset) {
    sensor_ = std::make_shared<chrono::sensor::ChLidarSensor>(
        body, parameters.GetRate(), offset, parameters.GetSamplesHorizontal(),
        parameters.GetSamplesVertical(), parameters.GetFOVHorizontal(),
        parameters.GetVerticalAngleMax(), parameters.GetVerticalAngleMin(),
        parameters.GetDistanceMax());

    // Set sensor frequency.
    sensor_->SetLag(parameters.GetLag());
    sensor_->SetCollectionWindow(1.0 / parameters.GetRate());

    // Initialise post-processing filter graph.
    // Add a filter for converting the depth information to a point cloud.
    sensor_->PushFilter(
        std::make_shared<chrono::sensor::ChFilterPCfromDepth>());
    // Add a filter to include positional and intensity noise.
    sensor_->PushFilter(
        std::make_shared<chrono::sensor::ChFilterLidarNoiseXYZI>(
            parameters.GetPositionNoiseStandardDeviation().x(),
            parameters.GetPositionNoiseStandardDeviation().y(),
            parameters.GetPositionNoiseStandardDeviation().z(),
            parameters.GetIntensityNoiseStandardDeviation()));

    // Add a filter for access to the position and intensity buffer.
    sensor_->PushFilter(std::make_shared<chrono::sensor::ChFilterXYZIAccess>());
}

std::shared_ptr<chrono::sensor::ChLidarSensor> LidarXYZI::GetSensor() {
    return sensor_;
}

}  // namespace Sensors
}  // namespace DYNO
