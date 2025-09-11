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
#include <chrono_sensor/filters/ChFilterCameraNoise.h>
#include <chrono_sensor/sensors/ChSegmentationCamera.h>

#include <dyno/models/camera_parameters.hpp>

namespace DYNO {
namespace Sensors {

class RGBACamera {
   public:
    RGBACamera(
        std::shared_ptr<chrono::ChBodyAuxRef> body, double exposure_time = 60.0,
        chrono::ChFrame<double> offset = chrono::ChFrame<double>(chrono::VNULL,
                                                                 chrono::QUNIT),
        int img_width = 640, int img_height = 480,
        double fov = chrono::CH_PI / 3.0, double alias_factor = 1.0,
        CameraLensModelType lens_model = CameraLensModelType::PINHOLE,
        double lag = 0.0, bool use_gi = false, double sigma_x = 0.0,
        double sigma_y = 0.0,
        Models::CameraNoiseModel noise_model = Models::CameraNoiseModel::NONE);
    std::shared_ptr<chrono::sensor::ChCameraSensor> GetSensor();

   private:
    std::shared_ptr<chrono::sensor::ChCameraSensor> sensor_;
    std::shared_ptr<chrono::ChBodyAuxRef> body_;
    double exposure_time_;
    chrono::ChFrame<double> offset_;
    int img_width_;
    int img_height_;
    double fov_;
    double alias_factor_;
    CameraLensModelType lens_model_;
    double lag_;
    bool use_gi_;
    double sigma_x_;
    double sigma_y_;
    Models::CameraNoiseModel noise_model_;
};
}  // namespace Sensors
}  // namespace DYNO
