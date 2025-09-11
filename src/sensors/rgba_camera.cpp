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

#include <dyno/sensors/rgba_camera.hpp>

namespace DYNO {
namespace Sensors {

RGBACamera::RGBACamera(std::shared_ptr<chrono::ChBodyAuxRef> body,
                       double exposure_time,
                       chrono::ChFrame<double> offset,
                       int img_width,
                       int img_height,
                       double fov,
                       double alias_factor,
                       CameraLensModelType lens_model,
                       double lag,
                       bool use_gi,
                       double sigma_x,
                       double sigma_y,
                       Models::CameraNoiseModel noise_model)
    : body_(body),
      exposure_time_(exposure_time),
      offset_(offset),
      img_width_(img_width),
      img_height_(img_height),
      fov_(fov),
      alias_factor_(alias_factor),
      lens_model_(lens_model),
      lag_(lag),
      use_gi_(use_gi),
      sigma_x_(sigma_x),
      sigma_y_(sigma_y),
      noise_model_(noise_model) {
    sensor_ = std::make_shared<chrono::sensor::ChCameraSensor>(body_,
                                                               exposure_time_,
                                                               offset_,
                                                               img_width_,
                                                               img_height_,
                                                               fov_,
                                                               alias_factor_,
                                                               lens_model_,
                                                               use_gi_,
                                                               2.5,
                                                               true);

    // Set sensor frequency.
    sensor_->SetLag(lag_);
    sensor_->SetCollectionWindow(exposure_time_);

    // Add a noise model filter to the camera sensor
    switch(noise_model_) {
    case DYNO::Models::CONST_NORMAL:
        sensor_->PushFilter(
            std::make_shared<chrono::sensor::ChFilterCameraNoiseConstNormal>(
                sigma_x_,
                sigma_y_));
        break;
    case DYNO::Models::PIXEL_DEPENDENT:
        sensor_->PushFilter(
            std::make_shared<chrono::sensor::ChFilterCameraNoisePixDep>(
                sigma_x_,
                sigma_y_));
        break;
    case DYNO::Models::NONE: break;
    }

    // Initialise post-processing filter graph.
    sensor_->PushFilter(
        std::make_shared<chrono::sensor::ChFilterRGBA8Access>());
}

std::shared_ptr<chrono::sensor::ChCameraSensor> RGBACamera::GetSensor() {
    return sensor_;
}

} // namespace Sensors
} // namespace DYNO