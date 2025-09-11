
#pragma once

// TODO: This should only include ChCamera instead.
#include <chrono_sensor/sensors/ChSegmentationCamera.h>

namespace DYNO {
namespace Models {

enum CameraNoiseModel {
    CONST_NORMAL,  // Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT,  // Pixel dependent gaussian noise
    NONE              // No noise model
};

struct CameraParameters {
    double exposure_time;
    double image_width;
    double image_height;
    double fov;
    double alias_factor;
    double lag;
    bool use_global_illumination;
    chrono::ChVector2d noise_pixel_std_dev;
    CameraLensModelType lens_model;
    CameraNoiseModel noise_model;
};

}  // namespace Models
}  // namespace DYNO
