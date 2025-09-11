#pragma once

#include <chrono/core/ChVector3.h>

namespace DYNO {
namespace Models {

struct LidarParameters {
    LidarParameters(uint samples_horizontal, uint samples_vertical,
                    double fov_horizontal, double vertical_angle_min,
                    double vertical_angle_max, double rate, double lag,
                    chrono::ChVector3d position_noise_std_dev,
                    double intensity_noise_std_dev)
        : samples_horizontal_(0) {}

    uint GetSamplesHorizontal() const { return samples_horizontal_; }

    uint GetSamplesVertical() const { return samples_vertical_; }

    double GetFOVHorizontal() const { return fov_horizontal_; }

    double GetVerticalAngleMin() const { return vertical_angle_min_; }

    double GetVerticalAngleMax() const { return vertical_angle_max_; }

    double GetDistanceMax() const { return distance_max_; }

    double GetRate() const { return rate_; }

    double GetLag() const { return lag_; }

    chrono::ChVector3d GetPositionNoiseStandardDeviation() const {
        return position_noise_std_dev_;
    }

    double GetIntensityNoiseStandardDeviation() const {
        return intensity_noise_std_dev_;
    }

    unsigned int samples_horizontal_;
    unsigned int samples_vertical_;
    double fov_horizontal_;
    double vertical_angle_min_;
    double vertical_angle_max_;
    double distance_max_;
    double rate_;
    double lag_;
    chrono::ChVector3d position_noise_std_dev_;
    double intensity_noise_std_dev_;
};

struct Ouster64 : LidarParameters {
    Ouster64()
        : LidarParameters(0, 0, 100.0, 120.0, 30.0, 10.0, 0.0,
                          chrono::ChVector3d(0.0, 0.0, 0.0), 0.0) {}
};

}  // namespace Models
}  // namespace DYNO
