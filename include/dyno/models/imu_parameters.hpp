#pragma once

#include <chrono/core/ChVector3.h>

namespace DYNO {
namespace Models {

struct ImuParameters {
    double update_rate;
    double lag;
    double collection_time;
    chrono::ChVector3d acceleration_noise_mean;
    chrono::ChVector3d acceleration_noise_std_dev;
    double acceleration_noise_bias;
    double acceleration_noise_tau;
    double gyro_noise_mean;
    double gyro_noise_std_dev;
    double gyro_noise_bias;
    double gyro_noise_tau;
};

}  // namespace Models
}  // namespace DYNO
