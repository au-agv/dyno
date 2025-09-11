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

#include <dyno/sensors/imu.hpp>

namespace DYNO {
namespace Sensors {

IMU::IMU(std::shared_ptr<chrono::ChBodyAuxRef> body,
         chrono::ChFrame<double>& offset,
         double update_rate,
         double lag,
         double collection_time,
         IMUNoiseModel acc_noise_model_type,
         IMUNoiseModel gyro_noise_model_type,
         chrono::ChVector3<double> acc_noise_mean,
         chrono::ChVector3<double> acc_noise_std_dev,
         double acc_noise_bias,
         double acc_noise_tau,
         chrono::ChVector3<double> gyro_noise_mean,
         chrono::ChVector3<double> gyro_noise_std_dev,
         double gyro_noise_bias,
         double gyro_noise_tau)
    : offset_(offset),
      update_rate_(update_rate),
      lag_(lag),
      collection_time_(collection_time),
      acc_noise_model_type_(acc_noise_model_type),
      gyro_noise_model_type_(gyro_noise_model_type),
      acc_noise_mean_(acc_noise_mean),
      acc_noise_std_dev_(acc_noise_std_dev),
      acc_noise_bias_(acc_noise_bias),
      acc_noise_tau_(acc_noise_tau),
      gyro_noise_mean_(gyro_noise_mean),
      gyro_noise_std_dev_(gyro_noise_std_dev),
      gyro_noise_bias_(gyro_noise_bias),
      gyro_noise_tau_(gyro_noise_tau) {
    acc_noise_model_ =
        std::make_shared<chrono::sensor::ChNoiseNormalDrift>(update_rate,
                                                             acc_noise_mean,
                                                             acc_noise_std_dev,
                                                             acc_noise_bias,
                                                             acc_noise_tau);

    acc_sensor_ = std::make_shared<chrono::sensor::ChAccelerometerSensor>(
        body, // body to which the IMU is attached
        update_rate, // update rate
        offset, // offset pose from body
        acc_noise_model_); // IMU noise model

    acc_sensor_->SetLag(lag);
    acc_sensor_->SetCollectionWindow(collection_time);
    acc_sensor_->PushFilter(
        std::make_shared<chrono::sensor::ChFilterAccelAccess>());

    gyro_noise_model_ =
        std::make_shared<chrono::sensor::ChNoiseNormalDrift>(update_rate,
                                                             gyro_noise_mean,
                                                             gyro_noise_std_dev,
                                                             gyro_noise_bias,
                                                             gyro_noise_tau);

    gyro_sensor_ = std::make_shared<chrono::sensor::ChGyroscopeSensor>(
        body, // body to which the IMU is attached
        update_rate, // update rate
        offset, // offset pose from body
        gyro_noise_model_); // IMU noise model
    gyro_sensor_->SetLag(lag);
    gyro_sensor_->SetCollectionWindow(collection_time);
    gyro_sensor_->PushFilter(
        std::make_shared<chrono::sensor::ChFilterGyroAccess>());
}

std::shared_ptr<chrono::sensor::ChAccelerometerSensor>
IMU::GetAccelerometerSensor() {
    return acc_sensor_;
}

chrono::sensor::UserAccelBufferPtr IMU::GetAccelerometerBuffer() {
    return acc_sensor_
        ->GetMostRecentBuffer<chrono::sensor::UserAccelBufferPtr>();
}

std::shared_ptr<chrono::sensor::ChGyroscopeSensor> IMU::GetGyroscopeSensor() {
    return gyro_sensor_;
}

chrono::sensor::UserGyroBufferPtr IMU::GetGyroscopeBuffer() {
    return gyro_sensor_
        ->GetMostRecentBuffer<chrono::sensor::UserGyroBufferPtr>();
}

} // namespace Sensors
} // namespace DYNO