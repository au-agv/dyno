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
#include <chrono_sensor/sensors/ChIMUSensor.h>

namespace DYNO {
namespace Sensors {

enum IMUNoiseModel { NORMAL_DRIFT, IMU_NONE };

class IMU {
  public:
    IMU(std::shared_ptr<chrono::ChBodyAuxRef> body,
        chrono::ChFrame<double>& offset,
        double update_rate = 1000.0,
        double lag = 0.0,
        double collection_time = 0.0,
        IMUNoiseModel acc_noise_model = IMUNoiseModel::IMU_NONE,
        IMUNoiseModel gyro_noise_model = IMUNoiseModel::IMU_NONE,
        chrono::ChVector3<double> acc_noise_mean =
            chrono::ChVector3({0.0, 0.0, 0.0}),
        chrono::ChVector3<double> acc_noise_std_dev =
            chrono::ChVector3({0.0, 0.0, 0.0}),
        double acc_noise_bias = 0.0,
        double acc_noise_tau = 0.0,
        chrono::ChVector3<double> gyro_noise_mean =
            chrono::ChVector3({0.0, 0.0, 0.0}),
        chrono::ChVector3<double> gyro_noise_std_dev =
            chrono::ChVector3({0.0, 0.0, 0.0}),
        double gyro_noise_bias = 0.0,
        double gyro_noise_tau = 0.0);

    /**
     * @brief Get a shared pointer to the accelerometer sensor.
     *
     * @return std::shared_ptr<chrono::sensor::ChAccelerometerSensor> Shared
     * pointer to the accelerometer sensor.
     */
    std::shared_ptr<chrono::sensor::ChAccelerometerSensor>
    GetAccelerometerSensor();

    /**
     * @brief Get a shared pointer to the accelerometer sensor buffer.
     *
     * @return chrono::sensor::UserAccelBufferPtr Shared pointer to the
     * accelerometer sensor buffer.
     */
    chrono::sensor::UserAccelBufferPtr GetAccelerometerBuffer();

    /**
     * @brief Get a shared pointer to the gyroscope sensor.
     *
     * @return std::shared_ptr<chrono::sensor::ChGyroscopeSensor> Shared pointer
     * to the gyroscope sensor.
     */
    std::shared_ptr<chrono::sensor::ChGyroscopeSensor> GetGyroscopeSensor();

    /**
     * @brief Get a shared pointer to the gyroscope sensor buffer.
     *
     * @return chrono::sensor::UserGyroBufferPtr Shared pointer to the gyroscope
     * sensor buffer.
     */
    chrono::sensor::UserGyroBufferPtr GetGyroscopeBuffer();

  private:
    chrono::ChFrame<double> offset_;

    /** @brief Sensor update rate in Hz. */
    double update_rate_;

    /** @brief Sensor reading collection lag. */
    double lag_;

    /** @brief Sensor sample collection time. */
    double collection_time_;

    // Accelerometer sensor
    // --------------------
    /** @brief Shared pointer to the accelerometer sensor. */
    std::shared_ptr<chrono::sensor::ChAccelerometerSensor> acc_sensor_;

    /** @brief Accelerometer noise model type. */
    IMUNoiseModel acc_noise_model_type_;

    /** @brief Shared pointer to the accelerometer noise model. */
    std::shared_ptr<chrono::sensor::ChNoiseModel> acc_noise_model_;

    chrono::ChVector3<double> acc_noise_mean_;

    chrono::ChVector3<double> acc_noise_std_dev_;

    double acc_noise_bias_;

    double acc_noise_tau_;
    // --------------------

    // Gyroscope sensor
    // ----------------
    /** @brief Shared pointer to the gyroscope sensor. */
    std::shared_ptr<chrono::sensor::ChGyroscopeSensor> gyro_sensor_;

    /** @brief Gyroscope noise model type. */
    IMUNoiseModel gyro_noise_model_type_;

    /** @brief Shared pointer to the gyroscope noise model. */
    std::shared_ptr<chrono::sensor::ChNoiseModel> gyro_noise_model_;

    chrono::ChVector3<double> gyro_noise_mean_;

    chrono::ChVector3<double> gyro_noise_std_dev_;

    double gyro_noise_bias_;

    double gyro_noise_tau_;
    // ----------------
};

} // namespace Sensors
} // namespace DYNO