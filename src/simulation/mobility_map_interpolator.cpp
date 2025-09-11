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

#include <dyno/simulation/mobility_map_interpolator.hpp>

namespace DYNO {
namespace Simulation {

MobilityMapInterpolator::MobilityMapInterpolator(const std::string& hdf5_file) {
    HighFive::File file(hdf5_file, HighFive::File::ReadOnly);

    if (!file.exist("angles") || !file.exist("speed_maps")) {
        throw std::runtime_error(
            "HDF5 file must contain datasets: angles, speed_maps");
    }

    HighFive::DataSet angles_ds = file.getDataSet("angles");
    HighFive::DataSet maps_ds = file.getDataSet("speed_maps");

    auto angles_dims = angles_ds.getSpace().getDimensions();
    auto maps_dims = maps_ds.getSpace().getDimensions();

    if (angles_dims.size() != 1) {
        throw std::runtime_error("angles dataset must be 1D");
    }

    if (maps_dims.size() != 3) {
        throw std::runtime_error("speed_maps dataset must be 3D (N, H, W)");
    }

    N_ = angles_dims[0];
    H_ = maps_dims[1];
    W_ = maps_dims[2];

    if (maps_dims[0] != N_) {
        throw std::runtime_error(
            "Mismatch between angles and speed_maps dimensions");
    }

    angles_.resize(N_);
    speed_maps_.resize(N_ * H_ * W_);

    angles_ds.read(angles_);
    maps_ds.read(speed_maps_);
}

std::vector<double> MobilityMapInterpolator::interpolate(
    double heading_angle) const {
    if (angles_.empty()) {
        throw std::runtime_error("Interpolator not initialized");
    }

    // Clamp outside range
    if (heading_angle <= angles_.front()) {
        size_t off = mapOffset(0);
        return {speed_maps_.begin() + off, speed_maps_.begin() + off + H_ * W_};
    }

    if (heading_angle >= angles_.back()) {
        size_t off = mapOffset(N_ - 1);
        return {speed_maps_.begin() + off, speed_maps_.begin() + off + H_ * W_};
    }

    // Find bracketing angles
    auto it = std::upper_bound(angles_.begin(), angles_.end(), heading_angle);

    size_t i1 = std::distance(angles_.begin(), it);
    size_t i0 = i1 - 1;

    double a0 = angles_[i0];
    double a1 = angles_[i1];
    double t = (heading_angle - a0) / (a1 - a0);

    std::vector<double> result(H_ * W_);

    size_t off0 = mapOffset(i0);
    size_t off1 = mapOffset(i1);

    for (size_t i = 0; i < H_ * W_; ++i) {
        result[i] =
            (1.0 - t) * speed_maps_[off0 + i] + t * speed_maps_[off1 + i];
    }

    return result;
}

}  // namespace Simulation
}  // namespace DYNO
