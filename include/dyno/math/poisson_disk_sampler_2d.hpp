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

#include <random>

#include <spdlog/spdlog.h>
#include <boost/algorithm/clamp.hpp>
#include <nlohmann/json.hpp>

#include <dyno/exceptions/exceptions.hpp>
#include <dyno/math/poisson_disk_sampler.hpp>
#include <dyno/math/poisson_point_2d.hpp>

namespace DYNO {
namespace Math {

class PoissonDiskSampler2D : public PoissonDiskSampler {
   public:
    using PoissonDiskSampler::Generate;

    PoissonDiskSampler2D(double disk_width, double disk_height,
                         double minimum_distance);

    void SetMinimumDistance(double distance);

    void Seed(unsigned int seed);

    std::vector<PoissonPoint2D> GetSamples();

    void Generate(unsigned int number_of_samples);

    void Serialize(nlohmann::json& data);

    void SetIterations(unsigned int iterations);

    void SetOrigin(double x, double y, double theta);


   private:
    PoissonPoint2D GenerateAroundPoint(const PoissonPoint2D& point);

    bool IsValid(const PoissonPoint2D& point);

    double GetDistance(const PoissonPoint2D& first_point,
                       const PoissonPoint2D& second_point);

    void InitializePRNG();

    void InitializeGrid();

    void LogGridStatistics();

    PoissonPoint2D GenerateRandomPoissonPoint();

    void SeedInitialPoint();

    void AddPointToActiveList(const PoissonPoint2D& point);

    void AddPointToGrid(const PoissonPoint2D& point);

    void TransformSamples();


    double disk_width_;
    double disk_height_;

    double grid_cell_size_;
    size_t grid_width_;
    size_t grid_height_;
    size_t grid_cells_num_;
    std::vector<PoissonPoint2D> grid_;

    std::vector<PoissonPoint2D> active_;
    std::vector<PoissonPoint2D> samples_;

    bool transform_samples_ = false;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_angle_ = 0.0;

    bool has_result_ = false;
};

}  // namespace Math
}  // namespace DYNO
