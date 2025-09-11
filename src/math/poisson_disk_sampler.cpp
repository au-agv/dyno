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

#include <dyno/math/poisson_disk_sampler.hpp>

namespace DYNO {
namespace Math {

PoissonDiskSampler::PoissonDiskSampler(double minimum_distance)
    : minimum_distance_(minimum_distance) {
    Seed();
    InitializePRNG();
};

void PoissonDiskSampler::Generate() {
    Generate(std::numeric_limits<unsigned int>::max());
}

void PoissonDiskSampler::Generate(unsigned int lower_bound,
                                  unsigned int upper_bound) {
    Generate(
        (unsigned int)(GenerateRandomNumber() * (upper_bound - lower_bound) +
                       lower_bound));
}

void PoissonDiskSampler::Generate(unsigned int number_of_samples) {
    throw DYNO::Exceptions::NotImplemented();
}

double PoissonDiskSampler::GenerateRandomNumber() {
    return uniform_distribution_(random_engine_);
}

void PoissonDiskSampler::SetMinimumDistance(double distance) {
    minimum_distance_ = distance;
    InitializeGrid();
}

void PoissonDiskSampler::InitializePRNG() {
    uniform_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);
    random_engine_ = std::mt19937(random_device_());
}

void PoissonDiskSampler::Seed() {
    random_engine_.seed();
}

void PoissonDiskSampler::Seed(unsigned int seed) {
    random_engine_.seed(seed);
}

void PoissonDiskSampler::SetIterations(unsigned int iterations) {
    iterations_ = iterations;
}

void PoissonDiskSampler::SetRadiusRange(double radius_min, double radius_max) {
    radius_min_ = radius_min;
    radius_max_ = radius_max;
    InitializeGrid();
}

double PoissonDiskSampler::GenerateSampleRadius() {
    return GenerateRandomNumber() * (radius_max_ - radius_min_) + radius_min_;
}

}  // namespace Math
}  // namespace DYNO
