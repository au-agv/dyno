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

#include <gtest/gtest.h>

#include <fstream>

#include <dyno/math/poisson_disk_sampler_2d.hpp>

using DYNO::Math::PoissonDiskSampler2D;


TEST(PoissonDisk, Serialization) {
    nlohmann::json data;
    PoissonDiskSampler2D sampler(100.0, 1500.0, 3.0);
    sampler.Generate(2);
    sampler.Serialize(data);
}

TEST(PoissonDisk, NoThreshold2D) {
    PoissonDiskSampler2D sampler(100.0, 100.0, 5.0);

    sampler.Generate();
}

TEST(PoissonDisk, FixedSamples2D) {
    PoissonDiskSampler2D sampler(100.0, 100.0, 5.0);
    sampler.Generate(100);
}

TEST(PoissonDisk, RangeSamples2D) {
    PoissonDiskSampler2D sampler(100.0, 100.0, 5.0);
    sampler.Generate(3, 100);
}

TEST(PoissonDisk, TransformSamples) {
    PoissonDiskSampler2D sampler(100.0, 100.0, 5.0);
    sampler.SetOrigin(10.0, 10.0, 5.0);
    sampler.Generate(3, 100);
}