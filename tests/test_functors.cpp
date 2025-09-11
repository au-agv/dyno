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

#include <dyno/environments/asymmetric_friction_functor.hpp>
#include <dyno/environments/asymmetric_soil_parameters_callback.hpp>

TEST(Functors, FrictionFunctor) {
    const auto functor =
        std::make_unique<DYNO::Environments::AsymmetricFrictionFunctor>(
            0.5,    // Left side friction
            0.25,   // Right side friction
            0.8,    // Base friction
            100.0,  // Split coordinate
            1.0     // Lateral offset
        );

    ASSERT_FLOAT_EQ(functor->operator()(chrono::ChVector3d(99.0, 0.0, 0.0)),
                    0.8);
    ASSERT_FLOAT_EQ(functor->operator()(chrono::ChVector3d(101.0, 2.0, 0.0)),
                    0.25);
    ASSERT_FLOAT_EQ(functor->operator()(chrono::ChVector3d(101.0, -2.0, 0.0)),
                    0.5);
}

TEST(Functors, SoilParametersCallback) {
    // TODO: Implement the soil parameters callback test.
    //std::make_unique<DYNO::Environments::AsymmetricSoilParametersCallback>();
}
