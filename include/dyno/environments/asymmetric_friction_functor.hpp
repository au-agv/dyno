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

#include <chrono_vehicle/ChTerrain.h>

namespace DYNO {
namespace Environments {

/**
 * @brief A functor to evaluate friction based on location on a surface with two
 * materials, split by a central axis and active past a given longitudinal
 * threshold.
 *
 * This class provides a mechanism to evaluate friction coefficients at
 * different locations within a terrain. It inherits from
 * chrono::vehicle::ChTerrain::FrictionFunctor and allows for the definition of
 * a minimum longitudinal threshold value, as well as a lateral shift of the
 * axis defining the split surface.
 */
class AsymmetricFrictionFunctor
    : public chrono::vehicle::ChTerrain::FrictionFunctor {
   public:
    /**
     * @brief Constructs an AsymmetricFrictionFunctor object with optional split
       coordinate.
     *
     * This constructor initializes the AsymmetricFrictionFunctor object with
     * values for the friction coefficients for the left side, right lane, and a
     * fallback value for all points before a defined longitudinal coordinate.
     * The axis defining the left and right side split may optinally be offset
     * laterally.
     *
     * @param left_side_friction The friction coefficient for the left side,
     * returned for all the values past the split coordinate and to the right of
     * the lateral offset.
     * @param right_side_friction The friction coefficient for the right side,
     * returned for all the values past the split coordinate and to the left of
     * the lateral offset.
     * @param base_friction The base friction coefficient, returned for all
     * values before the split coordinate.
     * @param split_coordinate An optional parameter to specify the longitudinal
     * coordinate at which the split friction functor takes effect. Defaults to
     * 0.0.
     * @param lateral_offset An optional parameter to specify the lateral offset
     * for the left and right split axis. Defaults to 0.0.
     */
    AsymmetricFrictionFunctor(float left_side_friction,
                              float right_lane_friction, float base_friction,
                              float split_coordinate = 0.0,
                              float lateral_offset = 0.0);

    /**
     * @brief Evaluates the friction coefficient at a given location.
     *
     * This operator function evaluates and returns the friction coefficient
     * based on the x and y coordinates of the specified location. The logic
     * determines the friction value by checking the signs of the x and y
     * components of the location vector.
     *
     * If the location is before the longitudinal coordinate specified by the
     * split coordinate, a fallback friction value is return instead.
     *
     * If a lateral offset is specified, the left and right friction
     * coefficients are evaluated relative to a laterally-shifted axis.
     *
     * @param location The location at which to evaluate the friction
     * coefficient.
     * @return The friction coefficient at the specified location.
     */
    float operator()(const chrono::ChVector3d& location);

   private:
    /**
     * @brief The base friction coefficient (for all locations before the
     * longitudinal split coordinate).
     */
    double base_friction_;

    /**
     * @brief The friction coefficient for the left side of the split axis.
     */
    double left_side_friction_;

    /**
     * @brief The friction coefficient for the right side of the split axis.
     */
    double right_side_friction_;

    /**
     * @brief The longitudinal coordinate used to determine the initial point of
     * the split surface.
     */
    double split_coordinate_;

    /**
     * @brief The lateral offset used to adjust the axis defining the left and
     * ride sides.
     */
    double lateral_offset_;
};

}  // namespace Environments
}  // namespace DYNO
