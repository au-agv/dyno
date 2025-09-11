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

#include <dyno/environments/asymmetric_friction_functor.hpp>

namespace DYNO {
namespace Environments {

AsymmetricFrictionFunctor::AsymmetricFrictionFunctor(float left_side_friction,
                                                     float right_side_friction,
                                                     float base_friction,
                                                     float split_coordinate,
                                                     float lateral_offset)
    : chrono::vehicle::ChTerrain::FrictionFunctor(),
      left_side_friction_(left_side_friction),
      right_side_friction_(right_side_friction),
      base_friction_(base_friction),
      split_coordinate_(split_coordinate),
      lateral_offset_(lateral_offset) {}

float AsymmetricFrictionFunctor::operator()(
    const chrono::ChVector3d& location) {
    if (location.x() > split_coordinate_) {
        if (location.y() < lateral_offset_) {
            return left_side_friction_;
        }
        return right_side_friction_;
    }
    return base_friction_;
}

}  // namespace Environments
}  // namespace DYNO
