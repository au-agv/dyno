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

#include <dyno/math/poisson_point_2d.hpp>

namespace DYNO {
namespace Math {

void to_json(nlohmann::json& data, const PoissonPoint2D& point) {
    data = nlohmann::json{point.GetX(), point.GetY(), point.GetRadius()};
}

void from_json(const nlohmann::json& data, PoissonPoint2D& point) {
    point = PoissonPoint2D(data[0], data[1], data[2]);
}

PoissonPoint2D::PoissonPoint2D() {}

PoissonPoint2D::PoissonPoint2D(double x, double y)
    : x_(x), y_(y), state_(PoissonPointState::ACTIVE) {}

PoissonPoint2D::PoissonPoint2D(double x, double y, double radius)
    : x_(x), y_(y), radius_(radius), state_(PoissonPointState::ACTIVE) {}

void PoissonPoint2D::Deactivate() {
    state_ = PoissonPointState::INACTIVE;
}

}  // namespace Math
}  // namespace DYNO
