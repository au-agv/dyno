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

#include <dyno/models/controller_tuning.hpp>

namespace DYNO {
namespace Models {

ControllerTuning::ControllerTuning(double proportional_gain,
                                   double integral_gain, double derivative_gain)
    : proportional_gain_(proportional_gain),
      integral_gain_(integral_gain),
      derivative_gain_(derivative_gain) {}

const double& ControllerTuning::GetProportionalGain() const {
    return proportional_gain_;
}

const double& ControllerTuning::GetIntegralGain() const {
    return integral_gain_;
}

const double& ControllerTuning::GetDerivativeGain() const {
    return derivative_gain_;
}

}  // namespace Models
}  // namespace DYNO
