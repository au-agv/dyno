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

#include <dyno/environments/asymmetric_soil_parameters_callback.hpp>

namespace DYNO {
namespace Environments {

AsymmetricSoilParametersCallback::AsymmetricSoilParametersCallback()
    : chrono::vehicle::SCMTerrain::SoilParametersCallback() {}

void AsymmetricSoilParametersCallback::Set(
    const chrono::ChVector3d& location, double& bekker_kphi, double& bekker_kc,
    double& bekker_n, double& mohr_cohesion, double& mohr_friction,
    double& janosi_shear, double& elastic_stiffness,
    double& damping_coefficient) {
    bekker_kphi = 4.0e6;
    bekker_kc = 0.0;
    bekker_n = 2.3;
    mohr_cohesion = 0.0;
    mohr_friction = 3.0e1;
    janosi_shear = 1.0e-2;
    elastic_stiffness = 4.0e7;
    damping_coefficient = 1.0e-2;

    if (location.x() >= 0.0) {
        if (location.y() < 3.0 / 2.0) {
            bekker_n = 2.3;
        } else {
            bekker_n = 0.95;
            bekker_kphi = 1.0e6;
        }
    } else {
        bekker_n = 0.15;
        bekker_kphi = 0.3e6;
    }
}

}  // namespace Environments
}  // namespace DYNO
