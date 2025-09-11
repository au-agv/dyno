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

#include <dyno/simulation/collision_checker.hpp>

namespace DYNO {
namespace Simulation {

SimplifiedCollisionChecker::SimplifiedCollisionChecker(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle, double vehicle_radius)
    : vehicle_(vehicle), vehicle_radius_(vehicle_radius) {}

void SimplifiedCollisionChecker::AddObstacle(double x, double y,
                                             double radius) {
    obstacles_.push_back({x, y, radius});
}

bool SimplifiedCollisionChecker::Check() {
    if (!vehicle_)
        return false;

    // Get vehicle's position from Chrono
    auto pos = vehicle_->GetChassis()->GetPos();  // ChVector
    CircleObject vehicle_circle = {pos.x(), pos.y(), vehicle_radius_};

    for (const auto& obs : obstacles_) {
        if (CirclesCollide(vehicle_circle, obs)) {
            return true;
        }
    }
    return false;
}

std::vector<int> SimplifiedCollisionChecker::GetCollidingObstacles() {
    std::vector<int> colliding_indices;
    if (!vehicle_)
        return colliding_indices;

    auto pos = vehicle_->GetChassis()->GetPos();
    CircleObject vehicle_circle = {pos.x(), pos.y(), vehicle_radius_};

    for (size_t i = 0; i < obstacles_.size(); ++i) {
        if (CirclesCollide(vehicle_circle, obstacles_[i])) {
            colliding_indices.push_back(static_cast<int>(i));
        }
    }
    return colliding_indices;
}

bool SimplifiedCollisionChecker::CirclesCollide(const CircleObject& c1,
                                                const CircleObject& c2) const {
    double dx = c1.x - c2.x;
    double dy = c1.y - c2.y;
    double distance_sq = dx * dx + dy * dy;
    double radius_sum = c1.radius + c2.radius;
    return distance_sq <= radius_sum * radius_sum;
}

}  // namespace Simulation
}  // namespace DYNO
