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

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <chrono_vehicle/ChVehicle.h>

namespace DYNO {
namespace Simulation {

struct CircleObject {
    double x;       // X position
    double y;       // Y position
    double radius;  // Radius of the circle
};

class SimplifiedCollisionChecker {
   public:
    // Constructor with vehicle pointer and radius
    SimplifiedCollisionChecker(
        std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
        double vehicle_radius);

    // Add an obstacle
    void AddObstacle(double x, double y, double radius);

    // Check for collisions at the current vehicle position
    bool Check();

    // Return indices of colliding obstacles
    std::vector<int> GetCollidingObstacles();

   private:
    double vehicle_radius_;
    std::vector<CircleObject> obstacles_;
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;

    bool CirclesCollide(const CircleObject& c1, const CircleObject& c2) const;
};

}  // namespace Simulation
}  // namespace DYNO
