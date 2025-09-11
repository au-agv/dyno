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

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <chrono_vehicle/ChVehicle.h>

namespace DYNO {
namespace Simulation {

struct Waypoint {
    double x;
    double y;
};

class PathTracker {
   public:
    PathTracker(std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
                const std::vector<Waypoint>& path, double tolerance);

    void Update();

    int GetNextWaypointIndex() const { return next_index_; }

    bool HasFailed() const { return failed_; }

    double GetCrossTrackError() const { return cross_track_error_; }

   private:
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle_;
    std::vector<Waypoint> path_;
    double tolerance_;  // distance to consider a waypoint reached

    int next_index_ = 0;
    bool failed_ = false;
    double cross_track_error_ = 0.0;

    double DistanceSquared(double x1, double y1, double x2, double y2) const;

    double ComputeCrossTrackError(double x, double y) const;
};

}  // namespace Simulation
}  // namespace DYNO
