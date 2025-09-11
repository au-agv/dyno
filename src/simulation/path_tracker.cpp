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

#include <dyno/simulation/path_tracker.hpp>

namespace DYNO {
namespace Simulation {

PathTracker::PathTracker(std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
                         const std::vector<Waypoint>& path, double tolerance)
    : vehicle_(vehicle), path_(path), tolerance_(tolerance) {}

void PathTracker::Update() {
    if (!vehicle_ || next_index_ >= path_.size())
        return;

    auto pos = vehicle_->GetChassis()->GetPos();
    double vx = pos.x();
    double vy = pos.y();

    // Compute cross-track error to the segment between previous and next
    // waypoint
    cross_track_error_ = ComputeCrossTrackError(vx, vy);

    const Waypoint& next_wp = path_[next_index_];

    double dist_sq = DistanceSquared(vx, vy, next_wp.x, next_wp.y);

    // Check if waypoint reached
    if (dist_sq <= tolerance_ * tolerance_) {
        next_index_++;
        failed_ = false;  // waypoint successfully reached
    } else {
        // Check if vehicle has moved past the waypoint without reaching it
        if (next_index_ > 0) {
            const Waypoint& prev_wp = path_[next_index_ - 1];

            double prev_dist_sq = DistanceSquared(vx, vy, prev_wp.x, prev_wp.y);

            // If moving away from previous waypoint and hasn't reached next
            if (prev_dist_sq > DistanceSquared(prev_wp.x, prev_wp.y, next_wp.x,
                                               next_wp.y) &&
                dist_sq > DistanceSquared(prev_wp.x, prev_wp.y, next_wp.x,
                                          next_wp.y)) {
                failed_ = true;
            }
        }
    }
}

double PathTracker::DistanceSquared(double x1, double y1, double x2,
                                    double y2) const {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return dx * dx + dy * dy;
}

double PathTracker::ComputeCrossTrackError(double x, double y) const {
    if (path_.empty())
        return 0.0;
    if (next_index_ == 0)
        return std::sqrt(DistanceSquared(x, y, path_[0].x, path_[0].y));

    const Waypoint& prev = path_[next_index_ - 1];
    const Waypoint& next = path_[next_index_];

    // Vector from prev to next waypoint
    double dx = next.x - prev.x;
    double dy = next.y - prev.y;

    double seg_len_sq = dx * dx + dy * dy;
    if (seg_len_sq == 0.0)
        return std::sqrt(DistanceSquared(x, y, prev.x, prev.y));

    // Project vehicle position onto segment
    double t = ((x - prev.x) * dx + (y - prev.y) * dy) / seg_len_sq;
    t = std::clamp(t, 0.0, 1.0);

    double proj_x = prev.x + t * dx;
    double proj_y = prev.y + t * dy;

    return std::sqrt(DistanceSquared(x, y, proj_x, proj_y));
}

}  // namespace Simulation
}  // namespace DYNO
