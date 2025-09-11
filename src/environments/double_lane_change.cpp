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

#include <dyno/environments/double_lane_change.hpp>

namespace DYNO {
namespace Environments {

DoubleLaneChange::DoubleLaneChange(double x_minimum,
                                   double acceleration_length,
                                   double vehicle_width,
                                   double vehicle_length,
                                   double wheelbase,
                                   double path_height,
                                   bool left_turn)
    : x_minimum_(x_minimum),
      acceleration_length_(acceleration_length),
      vehicle_length_(vehicle_length),
      vehicle_width_(vehicle_width),
      wheelbase_(wheelbase),
      path_height_(path_height),
      left_turn_(left_turn) {}

void DoubleLaneChange::Initialize() {
    // Define the lengths of the double lane change sections.
    section_1_length_ = 15.0;
    section_2_length_ = vehicle_length_ + 24.0;
    section_3_length_ = 25.0;
    section_4_length_ = section_2_length_;
    section_5_length_ = 15.0;

    // Define the widths of the double lane change sections.
    section_1_width_ = 1.1 * vehicle_width_ + 0.25;
    section_2_width_ = 1.2 * vehicle_width_ + 0.25;
    double section_3_width_ = section_2_width_;
    double section_4_width_ = section_2_width_;

    // Define the offsets.
    offset_ = left_turn_ ? 3.5 : -(3.5);
    m_ofsC = left_turn_ ? (section_4_width_ - section_1_width_) / 2
                        : (section_1_width_ - section_4_width_) / 2;

    double initial_point = x_minimum_ + acceleration_length_;

    // Define the gate positions.
    double gate_1_position = initial_point + section_1_length_;
    double gate_2_position = gate_1_position + section_2_length_;
    double gate_3_position = gate_2_position + section_3_length_;
    double gate_4_position = gate_3_position + section_4_length_;
    double gate_5_position = gate_4_position + section_5_length_;

    // Build section 0-1 of the NATO AVTP 03-160 Double Lane Change.
    line_center_.push_back(
        chrono::ChVector3<double>(x_minimum_, 0.0, path_height_));
    line_left_.push_back(
        chrono::ChVector3<double>(initial_point, section_1_width_ / 2.0, 0.0));
    line_right_.push_back(
        chrono::ChVector3<double>(initial_point, -section_1_width_ / 2.0, 0.0));

    // Build section 1-2 of the NATO AVTP 03-160 Double Lane Change.
    line_center_.push_back({gate_1_position, 0, path_height_});
    line_left_.push_back({gate_1_position, section_1_width_ / 2.0, 0});
    line_right_.push_back({gate_1_position, -section_1_width_ / 2.0, 0});

    // Build section 2-3 of the NATO AVTP 03-160 Double Lane Change.
    line_center_.push_back({gate_2_position, 0, path_height_});
    line_center_.back().y() += offset_;
    line_left_.push_back({gate_2_position, section_2_width_ / 2.0, 0});
    line_left_.back().y() += offset_;
    line_right_.push_back({gate_2_position, -section_2_width_ / 2.0, 0});
    line_right_.back().y() += offset_;

    // Build section 3-4 of the NATO AVTP 03-160 Double Lane Change.
    line_center_.push_back({gate_3_position, 0, path_height_});
    line_center_.back().y() += offset_;
    line_left_.push_back({gate_3_position, section_3_width_ / 2.0, 0});
    line_left_.back().y() += offset_;
    line_right_.push_back({gate_3_position, -section_3_width_ / 2.0, 0});
    line_right_.back().y() += offset_;

    // Build section 4-5 of the NATO AVTP 03-160 Double Lane Change.
    line_center_.push_back({gate_4_position, 0, path_height_});
    line_center_.back().y() += m_ofsC;
    line_left_.push_back({gate_4_position, section_4_width_ / 2.0, 0});
    line_left_.back().y() += m_ofsC;
    line_right_.push_back({gate_4_position, -section_4_width_ / 2.0, 0});
    line_right_.back().y() += m_ofsC;

    // Build the double lane change section 5-6.
    line_center_.push_back({gate_5_position, 0, path_height_});
    line_center_.back().y() += m_ofsC;
    line_left_.push_back({gate_5_position, section_4_width_ / 2.0, 0});
    line_left_.back().y() += m_ofsC;
    line_right_.push_back({gate_5_position, -section_4_width_ / 2.0, 0});
    line_right_.back().y() += m_ofsC;

    // Add the deceleration length to the centerline path.
    line_center_.back().x() += deceleration_length_;

    // Add road cones locations like in the standard
    for(auto& cone_position : {line_left_[0], // Cone 1
                               (line_left_[0] + line_left_[1]) / 2.0, // Cone 2
                               line_left_[1], // Cone 3
                               line_left_[2], // Cone 4
                               (line_left_[2] + line_left_[3]) / 2.0, // Cone 5
                               line_left_[3], // Cone 6
                               line_left_[4], // Cone 7
                               (line_left_[4] + line_left_[5]) / 2.0, // Cone 8
                               line_left_[5]}) { // Cone 9
        left_cones_positions_.push_back(cone_position);
    }
    // Cone 1A
    right_cones_positions_.push_back(line_right_[0]);
    // Cone 2A
    right_cones_positions_.push_back((line_right_[0] + line_right_[1]) / 2.0);
    // Cone 3A
    right_cones_positions_.push_back(line_right_[1]);
    // Cone 4A
    right_cones_positions_.push_back(line_right_[2]);
    // Cone 5A
    right_cones_positions_.push_back((line_right_[2] + line_right_[3]) / 2.0);
    // Cone 6A
    right_cones_positions_.push_back(line_right_[3]);
    // Cone 7A
    right_cones_positions_.push_back(line_right_[4]);
    // Cone 8A
    right_cones_positions_.push_back((line_right_[4] + line_right_[5]) / 2);
    // Cone 9A
    right_cones_positions_.push_back(line_right_[5]);

    // Prepare path spline definition. Here we use 1/3 of section 3 length as
    // control vertices for the Bezier curve.
    chrono::ChVector3<double> offset(section_3_length_ / 3, 0, 0);
    for(chrono::ChVector3<double>& node : line_center_) {
        control_vertices_in.push_back(node - offset);
        control_vertices_out.push_back(node + offset);
    }
    path_ = std::make_shared<chrono::ChBezierCurve>(line_center_,
                                                    control_vertices_in,
                                                    control_vertices_out);
}

void DoubleLaneChange::CreateSceneObjects(
    std::shared_ptr<chrono::vehicle::ChVehicleVisualSystem> visualization,
    int& sentinel_id,
    int& target_id) {
    // Add visualization for the sentinel point, keeping track of its
    // visualization ID.
    auto sentinel_sphere = std::make_shared<chrono::ChVisualShapeSphere>(0.1);
    sentinel_sphere->SetColor(chrono::ChColor(1.0, 0.0, 0.0));
    sentinel_id = visualization->AddVisualModel(sentinel_sphere,
                                                chrono::ChFrame<double>());

    // Add visualization for the target point, keeping track of its
    // visualization ID.
    auto target_sphere = std::make_shared<chrono::ChVisualShapeSphere>(0.1);
    target_sphere->SetColor(chrono::ChColor(0.0, 1, 0.0));
    target_id =
        visualization->AddVisualModel(target_sphere, chrono::ChFrame<double>());

    // Add the road cones.
    chrono::ChVector3<double> cone_offset(0.0, 0.21, 0.0);
    auto cone = std::make_shared<chrono::ChVisualShapeModelFile>();
    cone->SetFilename(chrono::vehicle::GetVehicleDataFile("trafficCone750mm.obj"));
    for(const auto& position : GetLeftConePositions())
        visualization->AddVisualModel(
            cone,
            chrono::ChFrame<double>(position + cone_offset));
    for(const auto& position : GetRightConePositions())
        visualization->AddVisualModel(
            cone,
            chrono::ChFrame<double>(position - cone_offset));
}

bool DoubleLaneChange::GateTestLeft(chrono::ChVector3<double>& position) {
    if(position.x() >= line_left_[0].x() && position.x() <= line_left_[1].x())
        return position.y() <= line_left_[0].y();
    if(position.x() >= line_left_[2].x() && position.x() <= line_left_[3].x())
        return position.y() <= line_left_[2].y();
    if(position.x() >= line_left_[4].x() && position.x() <= line_left_[5].x())
        return position.y() <= line_left_[4].y();

    return true;
}

bool DoubleLaneChange::GateTestRight(chrono::ChVector3<double>& position) {
    if(position.x() >= line_left_[0].x() && position.x() <= line_left_[1].x())
        return position.y() >= line_right_[0].y();
    if(position.x() >= line_left_[2].x() && position.x() <= line_left_[3].x())
        return position.y() >= line_right_[2].y();
    if(position.x() >= line_left_[4].x() && position.x() <= line_left_[5].x())
        return position.y() >= line_right_[4].y();

    return true;
}

const std::vector<chrono::ChVector3<double>>&
DoubleLaneChange::GetLeftConePositions() const {
    return left_cones_positions_;
}
const std::vector<chrono::ChVector3<double>>&
DoubleLaneChange::GetRightConePositions() const {
    return right_cones_positions_;
}

double DoubleLaneChange::GetManeuverLength() {
    return line_left_[5].x() - line_left_[0].x();
}

double DoubleLaneChange::GetXmax() { return line_left_[5].x(); }

double DoubleLaneChange::GetPathHeight() { return path_height_; }

void DoubleLaneChange::SetPathHeight(double path_height) {
    path_height_ = path_height;
}

bool DoubleLaneChange::HasFailed(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle) {
    // TODO: Check that the Z-coordinate of the vehicle point follows the NATO
    // standard.
    auto front_left_tire_position = vehicle->GetPointLocation(
        chrono::ChVector3<double>(0.0, vehicle_width_ / 2.0, 0.5));
    auto rear_left_tire_position = vehicle->GetPointLocation(
        chrono::ChVector3<double>(-wheelbase_, vehicle_width_ / 2.0, 0.5));
    auto front_right_tire_position = vehicle->GetPointLocation(
        chrono::ChVector3<double>(0.0, -vehicle_width_ / 2.0, 1.0));
    auto rear_right_tire_position = vehicle->GetPointLocation(
        chrono::ChVector3<double>(-wheelbase_, -vehicle_width_ / 2.0, 0.5));

    if(!GateTestLeft(front_left_tire_position)) { return true; }
    if(!GateTestLeft(rear_left_tire_position)) { return true; }
    if(!GateTestRight(front_right_tire_position)) { return true; }
    if(!GateTestRight(rear_right_tire_position)) { return true; }

    return false;
}

std::shared_ptr<chrono::ChBezierCurve> DoubleLaneChange::GetPath() {
    return path_;
}

} // namespace Environments
} // namespace DYNO
