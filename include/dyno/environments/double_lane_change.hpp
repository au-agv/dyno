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

#include <chrono/assets/ChVisualShapeModelFile.h>
#include <chrono/assets/ChVisualShapeSphere.h>
#include <chrono/core/ChBezierCurve.h>
#include <chrono/utils/ChUtils.h>
#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/ChVehicleDataPath.h>
#include <chrono_vehicle/ChVehicleVisualSystem.h>

namespace DYNO {
namespace Environments {

class DoubleLaneChange {
   public:
    DoubleLaneChange(double x_minimum, double acc_length, double vehicle_length,
                     double vehicle_width, double wheelbase, double path_height,
                     bool left_turn);

    void Initialize();

    double GetSectionGatePosition(unsigned int section_no);

    bool GateTestLeft(chrono::ChVector3<double>& position);

    bool GateTestRight(chrono::ChVector3<double>& position);

    const std::vector<chrono::ChVector3<double>>& GetLeftConePositions() const;

    const std::vector<chrono::ChVector3<double>>& GetRightConePositions() const;

    double GetManeuverLength();

    double GetXmax();

    std::shared_ptr<chrono::ChBezierCurve> GetPath();

    double GetPathHeight();

    void SetPathHeight(double path_height);

    void CreateSceneObjects(
        std::shared_ptr<chrono::vehicle::ChVehicleVisualSystem> vis,
        int& sentinelID, int& targetID);

    bool HasFailed(std::shared_ptr<chrono::vehicle::ChVehicle> vehicle);

   private:
    /** @brief Minimum X coordinate of the track. */
    double x_minimum_;

    /** @brief Length of the acceleration corridor. */
    double acceleration_length_;

    /** @brief Manouvre path. */
    std::shared_ptr<chrono::ChBezierCurve> path_;

    /** @brief Left line points. */
    std::vector<chrono::ChVector3<double>> line_left_;

    /** @brief Center line points. */
    std::vector<chrono::ChVector3<double>> line_center_;

    /** @brief Right line points. */
    std::vector<chrono::ChVector3<double>> line_right_;

    /** @brief Left cones positions. */
    std::vector<chrono::ChVector3<double>> left_cones_positions_;

    /** @brief Right cones positions.*/
    std::vector<chrono::ChVector3<double>> right_cones_positions_;

    /** @brief Length of the vehicle, measured at 0.5 meters from the ground. */
    double vehicle_length_;

    /** @brief Width of the vehicle. */
    double vehicle_width_;

    /** @brief Wheelbase of the vehicle. */
    double wheelbase_;

    double path_height_ = 0.5;

    /** @brief Whether or not to set up a left turn. */
    bool left_turn_;

    /** @brief Length of section 1. */
    double section_1_length_;

    /** @brief Width of section 1. */
    double section_1_width_;

    /** @brief Length of section 2. */
    double section_2_length_;

    /** @brief Width of section 2, without accounting for the offset. */
    double section_2_width_;

    double section_3_length_;

    /** @brief Section 2 offset.*/
    double offset_;

    double m_widthC;

    double section_5_length_;

    double m_lengthAB;

    double section_4_length_;

    double deceleration_length_ = 100.0;

    double m_ofsC;

    std::vector<chrono::ChVector3<double>> control_vertices_in;

    std::vector<chrono::ChVector3<double>> control_vertices_out;
};

}  // namespace Environments
}  // namespace DYNO
