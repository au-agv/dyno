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

#include <dyno/visualization/irrlicht.hpp>

namespace DYNO {
namespace Visualization {

Irrlicht::Irrlicht(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : Wrapper(vehicle, configuration) {}

void Irrlicht::Initialize() {
    if (is_enabled_) {
        SPDLOG_INFO("Initializing the Irrlicht visualization engine ...");

        // Irrlicht antli-aliasing configuration must be set before
        // initialization!
        visualization_->AttachVehicle(vehicle_.get());

        visualization_->SetAntialias(true);

        visualization_->SetWindowSize(
            configuration_->GetValue<double>("visualization/windowSize/width",
                                             1920),
            configuration_->GetValue<double>("visualization/windowSize/height",
                                             720));
        visualization_->EnableFullscreen(
            configuration_->GetValue<bool>("visualization/fullscreen", false));


        // Temporarily disable stderr output to avoid Irrlicht warnings for
        // video driver fallbacks.
        std::cerr.setstate(std::ios_base::failbit);
        visualization_->Initialize();
        std::cerr.clear();

        visualization_->EnableShadows();


        // Set up the chase camera parameters.
        SPDLOG_INFO("Configuring the chase camera ...");
        visualization_->SetChaseCamera(
            chrono::ChVector3<double>(-0.25, 0.0, 1.45), 4.25, 1.25);
        visualization_->SetChaseCameraAngle(45.0 * M_PI / 180.0);
        visualization_->SetChaseCameraMultipliers(0.5, 1.0);

        // Add light sources to the scene.
        SPDLOG_INFO("Adding light sources ...");
        const double top_light_plane_z = 1000.0;
        const double bottom_light_plane_z = 450.0;
        const auto white_light_color = chrono::ChColor(0.81f, 0.81f, 0.81f);
        const auto yellow_light_color =
            chrono::ChColor(255.0f / 255.0f, 205.0f / 255.0f, 162.0f / 255.0f);

        // Add a white skylight.
        visualization_->AddLight(
            chrono::ChVector3<double>(0.0, 0.0, top_light_plane_z),
            0.7 * top_light_plane_z, white_light_color);

        // Add a yellow-tinted 3-point illumination triad.
        visualization_->AddLight(
            chrono::ChVector3<double>(250.0, 500.0, bottom_light_plane_z),
            0.61 * bottom_light_plane_z, yellow_light_color);

        visualization_->AddLight(
            chrono::ChVector3<double>(250.0, -500.0, bottom_light_plane_z),
            0.61 * bottom_light_plane_z, yellow_light_color);

        visualization_->AddLight(
            chrono::ChVector3<double>(-top_light_plane_z / 2.0, 0.0,
                                      bottom_light_plane_z - 100.0),
            0.75 * bottom_light_plane_z, white_light_color);

        // Add the scene background and GUI overlay.
        SPDLOG_INFO("Adding GUI elements ...");
        visualization_->EnableStats(
            configuration_->GetValue("visualization/enabled", false));
        if (configuration_->GetValue("visualization/skybox", false)) {
            visualization_->AddSkyBox();
        };
        const auto color = configuration_->GetValue(
            "visualization/backgroundColor",
            std::vector<double>({58.0 / 255.0, 110.0 / 255.0, 155.0 / 255.0}));
        // TODO: Add call to SetBackgroundColor from future Chrono release
        /*
        visualization_->SetBackgroundColor(
        chrono::ChColor(color[0], color[1], color[2]));
    */
        // Set the rendered frames output directory.
        visualization_->SetImageOutputDirectory(output_frames_path_);
    }
}

void Irrlicht::Synchronize(const double& time_step,
                           const chrono::vehicle::DriverInputs& driver_inputs) {
    if (!is_enabled_)
        return;

    // Redraw the GUI elements.
    visualization_->Synchronize(simulation_time_, driver_inputs);
}

void Irrlicht::Advance(const double& time_step) {
    // Ignore the visualization advance step if the visualization is disabled
    // globally.
    if (!is_enabled_)
        return;

    // Advance the visualization by one time step.
    visualization_->Advance(time_step);
    simulation_time_ += time_step;

    if (IsTimeToRender()) {
        // Render the scene.
        if (!visualization_->Run())
            return;
        visualization_->BeginScene();
        visualization_->Render();
        visualization_->EndScene();

        if (save_output_) {
            visualization_->WriteImageToFile("frame_" + GetPaddedFrameIndex() +
                                             ".png");
        }

        frame_index_++;

        // Update the visualization time.
        if (use_system_clock_) {
            last_visualization_time_ = GetTimeInSeconds();
        } else {
            last_visualization_time_ = simulation_time_;
        }
    }
}

}  // namespace Visualization
}  // namespace DYNO
