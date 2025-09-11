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

#include <dyno/visualization/vulkan_scene_graph.hpp>

namespace DYNO {
namespace Visualization {

VulkanSceneGraph::VulkanSceneGraph(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : Wrapper(vehicle, configuration) {
    if (use_system_clock_) {
        last_visualization_time_ = GetTimeInSeconds();
    } else {
        last_visualization_time_ = simulation_time_;
    }

    // Parse the visualization options JSON document.
    ParseOptions();
}

void VulkanSceneGraph::Initialize() {
    if (!is_enabled_)
        return;

    visualization_->AttachVehicle(vehicle_.get());
    visualization_->Initialize();
}

void VulkanSceneGraph::Synchronize(
    const double& time_step,
    const chrono::vehicle::DriverInputs& driver_inputs) {
    if (!is_enabled_ || !IsTimeToRender())
        return;

    // Redraw the GUI elements.
    visualization_->Synchronize(simulation_time_, driver_inputs);
}

void VulkanSceneGraph::Advance(const double& time_step) {
    // Ignore the visualization advance step if the visualization is disabled
    // globally.
    if (!is_enabled_)
        return;

    // Advance the visualization by one time step.
    visualization_->Advance(time_step);
    simulation_time_ += time_step;

    // Update the visualization only if enough tim has passed to guarantee
    // the frames per second are respected for the simulation time rather
    // than the system clock.
    if (IsTimeToRender()) {
        if (!visualization_->Run())
            return;
        visualization_->BeginScene();
        visualization_->Render();
        visualization_->EndScene();

        if (save_output_) {
            visualization_->WriteImageToFile("frame_" + GetPaddedFrameIndex() +
                                             ".png");
        }

        if (use_system_clock_) {
            last_visualization_time_ = GetTimeInSeconds();
        } else {
            last_visualization_time_ = simulation_time_;
        }
    }
}

void VulkanSceneGraph::ParseOptions() {
    visualization_ =
        std::make_shared<chrono::vehicle::ChWheeledVehicleVisualSystemVSG>();

    visualization_->SetVerbose(false);
    visualization_->SetChaseCamera(chrono::ChVector3<double>(0.75, 1.75, 1.0),
                                   7.5, 2.25);
    visualization_->SetChaseCameraMultipliers(0.5, 2.0);

    visualization_->SetLogo(std::string(DYNO_DATA_DIR) +
                            "/textures/logo/dyno.png");
    visualization_->SetLightDirection(-45.0 * M_PI / 180.0,
                                      60.0 * M_PI / 180.0);
    visualization_->SetLightIntensity(
        configuration_->GetValue<double>("visualization/light", 1.0));
    visualization_->SetWindowTitle("DYNO");

    const auto color = configuration_->GetValue(
        "visualization/backgroundColor",
        std::vector<double>({255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0}));
    visualization_->SetBackgroundColor(
        chrono::ChColor(color[0], color[1], color[2]));

    visualization_->EnableSkyBox(
        configuration_->GetValue<bool>("visualization/skybox", false));

    visualization_->EnableFullscreen(
        configuration_->GetValue<bool>("visualization/fullscreen", false));

    visualization_->SetWindowSize(configuration_->GetValue<double>(
                                      "visualization/windowSize/width", 1920),
                                  configuration_->GetValue<double>(
                                      "visualization/windowSize/height", 1080));

    visualization_->SetGuiVisibility(
        configuration_->GetValue<bool>("visualization/gui/enabled", true));

    visualization_->SetGuiFontSize(
        configuration_->GetValue<double>("visualization/gui/fontSize", 18.0));
    visualization_->SetBaseGuiVisibility(false);
    visualization_->EnableShadows(
        configuration_->GetValue<bool>("visualization/shadows", true));

    if (save_output_) {
        visualization_->SetImageOutputDirectory("/home/robot/data");
        visualization_->SetImageOutput(true);
    }
}

}  // namespace Visualization
}  // namespace DYNO
