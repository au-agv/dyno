#include <dyno/visualization/wrapper.hpp>

namespace DYNO {
namespace Visualization {

Wrapper::Wrapper(
    std::shared_ptr<chrono::vehicle::ChVehicle> vehicle,
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : vehicle_(vehicle), configuration_(configuration) {
    if (use_system_clock_) {
        last_visualization_time_ = GetTimeInSeconds();
    } else {
        last_visualization_time_ = simulation_time_;
    }

    ParseOptions();
}

void Wrapper::ParseOptions() {
    SPDLOG_INFO("Parsing generic visualization system options ...");

    use_system_clock_ =
        configuration_->GetValue<bool>("visualization/useSystemClock", false);

    is_enabled_ =
        configuration_->GetValue<bool>("visualization/enabled", false);

    frame_rate_ = configuration_->GetValue<double>("visualization/fps", 30.0);

    save_output_ =
        configuration_->GetValue<bool>("visualization/export/enabled", false);

    output_frames_path_ = configuration_->GetValue<std::string>(
        "visualization/export/path", "./");
}

bool Wrapper::IsTimeToRender() {
    // Update the visualization only if enough tim has passed to guarantee
    // the frames per second are respected for the simulation time rather
    // than the system clock.

    if (use_system_clock_) {
        current_time_ = GetTimeInSeconds();
    } else {
        current_time_ = simulation_time_;
    }

    return (current_time_ - last_visualization_time_) > 1.0 / frame_rate_;
}

std::string Wrapper::GetPaddedFrameIndex() const {
    auto frame_index = std::to_string(frame_index_);
    size_t padding_zeros = 6;
    auto padded_frame_index =
        std::string(
            padding_zeros - std::min(padding_zeros, frame_index.length()),
            '0') +
        frame_index;

    return padded_frame_index;
}

}  // namespace Visualization
}  // namespace DYNO
