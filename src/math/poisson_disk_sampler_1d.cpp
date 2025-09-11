#include <dyno/math/poisson_disk_sampler_1d.hpp>

namespace DYNO {
namespace Math {

PoissonDiskSampler1D::PoissonDiskSampler1D(double disk_width,
                                           double minimum_distance)
    : PoissonDiskSampler(minimum_distance), disk_width_(disk_width) {

    InitializeGrid();
    SeedInitialPoint();
}

void PoissonDiskSampler1D::InitializeGrid() {
    // Define the size of a grid cell for the two-dimensional grid such that
    // all points will contain at most one sample.
    grid_cell_size_ = minimum_distance_ + radius_max_;

    // Define the grid geometry for the Poisson disk based on the calculate
    // grid cell size. Note that we always account for an extra grid cell.
    grid_width_ = std::ceil(disk_width_ / grid_cell_size_) + 1;
    grid_cells_num_ = grid_width_;

    // Resize the grid storage vectors to the appropriate sizes.
    grid_.clear();
    grid_.resize(grid_cells_num_);
}

void PoissonDiskSampler1D::Reset() {
    Seed();
    active_.clear();
    InitializeGrid();
    SeedInitialPoint();
}

void PoissonDiskSampler1D::LogGridStatistics() {
    SPDLOG_INFO("Poisson disk sampler parameters");
    SPDLOG_INFO("|- Minimum distance {}", minimum_distance_);
    SPDLOG_INFO("|- Disk width: {}", disk_width_);
    SPDLOG_INFO("|- Grid cell size: {:20.f}", grid_cell_size_);
    SPDLOG_INFO("|- Grid width: {}", grid_width_);
    SPDLOG_INFO("|- Grid number of cells: {}", grid_.size());
}

PoissonPoint1D PoissonDiskSampler1D::GenerateRandomPoissonPoint() {
    return PoissonPoint1D(GenerateRandomNumber() * disk_width_,
                          GenerateSampleRadius());
}

void PoissonDiskSampler1D::SeedInitialPoint() {
    auto seed_point = GenerateRandomPoissonPoint();
    AddPointToGrid(seed_point);
    AddPointToActiveList(seed_point);
}

void PoissonDiskSampler1D::AddPointToActiveList(const PoissonPoint1D& point) {
    SPDLOG_INFO("Adding point {:0.2f} to the active list ...", point.GetX());

    active_.push_back(point);
}

void PoissonDiskSampler1D::AddPointToGrid(const PoissonPoint1D& point) {
    // Calculate the flattened index array for the grid cell (i, j)
    auto i = size_t(std::floor(point.GetX() / grid_cell_size_));
    grid_[i] = point;

    SPDLOG_DEBUG("Adding point P = {:0.2f} to grid position {}", point.GetX(),
                 i);
}

void PoissonDiskSampler1D::TransformSamples() {
    for (auto& sample : samples_) {
        double x = sample.GetX() + origin_x_;
        double y = origin_y_;
        x = x * std::cos(origin_angle_) + y * std::sin(origin_angle_);
        sample = PoissonPoint1D(x, sample.GetRadius());
    }
}

void PoissonDiskSampler1D::SetOrigin(double x, double y, double theta) {
    origin_x_ = x;
    origin_y_ = y;
    origin_angle_ = theta;
    transform_samples_ = true;
}

std::vector<PoissonPoint1D> PoissonDiskSampler1D::GetSamples() {
    return samples_;
}

void PoissonDiskSampler1D::Generate(unsigned int number_of_samples) {
    SPDLOG_INFO("Starting random samples generation ...", number_of_samples);

    // Clear the previously generated samples before generating a new set.
    samples_.clear();

    while (!active_.empty()) {
        // Select a candidate point for the search.
        auto source_index =
            size_t(GenerateRandomNumber() * (active_.size() - 1));
        auto source_point = active_[source_index];

        SPDLOG_INFO("Selected source point {:0.2f} at index {}",
                    source_point.GetX(), source_index);

        bool found_valid_point = false;
        for (unsigned int i = 0; i < iterations_; ++i) {
            auto candidate_point = GenerateAroundPoint(source_point);

            if (IsValid(candidate_point)) {
                AddPointToGrid(candidate_point);
                active_.push_back(candidate_point);
                samples_.push_back(candidate_point);
                found_valid_point = true;
            }
        }

        if (!found_valid_point) {
            active_.erase(active_.begin() + source_index);
        }

        if (samples_.size() > number_of_samples) {
            SPDLOG_INFO("Successfully generated {} samples!", samples_.size());
            has_result_ = true;
            break;
        }
    }

    has_result_ = true;

    if (transform_samples_) {
        TransformSamples();
    }
}

PoissonPoint1D PoissonDiskSampler1D::GenerateAroundPoint(
    const PoissonPoint1D& point) {
    // Random angle
    double angle = 2.0 * M_PI * GenerateRandomNumber();
    // Random radius between r and 2r
    double radius =
        (minimum_distance_ + radius_max_) * (GenerateRandomNumber() + 1.0);

    // Convert polar coordinates to cartesian and viola,
    // a new point is generated around the source point (x, y)
    double new_x = point.GetX() + (radius * std::cos(angle));

    // Using min/max, we'll also constrain the new point to be within
    // the bounds of our grid.
    new_x = boost::algorithm::clamp(new_x, 0.0, disk_width_ - 1.0);

    return PoissonPoint1D(new_x, GenerateSampleRadius());
}

bool PoissonDiskSampler1D::IsValid(const PoissonPoint1D& point) {
    // Scale the source point onto the grid.
    size_t index_x = std::floor(point.GetX() / grid_cell_size_);

    // Determine the neighborhood around the source point.
    size_t start_x = std::max(index_x - 2.0, 0.0);
    size_t end_x = std::min(index_x + 2.0, grid_width_ - 1.0);

    // Check all non-empty neighbors cells and make sure the new point
    // is outside their radius.
    for (size_t i = start_x; i < end_x; ++i) {
        if (grid_[i].GetState() == PoissonPointState::ACTIVE ||
            grid_[i].GetState() == PoissonPointState::INACTIVE) {
            if (GetDistance(grid_[i], point) <=
                (minimum_distance_ + point.GetRadius() +
                 grid_[i].GetRadius())) {
                return false;
            }
        }
    }

    return true;
}

double PoissonDiskSampler1D::GetDistance(const PoissonPoint1D& first_point,
                                         const PoissonPoint1D& second_point) {
    return second_point.GetX() - first_point.GetX();
}

void PoissonDiskSampler1D::Serialize(nlohmann::json& data) {
    for (const auto& sample : samples_) {
        nlohmann::json data_sample;
        data.push_back(sample);
    }
}

}  // namespace Math
}  // namespace DYNO
