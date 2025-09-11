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

#include <dyno/simulation/grade_climbing.hpp>

namespace DYNO {
namespace Simulation {

GradeClimbing::GradeClimbing(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {}

void GradeClimbing::GetConfiguration() {
    SPDLOG_INFO("Reading configuration for the grade climbing scenario.");

    // ---------------------------------------------------------------------- //
    // Prepare the time stamps for the initial speed transient.
    // ---------------------------------------------------------------------- //
    time_to_initial_speed_ =
        configuration_->GetValue<double>("scenario/minimumTimeToInitialSpeed");
    const double time_speed_buffer = 0.1 * time_to_initial_speed_;
    time_speed_initial_start_ = warmup_time_;
    time_speed_initial_end_ =
        time_speed_initial_start_ + time_to_initial_speed_ + time_speed_buffer;

    initial_speed_tolerance_ =
        configuration_->GetValue<double>("scenario/initialSpeedTolerance", 0.1);

    // ---------------------------------------------------------------------- //
    // Initialize the sinusoidal step function for the speed transient.
    // ---------------------------------------------------------------------- //
    target_initial_speed_ =
        configuration_->GetValue<double>("scenario/targetInitialSpeed");
    try {
        experiment_speed_ =
            configuration_->GetValue<double>("scenario/experimentSpeed");
    } catch (const std::invalid_argument& exception) {
        SPDLOG_INFO(
            "No experiment speed specified: using initial speed as target.");
        experiment_speed_ = target_initial_speed_;
    }

    speed_ramp_ = std::make_shared<chrono::ChFunctionSineStep>(
        chrono::ChVector2(time_speed_initial_start_, 0.0),
        chrono::ChVector2(time_speed_initial_end_, target_initial_speed_));

    // ---------------------------------------------------------------------- //
    // Prepare the time stamps for the grade transient.
    // ---------------------------------------------------------------------- //
    time_to_target_grade_ =
        configuration_->GetValue<double>("scenario/timeToMaxGrade");
    target_grade_ =
        configuration_->GetValue<double>("scenario/targetGrade", 0.0);
    use_grade_ramp_ = configuration_->GetValue("scenario/useGradeRamp", true);

    friction_coefficient_ = configuration_->GetValue(
        "scenario/terrain/rigid/frictionCoefficient", 0.85);

    averaging_window_ =
        configuration_->GetValue<double>("scenario/averagingWindow", 0.5);
    speed_filter_ = std::make_shared<chrono::utils::ChRunningAverage>(
        int(averaging_window_ /
            configuration_->GetValue<double>("simulation/timeStep", 1.0e-3)));
    speed_tolerance_ =
        configuration_->GetValue<double>("scenario/speedTolerance", 0.2);

    terrain_type_ =
        configuration_->GetValue<std::string>("scenario/terrain/type", "rigid");
}

void GradeClimbing::Instantiate() {}

void GradeClimbing::InitializeGradeRamp(double initial_time) {
    time_grade_start_ = initial_time;
    time_grade_end_ = time_grade_start_ + time_to_target_grade_;

    grade_ramp_ = std::make_shared<chrono::ChFunctionSineStep>(
        chrono::ChVector2(time_grade_start_, 0.0),
        chrono::ChVector2(time_grade_end_, target_grade_));

    grade_ramp_initialized_ = true;
}

void GradeClimbing::InitializeDriver() {
    auto path_driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(),
        chrono::vehicle::StraightLinePath(
            chrono::VNULL, chrono::ChVector3<double>(1.0e5, 0.0, 0.0), 100),
        "path", 0.0);

    InitializeSteeringController(path_driver->GetSteeringController());
    InitializeSpeedController(path_driver->GetSpeedController());

    path_driver->Initialize();
    driver_ = path_driver;

    VehicleSimulation::InitializeDriver();
}

void GradeClimbing::InitializeTerrain() {
    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    terrain_->InitializeRigidTerrainSinglePatch(
        friction_coefficient_, chrono::ChVector2d(0.0, 0.0),
        chrono::ChVector2d(1.0, 1.0),
        std::filesystem::path(std::string(DYNO_DATA_DIR) +
                              "textures/terrain/checker_white.png"));
}

void GradeClimbing::PostInitializationHook() {
    failure_detector_ =
        std::make_shared<VehicleFailureDetector>(vehicle_->GetVehicle());
    failure_detector_->SetYawLimit(10.0, AngleUnit::Degrees);
    failure_detector_->SetYawRateLimit(10.0, RateUnit::DegreesPerSecond);
    failure_detector_->SetRollLimit(10.0, AngleUnit::Degrees);
    failure_detector_->SetRollRateLimit(10.0, RateUnit::DegreesPerSecond);

    steady_state_detector_ =
        std::make_shared<VehicleSteadyStateDetector>(vehicle_->GetVehicle());
    steady_state_detector_->SetTargetSpeed(experiment_speed_);

    downshift_latch_ =
        std::make_shared<GearDownshiftLatch>(vehicle_->GetVehicle());

    VehicleSimulation::PostInitializationHook();
}

void GradeClimbing::PreSynchronizationHook() {
    VehicleSimulation::PreSynchronizationHook();
}

void GradeClimbing::PostSynchronizationHook() {
    if (time_ < warmup_time_) {
        return;
    }

    // ---------------------------------------------------------------------- //
    // Vehicle speed estimation and update
    // ---------------------------------------------------------------------- //
    // Update the vehicle speed through the running average filter.
    double unfiltered_speed = GetVehicle()->GetSpeed();
    current_speed_ = speed_filter_->Add(unfiltered_speed);

    // ---------------------------------------------------------------------- //
    // Launch speed initial transient
    // ---------------------------------------------------------------------- //
    if (!speed_reached_) {
        double target_speed = speed_ramp_->GetVal(time_);
        std::dynamic_pointer_cast<chrono::vehicle::ChPathFollowerDriver>(
            driver_)
            ->SetDesiredSpeed(target_speed);

        // Report the state of the speed ramp process to the log.
        SPDLOG_DEBUG(
            "Speed ramp >> [{:0.1f}%% complete] | Time: "
            "({:0.0f} < {:0.0f} < {:0.0f}) s | Speed: ({:0.2f} [{:0.1f}] < "
            "{:0.1f}) "
            "m/s",
            100.0 * (time_ - time_speed_initial_start_) /
                (time_speed_initial_end_ - time_speed_initial_start_),
            time_speed_initial_start_, time_, time_speed_initial_end_,
            std::atan(current_speed_ / 100.0) * 180.0 / M_PI, unfiltered_speed,
            target_initial_speed_);

        // Note that we finalize the hook and quit early until the desired speed
        // has been reached.
        VehicleSimulation::PostSynchronizationHook();
        return;
    }

    // ---------------------------------------------------------------------- //
    // Grade ramp initial transient
    // ---------------------------------------------------------------------- //
    if (!grade_reached_) {
        if (use_grade_ramp_ && !grade_ramp_initialized_) {
            InitializeGradeRamp(time_);
        }

        // Get the target grade value for the current time step from the grade
        // functor.
        current_grade_ =
            use_grade_ramp_ ? grade_ramp_->GetVal(time_) : target_grade_;
        ApplyGrade();

        VehicleSimulation::PostSynchronizationHook();
        return;
    }

    if (time_ > start_time_) {
        std::dynamic_pointer_cast<chrono::vehicle::ChPathFollowerDriver>(
            driver_)
            ->SetDesiredSpeed(experiment_speed_);
    }

    VehicleSimulation::PostSynchronizationHook();
}

void GradeClimbing::ApplyGrade() {
    // Rotate the gravitational acceleration vector to apply the current grade.
    system_->SetGravitationalAcceleration(
        chrono::QuatFromRodrigues(
            chrono::ChVector3(0.0, std::atan(current_grade_ / 100.0), 0.0))
            .Rotate(chrono::ChVector3d(0.0, 0.0, -9.81)));

    const double grade_in_degrees =
        std::atan(current_grade_ / 100.0) * 180.0 / M_PI;
    const double grade_percentage = current_grade_;

    // Report the current grade in degrees to the log.
    if (use_grade_ramp_) {
        const double grade_ramp_percentage_completion =
            100.0 * (time_ - time_grade_start_) /
            (time_grade_end_ - time_grade_start_);

        SPDLOG_DEBUG(
            "Grade ramp >> [{:0.1f}%% complete] | Time: "
            "({:0.0f} < {:0.0f} < {:0.0f}) "
            "s | Grade: "
            "{:0.1f} deg [{:0.1f}%%]",
            grade_ramp_percentage_completion, time_grade_start_, time_,
            time_grade_end_, grade_in_degrees, grade_percentage);
    } else {
        SPDLOG_DEBUG("Grade {:0.1f} deg [{:0.1f}%%]", grade_in_degrees,
                     grade_percentage);
    }
}

void GradeClimbing::PostStepHook() {
    if (time_ > warmup_time_) {
        auto report = failure_detector_->Check(time_step_);
        if (!(report.type == VehicleFailureType::None)) {
            SPDLOG_ERROR(
                "Detected vehicle failure {} >> Value: {:0.2f} >> Limit: "
                "{:0.2f}",
                static_cast<int>(report.type), report.value, report.limit);
            is_completed_ = true;
            is_successful_ = false;
        }
    }

    if (time_ < start_time_) {

        if (!speed_reached_) {
            if (DYNO::Math::IsClose(current_speed_, target_initial_speed_,
                                    initial_speed_tolerance_)) {
                speed_reached_ = true;
                SPDLOG_INFO(
                    "Reached target speed {:0.2f} m/s at time {:0.2f} s",
                    current_speed_, time_);
            } else {
                // Quit early if the speed is not yet reached.
                VehicleSimulation::PostStepHook();
                return;
            }
        }

        if (!grade_reached_) {
            if (DYNO::Math::IsClose(current_grade_, target_grade_)) {

                grade_reached_ = true;
                start_time_ = time_ + 3.0;

                SPDLOG_INFO(
                    "Reached target grade {:0.2f} deg at time {:0.2f} s",
                    std::atan(current_grade_ / 100.0) * 180.0 / M_PI, time_);
            } else {
                // Quit early if the target slope has not yet been reached.
                VehicleSimulation::PostStepHook();
                return;
            }
        }

        // Add a check for the steady speed (on the steady detector)
        // Add a check on negative speed (on the failure detector)

        /*
        output_->AddResult("pose.position.real",
                           vehicle_->GetPositionX() / std::cos(current_slope_));
        */
    }

    if (steady_state_detector_->CheckSteadyState(time_step_)) {
        SPDLOG_INFO("Detected steady state, final speed {:0.2f} m/s",
                    vehicle_->GetSpeed());
        is_completed_ = true;
        is_successful_ = true;
    }

    if (!has_latched_) {
        downshift_latch_->Enforce(time_step_);
        if (downshift_latch_->GetState() == GearLatchState::LATCHED) {
            SPDLOG_INFO("Latch condition detected!");
            has_latched_ = true;
        }
    }

    VehicleSimulation::PostStepHook();
}

void GradeClimbing::WriteMetadata() {
    output_->AddMetadata("targetGrade", target_grade_);
    output_->AddMetadata("timeToTargetGrade", time_to_target_grade_);
    output_->AddMetadata("targetInitialSpeed", target_initial_speed_);
    output_->AddMetadata("minimumTimeToInitialSpeed", time_to_initial_speed_);
    output_->AddMetadata("success", is_successful_);

    if (terrain_type_ == "rigid") {
        output_->AddMetadata("frictionCoefficient",
                             configuration_->GetValue<double>(
                                 "scenario/terrain/rigid/frictionCoefficient"));
    }
}

}  // namespace Simulation
}  // namespace DYNO
