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

#include <dyno/simulation/vehicle_simulation.hpp>

namespace DYNO {
namespace Simulation {

using DYNO::Interfaces::JSON::GetValue;

VehicleSimulation::VehicleSimulation(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : configuration_(configuration) {
    InitializeLogger();
    InitializeTime();
    InitializeOutput();
}

void VehicleSimulation::Initialize() {
    GetConfiguration();
    Instantiate();
    chrono::SetChronoDataPath(CHRONO_DEFAULT_DATA_DIR);
    PreInitializeSystemHook();
    InitializeSystem();
    PostInitializeSystemHook();

    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);
    InitializeTerrain();
    PostInitializeTerrainHook();
    UpdateChronoVehicleDataPath();
    CreateVehicle();
    OverrideInitialPose();
    InitializeVehicle();
    ResetChronoVehicleDataPath();
    InitializeDriver();
    InitializeVisualization();
    InitializeAssets();
    FinalizeVisualization();
    SetOutput();
    PostInitializationHook();
}

const double& VehicleSimulation::GetTime() {
    return time_;
}

void VehicleSimulation::InitializeAssets() {}

void VehicleSimulation::FinalizeVisualization() {
    visualization_->Initialize();
}

std::shared_ptr<chrono::vehicle::ChDriver> VehicleSimulation::GetDriver() {
    return driver_;
}

std::shared_ptr<chrono::vehicle::ChTerrain> VehicleSimulation::GetTerrain() {
    return terrain_->GetTerrain();
}

void VehicleSimulation::Synchronize() {
    SynchronizeDriver();
    terrain_->Synchronize(time_);
    vehicle_->Synchronize(time_, current_driver_inputs_,
                          *terrain_->GetTerrain());
    visualization_->Synchronize(time_, current_driver_inputs_);
}

void VehicleSimulation::SynchronizeDriver() {
    if (time_ > warmup_time_) {
        current_driver_inputs_ = driver_->GetInputs();

        if (speed_controller_overridden_) {
            driver_->SetThrottle(current_driver_inputs_.m_throttle);
            driver_->SetBraking(current_driver_inputs_.m_braking);
            current_driver_inputs_.m_steering = driver_->GetSteering();
        }

        if (steering_controller_overridden_) {
            driver_->SetSteering(current_driver_inputs_.m_steering);
            current_driver_inputs_.m_throttle = driver_->GetThrottle();
            current_driver_inputs_.m_braking = driver_->GetBraking();
        }
    };

    driver_->Synchronize(time_);
}

void VehicleSimulation::OverrideControlsSpeed(const double& throttle,
                                              const double& braking) {
    driver_->SetBraking(braking);
    driver_->SetThrottle(throttle);

    current_driver_inputs_.m_throttle = throttle;
    current_driver_inputs_.m_braking = braking;

    speed_controller_overridden_ = true;
}

void VehicleSimulation::OverrideControlsSteering(const double& steering) {
    driver_->SetSteering(steering);

    current_driver_inputs_.m_steering = steering;

    steering_controller_overridden_ = true;
}

void VehicleSimulation::Advance() {
    driver_->Advance(time_step_);

    terrain_->Advance(time_step_);
    vehicle_->GetVehicle()->Advance(time_step_);
    system_->DoStepDynamics(time_step_);

    visualization_->Advance(time_step_);
}

void VehicleSimulation::Step() {
    time_ = system_->GetChTime();
    SPDLOG_DEBUG("Simulating time step: >> t = {:0.2f} s <<", time_);
    PreSynchronizationHook();
    Synchronize();
    PostSynchronizationHook();
    Advance();
    PostAdvanceHook();
    PostStepHook();
}

void VehicleSimulation::Loop() {
    while (time_ < end_time_) {
        Step();
    }
    output_->Dump();
}

bool VehicleSimulation::IsFinalTime() {
    return time_ > end_time_;
}

bool VehicleSimulation::IsCompleted() {
    return is_completed_;
}

void VehicleSimulation::SetOutput() {
    output_ = std::make_shared<DYNO::Serialization::HDF5Serializer>(
        vehicle_->GetVehicle(), driver_, terrain_->GetTerrain());
    output_->SetFilename(
        configuration_->GetValue<std::string>("output/filename", "untitled"));
    output_->CreateSubfolder(
        configuration_->GetValue<bool>("output/createSubfolder", false));
    output_->AddTimestamp(
        configuration_->GetValue<bool>("output/addTimestamp", false));
    output_->UseNameGenerator(
        configuration_->GetValue<bool>("output/useNameGenerator", false));
    output_->Initialize();
}

void VehicleSimulation::WriteMetadata() {}

void VehicleSimulation::PostStepHook() {
    if (time_ - last_output_time_ > 1.0 / output_frequency_ &&
        is_output_triggered_) {
        output_->Save(system_->GetChTime());
        last_output_time_ = time_;
    }

    if (vehicle_->GetPositionX() > output_trigger_position_ &&
        !is_output_triggered_) {
        SPDLOG_INFO("Logger output triggered - starting recording ...");

        is_output_triggered_ = true;
    }

    if (IsFinalTime() || IsCompleted()) {
        stop_loop_ = true;
    }
}

bool VehicleSimulation::ShouldStopNow() {
    return stop_loop_;
}

std::shared_ptr<DYNO::Models::Vehicle> VehicleSimulation::GetVehicle() {
    return vehicle_;
}

void VehicleSimulation::UpdateChronoVehicleDataPath() {
    const auto path_type =
        configuration_->GetValue<std::string>("vehicle/path/type", "chrono");
    if (path_type == "chrono") {
        chrono::vehicle::SetVehicleDataPath(CHRONO_DEFAULT_VEHICLE_DATA_DIR);
    } else if (path_type == "dyno") {
        chrono::vehicle::SetVehicleDataPath(DYNO_VEHICLE_DATA_DIR);
    } else if (path_type == "custom") {
        chrono::vehicle::SetVehicleDataPath(
            configuration_->GetValue<std::string>("vehicle/path/root"));
    }
}

void VehicleSimulation::ResetChronoVehicleDataPath() {
    chrono::SetChronoDataPath(CHRONO_DEFAULT_DATA_DIR);
    chrono::vehicle::SetVehicleDataPath(CHRONO_DEFAULT_VEHICLE_DATA_DIR);
}

void VehicleSimulation::InitializeSystem() {
    SPDLOG_INFO("Initializing the multibody system ...");

    const auto contacts_method = configuration_->GetValue<std::string>(
        "simulation/contacts/method", "nsc");
    if (contacts_method == "nsc") {
        system_ = std::make_shared<chrono::ChSystemNSC>();
    } else if (contacts_method == "smc") {
        system_ = std::make_shared<chrono::ChSystemSMC>();
    } else {
        throw std::invalid_argument("Unknown contacts method.");
    }

    system_->SetTimestepperType(
        chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    system_->SetSolverType(chrono::ChSolver::Type::BARZILAIBORWEIN);

    system_->GetSolver()->AsIterative()->SetMaxIterations(
        configuration_->GetValue<int>("solver/maximumIterations", 50));

    system_->SetGravitationalAcceleration(
        -9.81 * chrono::vehicle::ChWorldFrame::Vertical());

    system_->SetCollisionSystem(
        std::make_shared<chrono::ChCollisionSystemBullet>());
}

void VehicleSimulation::CreateVehicle() {
    SPDLOG_INFO("Creating the vehicle system ...");

    if (configuration_->GetValue<std::string>("vehicle/name") == "olav") {
        vehicle_ = std::make_shared<DYNO::Models::Olav>(system_);
    } else if (configuration_->GetValue<std::string>("vehicle/name") ==
               "genericTracked") {
        vehicle_ = std::make_shared<DYNO::Models::TrackedVehicle>(system_);
    } else if (configuration_->GetValue<std::string>("vehicle/name") ==
               "genericWheeled") {
        vehicle_ = std::make_shared<DYNO::Models::WheeledVehicle>(system_);

    } else {
        throw std::invalid_argument("Invalid vehicle name!");
    }
}

void VehicleSimulation::OverrideInitialPose() {}

void VehicleSimulation::InitializeVehicle() {
    SPDLOG_INFO("Initializing the vehicle system ...");

    vehicle_->Setup(configuration_);
    vehicle_->Initialize();
    vehicle_->GetVehicle()->EnableRealtime(
        configuration_->GetValue<bool>("simulation/realtime", false));

    if (configuration_->GetValue<bool>("vehicle/drag/enabled", true)) {
        double drag_coefficient;
        double reference_area;

        try {
            drag_coefficient =
                vehicle_->GetAerodynamicProperties().GetDragCoefficient();
            reference_area =
                vehicle_->GetAerodynamicProperties().GetReferenceArea();
        } catch (DYNO::Exceptions::NoAerodynamicParameters& exception) {
            SPDLOG_WARN(
                "No aerodynamic parameters configured, resorting to "
                "JSON file ...");

            drag_coefficient = configuration_->GetValue<double>(
                "vehicle/drag/dragCoefficient");
            reference_area =
                configuration_->GetValue<double>("vehicle/drag/referenceArea");
        }

        vehicle_->GetVehicle()->GetChassis()->SetAerodynamicDrag(
            drag_coefficient, reference_area,
            1.225  // Air density
        );
    }

    vehicle_->GetVehicle()->SetChassisCollide(false);
    if (vehicle_->GetVehicle()->HasBushings()) {
        throw std::invalid_argument(
            "Simulation with bushings is not available.");
    }

    SPDLOG_INFO("Initializing the vehicle at pose: [{:0.2f}, {:0.2f}, {:0.2f}]",
                vehicle_->GetPosition().x(), vehicle_->GetPosition().y(),
                vehicle_->GetPosition().z());
}

void VehicleSimulation::InitializeDriver() {
    SPDLOG_INFO("Initializing the driver model ...");

    current_driver_inputs_.m_braking = 1.0;
    current_driver_inputs_.m_throttle = 0.0;
    current_driver_inputs_.m_steering = 0.0;
    current_driver_inputs_.m_clutch = 0.0;
}

void VehicleSimulation::InitializeSensors() {
    throw std::invalid_argument("Sensors module not implemented.");
}

void VehicleSimulation::InitializeStraightLineDriver(
    const double& target_speed) {
    auto driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(),
        chrono::vehicle::StraightLinePath(
            chrono::ChVector3<double>(0.0, 0.0, 0.0),
            chrono::ChVector3<double>(1.0e5, 0.0, 0.0)),
        "straight_line_path", target_speed);

    InitializeSpeedController(driver->GetSpeedController());
    InitializeSteeringController(driver->GetSteeringController());

    driver->Initialize();
    driver_ = driver;
}

void VehicleSimulation::InitializeTerrain() {
    throw DYNO::Exceptions::NotImplemented();
}

void VehicleSimulation::InitializeRigidTerrain(
    const chrono::ChVector3d& position, const double& length,
    const double& width, const std::string& texture) {
    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());
    rigid_terrain
        ->AddPatch(
            GetContactMaterial(
                configuration_->GetValue(
                    "scenario/terrain/rigid/frictionCoefficient", 0.85),
                configuration_->GetValue(
                    "scenario/terrain/rigid/restitutionCoefficient", 0.001),
                configuration_->GetValue(
                    "scenario/terrain/rigid/elasticModulus", 4.87e6)),
            chrono::ChCoordsys(
                chrono::ChVector3(position.x(), position.y(), position.z()),
                chrono::QUNIT),
            length, width,
            0.25  // Patch thickness
            )
        ->SetTexture(
            std::string(DYNO_DATA_DIR) + "textures/terrain/" + texture + ".png",
            length, width);

    rigid_terrain->Initialize();

    terrain_->InitializeFrom(rigid_terrain);
}

void VehicleSimulation::InitializeSCMTerrain(const chrono::ChVector3d& position,
                                             const double& length,
                                             const double& width) {
    system_->SetNumThreads(std::min(8, chrono::ChOMP::GetNumProcs()),
                           std::min(8, chrono::ChOMP::GetNumProcs()), 1);

    auto scm_terrain =
        std::make_shared<chrono::vehicle::SCMTerrain>(system_.get());
    scm_terrain->SetSoilParameters(
        configuration_->GetValue("scenario/terrain/scm/bekkerKphi", 4.0e6),
        configuration_->GetValue("scenario/terrain/scm/bekkerKc", 0.0),
        configuration_->GetValue("scenario/terrain/scm/bekkerN", 1.1),
        configuration_->GetValue("scenario/terrain/scm/mohrCohesion", 0.0),
        configuration_->GetValue("scenario/terrain/scm/mohrFriction", 3.0e1),
        configuration_->GetValue("scenario/terrain/scm/janosiShear", 1.0e-2),
        2.0e8, 3.0e4);
    scm_terrain->SetReferenceFrame(chrono::ChCoordsys(position, chrono::QUNIT));
    scm_terrain->Initialize(
        length, width,
        configuration_->GetValue("scenario/terrain/scm/resolution", 0.075));
    scm_terrain->EnableBulldozing(true);
    scm_terrain->SetBulldozingParameters(
        configuration_->GetValue("scenario/terrain/scm/erosionAngle", 25.0),
        1.0, 3, 3);
    scm_terrain->SetPlotType(chrono::vehicle::SCMTerrain::PLOT_SINKAGE, 0.0,
                             0.05);
    scm_terrain->SetMeshWireframe(
        configuration_->GetValue("scenario/terrain/scm/wireframe", true));
    // vehicle_->AddActiveDomain(scm_terrain);
    terrain_->InitializeFrom(scm_terrain);
}

void VehicleSimulation::InitializeVisualization() {
#ifdef HAS_VSG_SUPPORT
    const auto engine = configuration_->GetValue<std::string>(
        "visualization/engine", "irrlicht");
#else
    const auto engine =
        configuration_->GetValue<std::string>("visualization/engine", "vsg");
#endif
    if (engine == "irrlicht") {
        if (configuration_->GetValue<std::string>("vehicle/name") ==
            "genericTracked") {
            visualization_ =
                std::make_shared<DYNO::Visualization::IrrlichtTracked>(
                    vehicle_->GetVehicle(), configuration_);
        } else if (configuration_->GetValue<std::string>("vehicle/name") ==
                   "olav") {
            visualization_ =
                std::make_shared<DYNO::Visualization::IrrlichtWheeled>(
                    vehicle_->GetVehicle(), configuration_);
        }
    }
#ifdef DYNO_HAS_VSG_SUPPORT
    else if (engine == "vsg") {
        visualization_ =
            std::make_shared<DYNO::Visualization::VulkanSceneGraph>(
                vehicle_->GetVehicle(), configuration_);

    }
#endif
    else {
        throw std::invalid_argument("Invalid visualization engine.");
    }
}

void VehicleSimulation::Instantiate() {}

void VehicleSimulation::GetConfiguration() {}

void VehicleSimulation::PreInitializeSystemHook() {}

void VehicleSimulation::PostInitializeSystemHook() {}

void VehicleSimulation::PostInitializeTerrainHook() {}

void VehicleSimulation::PostInitializationHook() {}

void VehicleSimulation::PreSynchronizationHook() {}

void VehicleSimulation::PostSynchronizationHook() {}

void VehicleSimulation::PostAdvanceHook() {}

void VehicleSimulation::InitializeLogger() {
    log_level_ = configuration_->GetValue<std::string>("log/level", "info");
    spdlog::set_level(spdlog::level::from_str(log_level_));
    SPDLOG_INFO("Logger level set to \"{}\"", log_level_);
}

void VehicleSimulation::InitializeTime() {
    time_step_ =
        configuration_->GetValue<double>("simulation/timeStep", 1.0e-3);
    warmup_time_ =
        configuration_->GetValue<double>("simulation/warmupTime", 5.0);
    end_time_ = configuration_->GetValue<double>("simulation/endTime", 1.0e5);
}

void VehicleSimulation::InitializeOutput() {
    output_frequency_ =
        configuration_->GetValue<double>("output/frequency", 100.0);
    output_trigger_position_ = configuration_->GetValue<double>(
        "output/triggerPosition", -std::numeric_limits<double>::infinity());
}

void VehicleSimulation::InitializeSpeedController(
    chrono::vehicle::ChSpeedController& controller) {
    // TODO: Consider a different implementation that preserves
    // encapsulation with respect to the ChDriver class.
    double proportional_gain;
    double integral_gain;
    double derivative_gain;

    try {
        proportional_gain =
            vehicle_->GetSpeedControllerTuning().GetProportionalGain();
        integral_gain = vehicle_->GetSpeedControllerTuning().GetIntegralGain();
        derivative_gain =
            vehicle_->GetSpeedControllerTuning().GetDerivativeGain();
    } catch (DYNO::Exceptions::NoControllerPreset& exception) {
        SPDLOG_WARN("No controller preset found, reverting to JSON file ...");

        proportional_gain = configuration_->GetValue<double>(
            "driver/speedController/proportionalGain");
        integral_gain = configuration_->GetValue<double>(
            "driver/speedController/integralGain");
        derivative_gain = configuration_->GetValue<double>(
            "driver/speedController/derivativeGain");
    }

    controller.SetGains(proportional_gain, integral_gain, derivative_gain);
}

void VehicleSimulation::InitializeSteeringController(
    chrono::vehicle::ChPathSteeringController& controller) {
    double proportional_gain;
    double integral_gain;
    double lookahead_distance;

    try {
        proportional_gain =
            vehicle_->GetSteeringControllerTuning().GetProportionalGain();
        integral_gain =
            vehicle_->GetSteeringControllerTuning().GetIntegralGain();
        lookahead_distance =
            vehicle_->GetSteeringControllerTuning().GetLookaheadDistance();
    } catch (DYNO::Exceptions::NoControllerPreset& exception) {
        SPDLOG_WARN("No controller preset found, reverting to JSON file ...");

        proportional_gain = configuration_->GetValue<double>(
            "driver/steeringController/proportionalGain");
        integral_gain = configuration_->GetValue<double>(
            "driver/steeringController/integralGain");
        lookahead_distance = configuration_->GetValue<double>(
            "driver/steeringController/lookaheadDistance");
    }

    controller.SetGains(proportional_gain, integral_gain, 0.0);

    controller.SetLookAheadDistance(lookahead_distance);
}

std::shared_ptr<chrono::ChContactMaterial>
VehicleSimulation::GetContactMaterial(const double& friction_coefficient,
                                      const double& restitution_coefficient,
                                      const double& elastic_modulus) const {
    chrono::ChContactMaterialData material_data;
    material_data.mu = configuration_->GetValue(
        "scenario/terrain/rigid/frictionCoefficient", 0.85);
    material_data.cr = restitution_coefficient;
    material_data.Y = elastic_modulus;
    return material_data.CreateMaterial(system_->GetContactMethod());
}

bool VehicleSimulation::ValidateVehicleRoll(const double& threshold) {
    if (vehicle_->GetVehicle()->GetRoll() > (180.0 / M_PI * threshold)) {
        SPDLOG_ERROR("Roll threshold exceeded!");
        return false;
    }

    return true;
}

bool VehicleSimulation::ValidateVehicleYaw(const double& threshold) {
    if (vehicle_->GetVehicle()->GetRot().GetCardanAnglesXYZ().z() >
        (180.0 / M_PI * threshold)) {
        SPDLOG_ERROR("Yaw threshold exceeded!");
        return false;
    }

    return true;
}

void VehicleSimulation::Dump() {
    WriteMetadata();
    output_->Dump();
}

const double& VehicleSimulation::GetEndTime() {
    return end_time_;
}

const double& VehicleSimulation::GetTimeStep() {
    return time_step_;
}

void VehicleSimulation::AddGate(double gate_position_x, unsigned int position) {
    gates_[position] = gate_position_x;
}

void VehicleSimulation::AddTrigger(const chrono::ChVector3d& trigger_position) {
    triggers_.push_back(trigger_position);
}

/// Teleport vehicle and set initial motion, without touching powertrain
void VehicleSimulation::TeleportVehicleMotion(
    chrono::vehicle::ChWheeledVehicle* vehicle, const chrono::ChVector3d& pos,
    const chrono::ChQuaterniond& rot, const chrono::ChVector3d& linear_vel,
    const chrono::ChVector3d& angular_vel) {
    // 1️⃣ Move chassis
    auto chassis = vehicle->GetChassisBody();
    chassis->SetPos(pos);
    chassis->SetRot(rot);
    chassis->SetPosDt(linear_vel);
    chassis->SetAngVelParent(angular_vel);

    // 2️⃣ Update wheels to match chassis linear velocity
    int n_wheels = 2;  // 2 wheels per axle
    for (int i = 0; i < n_wheels; ++i) {
        for (int j = 0; j < 2; ++j) {
            auto wheel = vehicle->GetWheel(
                j, static_cast<chrono::vehicle::VehicleSide>(i),
                chrono::vehicle::WheelLocation::SINGLE);

            // Compute wheel position relative to chassis
            chrono::ChVector3d rel_pos = wheel->GetPos() - chassis->GetPos();

            // Move wheel to match new chassis position

            wheel->GetSpindle()->SetPos(pos + rel_pos);
            wheel->GetSpindle()->SetRot(
                wheel->GetSpindle()->GetRot());  // keep current rotation

            // Match wheel linear velocity with chassis
            wheel->GetSpindle()->SetPosDt(linear_vel);
            wheel->GetSpindle()->SetAngVelParent(angular_vel);
        }
    }
}

bool VehicleSimulation::IsSuccessful() const {
    return is_successful_;
}

bool VehicleSimulation::IsCompleted() const {
    return is_completed_;
}


}  // namespace Simulation
}  // namespace DYNO
