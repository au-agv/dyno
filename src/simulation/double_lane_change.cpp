#include <dyno/simulation/double_lane_change.hpp>

namespace DYNO {
namespace Simulation {

DoubleLaneChange::DoubleLaneChange(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    GetConfiguration();
    Instantiate();
}

void DoubleLaneChange::GetConfiguration() {
    acceleration_length_ =
        configuration_->GetValue<double>("scenario/accelerationLength", 100.0);
    SPDLOG_INFO("Acceleration length: {:0.2f}", acceleration_length_);

    terrain_type_ =
        configuration_->GetValue<std::string>("scenario/terrain/type", "rigid");
    SPDLOG_INFO("Terrain type set to \"{}\"", terrain_type_);

    use_split_surface_ =
        configuration_->GetValue("scenario/useSplitSurface", false);
}

void DoubleLaneChange::Instantiate() {
    dlc_ = std::make_shared<DYNO::Environments::DoubleLaneChange>(
        -acceleration_length_ + 5.0, acceleration_length_, vehicle_length_,
        vehicle_width_,
        2.72,  // Wheelbase
        path_height_, left_turn_);
    dlc_->Initialize();
}

void DoubleLaneChange::InitializeDriver() {
    auto path_driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(), dlc_->GetPath(), "path",
        configuration_->GetValue<double>("scenario/targetSpeed"));

    InitializeSpeedController(path_driver->GetSpeedController());
    InitializeSteeringController(path_driver->GetSteeringController());

    path_driver->Initialize();
    driver_ = path_driver;

    VehicleSimulation::InitializeDriver();
}

void DoubleLaneChange::InitializeTerrain() {
    target_sideslope_ = configuration_->GetValue("scenario/sideslope", 0.0);

    system_->SetGravitationalAcceleration(
        chrono::QuatFromRodrigues(
            chrono::ChVector3(std::atan(target_sideslope_ / 100.0), 0.0, 0.0))
            .Rotate(chrono::ChVector3d(0.0, 0.0, -9.81)));

    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    terrain_type_ =
        configuration_->GetValue<std::string>("scenario/terrain/type", "rigid");

    const double patch_length =
        configuration_->GetValue("scenario/accelerationLength", 100.0) + 100.0;
    const double patch_width = configuration_->GetValue(
        "scenario/terrain/width", 5.0 * vehicle_width_);
    const double patch_position_x = patch_length / 2.0 - 10.0;
    const double patch_position_y = -vehicle_width_ / 2.0;

    if (terrain_type_ == "rigid") {
        const auto friction_coefficient = configuration_->GetValue(
            "scenario/terrain/rigid/frictionCoefficient", 0.85);
        auto rigid_terrain =
            std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());
        rigid_terrain
            ->AddPatch(GetContactMaterial(friction_coefficient),
                       chrono::ChCoordsys(
                           chrono::ChVector3(patch_position_x, 0.0, 0.0),
                           chrono::QUNIT),
                       patch_length, patch_width,
                       0.25  // Patch thickness
                       )
            ->SetTexture(
                std::string(DYNO_DATA_DIR) + "textures/terrain/checker_orange.png",
                patch_length, patch_width);

        if (use_split_surface_) {
            auto friction_functor =
                std::make_shared<DYNO::Environments::AsymmetricFrictionFunctor>(
                    configuration_->GetValue<double>(
                        "scenario/terrain/frictionCoefficient/left"),
                    configuration_->GetValue<double>(
                        "scenario/terrain/frictionCoefficient/right"),
                    friction_coefficient);
            rigid_terrain->RegisterFrictionFunctor(friction_functor);
            rigid_terrain->UseLocationDependentFriction(true);
        }

        rigid_terrain->Initialize();

        terrain_->InitializeFrom(rigid_terrain);
    } else if (terrain_type_ == "scm") {
        system_->SetNumThreads(std::min(8, chrono::ChOMP::GetNumProcs()),
                               std::min(8, chrono::ChOMP::GetNumProcs()), 1);

        auto scm_terrain =
            std::make_shared<chrono::vehicle::SCMTerrain>(system_.get());
        scm_terrain->SetSoilParameters(
            configuration_->GetValue("scenario/terrain/scm/bekkerKphi", 4.0e6),
            configuration_->GetValue("scenario/terrain/scm/bekkerKc", 0.0),
            configuration_->GetValue("scenario/terrain/scm/bekkerN", 1.1),
            configuration_->GetValue("scenario/terrain/scm/mohrCohesion", 0.0),
            configuration_->GetValue("scenario/terrain/scm/mohrFriction",
                                     3.0e1),
            configuration_->GetValue("scenario/terrain/scm/janosiShear",
                                     1.0e-2),
            4.0e7, 3.0e4);
        scm_terrain->SetReferenceFrame(chrono::ChCoordsys(
            chrono::ChVector3(patch_position_x, patch_position_y, 0.0),
            chrono::QUNIT));

        scm_terrain->Initialize(
            patch_length, patch_width,
            configuration_->GetValue("scenario/terrain/scm/resolution", 0.075));
        scm_terrain->EnableBulldozing(true);
        scm_terrain->SetBulldozingParameters(
            configuration_->GetValue("scenario/terrain/scm/erosionAngle", 25.0),
            1.0, 3, 3);
        scm_terrain->SetPlotType(chrono::vehicle::SCMTerrain::PLOT_SINKAGE, 0.0,
                                 0.2);

        auto soil_parameters_callback = std::make_shared<
            DYNO::Environments::AsymmetricSoilParametersCallback>();
        scm_terrain->RegisterSoilParametersCallback(soil_parameters_callback);

        // TODO: Reintroduce the active domain for the SCM terrain.
        // vehicle_->AddActiveDomain(scm_terrain);
        terrain_->InitializeFrom(scm_terrain);
    }
}

void DoubleLaneChange::OverrideInitialPose() {
    vehicle_->OverrideInitialPose(chrono::ChCoordsysd(
        chrono::ChVector3<double>(-acceleration_length_, 0.0, 0.1),
        chrono::QUNIT));
}

void DoubleLaneChange::PostSynchronizationHook() {
    // TODO: Perform a validation check for the violation of the double lane
    // change bounds.
    /*
    if(dlc_->HasFailed(vehicle_->GetVehicle())) {
        output_->AddResult("success", 0);
        if (configuration_->GetValue("scenario/haltOnFailure", false)) {
            SPDLOG_ERROR("FAIL!!!!");
            is_completed_ = true;
        }
    }
    if(vehicle_->GetPositionX() > (dlc_->GetXmax() + acceleration_length_ +
    100.0)) { is_completed_ = false;
    }
    */
}

}  // namespace Simulation
}  // namespace DYNO
