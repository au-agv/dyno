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

#include <dyno/simulation/terrain_mobility.hpp>

namespace DYNO {
namespace Simulation {

TerrainMobility::TerrainMobility(
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : VehicleSimulation(configuration) {
    SPDLOG_INFO("Instantiating the straight line acceleration simulation ...");

    LoadWaypoints();
}

void TerrainMobility::LoadWaypoints() {
    SPDLOG_INFO("Reading waypoints from file ...");

    std::ifstream waypoints_file(
        configuration_->GetValue<std::string>("scenario/waypoints"));
    nlohmann::json waypoints_data = nlohmann::json::parse(waypoints_file);

    for (const auto& waypoint_entry : waypoints_data["waypoints"]) {
        points_.emplace_back(
            chrono::ChVector3(waypoint_entry["map"]["x"].get<double>(),
                              waypoint_entry["map"]["y"].get<double>(),
                              waypoint_entry["elevation"].get<double>()));

        headings_.emplace_back(
            waypoint_entry["path"]["move_direction"].get<double>());

        speeds_.emplace_back(waypoint_entry["path"]["speed"].get<double>());
    }
}

void TerrainMobility::InitializeDriver() {
    SPDLOG_INFO("Initializing the straight line acceleration driver ...");

    auto path_driver = std::make_shared<chrono::vehicle::ChPathFollowerDriver>(
        *vehicle_->GetVehicle(),
        std::make_shared<chrono::ChBezierCurve>(points_), "path", 3.0);

    InitializeSpeedController(path_driver->GetSpeedController());
    InitializeSteeringController(path_driver->GetSteeringController());

    path_driver->Initialize();
    driver_ = path_driver;

    VehicleSimulation::InitializeDriver();
}

void TerrainMobility::InitializeTerrain() {
    SPDLOG_INFO(
        "Initializing the default terrain for the mobility mapping "
        "scenario ...");

    terrain_ =
        std::make_shared<DYNO::Environments::Terrain>(system_, configuration_);

    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());

    SPDLOG_INFO("Reading terrain information from file ...");
    std::ifstream terrain_file(
        configuration_->GetValue<std::string>("scenario/terrain/meta"));
    nlohmann::json terrain_info = nlohmann::json::parse(terrain_file);
    const auto patch_length = terrain_info["map_size"]["width"].get<double>();
    const auto patch_width = terrain_info["map_size"]["height"].get<double>();

    rigid_terrain
        ->AddPatch(
            GetContactMaterial(
                configuration_->GetValue("scenario/frictionCoefficient", 0.85),
                1.0e-5, 7.0e4),
            chrono::ChCoordsysd(chrono::ChVector3<double>(
                                    patch_length / 4.0, patch_width / 4.0, 0.0),
                                chrono::QUNIT),
            configuration_->GetValue<std::string>("scenario/terrain/path"),
            patch_length, patch_width,
            terrain_info["elevation"]["min"].get<double>(),
            terrain_info["elevation"]["max"].get<double>(), true)
        ->SetTexture(
            std::string(DYNO_DATA_DIR) + "textures/terrain/checker_orange.png",
            1.0, -1.0);
    rigid_terrain->Initialize();

    // system_->SetNumThreads(std::min(8, chrono::ChOMP::GetNumProcs()),
    //                        std::min(8, chrono::ChOMP::GetNumProcs()), 1);

    /*
// Soil parameters
// ---------------

scm_terrain->SetSoilParameters(
configuration_->GetValue("scenario/terrain/scm/bekkerKphi", 4.0e6),
configuration_->GetValue("scenario/terrain/scm/bekkerKc", 0.0),
configuration_->GetValue("scenario/terrain/scm/bekkerN", 1.1),
configuration_->GetValue("scenario/terrain/scm/mohrCohesion", 0.0),
configuration_->GetValue("scenario/terrain/scm/mohrFriction", 3.0e1),
configuration_->GetValue("scenario/terrain/scm/janosiShear", 1.0e-2),
2.0e8, 3.0e4);
scm_terrain->SetReferenceFrame(chrono::ChCoordsys(chrono::ChVector3d(0.0,
0.0, -10.0), chrono::QUNIT));


scm_terrain->Initialize(
configuration_->GetValue<std::string>("scenario/terrain/path"),
patch_length, patch_width,
terrain_info["elevation"]["min"].get<double>() - 15.0,
terrain_info["elevation"]["max"].get<double>() - 15.0, 2.0);

scm_terrain->Initialize(
patch_length, patch_width, 2.0);
// ---------------
scm_terrain->SetMeshWireframe(
configuration_->GetValue("terrain/scm/visualization/wireframe", true));
SPDLOG_DEBUG("Added 0.5 x 0.5 meter moving patch to wheels.");

// Enable bulldozing.
scm_terrain->EnableBulldozing(true);
scm_terrain->SetBulldozingParameters(45.0);

scm_terrain->SetPlotType(chrono::vehicle::SCMTerrain::PLOT_SINKAGE, 0.0,
                     0.05);

*/

    /*
    system_->SetNumThreads(std::min(8, chrono::ChOMP::GetNumProcs()),
                           std::min(8, chrono::ChOMP::GetNumProcs()), 1);

    scm_terrain_ =
    std::make_shared<chrono::vehicle::SCMTerrain>(system_.get());
    scm_terrain_->SetSoilParameters(
        configuration_->GetValue("scenario/terrain/scm/bekkerKphi", 4.0e6),
        configuration_->GetValue("scenario/terrain/scm/bekkerKc", 0.0),
        configuration_->GetValue("scenario/terrain/scm/bekkerN", 1.1),
        configuration_->GetValue("scenario/terrain/scm/mohrCohesion", 0.0),
        configuration_->GetValue("scenario/terrain/scm/mohrFriction", 3.0e1),
        configuration_->GetValue("scenario/terrain/scm/janosiShear", 1.0e-2),
        2.0e8, 3.0e4);
    scm_terrain_->SetReferenceFrame(
        chrono::ChCoordsys(chrono::ChVector3d(patch_width/2, patch_width/2,
    0.0), chrono::QUNIT)); scm_terrain_->Initialize(
        configuration_->GetValue<std::string>("scenario/terrain/path"),
        patch_width, patch_length,
        terrain_info["elevation"]["min"].get<double>(),
        terrain_info["elevation"]["max"].get<double>(),
        configuration_->GetValue("scenario/terrain/scm/resolution", 0.125));
    scm_terrain_->EnableBulldozing(true);
        scm_terrain_->SetBulldozingParameters(
            configuration_->GetValue("scenario/terrain/scm/erosionAngle", 35.0),
            1.0, 3, 1);
        scm_terrain_->SetPlotType(
            chrono::vehicle::SCMTerrain::DataPlotType::PLOT_SINKAGE, 0.001,
    0.05); scm_terrain_->SetColormap(chrono::ChColormap::Type::FAST);
    scm_terrain_->SetMeshWireframe(
        configuration_->GetValue("scenario/terrain/scm/wireframe", true));

*/

    terrain_->InitializeFrom(rigid_terrain);
}

void TerrainMobility::PrintTerrainHeights(
    chrono::vehicle::RigidTerrain& terrain, double spacing) {
    // Loop over all patches
    for (size_t i = 0; i < terrain.GetPatches().size(); ++i) {
        auto patch = terrain.GetPatches()[i];
        double length_x = 100.0;
        double length_y = 100.0;

        std::cout << "Patch " << i << " heights:\n";

        // Sample points on a grid in local coordinates
        for (double x = -10.0 * length_x; x <= 10.0 * length_x; x += spacing) {
            for (double y = -10.0 * length_y; y <= 10.0 * length_y;
                 y += spacing) {
                chrono::ChVector3d p_local(x, y, 100.0);
                double h = terrain.GetHeight(p_local);
                if (h > 0.0) {

                    std::cout << h << "H";
                }
            }
        }
        std::cout << std::endl;
    }
}

void TerrainMobility::PostInitializeTerrainHook() {
    SPDLOG_INFO("Detecting terrain height at the initial vehicle pose ...");

    auto rigid_terrain =
        std::dynamic_pointer_cast<chrono::vehicle::RigidTerrain>(
            terrain_->GetTerrain());

    // TODO: Perhaps there is a more elegant and reliable way to ensure the
    // raycast starting point is above the terrain mesh? Could this be an
    // std::numeric_limits<double>::infinity() instead?
    auto frame =
        rigid_terrain->GetPatches()[0]->GetGroundBody()->GetFrameRefToAbs();

    auto above_point = chrono::ChVector3d(5.0, 5.0, 10000000000.0);
    auto p_local = frame.TransformPointParentToLocal(above_point);

    auto normal = terrain_->GetTerrain()->GetNormal(above_point);
    SPDLOG_INFO("Orientation\nX:{:0.2f}\tY:{:0.2f}\tZ:{:0.2f}", normal.x(),
                normal.y(), normal.z());

    initial_height_ = terrain_->GetTerrain()->GetHeight(above_point);
    // initial_height_ = scm_terrain_->GetHeight(above_point);
    SPDLOG_INFO("Terrain initial height: {:0.2f}", initial_height_);
}

void TerrainMobility::OverrideInitialPose() {
    SPDLOG_ERROR("Setting initial pose ...");
    /*
    vehicle_->OverrideInitialPose(chrono::ChCoordsysd(
        chrono::ChVector3d(points_[0].x(), points_[0].y(), initial_height_),
        chrono::QUNIT));
        */

    vehicle_->OverrideInitialPose(chrono::ChCoordsysd(
        chrono::ChVector3d(5.0, 5.0, initial_height_), chrono::QUNIT));
}

void TerrainMobility::PostInitializationHook() {
    // TODO: Check whether a small system dynamics simulation step is required
    // before performing terrain raycasts.

    terrain_->Synchronize(0.0);
    system_->DoStepDynamics(0.001);
    terrain_->Advance(0.001);
    terrain_->Synchronize(0.001);

    auto rigid_terrain =
        std::dynamic_pointer_cast<chrono::vehicle::RigidTerrain>(
            terrain_->GetTerrain());
    // PrintTerrainHeights(*rigid_terrain);
    //  TODO: Must be conditional on SCM terrain being initialized.
    //  vehicle_->AddActiveDomain(scm_terrain_);

    auto wheeled_vehicle =
        std::dynamic_pointer_cast<chrono::vehicle::ChWheeledVehicle>(
            vehicle_->GetVehicle());

    vehicle_.reset();
    InitializeVehicle();
}

void TerrainMobility::PreSynchronizationHook() {}

void TerrainMobility::PostSynchronizationHook() {
    auto position = vehicle_->GetPosition();
    const auto distance = (points_.back() - position).Length();

    SPDLOG_DEBUG(
        "Current position {:0.2f} {:0.2f},  {:0.2f} {:0.2f}, {:0.2f} m",
        position.x(), position.y(), (points_.back()).x(), (points_.back()).y(),
        distance);

    if (distance < endpoint_tolerance_) {
        is_completed_ = true;

        output_->AddResult("transitTime", time_ - warmup_time_);
    }
}

void TerrainMobility::SynchronizeDriver() {
    if ((vehicle_->GetPosition() - points_[waypoint_index_]).Length() < 2.0) {
        SPDLOG_INFO("Reached waypoint {}, moving to waypoint {}",
                    waypoint_index_, waypoint_index_ + 1);

        waypoint_index_++;

        SPDLOG_INFO("Setting speed to {:0.2f} m/s", speeds_[waypoint_index_]);

        static_cast<chrono::vehicle::ChPathFollowerDriver*>(driver_.get())
            ->SetDesiredSpeed(
                std::max(std::min(3.0, vehicle_->GetSpeed()),
                         std::min(20.0, speeds_[waypoint_index_])));
    }

    if (vehicle_->GetVehicle()->GetRoll() > 30.0 * chrono::CH_PI / 180.0) {
        is_completed_ = true;
        SPDLOG_ERROR("FAILURE!");
    }

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

    driver_->Synchronize(time_step_);
}

}  // namespace Simulation
}  // namespace DYNO
