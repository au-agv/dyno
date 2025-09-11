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

#include <dyno/environments/terrain.hpp>

namespace DYNO {
namespace Environments {

Terrain::Terrain(
    std::shared_ptr<chrono::ChSystem> system,
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration)
    : configuration_(configuration) {
    system_ = system;
}

void Terrain::Synchronize(double time) {
    terrain_->Synchronize(time);
}

void Terrain::Advance(double time_step) {
    terrain_->Advance(time_step);
}

void Terrain::SelectTerrain() {
    auto terrain_type =
        configuration_->GetValue<std::string>("terrain/type", "rigid");
    if (terrain_type == "rigid") {
        terrain_type_ = TerrainType::RIGID;
    } else if (terrain_type == "scm") {
        terrain_type_ = TerrainType::SCM;
    } else {
        throw std::invalid_argument("Unknown terrain type");
    }
}

void Terrain::InitializeFrom(
    std::shared_ptr<chrono::vehicle::RigidTerrain>& terrain) {
    terrain_ = terrain;
}

void Terrain::InitializeFrom(
    std::shared_ptr<chrono::vehicle::SCMTerrain>& terrain) {
    terrain_ = terrain;
}

void Terrain::InitializeRigidTerrainSinglePatch(
    double friction_coefficient, chrono::ChVector2d patch_position,
    chrono::ChVector2d patch_size, std::filesystem::path texture_path) {
    SPDLOG_INFO("Initializing single patch rigid terrain ...");

    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());
    chrono::ChContactMaterialData minfo;
    minfo.mu = friction_coefficient;
    minfo.cr = 0.75f;
    minfo.Y = 2e7f;
    auto material = minfo.CreateMaterial(system_->GetContactMethod());

    rigid_terrain
        ->AddPatch(
            material,
            chrono::ChCoordsys(chrono::ChVector3(patch_position.x(), 0.0, 0.0),
                               chrono::QUNIT),
            patch_size.x(), patch_size.y(),
            0.25  // Patch thickness
            )
        ->SetTexture(
            std::string(DYNO_DATA_DIR) + "textures/terrain/checker_white.png",
            patch_size.x(), patch_size.y());

    rigid_terrain->Initialize();

    terrain_ = rigid_terrain;
}

void Terrain::InitializeSCM() {
    // Threads
    auto threads = configuration_->GetValue<int>("terrain/scm/threads", 1);
    system_->SetNumThreads(threads / 2, threads / 2, 1);

    auto scm_terrain =
        std::make_shared<chrono::vehicle::SCMTerrain>(system_.get());

    // Set SCM terrain position.
    auto position_x =
        configuration_->GetValue<double>("terrain/scm/position/x", 0.0);
    auto position_y =
        configuration_->GetValue<double>("terrain/scm/position/y", 0.0);
    scm_terrain->SetReferenceFrame(chrono::ChCoordsysd(
        chrono::ChVector3<double>(position_x, position_y, 0.0)));

    // Set SCM terrain geometry.
    auto length =
        configuration_->GetValue<double>("terrain/scm/geometry/length", 1.0);
    auto width =
        configuration_->GetValue<double>("terrain/scm/geometry/width", 1.0);

    // Set SCM terrain resolution.
    auto resolution =
        configuration_->GetValue<double>("terrain/scm/resolution", 0.1);

    // Initialize the SCM terrain.
    scm_terrain->Initialize(length, width, resolution);

    // Soil parameters
    // ---------------

    auto bekker_wong_k_phi =
        configuration_->GetValue<double>("terrain/scm/bekkerWongKphi", 0.8e6);
    auto bekker_wong_k_c =
        configuration_->GetValue<double>("terrain/scm/bekkerWongKc", 0.0);
    auto bekker_wong_n =
        configuration_->GetValue<double>("terrain/scm/bekkerWongN", 1.1);
    auto mohr_cohesive_limit =
        configuration_->GetValue<double>("terrain/scm/mohrCohesiveLimit", 0.0);
    auto mohr_friction_limit =
        configuration_->GetValue<double>("terrain/scm/mohrFrictionLimit", 0.0);
    auto janosi_shear_coefficient = configuration_->GetValue<double>(
        "terrain/scm/janosiShearCoefficient", 0.01);
    auto elastic_stiffness =
        configuration_->GetValue<double>("terrain/scm/elasticStiffness", 4.0e7);
    auto damping =
        configuration_->GetValue<double>("terrain/scm/damping", 3.0e4);

    // Set the soil parameters.
    scm_terrain->SetSoilParameters(
        bekker_wong_k_phi, bekker_wong_k_c, bekker_wong_n, mohr_cohesive_limit,
        mohr_friction_limit, janosi_shear_coefficient, elastic_stiffness,
        damping);

    // ---------------

    SPDLOG_DEBUG("Added 0.5 x 0.5 meter moving patch to wheels.");

    // Enable bulldozing.
    scm_terrain->EnableBulldozing(true);
    scm_terrain->SetBulldozingParameters(45.0);

    // Enable wireframe visualization.
    scm_terrain->SetMeshWireframe(
        configuration_->GetValue("terrain/scm/visualization/wireframe", true));

    scm_terrain->SetPlotType(
        configuration_->GetValue<chrono::vehicle::SCMTerrain::DataPlotType>(
            "terrain/scm/visualization/plot/type",
            chrono::vehicle::SCMTerrain::DataPlotType::PLOT_SINKAGE),
        configuration_->GetValue<double>(
            "terrain/scm/visualization/plot/minimum", 0.0),
        configuration_->GetValue<double>(
            "terrain/scm/visualization/plot/minimum", 1.0));

    if (configuration_->GetValue<bool>("terrain/scm/input/enabled", false)) {
        scm_terrain_reader_ =
            std::make_shared<DYNO::Interfaces::JSONTerrainOutput>(
                configuration_->GetValue<std::string>("terrain/scm/input/path"),
                scm_terrain);
        scm_terrain_reader_->LoadFromDisk();
    }

    if (configuration_->GetValue<bool>("terrain/scm/output/enabled", false)) {
        scm_terrain_writer_ =
            std::make_shared<DYNO::Interfaces::JSONTerrainOutput>(
                configuration_->GetValue<std::string>(
                    "terrain/scm/output/path"),
                scm_terrain);
    }
    terrain_ = scm_terrain;
}

void Terrain::WriteToDisk() {
    if (configuration_->GetValue<bool>("terrain/scm/serialization/output")) {
        SPDLOG_DEBUG("Writing SCM nodes to disk...");
        scm_terrain_writer_->WriteToDisk();
    }
}

std::shared_ptr<chrono::vehicle::ChTerrain> Terrain::GetTerrain() {
    return terrain_;
}

}  // namespace Environments
}  // namespace DYNO
