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

void Terrain::Synchronize(double time) { terrain_->Synchronize(time); }

void Terrain::Advance(double time_step) { terrain_->Advance(time_step); }

void Terrain::SelectTerrain() {
    auto terrain_type =
        configuration_->GetValue<std::string>("terrain/type", "rigid");
    if(terrain_type == "rigid") {
        terrain_type_ = TerrainType::RIGID;
    } else if(terrain_type == "scm") {
        terrain_type_ = TerrainType::SCM;
    }
    else {
        throw std::invalid_argument("Unknown terrain type");
    }
}

void Terrain::Initialize() {
    SelectTerrain();
    switch(terrain_type_) {
    case TerrainType::RIGID: InitializeRigid(); break;
    case TerrainType::SCM: InitializeSCM(); break;
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

void Terrain::InitializeRigid() {
    auto rigid_terrain =
        std::make_shared<chrono::vehicle::RigidTerrain>(system_.get());

    // Add patch to the rigid terrain.
    for(const auto& patch_definition :
        configuration_->GetValue<nlohmann::json::array_t>(
            "terrain/rigid/patches")) {
        // Define the rigid terrain patch contact material properties.
        auto material = std::make_shared<chrono::ChContactMaterialNSC>();

        // Initialize the default geometric, material and visualization
        // properties for the patch.
        auto position_x = 0.0;
        auto position_y = 0.0;
        auto patch_length = 100.0;
        auto patch_width = 100.0;
        double friction_coefficient = 0.85;

        if(patch_definition["material"].contains("frictionCoefficient")) {
            friction_coefficient =
                patch_definition["material"]["frictionCoefficient"]
                    .get<float>();
        } else {
            SPDLOG_DEBUG(
                "No friction coefficient specified, defaulting to 1.0");
        }
        material->SetFriction(friction_coefficient);

        SPDLOG_DEBUG("\nDefining new terrain material with the following "
                     "properties:\n\t> "
                     "Static friction: {:0.3f}" friction_coefficient, );

        // Check if the user specified a location for the patch.
        if(patch_definition.contains("position")) {
            if(patch_definition["position"].contains("x")) {
                position_x = patch_definition["position"]["x"].get<double>();
            } else {
                SPDLOG_DEBUG("No X coordinate specified for this patch, "
                             "defaulting to 0.0");
            }

            if(patch_definition["position"].contains("y")) {
                position_y = patch_definition["position"]["y"].get<double>();
            } else {
                SPDLOG_DEBUG("No Y coordinate specified for this patch, "
                             "defaulting to 0.0");
            }
        } else {
            SPDLOG_DEBUG(
                "No patch position specified, defaulting to (0.0, 0.0)");
        }

        // Check if the user specified a location for the patch.
        chrono::ChQuaternion<double> orientation = chrono::QUNIT;
        if(patch_definition.contains("orientation")) {
            if(patch_definition["orientation"]["component"]
                   .get<std::string>() == "y") {
                orientation.SetFromAngleY(
                    -chrono::CH_DEG_TO_RAD *
                    patch_definition["orientation"]["magnitude"].get<double>());
            } else if(patch_definition["orientation"]["component"]
                          .get<std::string>() == "x") {
                orientation.SetFromAngleX(
                    -chrono::CH_DEG_TO_RAD *
                    patch_definition["orientation"]["magnitude"].get<double>());
            } else if(patch_definition["orientation"]["component"]
                          .get<std::string>() == "z") {
                orientation.SetFromAngleZ(
                    -chrono::CH_DEG_TO_RAD *
                    patch_definition["orientation"]["magnitude"].get<double>());
            }
        }

        // Check if the user specified a patch size.
        if(patch_definition.contains("size")) {
            if(patch_definition["size"].contains("length")) {
                patch_length = patch_definition["size"]["length"].get<double>();
            } else {
                SPDLOG_DEBUG("No patch length specified, defaulting to 100.0");
            }

            if(patch_definition["size"].contains("width")) {
                patch_width = patch_definition["size"]["width"].get<double>();
            } else {
                SPDLOG_DEBUG("No patch width specified, defaulting to 100.0");
            }
        } else {
            SPDLOG_DEBUG(
                "No patch size specified, defaulting to 100.0 x 100.0");
        }

        std::shared_ptr<chrono::vehicle::RigidTerrain::Patch> patch;
        auto position_z = 0.0;

        if(patch_definition.contains("slope")) {
            position_x -=
                patch_length / 2.0 *
                std::cos(
                    chrono::CH_DEG_TO_RAD *
                    patch_definition["orientation"]["magnitude"].get<double>());

            position_z +=
                patch_length / 2.0 *
                std::sin(
                    chrono::CH_DEG_TO_RAD *
                    patch_definition["orientation"]["magnitude"].get<double>());
        }

        if(patch_definition.contains("heightmap")) {
            patch = rigid_terrain->AddPatch(
                material,
                chrono::ChCoordsys(chrono::ChVector3<double>(position_x,
                                                             position_y,
                                                             position_z),
                                   orientation),
                patch_definition["heightmap"]["file"].get<std::string>(),
                patch_length,
                patch_width,
                patch_definition["heightmap"]["lowest"].get<double>(),
                patch_definition["heightmap"]["highest"].get<double>());
        } else {
            patch = rigid_terrain->AddPatch(
                material,
                chrono::ChCoordsys(chrono::ChVector3<double>(position_x,
                                                             position_y,
                                                             position_z),
                                   orientation),
                patch_length,
                patch_width,
                0.1,
                false,
                1.0);
        }

        auto red = 127;
        auto green = 127;
        auto blue = 127;

        if(patch_definition.contains("visualization")) {
            if(patch_definition["visualization"].contains("color")) {
                red = patch_definition["visualization"]["color"][0]
                          .get<double>() /
                    255.0;
                green = patch_definition["visualization"]["color"][1]
                            .get<double>() /
                    255.0;
                blue = patch_definition["visualization"]["color"][2]
                           .get<double>() /
                    255.0;
            } else {
                SPDLOG_DEBUG("No patch color specified, defaulting to grey");
            }
            patch->SetColor(chrono::ChColor(red, green, blue));

            if(patch_definition["visualization"].contains("texture") &&
               patch_definition["visualization"]["texture"].contains("path")) {
                double scale_x =
                    patch_length / patch_width * patch_length / 5.0;
                double scale_y = 1.0 * patch_length / 5.0;

                patch->SetTexture(
                    DYNO_DATA_DIR +
                        patch_definition["visualization"]["texture"]["path"]
                            .get<std::string>(),
                    scale_x,
                    scale_y);
            }
        }

        SPDLOG_DEBUG(
            "Added terrain patch of size {}x{} m at location ({}, {})...",
            patch_length,
            patch_width,
            position_x,
            position_y);
    }

    // Finalize the rigid terrain initialization.
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
    auto janosi_shear_coefficient =
        configuration_->GetValue<double>("terrain/scm/janosiShearCoefficient",
                                         0.01);
    auto elastic_stiffness =
        configuration_->GetValue<double>("terrain/scm/elasticStiffness", 4.0e7);
    auto damping =
        configuration_->GetValue<double>("terrain/scm/damping", 3.0e4);

    // Set the soil parameters.
    scm_terrain->SetSoilParameters(bekker_wong_k_phi,
                                   bekker_wong_k_c,
                                   bekker_wong_n,
                                   mohr_cohesive_limit,
                                   mohr_friction_limit,
                                   janosi_shear_coefficient,
                                   elastic_stiffness,
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
            "terrain/scm/visualization/plot/minimum",
            0.0),
        configuration_->GetValue<double>(
            "terrain/scm/visualization/plot/minimum",
            1.0));

    if(configuration_->GetValue<bool>("terrain/scm/input/enabled", false)) {
        scm_terrain_reader_ =
            std::make_shared<DYNO::Interfaces::JSONTerrainOutput>(
                configuration_->GetValue<std::string>("terrain/scm/input/path"),
                scm_terrain);
        scm_terrain_reader_->LoadFromDisk();
    }

    if(configuration_->GetValue<bool>("terrain/scm/output/enabled", false)) {
        scm_terrain_writer_ =
            std::make_shared<DYNO::Interfaces::JSONTerrainOutput>(
                configuration_->GetValue<std::string>(
                    "terrain/scm/output/path"),
                scm_terrain);
    }
    terrain_ = scm_terrain;
}

void Terrain::WriteToDisk() {
    if(configuration_->GetValue<bool>("terrain/scm/serialization/output")) {
        SPDLOG_DEBUG("Writing SCM nodes to disk...");
        scm_terrain_writer_->WriteToDisk();
    }
}

std::shared_ptr<chrono::vehicle::ChTerrain> Terrain::GetTerrain() {
    return terrain_;
}

} // namespace Environments
} // namespace DYNO