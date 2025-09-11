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

#pragma once

#include <filesystem>

#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/terrain/SCMTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include <dyno/interfaces/json_configuration.hpp>
#include <dyno/interfaces/json_terrain_output.hpp>

namespace DYNO {
namespace Environments {

enum TerrainType { RIGID = 0, SCM = 1 };

class Terrain {
   public:
    Terrain(std::shared_ptr<chrono::ChSystem> system,
            std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration);

    void Advance(double time_step);

    void Synchronize(double time);

    void WriteToDisk();

    std::shared_ptr<chrono::vehicle::ChTerrain> GetTerrain();

    void InitializeFrom(
        std::shared_ptr<chrono::vehicle::RigidTerrain>& terrain);

    void InitializeFrom(std::shared_ptr<chrono::vehicle::SCMTerrain>& terrain);

    void InitializeRigidTerrainSinglePatch(double friction_coefficient,
                                           chrono::ChVector2d patch_position,
                                           chrono::ChVector2d patch_size,
                                           std::filesystem::path texture_path);

   private:
    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration_;

    void SelectTerrain();

    void InitializeSCM();

    std::shared_ptr<chrono::ChSystem> system_;

    std::shared_ptr<chrono::vehicle::ChTerrain> terrain_;

    std::shared_ptr<DYNO::Interfaces::JSONTerrainOutput> scm_terrain_reader_;

    std::shared_ptr<DYNO::Interfaces::JSONTerrainOutput> scm_terrain_writer_;

    TerrainType terrain_type_;
};

}  // namespace Environments
}  // namespace DYNO
