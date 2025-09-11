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

#include <dyno/interfaces/json_terrain_output.hpp>

namespace DYNO {
namespace Interfaces {

JSONTerrainOutput::JSONTerrainOutput(
    std::string path,
    std::shared_ptr<chrono::vehicle::SCMTerrain> terrain)
    : path_(path),
      terrain_(terrain) {}

void JSONTerrainOutput::LoadFromDisk() {
    std::ifstream file(path_);

    nlohmann::json serialized_terrain = nlohmann::json::parse(file);

    std::vector<chrono::vehicle::SCMTerrain::NodeLevel> node_levels;

    for(auto& node : serialized_terrain["nodes"]) {
        node_levels.push_back(chrono::vehicle::SCMTerrain::NodeLevel(
            {chrono::ChVector2<int>(node["x"], node["y"]), node["d"]}));
    }

    terrain_->SetModifiedNodes(node_levels);
}

void JSONTerrainOutput::WriteToDisk() {
    auto modified_nodes = terrain_->GetModifiedNodes(true);

    std::ofstream file(path_);

    nlohmann::json serialized_terrain;
    for(auto& current_modified_node : modified_nodes) {
        nlohmann::json serialized_node;
        serialized_node["x"] = current_modified_node.first.x();
        serialized_node["y"] = current_modified_node.first.y();
        serialized_node["d"] = current_modified_node.second;
        serialized_terrain["nodes"].push_back(serialized_node);
    }

    file << serialized_terrain;
}
} // namespace Interfaces
} // namespace DYNO