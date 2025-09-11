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

#include <dyno/interfaces/json_utilities.hpp>

namespace chrono {
void to_json(nlohmann::json& data, const chrono::ChVector3d& vector) {
    data = nlohmann::json{{vector.x(), vector.y(), vector.z()}};
}

void from_json(const nlohmann::json& data, chrono::ChVector3d& vector) {
    vector.x() = data[0];
    vector.y() = data[1];
    vector.z() = data[2];
}
}  // namespace chrono

namespace DYNO {
namespace Interfaces {
namespace JSON {

/**
 * @brief Split a dictionary key sequence into a vector of keys through a
 * specified delimiter.
 *
 * @param keys List of keys concated into a single string through the
 * specified delimiter.
 * @param delimiter The delimiter used to split the key sequence.
 * @return std::vector<std::string>
 */

std::vector<std::string> SplitKeys(std::string const& str,
                                   char const delimiter) noexcept {
    std::vector<std::string> res = {};
    std::size_t start{0};
    std::size_t end{0};

    while ((start = str.find_first_not_of(delimiter, end)) !=
           std::string::npos) {
        end = str.find(delimiter, start);
        res.push_back(str.substr(start, end - start));
    }

    return res;
}

nlohmann::json GetValue(nlohmann::json const& dictionary,
                        std::string const& keys) {
    std::vector<std::string> const names = SplitKeys(keys, '/');
    nlohmann::json const* leaf = &dictionary;
    for (auto const& name : names) {
        if (leaf->contains(name)) {
            leaf = &leaf->at(name);
        } else {
            throw std::invalid_argument("Invalid dictionary key!");
        }
    }
    return *leaf;
}

std::string GetPath(std::string user_path) {
    std::experimental::filesystem::path filesystem_path(user_path);
    if (filesystem_path.is_relative()) {
        return DYNO_DATA_DIR + user_path;
    }
    return user_path;
}

}  // namespace JSON
}  // namespace Interfaces
}  // namespace DYNO
