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

#include <experimental/filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <chrono/core/ChVector3.h>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace chrono {
void to_json(nlohmann::json& data, const chrono::ChVector3d& vector); 
void from_json(const nlohmann::json& data, chrono::ChVector3d& vector);
}

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
std::vector<std::string> SplitKeys(std::string const& keys,
                                   char const delimiter) noexcept;

nlohmann::json GetValue(nlohmann::json const& dictionary,
                        std::string const& keys);

std::string GetPath(std::string user_path);

template <class T>
T GetValue(const nlohmann::json& dictionary, const std::string& keys) {
    try {
        return GetValue(dictionary, keys).get<T>();
    } catch (const std::invalid_argument& exception) {
        SPDLOG_ERROR("Value at key \"{}\" not found!", keys);
        throw std::invalid_argument("Invalid dictionary key!");
    }
}

template <class T>
T GetValue(const nlohmann::json& dictionary, const std::string& keys,
           T default_value) {
    try {
        return GetValue(dictionary, keys).get<T>();
    } catch (const std::invalid_argument& exception) {
        SPDLOG_DEBUG(
            "Value at key \"{}\" not found, using default value \"{}\" ...",
            keys, default_value);
        return default_value;
    }
}

}  // namespace JSON
}  // namespace Interfaces
}  // namespace DYNO
