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

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include <opencv2/opencv.hpp>

namespace DYNO {
namespace Environments {

/**
 * @brief Class for processing heightmap data from a file.
 */
class HeightmapProcessor {
   public:
    /**
     * @brief Constructs a _HeightmapProcessor_ object and loads the heightmap
     *        from the specified file path.
     *
     * @param path The file path to the heightmap data.
     */
    HeightmapProcessor(const std::filesystem::path& path);

    /**
     * @brief Retrieves the height value at the specified (x, y) coordinates.
     *
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     *
     * @return The height value at the specified coordinates.
     */
    double GetHeight(double x, double y);

    /**
     * @brief Sets the size of the heightmap in both x and y dimensions.
     *
     * @param size_x The size in the x dimension.
     * @param size_y The size in the y dimension.
     */
    void SetSize(double size_x, double size_y);

    /**
     * @brief Sets the size of the heightmap in the x dimension.
     *
     * @param size The size in the x dimension.
     */
    void SetSizeX(double size);

    /**
     * @brief Sets the size of the heightmap in the y dimension.
     *
     * @param size The size in the y dimension.
     */
    void SetSizeY(double size);

    /**
     * @brief Sets the minimum and maximum height values for the heightmap.
     *
     * @param height_min The minimum height value.
     * @param height_max The maximum height value.
     */
    void SetHeightRange(double height_min, double height_max);

    /**
     * @brief Sets the minimum height value for the heightmap.
     *
     * @param height The minimum height value.
     */
    void SetHeightMin(double height);

    /**
     * @brief Sets the maximum height value for the heightmap.
     *
     * @param height The maximum height value.
     */
    void SetHeightMax(double height);

    /**
     * @brief Sets the origin (starting point) of the heightmap in both x and y
     *        dimensions.
     *
     * @param x The x-coordinate of the origin.
     * @param y The y-coordinate of the origin.
     */
    void SetOrigin(double x, double y);

    /**
     * @brief Sets the x-coordinate of the origin of the heightmap.
     *
     * @param x The x-coordinate of the origin.
     */
    void SetOriginX(double x);

    /**
     * @brief Sets the y-coordinate of the origin of the heightmap.
     *
     * @param y The y-coordinate of the origin.
     */
    void SetOriginY(double y);

   private:
    /**
     * @brief Loads the heightmap data from the file specified during
     *        construction.
     *
     * @param path The file path to the heightmap data.
     */
    void LoadHeightmap(const std::filesystem::path& path);

    /** @brief The heightmap data stored as a matrix. */
    cv::Mat heightmap_;

    /** @brief The size of the heightmap in the x dimension. */
    double size_x_ = 1.0;

    /** @brief The size of the heightmap in the y dimension. */
    double size_y_ = 1.0;

    /** @brief The minimum height value for the heightmap. */
    double height_min_ = 0.0;

    /** @brief The maximum height value for the heightmap. */
    double height_max_ = 1.0;

    /** @brief The x-coordinate of the origin of the heightmap. */
    double origin_x_ = 0.0;

    /** @brief The y-coordinate of the origin of the heightmap. */
    double origin_y_ = 0.0;
};

}  // namespace Environments
}  // namespace DYNO
