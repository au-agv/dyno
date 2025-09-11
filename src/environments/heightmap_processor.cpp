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

#include <dyno/environments/heightmap_processor.hpp>

namespace DYNO {
namespace Environments {

HeightmapProcessor::HeightmapProcessor(const std::filesystem::path& path) {
    LoadHeightmap(path);
}

double HeightmapProcessor::GetHeight(double x, double y) {
    if (heightmap_.empty()) {
        throw std::runtime_error(
            "Could not retrieve height: heightmap not loaded!");
    }

    // Transform from world cordinates to local terrain coordinates.
    double local_x = x - origin_x_;
    double local_y = y - origin_y_;

    int width = heightmap_.cols;
    int height = heightmap_.rows;

    // Transform from local cordinates to pixel heightmap coordinates.
    int pixel_x = static_cast<int>((local_x / size_x_) * (width - 1));
    int pixel_y =
        static_cast<int>(((size_y_ - local_y) / size_y_) * (height - 1));

    // Clamp to the pixel coordinate bounds if the requested coordinates lie
    // outside the map bounds.
    pixel_x = std::clamp(pixel_x, 0, width - 1);
    pixel_y = std::clamp(pixel_y, 0, height - 1);

    // Convert the pixel value to a [0 - 1] range normalized floating-point
    // value mappable to the heightmap range.
    auto pixel_value =
        static_cast<double>(heightmap_.at<unsigned char>(pixel_y, pixel_x)) /
        255.0;

    return height_min_ + (height_max_ - height_min_) * pixel_value;
}

void HeightmapProcessor::LoadHeightmap(const std::filesystem::path& path) {
    cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        throw std::runtime_error("Could not load heightmap at \"" +
                                 path.string() + "\"");
    }

    if (image.channels() == 1) {
        heightmap_ = image;
    } else {
        cv::cvtColor(image, heightmap_, cv::COLOR_BGR2GRAY);
    }
}

void HeightmapProcessor::SetSize(double size_x, double size_y) {
    size_x_ = size_x;
    size_y_ = size_y;
}

void HeightmapProcessor::SetSizeX(double size) {
    size_x_ = size;
}

void HeightmapProcessor::SetSizeY(double size) {
    size_y_ = size;
}

void HeightmapProcessor::SetHeightRange(double height_min, double height_max) {
    height_min_ = height_min;
    height_max_ = height_max;
}

void HeightmapProcessor::SetHeightMin(double height) {
    height_min_ = height;
}

void HeightmapProcessor::SetHeightMax(double height) {
    height_max_ = height;
}

void HeightmapProcessor::SetOrigin(double x, double y) {
    origin_x_ = x;
    origin_y_ = y;
}

void HeightmapProcessor::SetOriginX(double x) {
    origin_x_ = x;
}

void HeightmapProcessor::SetOriginY(double y) {
    origin_y_ = y;
}

}  // namespace Environments
}  // namespace DYNO
