//
//  image.h
//  engine::core
//
//  CPU-side image data, decoupled from GPU texture creation. Backends map ImageFormat
//  to their own pixel formats (VkFormat / MTLPixelFormat) and upload.
//

#pragma once

#include <cstdint>
#include <vector>

namespace engine {

enum class ImageFormat {
    R8,
    RG8,
    RGBA8,
    RGBA8_SRGB,
};

struct Image {
    uint32_t              width  = 0;
    uint32_t              height = 0;
    ImageFormat           format = ImageFormat::RGBA8;
    std::vector<uint8_t>  pixels;
};

} // namespace engine
