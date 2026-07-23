//
//  image.cpp
//  engine::core / image
//
//  stb_image-backed loader. This is the single translation unit that defines the stb_image
//  implementation (stb is header-only / INTERFACE — no other TU compiles it). Gated on
//  ENGINE_ASSET_LOADERS so the headless training build (which does not link stb) still compiles;
//  there the loaders return an invalid Image.
//

#include "engine/core/image/image.h"

#if defined(ENGINE_ASSET_LOADERS)

#include <cstring>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO_LINK_WARNING
#include <stb_image.h>

namespace engine::core {

namespace {
Image adopt(unsigned char* data, int w, int h) {
    Image img;
    if (!data || w <= 0 || h <= 0) {
        if (data) stbi_image_free(data);
        return img;
    }
    img.width    = static_cast<uint32_t>(w);
    img.height   = static_cast<uint32_t>(h);
    img.channels = 4;
    img.pixels.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
    std::memcpy(img.pixels.data(), data, img.pixels.size());
    stbi_image_free(data);
    return img;
}
} // namespace

Image loadImage(std::string_view path, bool flipVertically) {
    stbi_set_flip_vertically_on_load(flipVertically ? 1 : 0);
    int w = 0, h = 0, comp = 0;
    unsigned char* data = stbi_load(std::string(path).c_str(), &w, &h, &comp, 4);   // force RGBA
    return adopt(data, w, h);
}

Image loadImageFromMemory(std::span<const std::byte> encoded, bool flipVertically) {
    if (encoded.empty()) return {};
    stbi_set_flip_vertically_on_load(flipVertically ? 1 : 0);
    int w = 0, h = 0, comp = 0;
    unsigned char* data = stbi_load_from_memory(
        reinterpret_cast<const stbi_uc*>(encoded.data()),
        static_cast<int>(encoded.size()), &w, &h, &comp, 4);   // force RGBA
    return adopt(data, w, h);
}

} // namespace engine::core

#else   // !ENGINE_ASSET_LOADERS — headless training build, no image decoder linked.

namespace engine::core {
Image loadImage(std::string_view, bool) { return {}; }
Image loadImageFromMemory(std::span<const std::byte>, bool) { return {}; }
} // namespace engine::core

#endif
