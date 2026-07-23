//
//  image.h
//  engine::core / image
//
//  CPU-side decoded image + loaders. Backend-agnostic: an `Image` is tightly-packed 8-bit
//  RGBA (4 channels), row-major, top-left origin — the layout the RHI uploads straight into a
//  texture. The declarations are dependency-free; the implementation uses stb_image and is
//  compiled only in a full engine build (ENGINE_ASSET_LOADERS). In a headless training build
//  the loaders return an invalid Image rather than pulling in a decoder.
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace engine::core {

// Decoded 8-bit RGBA image. `pixels` is width*height*4 bytes, row-major, top-left origin.
struct Image {
    uint32_t               width    = 0;
    uint32_t               height   = 0;
    uint32_t               channels = 4;   // always 4 (RGBA) once loaded; kept for clarity
    std::vector<std::byte> pixels;

    bool        valid()    const { return width > 0 && height > 0 && !pixels.empty(); }
    std::size_t byteSize() const { return pixels.size(); }
};

// Loads an image file (PNG/JPG/TGA/BMP/PSD/GIF/HDR→LDR via stb_image) as 8-bit RGBA.
// Returns an invalid (empty) Image on failure (bad path, decode error, or loaders not built).
// `flipVertically` flips rows to a bottom-up origin (for APIs/UV conventions that need it).
Image loadImage(std::string_view path, bool flipVertically = false);

// Same, decoding from an in-memory encoded buffer (e.g. bytes from io::readFile or an archive).
Image loadImageFromMemory(std::span<const std::byte> encoded, bool flipVertically = false);

} // namespace engine::core
