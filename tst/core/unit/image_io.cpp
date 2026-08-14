#include "harness/harness.h"
//
//  image_io.cpp
//  engine::tst — core / unit
//
//  Verifies core::io::readFile + core::image::loadImage / loadImageFromMemory. Builds a known
//  RGBA pattern as a minimal uncompressed 32-bit TGA (self-contained — no encoder dependency),
//  then loads it back both from an in-memory buffer and from a temp file (via io::readFile),
//  checking dimensions + pixel values survive. Also checks the failure path.
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#include "engine/core/image/image.h"
#include "engine/core/io/io.h"

using namespace engine;

namespace {

// Build an uncompressed 32-bit TGA (top-left origin). `rgba` is width*height*4 RGBA bytes.
std::vector<std::byte> makeTGA(int w, int h, const std::vector<uint8_t>& rgba) {
    std::vector<uint8_t> b;
    b.reserve(18 + rgba.size());
    auto push = [&](uint8_t v) { b.push_back(v); };
    push(0);            // id length
    push(0);            // no color map
    push(2);            // uncompressed true-color
    for (int i = 0; i < 5; ++i) push(0);   // color map spec
    push(0); push(0);   // x-origin
    push(0); push(0);   // y-origin
    push(uint8_t(w & 0xFF)); push(uint8_t((w >> 8) & 0xFF));
    push(uint8_t(h & 0xFF)); push(uint8_t((h >> 8) & 0xFF));
    push(32);           // bits per pixel
    push(0x28);         // descriptor: 8 alpha bits + top-to-bottom origin
    // TGA 32-bit pixel order is BGRA.
    for (int i = 0; i < w * h; ++i) {
        push(rgba[i * 4 + 2]);  // B
        push(rgba[i * 4 + 1]);  // G
        push(rgba[i * 4 + 0]);  // R
        push(rgba[i * 4 + 3]);  // A
    }
    std::vector<std::byte> out(b.size());
    std::memcpy(out.data(), b.data(), b.size());
    return out;
}

} // namespace

TST_CASE(core, unit, image_io_roundtrip) {
    constexpr int W = 4, H = 3;

    // Known pattern: R ramps with x, G ramps with y, B constant, A = 255.
    std::vector<uint8_t> src(static_cast<size_t>(W) * H * 4);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            uint8_t* p = &src[(static_cast<size_t>(y) * W + x) * 4];
            p[0] = static_cast<uint8_t>(x * 60);
            p[1] = static_cast<uint8_t>(y * 80);
            p[2] = 40;
            p[3] = 255;
        }
    const std::vector<std::byte> tga = makeTGA(W, H, src);

    auto pixel = [](const core::Image& im, int x, int y) {
        return reinterpret_cast<const uint8_t*>(im.pixels.data()) + (static_cast<size_t>(y) * im.width + x) * 4;
    };

    // (1) decode straight from memory
    core::Image img = core::loadImageFromMemory(std::span<const std::byte>(tga));
    TST_REQUIRE_MSG(img.valid(), "loadImageFromMemory returned an invalid image");
    TST_REQUIRE_MSG(img.width == W && img.height == H && img.channels == 4, "unexpected dimensions");
    TST_REQUIRE_MSG(img.byteSize() == static_cast<size_t>(W) * H * 4, "unexpected byte size");
    const uint8_t* c = pixel(img, W - 1, H - 1);
    std::printf("image_io: %ux%u last px = %u %u %u %u\n", img.width, img.height, c[0], c[1], c[2], c[3]);
    TST_REQUIRE_MSG(c[0] == (W - 1) * 60 && c[1] == (H - 1) * 80 && c[2] == 40 && c[3] == 255,
                    "pixel value did not survive the TGA round trip");

    // (2) write to a temp file, then io::readFile + loadImage from disk should match byte-for-byte.
    const auto path = (std::filesystem::temp_directory_path() / "engine_image_io_test.tga").string();
    { std::ofstream f(path, std::ios::binary); f.write(reinterpret_cast<const char*>(tga.data()), tga.size()); }

    std::vector<std::byte> bytes = core::io::readFile(path);
    TST_REQUIRE_MSG(bytes.size() == tga.size() && bytes == tga, "io::readFile did not return the written bytes");

    core::Image img2 = core::loadImage(path);
    TST_REQUIRE_MSG(img2.valid() && img2.width == img.width && img2.height == img.height, "loadImage from disk mismatch");
    TST_REQUIRE_MSG(img2.pixels == img.pixels, "from-disk pixels differ from from-memory pixels");

    // (3) failure path
    core::Image missing = core::loadImage((std::filesystem::temp_directory_path() / "does_not_exist_zzz.tga").string());
    TST_REQUIRE_MSG(!missing.valid(), "loading a missing file should yield an invalid image");

    std::error_code ec;
    std::filesystem::remove(path, ec);
    std::printf("image io ok\n");
}
