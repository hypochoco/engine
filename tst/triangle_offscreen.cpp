//
//  triangle_offscreen.cpp
//  engine::tst
//
//  Renders one triangle into an offscreen RGBA8 texture using the RHI (Metal backend) and a
//  Slang-compiled shader, then reads the pixels back and checks the center is the triangle
//  (not the clear color). Fully headless — no window. Proves shader load → pipeline →
//  encoder → draw → readback end to end.
//

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "engine/graphics/rhi/rhi.h"

namespace {
std::vector<std::byte> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> data(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}
}

int main() {
    using namespace engine::rhi;

    constexpr uint32_t W = 64, H = 64;

    Device device = Device::createHeadless({});

    // --- shader (Slang-compiled metallib; ENGINE_SHADER_DIR set by CMake) ---
    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/triangle.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: could not read %s\n", metallib.c_str()); return 1; }

    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);
    if (!vs.valid() || !fs.valid()) { std::printf("FAIL: shader load\n"); return 1; }

    // --- pipeline: vertex = {float3 position; float3 color;} (stride 24) ---
    VertexLayout layout;
    layout.stride = 24;
    layout.attributes = {
        { .location = 0, .format = VertexFormat::Float3, .offset = 0 },
        { .location = 1, .format = VertexFormat::Float3, .offset = 12 },
    };
    const Format colorFormat = Format::RGBA8Unorm;

    GraphicsPipelineDesc pd;
    pd.vertex = vs;
    pd.fragment = fs;
    pd.vertexLayout = layout;
    pd.topology = Topology::TriangleList;
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    if (!pipe.valid()) { std::printf("FAIL: pipeline\n"); return 1; }

    // --- offscreen render target ---
    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm,
          .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle rt = device.createRenderTarget(color);

    // --- geometry: 3 verts (clip-space pos + rgb color) ---
    const std::array<float, 18> verts = {
        //   x      y     z      r   g   b
         0.0f, -0.5f, 0.0f,   1, 0, 0,
         0.5f,  0.5f, 0.0f,   0, 1, 0,
        -0.5f,  0.5f, 0.0f,   0, 0, 1,
    };
    const auto vbytes = std::as_bytes(std::span<const float>(verts));
    BufferHandle vbuf = device.createBuffer(
        { .size = vbytes.size_bytes(), .usage = BufferUsage::Vertex, .memory = MemoryMode::CpuToGpu },
        vbytes);

    // --- render ---
    FrameContext frame = device.beginFrame();
    CommandList cl = device.commandList(frame);

    ColorAttachment ca;
    ca.target = rt;
    ca.load = LoadOp::Clear;
    ca.store = StoreOp::Store;
    ca.clearColor[0] = 0.1f; ca.clearColor[1] = 0.1f; ca.clearColor[2] = 0.1f; ca.clearColor[3] = 1.0f;

    RenderTargetDesc rtd;
    rtd.color = std::span<const ColorAttachment>(&ca, 1);
    rtd.width = W; rtd.height = H;

    cl.beginRendering(rtd);
    cl.bindPipeline(pipe);
    cl.setViewport(0, 0, float(W), float(H));
    cl.bindVertexBuffer(vbuf, 0);
    cl.draw(3, 1, 0, 0);
    cl.endRendering();

    device.submit(frame, cl);
    device.endFrame(std::move(frame));

    // --- readback + verify ---
    std::vector<uint8_t> pixels(static_cast<size_t>(W) * H * 4);
    device.readback(color, std::as_writable_bytes(std::span<uint8_t>(pixels)));

    auto px = [&](uint32_t x, uint32_t y) { return &pixels[(static_cast<size_t>(y) * W + x) * 4]; };
    const uint8_t* center = px(W / 2, H / 2);
    const uint8_t* corner = px(1, 1);   // near top-left: outside the triangle → clear color

    std::printf("center rgba = %u %u %u %u\n", center[0], center[1], center[2], center[3]);
    std::printf("corner rgba = %u %u %u %u\n", corner[0], corner[1], corner[2], corner[3]);

    const bool centerIsTriangle = (center[0] + center[1] + center[2]) > 90 && center[3] == 255;
    const bool cornerIsClear    = corner[0] < 60 && corner[1] < 60 && corner[2] < 60;
    if (!centerIsTriangle) { std::printf("FAIL: center pixel is not the triangle\n"); return 1; }
    if (!cornerIsClear)    { std::printf("FAIL: corner pixel is not the clear color\n"); return 1; }

    std::printf("triangle offscreen ok\n");
    return 0;
}
