//
//  mesh_offscreen.cpp
//  engine::tst
//
//  Renders a core primitive mesh (a UV sphere) offscreen with indexed drawing + depth,
//  through the GeometryStore and the canonical vertex layout, then reads pixels back and
//  checks the lit sphere covers the center while the corner stays the clear color.
//  Ties together: core geometry → GeometryStore → RHI indexed draw → Metal backend.
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"

#include <glm/glm.hpp>

namespace {
struct Uniforms { glm::mat4 mvp{1.0f}; glm::mat4 normalMatrix{1.0f}; };
}

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
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 128, H = 128;

    Device device = Device::createHeadless({});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: could not read %s\n", metallib.c_str()); return 1; }

    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);
    if (!vs.valid() || !fs.valid()) { std::printf("FAIL: shader load\n"); return 1; }

    // Pipeline uses the canonical core::Vertex layout + a depth buffer.
    const rhi::VertexLayout layout = render::coreVertexLayout();
    const Format colorFormat = Format::RGBA8Unorm;

    GraphicsPipelineDesc pd;
    pd.vertex = vs;
    pd.fragment = fs;
    pd.vertexLayout = layout;
    pd.topology = Topology::TriangleList;
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    if (!pipe.valid()) { std::printf("FAIL: pipeline\n"); return 1; }

    // Render targets: color (readable) + depth.
    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm,
          .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    TextureHandle depth = device.createTexture(
        { .width = W, .height = H, .format = Format::Depth32Float,
          .usage = TextureUsage::DepthTarget });
    RenderTargetHandle colorRT = device.createRenderTarget(color);
    RenderTargetHandle depthRT = device.createRenderTarget(depth);

    // Upload a sphere via the GeometryStore (positions already in clip-space range).
    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.7f, 24, 48));
    const auto range = geometry.range(sphere);

    // Identity camera uniform: positions pass through to clip space as before.
    Uniforms u{};
    BufferHandle ubuf = device.createBuffer(
        { .size = sizeof(Uniforms), .usage = BufferUsage::Uniform, .memory = MemoryMode::CpuToGpu },
        std::as_bytes(std::span<const Uniforms>(&u, 1)));

    // Render.
    FrameContext frame = device.beginFrame();
    CommandList cl = device.commandList(frame);

    ColorAttachment ca;
    ca.target = colorRT;
    ca.load = LoadOp::Clear; ca.store = StoreOp::Store;
    ca.clearColor[0] = 0.1f; ca.clearColor[1] = 0.1f; ca.clearColor[2] = 0.1f; ca.clearColor[3] = 1.0f;

    DepthAttachment da;
    da.target = depthRT;
    da.load = LoadOp::Clear; da.store = StoreOp::DontCare;
    da.clearDepth = 1.0f;

    RenderTargetDesc rtd;
    rtd.color = std::span<const ColorAttachment>(&ca, 1);
    rtd.depth = &da;
    rtd.width = W; rtd.height = H;

    cl.beginRendering(rtd);
    cl.bindPipeline(pipe);
    cl.setViewport(0, 0, float(W), float(H));
    BufferBinding ub{ .binding = 0, .buffer = ubuf };
    ResourceBindings rb; rb.buffers = std::span<const BufferBinding>(&ub, 1);
    cl.bindResources(rb);
    cl.bindVertexBuffer(geometry.vertexBuffer(), 0);
    cl.bindIndexBuffer(geometry.indexBuffer(), IndexType::Uint32);
    cl.drawIndexed(range.indexCount, 1, range.firstIndex, range.vertexOffset, 0);
    cl.endRendering();

    device.submit(frame, cl);
    device.endFrame(std::move(frame));

    // Readback + verify.
    std::vector<uint8_t> pixels(static_cast<size_t>(W) * H * 4);
    device.readback(color, std::as_writable_bytes(std::span<uint8_t>(pixels)));

    auto px = [&](uint32_t x, uint32_t y) { return &pixels[(static_cast<size_t>(y) * W + x) * 4]; };
    const uint8_t* center = px(W / 2, H / 2);
    const uint8_t* corner = px(2, 2);

    std::printf("center rgba = %u %u %u %u\n", center[0], center[1], center[2], center[3]);
    std::printf("corner rgba = %u %u %u %u\n", corner[0], corner[1], corner[2], corner[3]);
    std::printf("sphere: indexCount=%u firstIndex=%u vertexOffset=%d\n",
                range.indexCount, range.firstIndex, range.vertexOffset);

    const bool centerLit    = (center[0] + center[1] + center[2]) > 120 && center[3] == 255;
    const bool cornerIsClear = corner[0] < 60 && corner[1] < 60 && corner[2] < 60;
    if (!centerLit)     { std::printf("FAIL: center is not the lit sphere\n"); return 1; }
    if (!cornerIsClear) { std::printf("FAIL: corner is not the clear color\n"); return 1; }

    std::printf("mesh offscreen ok\n");
    return 0;
}
