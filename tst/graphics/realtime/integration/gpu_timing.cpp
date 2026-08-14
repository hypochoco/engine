#include "harness/harness.h"
//
//  gpu_timing.cpp
//  engine::tst — graphics / integration
//
//  Verifies Device::lastGpuFrameMs() reports a positive GPU busy time after real GPU work. Renders
//  a triangle into an offscreen target for several headless frames, drains, then asserts the
//  reported GPU time is > 0 and plausibly small. Proves the Metal completion-handler timing hook.
//

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <span>
#include <string>
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

TST_CASE(graphics, integration, gpu_frame_timing) {
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    // Before any frame, no timing is available.
    TST_REQUIRE_MSG(device.lastGpuFrameMs() == 0.0, "no GPU time before the first frame");

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/triangle.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: could not read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup/verification failed"); }

    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);
    TST_REQUIRE_MSG(vs.valid() && fs.valid(), "shader load");

    VertexLayout layout;
    layout.stride = 24;
    layout.attributes = {
        { .location = 0, .format = VertexFormat::Float3, .offset = 0 },
        { .location = 1, .format = VertexFormat::Float3, .offset = 12 },
    };
    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs; pd.vertexLayout = layout;
    pd.topology = Topology::TriangleList;
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    TST_REQUIRE_MSG(pipe.valid(), "pipeline");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm,
          .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle rt = device.createRenderTarget(color);

    const std::array<float, 18> verts = {
         0.0f, -0.8f, 0.0f, 1, 0, 0,
         0.8f,  0.8f, 0.0f, 0, 1, 0,
        -0.8f,  0.8f, 0.0f, 0, 0, 1,
    };
    const auto vbytes = std::as_bytes(std::span<const float>(verts));
    BufferHandle vbuf = device.createBuffer(
        { .size = vbytes.size_bytes(), .usage = BufferUsage::Vertex, .memory = MemoryMode::CpuToGpu },
        vbytes);

    // Render several frames of real GPU work.
    for (int i = 0; i < 4; ++i) {
        FrameContext frame = device.beginFrame();
        CommandList cl = device.commandList(frame);
        ColorAttachment ca;
        ca.target = rt; ca.load = LoadOp::Clear; ca.store = StoreOp::Store;
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
    }
    device.waitIdle();   // ensures every completion handler has run → lastGpuFrameMs is populated

    const double gpuMs = device.lastGpuFrameMs();
    std::printf("lastGpuFrameMs = %.4f ms\n", gpuMs);
    TST_REQUIRE_MSG(gpuMs > 0.0, "GPU frame time should be positive after rendering");
    TST_REQUIRE_MSG(gpuMs < 1000.0, "a trivial triangle frame should be well under a second");

    std::printf("gpu frame timing ok\n");
}
