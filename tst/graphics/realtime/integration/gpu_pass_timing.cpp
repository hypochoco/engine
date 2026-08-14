#include "harness/harness.h"
//
//  gpu_pass_timing.cpp
//  engine::tst — graphics / integration
//
//  Verifies the RHI per-pass GPU timestamp mechanism: create a timestamp pool, sample a render
//  pass at its boundaries (RenderTargetDesc::timestampBegin/End), drain, then resolve and assert
//  the end timestamp is after the begin and the derived duration is plausible. Skips gracefully if
//  the device doesn't support stage-boundary counter sampling.
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

TST_CASE(graphics, integration, gpu_pass_timing) {
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    if (!device.gpuTimestampsSupported()) {
        std::printf("gpu timestamps unsupported on this device — skipping\n");
        return;   // graceful skip (still counts as a pass)
    }

    TimestampPoolHandle pool = device.createTimestampPool(4);
    TST_REQUIRE_MSG(pool.valid(), "createTimestampPool should succeed when supported");

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/triangle.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: could not read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup/verification failed"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

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

    // One frame: sample the pass into slots 0 (begin) and 1 (end).
    FrameContext frame = device.beginFrame();
    CommandList cl = device.commandList(frame);
    ColorAttachment ca;
    ca.target = rt; ca.load = LoadOp::Clear; ca.store = StoreOp::Store;
    ca.clearColor[0] = 0.1f; ca.clearColor[1] = 0.1f; ca.clearColor[2] = 0.1f; ca.clearColor[3] = 1.0f;
    RenderTargetDesc rtd;
    rtd.color = std::span<const ColorAttachment>(&ca, 1);
    rtd.width = W; rtd.height = H;
    rtd.timestampPool = pool; rtd.timestampBegin = 0; rtd.timestampEnd = 1;
    cl.beginRendering(rtd);
    cl.bindPipeline(pipe);
    cl.setViewport(0, 0, float(W), float(H));
    cl.bindVertexBuffer(vbuf, 0);
    cl.draw(3, 1, 0, 0);
    cl.endRendering();
    device.submit(frame, cl);
    device.endFrame(std::move(frame));
    device.waitIdle();

    std::array<uint64_t, 4> vals{};
    const bool ok = device.resolveTimestamps(pool, std::span<uint64_t>(vals.data(), vals.size()));
    TST_REQUIRE_MSG(ok, "resolveTimestamps should succeed");
    std::printf("pass timestamps: begin=%llu end=%llu\n",
                static_cast<unsigned long long>(vals[0]), static_cast<unsigned long long>(vals[1]));
    TST_REQUIRE_MSG(vals[1] > vals[0], "end timestamp must be after begin");

    const double ms = static_cast<double>(vals[1] - vals[0]) * 1e-6;   // ns → ms (Apple GPU timebase)
    std::printf("pass GPU time = %.4f ms\n", ms);
    TST_REQUIRE_MSG(ms > 0.0 && ms < 1000.0, "pass duration should be plausible");

    device.destroy(pool);
    std::printf("gpu pass timing ok\n");
}
