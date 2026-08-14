#include "harness/harness.h"
//
//  bindless_textures.cpp (graphics benchmark)
//  engine::tst
//
//  Phase-1 texture foundation baseline:
//    - texture upload + mipmap generation throughput (create + replaceRegion + blit generateMipmaps),
//    - frame time for an instanced draw sampling a bindless albedo texture vs the untextured path.
//  Headless endFrame waits for GPU completion. Absolute numbers are machine/thermal/build dependent
//  — a relative before/after baseline on the SAME machine.
//

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <span>
#include <string>
#include <vector>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"

using Clock = std::chrono::steady_clock;

namespace {
std::vector<std::byte> readBin(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> d(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(d.data()), size);
    return d;
}
double ms(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }
}

TST_CASE(graphics, benchmark, bindless_textures) {
    using namespace engine;
    using namespace engine::rhi;

#if defined(NDEBUG)
    std::printf("[build: optimized]\n");
#else
    std::printf("[build: DEBUG — timings not representative]\n");
#endif

    constexpr uint32_t W = 512, H = 512;
    Device device = Device::createHeadless({});

    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    if (blob.empty()) { TST_REQUIRE_MSG(false, "read mesh.metallib"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const Format colorFmt = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs;
    pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&colorFmt, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    SamplerHandle matSamp = device.createSampler({ .mipmap = MipmapMode::Linear });

    // --- (A) texture upload + mipmap generation throughput ---
    auto uploadBatch = [&](uint32_t count, uint32_t dim, uint32_t mips) {
        std::vector<uint8_t> px(static_cast<size_t>(dim) * dim * 4, 200);
        std::vector<TextureHandle> texs; texs.reserve(count);
        const auto t0 = Clock::now();
        for (uint32_t i = 0; i < count; ++i) {
            TextureHandle t = device.createTexture(
                { .width = dim, .height = dim, .mipLevels = mips, .format = Format::RGBA8Unorm,
                  .usage = TextureUsage::Sampled },
                std::as_bytes(std::span<const uint8_t>(px)));
            if (mips > 1) device.generateMipmaps(t);
            texs.push_back(t);
        }
        const double dt = ms(Clock::now() - t0);
        for (auto t : texs) device.destroy(t);
        return dt;
    };
    std::printf("\ntexture upload+mip throughput (512x512 RGBA8):\n");
    for (uint32_t count : {16u, 64u, 256u}) {
        const double withMip = uploadBatch(count, 512, 10);
        const double noMip    = uploadBatch(count, 512, 1);
        std::printf("  %4u textures: mip-chain %7.3f ms (%.1fk/s)   mip0-only %7.3f ms\n",
                    count, withMip, count / withMip, noMip);
    }

    // --- (B) textured instanced draw vs untextured ---
    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 24, 48));
    render::Renderer renderer(device, geometry);
    render::RenderResources res; res.mesh = pipe; res.materialSampler = matSamp;
    renderer.setResources(res);

    std::vector<uint8_t> tpx(4 * 4 * 4, 180);
    const uint32_t slot = device.registerBindlessTexture(device.createTexture(
        { .width = 4, .height = 4, .format = Format::RGBA8Unorm, .usage = TextureUsage::Sampled },
        std::as_bytes(std::span<const uint8_t>(tpx))));

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFmt, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    auto benchDraw = [&](uint32_t instances, bool textured) {
        std::vector<render::InstanceData> insts(instances);
        for (uint32_t i = 0; i < instances; ++i) {
            const float a = float(i) * 0.3f;
            insts[i].model = glm::translate(glm::mat4(1.0f), glm::vec3(std::sin(a) * 3, std::cos(a) * 3, -float(i % 20)));
            insts[i].normalModel = insts[i].model;
            insts[i].materialIndex = 0;
        }
        render::MaterialGPU m; m.baseColorFactor = glm::vec4(1.0f);
        m.baseColorTexture = textured ? int(slot) : -1;
        render::RenderItem item{ sphere, 0, instances };
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0, 0, 8), glm::vec3(0), glm::vec3(0, 1, 0));
        v.proj = glm::perspective(glm::radians(60.0f), float(W) / float(H), 0.1f, 100.0f);
        v.target = colorRT; v.width = W; v.height = H;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(insts);
        v.materials = std::span<const render::MaterialGPU>(&m, 1);

        const int frames = 60;
        double record = 0.0;
        const auto t0 = Clock::now();
        for (int f = 0; f < frames; ++f) {
            FrameContext frame = device.beginFrame();
            const auto r0 = Clock::now();
            renderer.render(frame, std::span<const render::RenderView>(&v, 1));
            record += ms(Clock::now() - r0);
            device.endFrame(std::move(frame));
        }
        const double frame = ms(Clock::now() - t0) / frames;
        std::printf("  %5u inst %s: frame %6.3f ms  record %6.3f ms\n",
                    instances, textured ? "textured  " : "untextured", frame, record / frames);
    };
    std::printf("\ninstanced draw (bindless albedo vs none), 512x512:\n");
    for (uint32_t inst : {256u, 4096u, 16384u}) {
        benchDraw(inst, false);
        benchDraw(inst, true);
    }

    device.destroy(matSamp);
    std::printf("\nbindless textures benchmark ok\n");
}
