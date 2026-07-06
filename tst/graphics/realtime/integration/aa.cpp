#include "harness/harness.h"
//
//  aa.cpp
//  engine::tst
//
//  MSAA anti-aliasing (RF6 AA). A bright slab rotated in-plane presents diagonal silhouette edges
//  against a black background. Without MSAA the edge pixels are almost all fully foreground or
//  fully background; with 4× MSAA the rasterizer's coverage samples produce many PARTIAL-coverage
//  (intermediate-brightness) pixels along the edge — the measurable signature of anti-aliasing.
//  Interior + background pixels are unchanged (MSAA only touches edges).
//

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

namespace {
std::vector<std::byte> readFileBin(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> data(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}
}

TST_CASE(graphics, integration, aa) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    auto fxaaBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/fxaa.metallib");
    if (meshBlob.empty() || fxaaBlob.empty()) { TST_REQUIRE_MSG(false, "read shaders failed"); }
    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle fxvs = device.createShader(fxaaBlob, ShaderStage::Vertex);
    ShaderHandle fxfs = device.createShader(fxaaBlob, ShaderStage::Fragment);

    const Format colorFormat = Format::RGBA8Unorm;
    auto makePipe = [&](uint32_t samples) {
        GraphicsPipelineDesc pd;
        pd.vertex = vs; pd.fragment = fs;
        pd.vertexLayout = render::coreVertexLayout();
        pd.colorFormats = std::span<const Format>(&colorFormat, 1);
        pd.depthFormat = Format::Depth32Float;
        pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
        pd.sampleCount = samples;
        return device.createGraphicsPipeline(pd);
    };
    PipelineHandle pipe1 = makePipe(1);   // no MSAA
    PipelineHandle pipe4 = makePipe(4);   // 4× MSAA
    TST_REQUIRE_MSG(pipe1.valid() && pipe4.valid(), "pipeline creation failed");

    // FXAA post pipeline (fullscreen, LDR, no depth/cull) + a linear-clamp sampler.
    GraphicsPipelineDesc fxd;
    fxd.vertex = fxvs; fxd.fragment = fxfs;
    fxd.colorFormats = std::span<const Format>(&colorFormat, 1);
    fxd.depthFormat = Format::Undefined;
    fxd.depth = { .test = false, .write = false, .op = CompareOp::Always };
    fxd.raster.cull = CullMode::None;
    PipelineHandle fxaaPipe = device.createGraphicsPipeline(fxd);
    SamplerHandle fxaaSamp = device.createSampler({ .addressU = AddressMode::ClampToEdge,
                                                    .addressV = AddressMode::ClampToEdge });
    TST_REQUIRE_MSG(fxaaPipe.valid() && fxaaSamp.valid(), "FXAA pipeline/sampler creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle box = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::Renderer renderer(device, geometry);

    render::MaterialGPU white; white.baseColorFactor = {1.0f, 1.0f, 1.0f, 1.0f};
    // A thin slab rotated ~27° in-plane ⇒ diagonal silhouette edges (not axis-aligned).
    render::InstanceData inst;
    inst.model = glm::rotate(glm::mat4(1.0f), glm::radians(27.0f), glm::vec3(0, 0, 1)) *
                 glm::scale(glm::mat4(1.0f), glm::vec3(2.6f, 2.6f, 0.1f));
    inst.normalModel = inst.model; inst.materialIndex = 0;
    render::RenderItem item{ box, 0, 1 };

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 0, 4), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(45.0f), float(W) / float(H), 0.1f, 100.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.intensity = 0.0f; view.light.ambient = glm::vec3(1.0f);   // flat, uniformly bright slab
    view.clearColor[0] = 0.0f; view.clearColor[1] = 0.0f; view.clearColor[2] = 0.0f; view.clearColor[3] = 1.0f;
    view.instances = std::span<const render::InstanceData>(&inst, 1);
    view.materials = std::span<const render::MaterialGPU>(&white, 1);

    auto renderTo = [&](std::vector<uint8_t>& out) {
        out.assign(static_cast<size_t>(W) * H * 4, 0);
        view.items = std::span<const render::RenderItem>(&item, 1);
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(out)));
    };
    auto g = [&](const std::vector<uint8_t>& img, uint32_t x, uint32_t y) {
        return int(img[(static_cast<size_t>(y) * W + x) * 4 + 1]);
    };

    std::vector<uint8_t> off, on;
    renderer.setMeshPipeline(pipe1); renderer.setMSAA(1); renderTo(off);
    renderer.setMeshPipeline(pipe4); renderer.setMSAA(4); renderTo(on);

    const int gIn = g(on, W / 2, H / 2);   // slab interior (bright)
    const int gBg = g(on, 4, 4);           // corner background (dark)
    // Intermediate = partial-coverage edge pixels (strictly between bg and interior).
    auto intermediate = [&](const std::vector<uint8_t>& img) {
        size_t c = 0;
        for (size_t p = 0; p < static_cast<size_t>(W) * H; ++p) {
            int v = img[p * 4 + 1];
            if (v > gBg + 25 && v < gIn - 25) ++c;
        }
        return c;
    };
    const size_t interOff = intermediate(off);
    const size_t interOn  = intermediate(on);
    std::printf("MSAA edge pixels (partial coverage): off=%zu 4x=%zu (interior=%d bg=%d)\n",
                interOff, interOn, gIn, gBg);

    TST_REQUIRE_MSG(gIn > 200 && gBg < 30, "expected a bright slab over a dark background");
    TST_REQUIRE_MSG(interOn > 200, "4x MSAA should produce many partial-coverage edge pixels");
    TST_REQUIRE_MSG(interOn > interOff * 3, "MSAA should smooth edges far more than no-MSAA");
    // Parity: interior + background are untouched by MSAA (only edges change).
    TST_REQUIRE_MSG(g(off, W / 2, H / 2) > 200 && g(off, 4, 4) < 30, "non-edge pixels unchanged by MSAA");

    // --- FXAA (post, no MSAA): softens the same hard edge into partial-coverage pixels. ---
    std::vector<uint8_t> fx;
    renderer.setMeshPipeline(pipe1); renderer.setMSAA(1); renderer.setFXAA(fxaaPipe, fxaaSamp); renderTo(fx);
    renderer.setFXAA({}, {});
    const size_t interFX = intermediate(fx);
    std::printf("FXAA edge pixels (partial coverage): no-AA=%zu FXAA=%zu\n", interOff, interFX);
    TST_REQUIRE_MSG(interFX > 100, "FXAA should soften the hard edge into intermediate pixels");
    TST_REQUIRE_MSG(interFX > interOff + 100, "FXAA should add many more edge pixels than no-AA");
    TST_REQUIRE_MSG(g(fx, W / 2, H / 2) > 200 && g(fx, 4, 4) < 30, "FXAA leaves flat interior/bg alone");

    device.destroy(fxaaSamp);
    std::printf("aa (MSAA + FXAA) ok\n");
}
