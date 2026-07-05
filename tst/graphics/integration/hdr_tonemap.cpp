#include "harness/harness.h"
//
//  hdr_tonemap.cpp
//  engine::tst
//
//  RF5: HDR render target + ACES tone-mapping resolve. Renders a bright surface (ambient > 1,
//  which clips to white in 8-bit) two ways:
//    - tonemap OFF: forward writes straight to an RGBA8 target ⇒ the bright value clamps to 255.
//    - tonemap ON:  forward writes to an RGBA16F HDR target, then a fullscreen ACES tonemap pass
//                   resolves it ⇒ the bright value is pulled below 255 (graceful highlight).
//  Asserting ON < OFF (and ON still bright) proves the HDR path + texture sampling + tonemap all
//  work. Also exercises the render graph's RenderTarget->ShaderRead barrier between the passes.
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

TST_CASE(graphics, integration, hdr_tonemap) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 128, H = 128;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    auto tmBlob   = readFileBin(std::string(ENGINE_SHADER_DIR) + "/tonemap.metallib");
    if (meshBlob.empty() || tmBlob.empty()) { std::printf("FAIL: read shaders\n"); TST_REQUIRE_MSG(false, "setup failed"); }

    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle tmvs = device.createShader(tmBlob, ShaderStage::Vertex);
    ShaderHandle tmfs = device.createShader(tmBlob, ShaderStage::Fragment);

    auto meshPipe = [&](Format colorFmt) {
        GraphicsPipelineDesc p;
        p.vertex = vs; p.fragment = fs;
        p.vertexLayout = render::coreVertexLayout();
        p.colorFormats = std::span<const Format>(&colorFmt, 1);
        p.depthFormat = Format::Depth32Float;
        p.depth = { .test = true, .write = true, .op = CompareOp::Less };
        return device.createGraphicsPipeline(p);
    };
    PipelineHandle pipeLDR = meshPipe(Format::RGBA8Unorm);   // forward straight to 8-bit (tonemap off)
    PipelineHandle pipeHDR = meshPipe(Format::RGBA16Float);  // forward to HDR (tonemap on)

    // Tonemap pipeline: fullscreen (no vertex layout), no depth, no cull, outputs RGBA8.
    GraphicsPipelineDesc tp;
    tp.vertex = tmvs; tp.fragment = tmfs;
    const Format ldr = Format::RGBA8Unorm;
    tp.colorFormats = std::span<const Format>(&ldr, 1);
    tp.depthFormat = Format::Undefined;
    tp.depth = { .test = false, .write = false, .op = CompareOp::Always };
    tp.raster.cull = CullMode::None;
    PipelineHandle tonemapPipe = device.createGraphicsPipeline(tp);
    SamplerHandle sampler = device.createSampler({ .minFilter = Filter::Linear, .magFilter = Filter::Linear,
                                                   .addressU = AddressMode::ClampToEdge,
                                                   .addressV = AddressMode::ClampToEdge });
    TST_REQUIRE_MSG(pipeLDR.valid() && pipeHDR.valid() && tonemapPipe.valid() && sampler.valid(),
                    "pipeline/sampler creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.9f, 32, 64));
    render::Renderer renderer(device, geometry);

    render::MaterialGPU white; white.baseColorFactor = {1.0f, 1.0f, 1.0f, 1.0f};
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;

    auto makeView = [&](PipelineHandle meshPipe) {
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));
        v.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 20.0f);
        v.target = colorRT; v.width = W; v.height = H;
        // Bright ambient (>1) so the surface is over-white in linear space.
        v.light.intensity = 0.0f;
        v.light.ambient = glm::vec3(1.6f, 1.6f, 1.6f);
        static render::RenderItem item;   // static so its address stays valid in the span
        item = render::RenderItem{ sphere, meshPipe, 0, 1 };
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&white, 1);
        return v;
    };

    auto centerR = [&]() {
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        return int(px[(static_cast<size_t>(H / 2) * W + (W / 2)) * 4 + 0]);
    };
    auto renderView = [&](const render::RenderView& v) {
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(frame));
    };

    // (1) tonemap OFF — forward straight to 8-bit; bright ambient clips to 255.
    renderer.setTonemap({}, {});
    renderView(makeView(pipeLDR));
    const int offR = centerR();

    // (2) tonemap ON — forward to HDR, ACES resolve; bright value pulled below 255.
    renderer.setTonemap(tonemapPipe, sampler);
    renderView(makeView(pipeHDR));
    const int onR = centerR();

    std::printf("bright surface center R: tonemap OFF=%d  ON=%d\n", offR, onR);

    TST_REQUIRE_MSG(offR >= 254, "without tonemap, bright ambient should clip to ~255");
    TST_REQUIRE_MSG(onR < 250, "ACES tonemap should pull the bright highlight below 255");
    TST_REQUIRE_MSG(onR > 150, "tonemapped surface should still be clearly bright (pass ran)");

    device.destroy(sampler);
    std::printf("hdr tonemap ok\n");
}
