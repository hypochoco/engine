#include "harness/harness.h"
//
//  shadow_map.cpp
//  engine::tst
//
//  Directional shadow mapping. A caster box floats over a wide ground box, lit by a straight-down
//  sun. Rendered twice — shadows OFF then ON — and we count pixels that DARKEN when shadows are
//  enabled. A localized shadow patch under the caster must appear (many darkened pixels), but not
//  the whole image (shadows are local), proving the shadow pass + PCF sampling work end-to-end.
//  Also exercises a depth-only pipeline, a sampled depth texture, and the graph's
//  RenderTarget->ShaderRead barrier on the shadow map.
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

TST_CASE(graphics, integration, shadow_map) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    auto shBlob   = readFileBin(std::string(ENGINE_SHADER_DIR) + "/shadow.metallib");
    if (meshBlob.empty() || shBlob.empty()) { std::printf("FAIL: read shaders\n"); TST_REQUIRE_MSG(false, "setup failed"); }

    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle shvs = device.createShader(shBlob, ShaderStage::Vertex);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs;
    pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);

    // Depth-only shadow pipeline: vertex only (no fragment), no color attachment.
    GraphicsPipelineDesc sd;
    sd.vertex = shvs;                       // fragment left invalid
    sd.vertexLayout = render::coreVertexLayout();
    sd.depthFormat = Format::Depth32Float;
    sd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle shadowPipe = device.createGraphicsPipeline(sd);
    SamplerHandle shadowSamp = device.createSampler({ .minFilter = Filter::Nearest, .magFilter = Filter::Nearest,
                                                      .mipmap = MipmapMode::Nearest,
                                                      .addressU = AddressMode::ClampToEdge,
                                                      .addressV = AddressMode::ClampToEdge });
    TST_REQUIRE_MSG(pipe.valid() && shadowPipe.valid() && shadowSamp.valid(), "pipeline/sampler creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle box = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));   // unit cube
    render::Renderer renderer(device, geometry);

    render::MaterialGPU white; white.baseColorFactor = {0.85f, 0.85f, 0.85f, 1.0f};
    // Single mesh, one item (instanceCount=2, firstInstance=0) so SV_InstanceID maps directly.
    std::vector<render::InstanceData> inst(2);
    inst[0].model = glm::scale(glm::mat4(1.0f), glm::vec3(30.0f, 0.1f, 30.0f));                    // ground slab
    inst[1].model = glm::translate(glm::mat4(1.0f), glm::vec3(0, 4, 0)) * glm::scale(glm::mat4(1.0f), glm::vec3(3.0f)); // caster
    for (auto& in : inst) { in.normalModel = in.model; in.materialIndex = 0; }
    render::RenderItem item{ box, 0, 2 };
    renderer.setMeshPipeline(pipe);

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 14, 16), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 100.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.direction = glm::vec3(0.0f, -1.0f, 0.0f);   // straight-down sun
    view.light.intensity = 1.0f;
    view.light.color = glm::vec3(1.0f);
    view.light.ambient = glm::vec3(0.12f);
    view.clearColor[0] = 0.0f; view.clearColor[1] = 0.0f; view.clearColor[2] = 0.0f; view.clearColor[3] = 1.0f;
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(inst);
    view.materials = std::span<const render::MaterialGPU>(&white, 1);

    auto renderTo = [&](std::vector<uint8_t>& out) {
        out.assign(static_cast<size_t>(W) * H * 4, 0);
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(out)));
    };

    // (1) shadows OFF.
    renderer.setShadows({}, {});
    std::vector<uint8_t> off;  renderTo(off);

    // (2) shadows ON.
    renderer.setShadows(shadowPipe, shadowSamp, 12.0f, 100.0f);
    std::vector<uint8_t> on;  renderTo(on);

    // Count pixels that darkened notably (the shadow patch) and pixels that stayed lit.
    size_t darkened = 0, litBoth = 0;
    for (size_t p = 0; p < static_cast<size_t>(W) * H; ++p) {
        int offG = off[p * 4 + 1], onG = on[p * 4 + 1];
        if (offG > 40 && offG - onG > 30) ++darkened;   // was lit, now shadowed
        if (offG > 40 && onG > 40) ++litBoth;
    }
    const size_t total = static_cast<size_t>(W) * H;
    std::printf("shadow: darkened=%zu litBoth=%zu / %zu pixels\n", darkened, litBoth, total);

    TST_REQUIRE_MSG(darkened > 500, "no shadow patch appeared under the caster");
    TST_REQUIRE_MSG(darkened < total / 2, "shadow darkened too much (should be a local patch)");
    TST_REQUIRE_MSG(litBoth > 1000, "scene should still have lit ground outside the shadow");

    device.destroy(shadowSamp);
    std::printf("shadow map ok\n");
}
