#include "harness/harness.h"
//
//  dynamic_resolution.cpp
//  engine::tst — graphics / integration
//
//  GraphicsConfig::renderScale (dynamic resolution). With FXAA enabled, the scene renders into a
//  renderScale× intermediate and the FXAA pass upscales it to the full-res view target. This test
//  renders a full-screen lit quad at renderScale 1.0 and 0.5 into the SAME full-size target and
//  checks that BOTH completely fill it (center + spread points bright) — i.e. the 0.5 render is
//  upscaled to cover every pixel, not left as a quarter-image or garbage. Proves the scaled-scene →
//  full-res-upscale path is wired correctly.
//

#include <array>
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

#include "engine/core/geometry/mesh.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"

using namespace engine;
using namespace engine::rhi;

namespace {
std::vector<std::byte> readBin(const std::string& p) {
    std::ifstream f(p, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto n = static_cast<std::streamsize>(f.tellg()); f.seekg(0);
    std::vector<std::byte> d(static_cast<size_t>(n)); f.read(reinterpret_cast<char*>(d.data()), n); return d;
}
MeshData bigQuad() {   // covers the whole viewport at the camera below
    MeshData m;
    auto v = [](float x, float y) { Vertex t; t.position={x,y,0.0f}; t.normal={0,0,1}; t.uv={0,0}; t.color={1,1,1}; return t; };
    m.vertices = { v(-10,-10), v(10,-10), v(10,10), v(-10,10) };
    m.indices = { 0,1,2, 0,2,3 };
    return m;
}
} // namespace

TST_CASE(graphics, integration, dynamic_resolution) {
    constexpr uint32_t W = 128, H = 128;
    Device device = Device::createHeadless({});
    const auto mesh = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    const auto fxaa = readBin(std::string(ENGINE_SHADER_DIR) + "/fxaa.metallib");
    TST_REQUIRE_MSG(!mesh.empty() && !fxaa.empty(), "read shaders");
    ShaderHandle vs = device.createShader(mesh, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(mesh, ShaderStage::Fragment);
    ShaderHandle fxv = device.createShader(fxaa, ShaderStage::Vertex);
    ShaderHandle fxf = device.createShader(fxaa, ShaderStage::Fragment);

    const Format fmt = Format::RGBA8Unorm;
    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = fmt, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle quad = geometry.upload(bigQuad());
    render::Renderer renderer(device, geometry);

    // Mesh pipeline must target the FXAA intermediate format (RGBA8) since FXAA is on.
    PipelineHandle meshPipe = renderer.createMeshPipeline({ .vertex = vs, .fragment = fs }, fmt);
    GraphicsPipelineDesc fxd;
    fxd.vertex = fxv; fxd.fragment = fxf;
    fxd.colorFormats = std::span<const Format>(&fmt, 1);
    fxd.depthFormat = Format::Undefined;
    fxd.depth = { .test = false, .write = false, .op = CompareOp::Always };
    fxd.raster.cull = CullMode::None;
    PipelineHandle fxaaPipe = device.createGraphicsPipeline(fxd);
    SamplerHandle fxaaSamp = device.createSampler({ .minFilter = Filter::Linear, .magFilter = Filter::Linear,
                                                    .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });
    render::RenderResources res; res.mesh = meshPipe; renderer.setResources(res);
    renderer.setFXAA(fxaaPipe, fxaaSamp);

    render::DirectionalLight lit; lit.intensity = 0.0f; lit.ambient = glm::vec3(1.0f);   // flat white
    render::MaterialGPU mat; mat.baseColorFactor = glm::vec4(0.2f, 0.9f, 0.3f, 1.0f);
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f);
    render::RenderItem item{ quad, 0, 1 };

    auto run = [&](float scale) {
        renderer.setRenderScale(scale);
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0,0,3), glm::vec3(0), glm::vec3(0,1,0));
        v.proj = glm::perspective(glm::radians(60.0f), float(W)/float(H), 0.1f, 50.0f);
        v.target = colorRT; v.width = W; v.height = H; v.light = lit;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&mat, 1);
        FrameContext fr = device.beginFrame();
        renderer.render(fr, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(fr));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        return px;
    };
    auto green = [&](const std::vector<uint8_t>& px, uint32_t x, uint32_t y) {
        return int(px[(static_cast<size_t>(y) * W + x) * 4 + 1]);
    };

    const auto full = run(1.0f);
    const auto half = run(0.5f);

    // Both must fill the FULL target: sample a spread of points (the quad covers the whole viewport).
    const std::array<std::pair<uint32_t,uint32_t>,5> pts{{ {W/2,H/2},{W/4,H/2},{3*W/4,H/2},{W/2,H/4},{W/2,3*H/4} }};
    for (auto [x,y] : pts) {
        TST_REQUIRE_MSG(green(full, x, y) > 180, "renderScale=1.0 should fill the target");
        TST_REQUIRE_MSG(green(half, x, y) > 180, "renderScale=0.5 must UPSCALE to fill the full-res target");
    }
    std::printf("dynamic_resolution: full+0.5x both fill %ux%u (center green full=%d half=%d)\n",
                W, H, green(full, W/2, H/2), green(half, W/2, H/2));
    renderer.setRenderScale(1.0f);
    device.destroy(fxaaSamp);
    std::printf("dynamic resolution ok\n");
}
