#include "harness/harness.h"
//
//  backface_cull.cpp
//  engine::tst — graphics / integration
//
//  Proves back-face culling actually works now (RasterState.cull applied in the backend with the
//  correct winding). A CCW-outward quad facing the camera is drawn under a cull=Back pipeline; the
//  same quad rotated 180° (now facing away) is culled ⇒ background. A cull=None pipeline draws both.
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
// CCW-outward quad, normal +Z (matches the engine convention).
MeshData quadXY() {
    MeshData m;
    auto v = [](glm::vec3 p) { Vertex x; x.position=p; x.normal={0,0,1}; x.uv={0,0}; x.color={1,1,1}; return x; };
    m.vertices = { v({-1,-1,0}), v({1,-1,0}), v({1,1,0}), v({-1,1,0}) };
    m.indices = { 0,1,2, 0,2,3 };
    return m;
}
} // namespace

TST_CASE(graphics, integration, backface_cull) {
    constexpr uint32_t W = 96, H = 96;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    render::GeometryStore geometry(device);
    render::MeshHandle quad = geometry.upload(quadXY());
    render::Renderer renderer(device, geometry);
    PipelineHandle cullBack = renderer.createMeshPipeline({ .vertex = vs, .fragment = fs, .cull = CullMode::Back });
    PipelineHandle cullNone = renderer.createMeshPipeline({ .vertex = vs, .fragment = fs, .cull = CullMode::None });
    TST_REQUIRE_MSG(cullBack.valid() && cullNone.valid(), "pipeline creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);
    render::MaterialGPU white; white.baseColorFactor = glm::vec4(1.0f);

    auto centerLuma = [&](const glm::mat4& model, PipelineHandle pipe) {
        renderer.setMeshPipeline(pipe);
        render::InstanceData inst; inst.model = model; inst.normalModel = model; inst.materialIndex = 0;
        render::RenderItem item{ quad, 0, 1 };
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0,0,3), glm::vec3(0), glm::vec3(0,1,0));   // camera at +Z, looks -Z
        v.proj = glm::perspective(glm::radians(50.0f), float(W)/float(H), 0.1f, 50.0f);
        v.target = colorRT; v.width = W; v.height = H;
        v.light.intensity = 0.0f; v.light.ambient = glm::vec3(1.0f);   // flat lit
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&white, 1);
        FrameContext fr = device.beginFrame();
        renderer.render(fr, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(fr));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        const size_t c = (static_cast<size_t>(H/2)*W + W/2) * 4;
        return int(px[c]) + int(px[c+1]) + int(px[c+2]);
    };

    const glm::mat4 facing = glm::mat4(1.0f);                                         // +Z normal toward camera → front
    const glm::mat4 away   = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0,1,0));  // normal → -Z → back

    const int frontBack = centerLuma(facing, cullBack);   // front-facing under cull=Back → drawn
    const int backBack  = centerLuma(away,   cullBack);   // back-facing under cull=Back → culled
    const int backNone  = centerLuma(away,   cullNone);   // back-facing under cull=None → drawn
    std::printf("backface cull: front(cullBack)=%d  back(cullBack)=%d  back(cullNone)=%d\n",
                frontBack, backBack, backNone);

    TST_REQUIRE_MSG(frontBack > 200, "front-facing quad should be drawn under cull=Back");
    TST_REQUIRE_MSG(backBack  < 120, "back-facing quad should be CULLED under cull=Back (≈ clear color)");
    TST_REQUIRE_MSG(backNone  > 200, "back-facing quad should still draw under cull=None");
    std::printf("backface cull ok\n");
}
