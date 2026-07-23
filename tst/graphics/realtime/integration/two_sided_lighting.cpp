#include "harness/harness.h"
//
//  two_sided_lighting.cpp
//  engine::tst — graphics / integration
//
//  MaterialFlagDoubleSided (two-sided lighting): with a CullMode::None pipeline (built via the
//  engine's Renderer::createMeshPipeline factory — no backend code, no hand-built pipeline desc), a
//  back-facing surface lit from the viewer's side is dark by default but lit when the double-sided
//  flag flips the normal to face the viewer. Also confirms the factory produces a working pipeline.
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
MeshData quadXY() {
    MeshData m;
    auto v = [](glm::vec3 p) { Vertex x; x.position=p; x.normal={0,0,1}; x.uv={0,0}; x.color={1,1,1}; return x; };
    m.vertices = { v({-1,-1,0}), v({1,-1,0}), v({1,1,0}), v({-1,1,0}) };
    m.indices = { 0,1,2, 0,2,3 };
    return m;
}
} // namespace

TST_CASE(graphics, integration, two_sided_lighting) {
    constexpr uint32_t W = 128, H = 128;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle quad = geometry.upload(quadXY());
    render::Renderer renderer(device, geometry);

    // Build the (double-sided) pipeline via the engine factory — the app supplies only shaders + cull.
    PipelineHandle cullNone = renderer.createMeshPipeline(
        { .vertex = vs, .fragment = fs, .cull = CullMode::None, .debugName = "foliage" });
    TST_REQUIRE_MSG(cullNone.valid(), "createMeshPipeline factory failed");
    render::RenderResources res; res.mesh = cullNone; renderer.setResources(res);

    // View the CCW-outward +Z quad's BACK (-Z) side from -Z; sun also on the -Z side (light travels
    // +Z). The geometric +Z normal points away ⇒ the back face is dark by default; the double-sided
    // flip turns the normal toward the viewer/light ⇒ it lights up.
    render::DirectionalLight light; light.intensity = 1.0f; light.color = glm::vec3(1.0f);
    light.ambient = glm::vec3(0.0f); light.direction = glm::vec3(0, 0, 1);

    auto centerLuma = [&](const render::MaterialGPU& m) {
        render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
        render::RenderItem item{ quad, 0, 1 };
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0,0,-3), glm::vec3(0), glm::vec3(0,1,0));
        v.proj = glm::perspective(glm::radians(50.0f), float(W)/float(H), 0.1f, 50.0f);
        v.target = colorRT; v.width = W; v.height = H; v.light = light;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&m, 1);
        FrameContext fr = device.beginFrame();
        renderer.render(fr, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(fr));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        const size_t c = (static_cast<size_t>(H/2)*W + W/2) * 4;
        return int(px[c]) + int(px[c+1]) + int(px[c+2]);
    };

    render::MaterialGPU oneSided; oneSided.baseColorFactor = glm::vec4(1.0f); oneSided.flags = render::MaterialFlagNone;
    render::MaterialGPU twoSided = oneSided; twoSided.flags = render::MaterialFlagDoubleSided;
    const int oneL = centerLuma(oneSided), twoL = centerLuma(twoSided);
    std::printf("two-sided lighting: back face one-sided=%d double-sided=%d\n", oneL, twoL);
    TST_REQUIRE_MSG(twoL > oneL + 120, "double-sided flag should light the back face (normal flip)");
    std::printf("two-sided lighting ok\n");
}
