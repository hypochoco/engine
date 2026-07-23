#include "harness/harness.h"
//
//  pipeline_variants.cpp
//  engine::tst — graphics / integration
//
//  Per-item pipeline selection + the Renderer::createMeshPipeline factory. A near GREEN quad is
//  drawn first (writes depth); a far RED quad is drawn second. With the default pipeline (depth
//  test Less) the far quad is occluded ⇒ GREEN. Routing the far item to a factory-built variant
//  with depthCompare = Always makes it overwrite ⇒ RED. The ONLY difference between the two runs is
//  which pipeline the far item uses, so this proves (a) the factory builds working forward-pass
//  pipelines from just shaders + knobs, and (b) RenderItem.pipeline selects a per-item pipeline.
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
// A +Z-facing quad at the given z, covering the view.
MeshData quadAt(float z) {
    MeshData m;
    auto v = [z](float x, float y) { Vertex t; t.position={x,y,z}; t.normal={0,0,1}; t.uv={0,0}; t.color={1,1,1}; return t; };
    m.vertices = { v(-1,-1), v(1,-1), v(1,1), v(-1,1) };
    m.indices = { 0,1,2, 0,2,3 };
    return m;
}
} // namespace

TST_CASE(graphics, integration, pipeline_variants) {
    constexpr uint32_t W = 96, H = 96;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle nearQuad = geometry.upload(quadAt(0.5f));   // closer to a +Z camera
    render::MeshHandle farQuad  = geometry.upload(quadAt(0.0f));   // farther
    render::Renderer renderer(device, geometry);

    // Both pipelines via the factory: default (depth Less) and an overlay variant (depth Always).
    PipelineHandle defaultPipe = renderer.createMeshPipeline({ .vertex = vs, .fragment = fs });
    PipelineHandle overlayPipe = renderer.createMeshPipeline(
        { .vertex = vs, .fragment = fs, .depthWrite = false, .depthCompare = CompareOp::Always });
    TST_REQUIRE_MSG(defaultPipe.valid() && overlayPipe.valid(), "factory pipeline creation failed");
    render::RenderResources res; res.mesh = defaultPipe; renderer.setResources(res);

    render::DirectionalLight lit; lit.intensity = 0.0f; lit.ambient = glm::vec3(1.0f);   // flat lit
    render::MaterialGPU green; green.baseColorFactor = glm::vec4(0, 1, 0, 1);
    render::MaterialGPU red;   red.baseColorFactor   = glm::vec4(1, 0, 0, 1);
    std::array<render::MaterialGPU, 2> mats{ green, red };
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f);

    auto runCenter = [&](PipelineHandle farItemPipe) {
        // instance 0 (green) → near quad; instance 1 (red) → far quad.
        std::array<render::InstanceData, 2> insts{ inst, inst };
        insts[0].materialIndex = 0; insts[1].materialIndex = 1;
        std::array<render::RenderItem, 2> items{
            render::RenderItem{ nearQuad, 0, 1, {} },            // green, near, default pipeline
            render::RenderItem{ farQuad,  1, 1, farItemPipe },   // red, far, pipeline under test
        };
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0,0,3), glm::vec3(0), glm::vec3(0,1,0));
        v.proj = glm::perspective(glm::radians(50.0f), float(W)/float(H), 0.1f, 50.0f);
        v.target = colorRT; v.width = W; v.height = H; v.light = lit;
        v.items = std::span<const render::RenderItem>(items.data(), items.size());
        v.instances = std::span<const render::InstanceData>(insts.data(), insts.size());
        v.materials = std::span<const render::MaterialGPU>(mats.data(), mats.size());
        FrameContext fr = device.beginFrame();
        renderer.render(fr, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(fr));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        const size_t c = (static_cast<size_t>(H/2)*W + W/2) * 4;
        return std::array<int,3>{ px[c], px[c+1], px[c+2] };
    };

    const auto def = runCenter({});           // far item uses default pipeline (depth Less) ⇒ occluded ⇒ green
    const auto ovl = runCenter(overlayPipe);   // far item uses overlay variant (depth Always) ⇒ overwrites ⇒ red
    std::printf("far-item default-pipe rgb = %d %d %d   overlay-variant rgb = %d %d %d\n",
                def[0], def[1], def[2], ovl[0], ovl[1], ovl[2]);
    TST_REQUIRE_MSG(def[1] > 200 && def[0] < 60, "far item on the default pipeline should be depth-occluded (green shows)");
    TST_REQUIRE_MSG(ovl[0] > 200 && ovl[1] < 60, "far item routed to the overlay variant should overwrite (red shows)");
    std::printf("pipeline variants ok\n");
}
