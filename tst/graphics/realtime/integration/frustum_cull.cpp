#include "harness/harness.h"
//
//  frustum_cull.cpp
//  engine::tst — graphics / integration
//
//  scene::cullToFrustum — keeps only instances whose world AABB intersects the view frustum, and a
//  RenderView pointed at the culled scene renders the same visible pixels as one pointed at the full
//  scene (culling only drops off-screen instances). Also checks the no-bounds path is a passthrough.
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

#include "engine/core/geometry/bounds.h"
#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
#include "engine/scene/extract.h"

using namespace engine;
using namespace engine::rhi;

namespace {
std::vector<std::byte> readBin(const std::string& p) {
    std::ifstream f(p, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto n = static_cast<std::streamsize>(f.tellg()); f.seekg(0);
    std::vector<std::byte> d(static_cast<size_t>(n)); f.read(reinterpret_cast<char*>(d.data()), n); return d;
}
} // namespace

TST_CASE(graphics, integration, frustum_cull) {
    // --- Build a scene: one visible instance at the origin + several off-screen ones. ------------
    const MeshData meshData = primitives::makeSphere(0.5f, 16, 32);
    const core::Aabb localBounds = computeBounds(meshData);

    scene::ExtractedScene full;
    const glm::vec3 positions[] = {
        {   0,   0,   0},   // visible (in front of the camera)
        { 100,   0,   0},   // far right → culled
        {-100,   0,   0},   // far left  → culled
        {   0,   0,  50},   // behind the camera → culled
        {   0, 200,   0},   // far above → culled
    };
    for (const glm::vec3& p : positions) {
        render::InstanceData d;
        d.model = glm::translate(glm::mat4(1.0f), p);
        d.normalModel = d.model;
        d.materialIndex = 0;
        full.instances.push_back(d);
    }
    render::RenderItem item{ render::MeshHandle{0, 0}, 0, static_cast<uint32_t>(full.instances.size()) };
    full.items.push_back(item);
    const std::array<core::Aabb, 1> itemBounds{ localBounds };

    // Camera at (0,0,5) looking toward the origin.
    render::RenderView probe;
    probe.view = glm::lookAt(glm::vec3(0, 0, 5), glm::vec3(0), glm::vec3(0, 1, 0));
    probe.proj = glm::perspective(glm::radians(50.0f), 1.0f, 0.1f, 40.0f);
    const core::Frustum frustum = scene::viewFrustum(probe);

    scene::ExtractedScene culled;
    scene::cullToFrustum(frustum, full, std::span<const core::Aabb>(itemBounds), culled);
    std::size_t keptInstances = 0;
    for (const auto& it : culled.items) keptInstances += it.instanceCount;
    std::printf("frustum cull: %zu/%zu instances kept, %zu items\n",
                keptInstances, full.instances.size(), culled.items.size());
    TST_REQUIRE_MSG(keptInstances == 1, "only the origin instance should survive the frustum");

    // No-bounds path is a straight passthrough.
    scene::ExtractedScene passthrough;
    scene::cullToFrustum(frustum, full, std::span<const core::Aabb>(), passthrough);
    TST_REQUIRE_MSG(passthrough.instances.size() == full.instances.size(), "empty bounds ⇒ no culling");

    // --- Render equivalence: full vs culled must produce identical pixels. -----------------------
    constexpr uint32_t W = 96, H = 96;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(meshData);   // MeshHandle{0,0} — matches item.mesh
    render::Renderer renderer(device, geometry);
    renderer.setMeshPipeline(renderer.createMeshPipeline({ .vertex = vs, .fragment = fs }));

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);
    render::MaterialGPU mat; mat.baseColorFactor = glm::vec4(0.9f, 0.3f, 0.2f, 1.0f);

    auto renderScene = [&](const scene::ExtractedScene& s) {
        render::RenderView v = probe;
        v.target = colorRT; v.width = W; v.height = H;
        v.light.intensity = 0.0f; v.light.ambient = glm::vec3(1.0f);
        v.items = std::span<const render::RenderItem>(s.items);
        v.instances = std::span<const render::InstanceData>(s.instances);
        v.materials = std::span<const render::MaterialGPU>(&mat, 1);
        FrameContext fr = device.beginFrame();
        renderer.render(fr, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(fr));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        return px;
    };
    const auto pxFull   = renderScene(full);
    const auto pxCulled = renderScene(culled);
    int diffs = 0;
    for (size_t i = 0; i < pxFull.size(); i += 4)
        if (pxFull[i] != pxCulled[i] || pxFull[i+1] != pxCulled[i+1] || pxFull[i+2] != pxCulled[i+2]) ++diffs;
    std::printf("frustum cull: full-vs-culled pixel diffs = %d (expect 0)\n", diffs);
    TST_REQUIRE_MSG(diffs == 0, "culling only off-screen instances must not change the rendered image");

    std::printf("frustum cull ok\n");
}
