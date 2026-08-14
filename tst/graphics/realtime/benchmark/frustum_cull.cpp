#include "harness/harness.h"
//
//  frustum_cull.cpp (graphics benchmark)
//  engine::tst
//
//  Two views of frustum culling:
//    (1) cullToFrustum CPU throughput (instances/s) + kept ratio.
//    (2) END-TO-END render frame time WITH vs WITHOUT culling — same scene, camera sees a small
//        fraction. "Without" submits all N instances (full per-frame upload + GPU draw); "with"
//        culls each frame then submits only the survivors (cull cost included). Shows the real win.
//  Headless endFrame waits for GPU completion. Relative before/after baseline on the SAME machine.
//

#include <array>
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

#include "engine/core/geometry/bounds.h"
#include "engine/core/geometry/primitives.h"
#include "engine/core/math/bounds.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
#include "engine/graphics/view/render_view.h"
#include "engine/scene/extract.h"

using Clock = std::chrono::steady_clock;
namespace {
double ms(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }
std::vector<std::byte> readBin(const std::string& p) {
    std::ifstream f(p, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto n = static_cast<std::streamsize>(f.tellg()); f.seekg(0);
    std::vector<std::byte> d(static_cast<size_t>(n)); f.read(reinterpret_cast<char*>(d.data()), n); return d;
}
// Deterministic scatter of N unit-box instances across a ±half box centered on the origin.
std::vector<engine::render::InstanceData> scatter(uint32_t n, float half) {
    using namespace engine;
    std::vector<render::InstanceData> v; v.reserve(n);
    uint32_t s = 12345u;
    auto rnd = [&]() { s = s * 1664525u + 1013904223u; return (s >> 8) / float(1u << 24); };
    for (uint32_t i = 0; i < n; ++i) {
        const glm::vec3 p{ (rnd() - 0.5f) * 2 * half, (rnd() - 0.5f) * 2 * half, (rnd() - 0.5f) * 2 * half };
        render::InstanceData d; d.model = glm::translate(glm::mat4(1.0f), p);
        d.normalModel = d.model; d.materialIndex = 0;
        v.push_back(d);
    }
    return v;
}
} // namespace

TST_CASE(graphics, benchmark, frustum_cull) {
    using namespace engine;
    using namespace engine::rhi;
#if defined(NDEBUG)
    std::printf("[build: optimized]\n");
#else
    std::printf("[build: DEBUG — timings not representative]\n");
#endif

    // Camera at the origin looking down -Z; sees a forward cone (far 200).
    render::RenderView cam;
    cam.view = glm::lookAt(glm::vec3(0, 0, 0), glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
    cam.proj = glm::perspective(glm::radians(60.0f), 1.0f, 0.1f, 200.0f);
    const core::Frustum frustum = scene::viewFrustum(cam);
    const std::array<core::Aabb, 1> unitBounds{ core::Aabb{ glm::vec3(-0.5f), glm::vec3(0.5f) } };

    // --- (1) CPU cull throughput ---------------------------------------------------------------
    std::printf("\nscene::cullToFrustum throughput (instances spread over a ±400 box):\n");
    for (uint32_t n : {10000u, 100000u, 500000u}) {
        scene::ExtractedScene full;
        full.instances = scatter(n, 400.0f);
        full.items.push_back(render::RenderItem{ render::MeshHandle{0,0}, 0, n });
        scene::ExtractedScene culled;
        double best = 1e30; std::size_t kept = 0;
        for (int pass = 0; pass < 5; ++pass) {
            const auto t0 = Clock::now();
            scene::cullToFrustum(frustum, full, std::span<const core::Aabb>(unitBounds), culled);
            best = std::min(best, ms(Clock::now() - t0));
            kept = culled.instances.size();
        }
        std::printf("  %7u inst: cull %7.3f ms  (%.0f M inst/s)  kept %zu (%.1f%%)\n",
                    n, best, (n / best) / 1000.0, kept, 100.0 * double(kept) / n);
    }

    // --- (2) End-to-end render frame time: WITHOUT vs WITH culling ------------------------------
    constexpr uint32_t W = 512, H = 512;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    if (blob.empty()) { TST_REQUIRE_MSG(false, "read mesh.metallib"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    render::GeometryStore geometry(device);
    render::MeshHandle box    = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));      // ~12 tris
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 24, 48));      // ~2.3K tris
    render::Renderer renderer(device, geometry);
    renderer.setMeshPipeline(renderer.createMeshPipeline({ .vertex = vs, .fragment = fs }));

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);
    render::MaterialGPU mat; mat.baseColorFactor = glm::vec4(0.8f);

    auto timeFrames = [&](auto perFrameSubmit) {
        const int frames = 30;
        double frameMs = 0.0, cpuMs = 0.0;
        for (int f = 0; f < frames; ++f) {
            const auto t0 = Clock::now();
            std::span<const render::RenderItem>   items;
            std::span<const render::InstanceData> insts;
            const auto c0 = Clock::now();
            perFrameSubmit(items, insts);           // may cull (CPU work counted below)
            cpuMs += ms(Clock::now() - c0);
            render::RenderView v = cam;
            v.target = colorRT; v.width = W; v.height = H;
            v.light.intensity = 0.0f; v.light.ambient = glm::vec3(1.0f);
            v.items = items; v.instances = insts;
            v.materials = std::span<const render::MaterialGPU>(&mat, 1);
            FrameContext fr = device.beginFrame();
            renderer.render(fr, std::span<const render::RenderView>(&v, 1));
            device.endFrame(std::move(fr));
            frameMs += ms(Clock::now() - t0);
        }
        return std::pair{ frameMs / frames, cpuMs / frames };
    };

    auto compare = [&](render::MeshHandle mesh, const char* label, uint32_t n) {
        scene::ExtractedScene full;
        full.instances = scatter(n, 400.0f);
        full.items.push_back(render::RenderItem{ mesh, 0, n });
        // WITHOUT: submit all N every frame.
        auto [noCullFrame, noCullCpu] = timeFrames(
            [&](std::span<const render::RenderItem>& items, std::span<const render::InstanceData>& insts) {
                items = std::span<const render::RenderItem>(full.items);
                insts = std::span<const render::InstanceData>(full.instances);
            });
        // WITH: cull each frame, submit survivors (cull cost included).
        scene::ExtractedScene culled;
        std::size_t kept = 0;
        auto [cullFrame, cullCpu] = timeFrames(
            [&](std::span<const render::RenderItem>& items, std::span<const render::InstanceData>& insts) {
                scene::cullToFrustum(frustum, full, std::span<const core::Aabb>(unitBounds), culled);
                kept = culled.instances.size();
                items = std::span<const render::RenderItem>(culled.items);
                insts = std::span<const render::InstanceData>(culled.instances);
            });
        std::printf("  %-14s %7u inst: WITHOUT %7.3f ms | WITH %7.3f ms (cull %.3f) | kept %zu (%.1f%%)  %.1fx\n",
                    label, n, noCullFrame, cullFrame, cullCpu, kept, 100.0 * double(kept) / n,
                    noCullFrame / cullFrame);
    };

    std::printf("\nend-to-end frame time (512x512), WITHOUT vs WITH frustum culling:\n");
    compare(box,    "box ~12t",    50000);
    compare(box,    "box ~12t",    200000);
    compare(sphere, "sphere ~2.3Kt", 20000);
    compare(sphere, "sphere ~2.3Kt", 50000);

    std::printf("\nfrustum cull benchmark ok\n");
}
