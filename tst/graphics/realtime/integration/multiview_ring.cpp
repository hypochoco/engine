#include "harness/harness.h"
//
//  multiview_ring.cpp
//  engine::tst
//
//  RF2 regression test for the FrameRingAllocator + per-view sub-allocation (render-framework
//  note §1). Renders TWO views in ONE frame into two separate offscreen targets, each with
//  distinct content (a red sphere vs a blue sphere). Before the fix, both views shared one
//  instance/material/camera buffer, so the second view's data clobbered the first and BOTH
//  targets showed the last view's content. We assert each target keeps its OWN content.
//
//  Then it repeats for several frames (> framesInFlight) to exercise the per-frame arena
//  cycling + the Device frames-in-flight throttle, checking correctness on the final frame.
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

TST_CASE(graphics, integration, multiview_ring) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 128, H = 128;

    // framesInFlight = 2 (default). We render 3 frames so the ring arenas cycle 0,1,0 and the
    // throttle semaphore is exercised.
    Device device = Device::createHeadless({ .framesInFlight = 2 });

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFileBin(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup/verification failed"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = render::coreVertexLayout();
    pdesc.topology = Topology::TriangleList;
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    TST_REQUIRE_MSG(pipe.valid(), "pipeline creation failed");

    // Two offscreen targets, one per view.
    TextureHandle texA = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    TextureHandle texB = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle rtA = device.createRenderTarget(texA);
    RenderTargetHandle rtB = device.createRenderTarget(texB);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 24, 48));
    render::Renderer renderer(device, geometry);

    // View A = red sphere; View B = blue sphere. Distinct per-view materials + instances so a
    // clobber would show up as the wrong color in a target.
    render::MaterialGPU matA; matA.baseColorFactor = {1.0f, 0.15f, 0.15f, 1.0f};   // red
    render::MaterialGPU matB; matB.baseColorFactor = {0.15f, 0.15f, 1.0f, 1.0f};   // blue

    render::InstanceData instA; instA.model = glm::mat4(1.0f); instA.normalModel = glm::mat4(1.0f); instA.materialIndex = 0;
    render::InstanceData instB = instA;

    render::RenderItem itemA{ sphere, 0, 1 };
    render::RenderItem itemB{ sphere, 0, 1 };
    renderer.setMeshPipeline(pipe);

    auto makeView = [&](RenderTargetHandle target, const render::RenderItem& item,
                        const render::InstanceData& inst, const render::MaterialGPU& mat) {
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));
        v.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 20.0f);
        v.target = target; v.width = W; v.height = H;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&mat, 1);
        return v;
    };

    int lastAr = 0, lastAb = 0, lastBr = 0, lastBb = 0;
    constexpr int FRAMES = 3;
    for (int f = 0; f < FRAMES; ++f) {
        render::RenderView views[2] = {
            makeView(rtA, itemA, instA, matA),
            makeView(rtB, itemB, instB, matB),
        };
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(views, 2));
        device.endFrame(std::move(frame));

        std::vector<uint8_t> pa(static_cast<size_t>(W) * H * 4), pb(static_cast<size_t>(W) * H * 4);
        device.readback(texA, std::as_writable_bytes(std::span<uint8_t>(pa)));
        device.readback(texB, std::as_writable_bytes(std::span<uint8_t>(pb)));
        const size_t c = (static_cast<size_t>(H / 2) * W + (W / 2)) * 4;
        lastAr = pa[c + 0]; lastAb = pa[c + 2];
        lastBr = pb[c + 0]; lastBb = pb[c + 2];
    }

    std::printf("view A center r=%d b=%d ; view B center r=%d b=%d (after %d frames)\n",
                lastAr, lastAb, lastBr, lastBb, FRAMES);

    // Target A must be red-dominant (its own content), target B blue-dominant. A clobber would
    // make A look like B (blue-dominant) or vice versa.
    const bool aIsRed  = lastAr > lastAb + 40;
    const bool bIsBlue = lastBb > lastBr + 40;
    TST_REQUIRE_MSG(aIsRed,  "view A target was clobbered (not red-dominant)");
    TST_REQUIRE_MSG(bIsBlue, "view B target was clobbered (not blue-dominant)");

    std::printf("multiview ring ok\n");
}
