#include "harness/harness.h"
//
//  clustered_forward.cpp
//  engine::tst
//
//  RF4b end-to-end: clustered forward+ shading must match the loop-all reference. Renders the
//  same multi-light scene twice through the Renderer — once with clustering off (loop every
//  light per fragment) and once with clustering on (froxel binning compute pass + per-cluster
//  light loop) — and asserts the two images are near-identical (mean abs diff small). Binning is
//  conservative, so every light that affects a fragment is in that fragment's cluster ⇒ the
//  clustered image should equal the reference up to rounding.
//

#include <cmath>
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

TST_CASE(graphics, integration, clustered_forward) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    auto clusBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/cluster.metallib");
    if (meshBlob.empty() || clusBlob.empty()) { std::printf("FAIL: read shaders\n"); TST_REQUIRE_MSG(false, "setup failed"); }

    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle cs = device.createShader(clusBlob, ShaderStage::Compute);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = render::coreVertexLayout();
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    PipelineHandle clusterPipe = device.createComputePipeline({ .compute = cs });
    TST_REQUIRE_MSG(pipe.valid() && clusterPipe.valid(), "pipeline creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.9f, 32, 64));
    render::Renderer renderer(device, geometry);

    render::MaterialGPU white; white.baseColorFactor = {0.9f, 0.9f, 0.9f, 1.0f};
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
    render::RenderItem item{ sphere, pipe, 0, 1 };

    // Several point lights around the sphere so shading varies across the surface (exercises many
    // froxels in x/y/z — a mis-mapped cluster lookup would diverge from the reference).
    std::vector<render::PointLight> lights = {
        { { 1.3f, 0.0f, 1.0f}, 4.0f, {1.0f, 0.3f, 0.3f}, 3.0f },
        { {-1.3f, 0.0f, 1.0f}, 4.0f, {0.3f, 1.0f, 0.3f}, 3.0f },
        { { 0.0f, 1.3f, 1.0f}, 4.0f, {0.3f, 0.3f, 1.0f}, 3.0f },
        { { 0.0f, 0.0f, 1.6f}, 4.0f, {1.0f, 1.0f, 1.0f}, 2.0f },
    };

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 20.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.intensity = 0.0f; view.light.ambient = glm::vec3(0.04f);
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(&inst, 1);
    view.materials = std::span<const render::MaterialGPU>(&white, 1);
    view.pointLights = std::span<const render::PointLight>(lights);

    auto renderTo = [&](std::vector<uint8_t>& out) {
        out.assign(static_cast<size_t>(W) * H * 4, 0);
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(out)));
    };

    // (1) reference: loop-all (clustering disabled).
    renderer.setClusterBinning({});
    std::vector<uint8_t> ref;  renderTo(ref);

    // (2) clustered.
    renderer.setClusterBinning(clusterPipe);
    std::vector<uint8_t> clus; renderTo(clus);

    // Mean absolute difference over all channels + max diff.
    double sum = 0; int maxd = 0; size_t lit = 0;
    for (size_t i = 0; i < ref.size(); ++i) {
        int d = std::abs(int(ref[i]) - int(clus[i]));
        sum += d; if (d > maxd) maxd = d;
        if ((i % 4) == 0 && ref[i] > 20) ++lit;   // count clearly-lit red-channel pixels
    }
    double mad = sum / ref.size();
    const size_t cc = (static_cast<size_t>(H / 2) * W + (W / 2)) * 4;
    std::printf("clustered vs loop-all: MAD=%.4f maxDiff=%d litPixels=%zu | ref center rgb=%d,%d,%d clus=%d,%d,%d\n",
                mad, maxd, lit, ref[cc], ref[cc+1], ref[cc+2], clus[cc], clus[cc+1], clus[cc+2]);

    TST_REQUIRE_MSG(lit > 1000, "reference scene is not meaningfully lit");
    TST_REQUIRE_MSG(mad < 1.0, "clustered image diverges from loop-all reference");
    TST_REQUIRE_MSG(maxd <= 16, "clustered image has large per-pixel divergence from reference");

    std::printf("clustered forward+ ok\n");
}
