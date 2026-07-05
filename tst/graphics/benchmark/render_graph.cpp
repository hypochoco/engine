#include "harness/harness.h"
//
//  render_graph.cpp (graphics benchmark)
//  engine::tst
//
//  Performance baseline for the render-framework path: the RenderGraph + FrameRingAllocator +
//  instanced forward+ draw, and the multi-light shading loop. Reports:
//    - frame time vs instance count (per-frame ring upload + one instanced drawIndexed),
//    - frame time vs point-light count (the fragment light loop).
//  Headless endFrame waits for GPU completion, so "frame ms" is CPU-record + GPU; "record ms"
//  times render() alone (CPU encode). Absolute numbers are hardware/thermal/build dependent —
//  a relative baseline for before/after on the SAME machine (build type printed).
//

#include <chrono>
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

using Clock = std::chrono::steady_clock;

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
double ms(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }
}

TST_CASE(graphics, benchmark, render_graph) {
    using namespace engine;
    using namespace engine::rhi;

#if defined(NDEBUG)
    std::printf("[build: optimized]\n");
#else
    std::printf("[build: DEBUG — timings not representative]\n");
#endif

    constexpr uint32_t W = 512, H = 512;
    Device device = Device::createHeadless({});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFileBin(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup failed"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = render::coreVertexLayout();
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    TST_REQUIRE_MSG(pipe.valid(), "pipeline creation failed");

    const auto clusBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/cluster.metallib");
    ShaderHandle cs = device.createShader(clusBlob, ShaderStage::Compute);
    PipelineHandle clusterPipe = device.createComputePipeline({ .compute = cs });
    TST_REQUIRE_MSG(clusterPipe.valid(), "cluster pipeline creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.4f, 12, 24));
    render::Renderer renderer(device, geometry);
    render::MaterialGPU mat; mat.baseColorFactor = {0.8f, 0.8f, 0.85f, 1.0f};

    auto buildInstances = [](uint32_t n) {
        std::vector<render::InstanceData> v(n);
        const int side = std::max(1, (int)std::ceil(std::cbrt((double)n)));
        for (uint32_t i = 0; i < n; ++i) {
            int x = i % side, y = (i / side) % side, z = i / (side * side);
            glm::vec3 p = glm::vec3(x - side * 0.5f, y - side * 0.5f, z - side * 0.5f) * 1.2f;
            v[i].model = glm::translate(glm::mat4(1.0f), p);
            v[i].normalModel = v[i].model;
            v[i].materialIndex = 0;
        }
        return v;
    };

    auto runFrames = [&](const render::RenderView& view, int frames) {
        double recordTotal = 0, frameTotal = 0;
        for (int f = 0; f < frames; ++f) {
            auto t0 = Clock::now();
            FrameContext frame = device.beginFrame();
            renderer.render(frame, std::span<const render::RenderView>(&view, 1));
            auto t1 = Clock::now();
            device.endFrame(std::move(frame));
            auto t2 = Clock::now();
            recordTotal += ms(t1 - t0);
            frameTotal  += ms(t2 - t0);
        }
        return std::pair<double, double>(recordTotal / frames, frameTotal / frames);
    };

    auto makeView = [&](std::span<const render::InstanceData> inst, const render::RenderItem& item,
                        std::span<const render::PointLight> lights) {
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0, 0, 40), glm::vec3(0), glm::vec3(0, 1, 0));
        v.proj = glm::perspective(glm::radians(60.0f), float(W) / float(H), 0.1f, 200.0f);
        v.target = colorRT; v.width = W; v.height = H;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = inst; v.materials = std::span<const render::MaterialGPU>(&mat, 1);
        v.pointLights = lights;
        return v;
    };

    std::printf("\n--- instance-count sweep (0 point lights) ---\n");
    std::printf("%10s %12s %12s %16s\n", "instances", "record ms", "frame ms", "Minst/s(frame)");
    for (uint32_t n : {256u, 4096u, 16384u, 65536u}) {
        auto instances = buildInstances(n);
        render::RenderItem item{ sphere, pipe, 0, n };
        auto view = makeView(instances, item, {});
        runFrames(view, 3);                       // warmup
        auto [rec, frame] = runFrames(view, 20);
        double minstps = (n / (frame / 1000.0)) / 1e6;
        std::printf("%10u %12.3f %12.3f %16.1f\n", n, rec, frame, minstps);
    }

    std::printf("\n--- point-light sweep (4096 instances): loop-all vs clustered forward+ ---\n");
    std::printf("(lights are LOCAL: small radius, spread across the scene — clustering's use case)\n");
    std::printf("%12s %16s %16s\n", "lights", "loop-all ms", "clustered ms");
    auto instances = buildInstances(4096);
    render::RenderItem item{ sphere, pipe, 0, 4096 };
    std::vector<render::PointLight> lights;
    for (uint32_t nl : {0u, 16u, 64u, 256u, 1024u}) {
        lights.clear();
        // Scatter small-radius lights across the instance cube's extent so each touches only a few
        // froxels (the locality clustering exploits). ~19-unit cube (buildInstances spacing 1.2).
        for (uint32_t i = 0; i < nl; ++i) {
            float a = float(i) * 2.3999632f;   // golden-angle scatter
            float r = 9.0f * std::sqrt(float(i + 1) / float(nl + 1));
            lights.push_back({ {std::cos(a) * r, std::sin(a) * r, std::sin(a * 0.7f) * 9.0f}, 2.5f,
                               {1.0f, 1.0f, 1.0f}, 1.0f });
        }
        auto view = makeView(instances, item, std::span<const render::PointLight>(lights));

        renderer.setClusterBinning({});                 // loop-all
        runFrames(view, 3);
        auto [rec0, frameLoop] = runFrames(view, 20);

        renderer.setClusterBinning(clusterPipe);         // clustered forward+
        runFrames(view, 3);
        auto [rec1, frameClus] = runFrames(view, 20);
        (void)rec0; (void)rec1;

        std::printf("%12u %16.3f %16.3f\n", nl, frameLoop, frameClus);
    }
    renderer.setClusterBinning({});
    std::printf("\nrender_graph benchmark done\n");
}
