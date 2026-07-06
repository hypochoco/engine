#include "harness/harness.h"
//
//  clustered_lights.cpp
//  engine::tst
//
//  Windowed demo of the render framework: a field of spheres lit by many moving COLORED point
//  lights via clustered forward+ (the froxel binning compute pass + per-cluster forward shading).
//  Low ambient so the lights read as moving colored pools. Orbiting camera. Close to exit.
//
//  Run:  ./build/tst/visuals clustered_lights     (optionally ENGINE_GRID=N, ENGINE_LIGHTS=M)
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
#include "graphics/realtime/visual/demo_toggles.h"

namespace {
std::vector<std::byte> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> data(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}
}

TST_CASE(graphics, visual, clustered_lights) {
    using namespace engine;
    using namespace engine::rhi;

    int grid = 24;
    if (const char* g = std::getenv("ENGINE_GRID")) { grid = std::atoi(g); if (grid < 1) grid = 1; }
    int numLights = 16;
    if (const char* l = std::getenv("ENGINE_LIGHTS")) { numLights = std::atoi(l); if (numLights < 1) numLights = 1; }
    const int count = grid * grid;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1100, 800, "engine — clustered forward+ lights", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return; }

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw), H = static_cast<uint32_t>(fbh);

    WindowSurface surface{ window, W, H };
    Device device = Device::createWindowed(surface, {});

    const auto meshBlob = readFile(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    const auto clusBlob = readFile(std::string(ENGINE_SHADER_DIR) + "/cluster.metallib");
    const auto tmBlob   = readFile(std::string(ENGINE_SHADER_DIR) + "/tonemap.metallib");
    if (meshBlob.empty() || clusBlob.empty() || tmBlob.empty()) { std::printf("FAIL: read shaders\n"); return; }
    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle cs = device.createShader(clusBlob, ShaderStage::Compute);
    ShaderHandle tmvs = device.createShader(tmBlob, ShaderStage::Vertex);
    ShaderHandle tmfs = device.createShader(tmBlob, ShaderStage::Fragment);

    const Format colorFormat = Format::BGRA8Unorm;   // swapchain (final target)
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = render::coreVertexLayout();
    pdesc.topology = Topology::TriangleList;
    const Format hdrFormat = Format::RGBA16Float;    // forward renders into HDR
    pdesc.colorFormats = std::span<const Format>(&hdrFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    PipelineHandle clusterPipe = device.createComputePipeline({ .compute = cs });

    // Fullscreen ACES tonemap pipeline (no vertex layout / depth / cull), outputs to the swapchain.
    GraphicsPipelineDesc tp;
    tp.vertex = tmvs; tp.fragment = tmfs;
    tp.colorFormats = std::span<const Format>(&colorFormat, 1);
    tp.depthFormat = Format::Undefined;
    tp.depth = { .test = false, .write = false, .op = CompareOp::Always };
    tp.raster.cull = CullMode::None;
    PipelineHandle tonemapPipe = device.createGraphicsPipeline(tp);
    SamplerHandle sampler = device.createSampler({ .addressU = AddressMode::ClampToEdge,
                                                   .addressV = AddressMode::ClampToEdge });

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.4f, 16, 32));
    render::Renderer renderer(device, geometry);
    renderer.setClusterBinning(clusterPipe);   // clustered forward+ on
    renderer.setTonemap(tonemapPipe, sampler);  // HDR + ACES tone mapping on

    // A flat field of spheres (single instanced draw — one item, so SV_InstanceID starts at 0).
    const float spacing = 1.0f;
    const float extent  = (grid - 1) * spacing * 0.5f;
    std::vector<render::MaterialGPU> materials(count);
    std::vector<render::InstanceData> instances(count);
    for (int iz = 0; iz < grid; ++iz) {
        for (int ix = 0; ix < grid; ++ix) {
            const int i = iz * grid + ix;
            materials[i].baseColorFactor = glm::vec4(0.75f, 0.75f, 0.78f, 1.0f);   // neutral, so light color shows
            glm::vec3 p(ix * spacing - extent, 0.0f, iz * spacing - extent);
            instances[i].model = glm::translate(glm::mat4(1.0f), p);
            instances[i].normalModel = instances[i].model;
            instances[i].materialIndex = static_cast<uint32_t>(i);
        }
    }
    render::RenderItem item{ sphere, 0, static_cast<uint32_t>(count) };
    renderer.setMeshPipeline(pipe);

    // A palette for the moving lights.
    const glm::vec3 palette[] = {
        {1.0f, 0.2f, 0.2f}, {0.2f, 1.0f, 0.3f}, {0.3f, 0.4f, 1.0f}, {1.0f, 0.8f, 0.2f},
        {1.0f, 0.3f, 0.9f}, {0.2f, 1.0f, 1.0f}, {1.0f, 0.5f, 0.1f}, {0.6f, 0.3f, 1.0f},
    };
    std::vector<render::PointLight> lights(numLights);

    std::printf("clustered_lights: %d spheres (%dx%d), %d moving colored lights, %ux%u — close to exit.\n",
                count, grid, grid, numLights, W, H);
    std::printf("  keys:  C = clustered forward+ vs loop-all\n");

    engine::demo::KeyToggle clusteredT{ GLFW_KEY_C, true, "clustered" };

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (clusteredT.poll(window)) renderer.setClusterBinning(clusteredT.on ? clusterPipe : PipelineHandle{});
        const float t = static_cast<float>(glfwGetTime());

        // Animate the point lights: each orbits its own circle just above the field.
        for (int i = 0; i < numLights; ++i) {
            const float ph = float(i) / numLights * 6.2831853f;
            const float sp = 0.4f + 0.15f * (i % 3);
            const float rad = extent * (0.3f + 0.6f * float((i * 7) % numLights) / numLights);
            lights[i].position = glm::vec3(std::cos(t * sp + ph) * rad,
                                           0.8f + 0.5f * std::sin(t * 0.7f + ph),
                                           std::sin(t * sp + ph) * rad);
            lights[i].radius = 4.0f;
            lights[i].color = palette[i % 8];
            lights[i].intensity = 5.0f;   // bright — ACES tonemapping rolls off the highlights
        }

        const float r = extent * 2.0f + 4.0f;
        glm::vec3 eye(std::sin(t * 0.15f) * r, extent * 0.7f + 4.0f, std::cos(t * 0.15f) * r);
        render::RenderView view;
        view.view = glm::lookAt(eye, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
        view.proj = glm::perspective(glm::radians(55.0f), float(W) / float(H), 0.1f, 200.0f);
        view.width = W; view.height = H;
        view.clearColor[0] = 0.02f; view.clearColor[1] = 0.02f; view.clearColor[2] = 0.03f; view.clearColor[3] = 1.0f;
        view.light.intensity = 0.0f;                       // no sun — let the point lights carry it
        view.light.ambient = glm::vec3(0.03f, 0.03f, 0.04f);
        view.items = std::span<const render::RenderItem>(&item, 1);
        view.instances = std::span<const render::InstanceData>(instances);
        view.materials = std::span<const render::MaterialGPU>(materials);
        view.pointLights = std::span<const render::PointLight>(lights);

        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        view.target = frame.swapchainTarget();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("clustered_lights: closed.\n");
}
