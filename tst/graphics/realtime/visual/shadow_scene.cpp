#include "harness/harness.h"
//
//  shadow_scene.cpp
//  engine::tst
//
//  Windowed demo of directional shadow mapping: a grid of cubes of varying heights on a ground
//  slab, lit by a slowly rotating sun that casts moving shadows, with HDR + ACES tone mapping.
//  Close to exit.
//
//  Run:  ./build/tst/visuals shadow_scene
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

TST_CASE(graphics, visual, shadow_scene) {
    using namespace engine;
    using namespace engine::rhi;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1100, 800, "engine — directional shadows", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return; }
    int fbw = 0, fbh = 0; glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw), H = static_cast<uint32_t>(fbh);

    WindowSurface surface{ window, W, H };
    Device device = Device::createWindowed(surface, {});

    const auto meshBlob = readFile(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    const auto shBlob   = readFile(std::string(ENGINE_SHADER_DIR) + "/shadow.metallib");
    const auto tmBlob   = readFile(std::string(ENGINE_SHADER_DIR) + "/tonemap.metallib");
    const auto skyBlob  = readFile(std::string(ENGINE_SHADER_DIR) + "/sky.metallib");
    if (meshBlob.empty() || shBlob.empty() || tmBlob.empty() || skyBlob.empty()) { std::printf("FAIL: read shaders\n"); return; }
    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle shvs = device.createShader(shBlob, ShaderStage::Vertex);
    ShaderHandle tmvs = device.createShader(tmBlob, ShaderStage::Vertex);
    ShaderHandle tmfs = device.createShader(tmBlob, ShaderStage::Fragment);
    ShaderHandle skvs = device.createShader(skyBlob, ShaderStage::Vertex);
    ShaderHandle skfs = device.createShader(skyBlob, ShaderStage::Fragment);

    const Format swap = Format::BGRA8Unorm, hdr = Format::RGBA16Float;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs; pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&hdr, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);

    GraphicsPipelineDesc sd;
    sd.vertex = shvs; sd.vertexLayout = render::coreVertexLayout();
    sd.depthFormat = Format::Depth32Float;
    sd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle shadowPipe = device.createGraphicsPipeline(sd);

    GraphicsPipelineDesc tp;
    tp.vertex = tmvs; tp.fragment = tmfs;
    tp.colorFormats = std::span<const Format>(&swap, 1);
    tp.depthFormat = Format::Undefined;
    tp.depth = { .test = false, .write = false, .op = CompareOp::Always };
    tp.raster.cull = CullMode::None;
    PipelineHandle tonemapPipe = device.createGraphicsPipeline(tp);

    // Sky pipeline: fullscreen, HDR target (tonemapping is on), depth test LessEqual + no write.
    GraphicsPipelineDesc skd;
    skd.vertex = skvs; skd.fragment = skfs;
    skd.colorFormats = std::span<const Format>(&hdr, 1);
    skd.depthFormat = Format::Depth32Float;
    skd.depth = { .test = true, .write = false, .op = CompareOp::LessEqual };
    skd.raster.cull = CullMode::None;
    PipelineHandle skyPipe = device.createGraphicsPipeline(skd);

    SamplerHandle shadowSamp = device.createSampler({ .minFilter = Filter::Nearest, .magFilter = Filter::Nearest,
                                                      .mipmap = MipmapMode::Nearest,
                                                      .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });
    SamplerHandle linSamp = device.createSampler({ .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });

    render::GeometryStore geometry(device);
    render::MeshHandle box = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::Renderer renderer(device, geometry);
    renderer.setTonemap(tonemapPipe, linSamp);
    // Feature enablers (wrapped so the A/B toggles can re-apply them).
    auto enableShadows = [&]() { renderer.setShadows(shadowPipe, shadowSamp, 28.0f, 120.0f); };
    auto enableFog = [&]() {
        renderer.setFog(/*density*/ 0.008f, /*heightFalloff*/ 0.02f, /*baseHeight*/ 0.0f,
                        /*color*/ glm::vec3(0.55f, 0.62f, 0.78f), /*inscatter*/ glm::vec3(1.4f, 1.0f, 0.6f),
                        /*inscatterExp*/ 8.0f);
    };
    enableShadows();
    renderer.setSky(skyPipe);
    enableFog();

    // Ground slab (instance 0) + a grid of cubes of varying height (single mesh, one draw).
    const int G = 7;
    std::vector<render::InstanceData> inst;
    std::vector<render::MaterialGPU> mats;
    mats.push_back({}); mats[0].baseColorFactor = {0.5f, 0.5f, 0.55f, 1.0f};   // ground
    { render::InstanceData g; g.model = glm::scale(glm::mat4(1.0f), glm::vec3(50.0f, 0.4f, 50.0f)); g.normalModel = g.model; g.materialIndex = 0; inst.push_back(g); }
    for (int iz = 0; iz < G; ++iz)
        for (int ix = 0; ix < G; ++ix) {
            float hx = std::sin(float(ix) * 12.9898f + float(iz) * 78.233f) * 43758.5f;
            float ht = 1.0f + 3.0f * (hx - std::floor(hx));   // pseudo-random height 1..4
            glm::vec3 p((ix - G / 2) * 5.0f, ht * 0.5f, (iz - G / 2) * 5.0f);
            render::InstanceData c;
            c.model = glm::translate(glm::mat4(1.0f), p) * glm::scale(glm::mat4(1.0f), glm::vec3(1.6f, ht, 1.6f));
            c.normalModel = c.model; c.materialIndex = static_cast<uint32_t>(mats.size());
            inst.push_back(c);
            render::MaterialGPU m; m.baseColorFactor = glm::vec4(0.4f + 0.5f * float(ix) / G, 0.55f, 0.4f + 0.5f * float(iz) / G, 1.0f);
            mats.push_back(m);
        }
    render::RenderItem item{ box, 0, static_cast<uint32_t>(inst.size()) };
    renderer.setMeshPipeline(pipe);

    std::printf("shadow_scene: %zu boxes on a ground, rotating sun casting shadows, HDR. Close to exit.\n", inst.size() - 1);
    std::printf("  keys:  S = shadows   K = sky   F = fog\n");

    engine::demo::KeyToggle shadowsT{ GLFW_KEY_S, true, "shadows" };
    engine::demo::KeyToggle skyT    { GLFW_KEY_K, true, "sky" };
    engine::demo::KeyToggle fogT    { GLFW_KEY_F, true, "fog" };

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (shadowsT.poll(window)) { if (shadowsT.on) enableShadows(); else renderer.setShadows({}, {}); }
        if (skyT.poll(window))     renderer.setSky(skyT.on ? skyPipe : PipelineHandle{});
        if (fogT.poll(window))     { if (fogT.on) enableFog(); else renderer.setFog(0.0f); }
        const float t = static_cast<float>(glfwGetTime());

        render::RenderView view;
        glm::vec3 eye(std::sin(t * 0.1f) * 42.0f, 30.0f, std::cos(t * 0.1f) * 42.0f);
        view.view = glm::lookAt(eye, glm::vec3(0, 2, 0), glm::vec3(0, 1, 0));
        view.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.5f, 300.0f);
        view.width = W; view.height = H;
        view.clearColor[0] = 0.05f; view.clearColor[1] = 0.06f; view.clearColor[2] = 0.09f; view.clearColor[3] = 1.0f;
        // Rotating sun (kept fairly high so shadows sweep without going flat).
        view.light.direction = glm::normalize(glm::vec3(std::cos(t * 0.25f) * 0.6f, -1.0f, std::sin(t * 0.25f) * 0.6f));
        view.light.color = glm::vec3(1.0f, 0.96f, 0.88f);
        view.light.intensity = 1.6f;
        view.light.ambient = glm::vec3(0.10f, 0.11f, 0.14f);
        view.items = std::span<const render::RenderItem>(&item, 1);
        view.instances = std::span<const render::InstanceData>(inst);
        view.materials = std::span<const render::MaterialGPU>(mats);

        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        view.target = frame.swapchainTarget();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("shadow_scene: closed.\n");
}
