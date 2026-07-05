#include "harness/harness.h"
//
//  atmosphere_scene.cpp
//  engine::tst
//
//  Windowed showcase for aerial-perspective + height fog (RF6 atmosphere). A long avenue of pillars
//  of varying height recedes toward the horizon under a low sun, with the procedural sky behind.
//  Near pillars are crisp; distant ones wash out toward the sky colour and glow warm toward the sun
//  (Mie in-scatter). Height fog pools low so pillar bases are hazier than their tops.
//
//  Press F to toggle fog on/off (direct A/B). Close the window to exit.
//
//  Run:  ./build/tst/visuals atmosphere_scene
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
#include "graphics/visual/demo_toggles.h"

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

TST_CASE(graphics, visual, atmosphere_scene) {
    using namespace engine;
    using namespace engine::rhi;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1100, 800, "engine — atmosphere (F: toggle fog)", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return; }
    int fbw = 0, fbh = 0; glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw), H = static_cast<uint32_t>(fbh);

    WindowSurface surface{ window, W, H };
    Device device = Device::createWindowed(surface, {});

    const auto meshBlob = readFile(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    const auto tmBlob   = readFile(std::string(ENGINE_SHADER_DIR) + "/tonemap.metallib");
    const auto skyBlob  = readFile(std::string(ENGINE_SHADER_DIR) + "/sky.metallib");
    const auto fxaaBlob = readFile(std::string(ENGINE_SHADER_DIR) + "/fxaa.metallib");
    if (meshBlob.empty() || tmBlob.empty() || skyBlob.empty() || fxaaBlob.empty()) { std::printf("FAIL: read shaders\n"); return; }
    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle tmvs = device.createShader(tmBlob, ShaderStage::Vertex);
    ShaderHandle tmfs = device.createShader(tmBlob, ShaderStage::Fragment);
    ShaderHandle skvs = device.createShader(skyBlob, ShaderStage::Vertex);
    ShaderHandle skfs = device.createShader(skyBlob, ShaderStage::Fragment);
    ShaderHandle fxvs = device.createShader(fxaaBlob, ShaderStage::Vertex);
    ShaderHandle fxfs = device.createShader(fxaaBlob, ShaderStage::Fragment);

    const Format swap = Format::BGRA8Unorm, hdr = Format::RGBA16Float, ldr = Format::RGBA8Unorm;
    // Mesh + sky pipelines in 1× and 4× sample-count variants (MSAA needs matching sampleCount).
    auto meshPipe = [&](uint32_t s) {
        GraphicsPipelineDesc pd;
        pd.vertex = vs; pd.fragment = fs; pd.vertexLayout = render::coreVertexLayout();
        pd.colorFormats = std::span<const Format>(&hdr, 1);
        pd.depthFormat = Format::Depth32Float;
        pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
        pd.sampleCount = s;
        return device.createGraphicsPipeline(pd);
    };
    auto skyPipeN = [&](uint32_t s) {
        GraphicsPipelineDesc skd;
        skd.vertex = skvs; skd.fragment = skfs;
        skd.colorFormats = std::span<const Format>(&hdr, 1);
        skd.depthFormat = Format::Depth32Float;
        skd.depth = { .test = true, .write = false, .op = CompareOp::LessEqual };
        skd.raster.cull = CullMode::None;
        skd.sampleCount = s;
        return device.createGraphicsPipeline(skd);
    };
    PipelineHandle pipeMesh1 = meshPipe(1), pipeMesh4 = meshPipe(4);
    PipelineHandle pipeSky1  = skyPipeN(1), pipeSky4  = skyPipeN(4);

    // Tonemap: outputs the swapchain when FXAA is off, or the RGBA8 intermediate when FXAA is on.
    auto tmPipe = [&](Format out) {
        GraphicsPipelineDesc tp;
        tp.vertex = tmvs; tp.fragment = tmfs;
        tp.colorFormats = std::span<const Format>(&out, 1);
        tp.depthFormat = Format::Undefined;
        tp.depth = { .test = false, .write = false, .op = CompareOp::Always };
        tp.raster.cull = CullMode::None;
        return device.createGraphicsPipeline(tp);
    };
    PipelineHandle tmSwap = tmPipe(swap), tmLDR = tmPipe(ldr);

    GraphicsPipelineDesc fxd;   // FXAA outputs the swapchain
    fxd.vertex = fxvs; fxd.fragment = fxfs;
    fxd.colorFormats = std::span<const Format>(&swap, 1);
    fxd.depthFormat = Format::Undefined;
    fxd.depth = { .test = false, .write = false, .op = CompareOp::Always };
    fxd.raster.cull = CullMode::None;
    PipelineHandle fxPipe = device.createGraphicsPipeline(fxd);

    SamplerHandle linSamp = device.createSampler({ .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });

    render::GeometryStore geometry(device);
    render::MeshHandle box = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::Renderer renderer(device, geometry);
    renderer.setSky(pipeSky1);

    // Fog config (sky-consistent pale base + warm sun in-scatter). Toggled by F.
    auto enableFog = [&]() {
        renderer.setFog(/*density*/ 0.012f, /*heightFalloff*/ 0.02f, /*baseHeight*/ 0.0f,
                        /*color*/ glm::vec3(0.55f, 0.63f, 0.80f), /*inscatter*/ glm::vec3(2.4f, 1.6f, 0.8f),
                        /*inscatterExp*/ 6.0f);
    };
    enableFog();   // fog starts on; F toggles it via the KeyToggle below

    // AA state: reconfigures pipelines + renderer whenever MSAA (M) or FXAA (X) toggles.
    bool msaaOn = true, fxaaOn = false, skyOn = true;
    render::MeshHandle boxMesh = box;
    render::RenderItem item{ boxMesh, pipeMesh1, 0, 0 };   // instanceCount filled after building instances
    auto applyAA = [&]() {
        renderer.setMSAA(msaaOn ? 4 : 1);
        item.pipeline = msaaOn ? pipeMesh4 : pipeMesh1;
        renderer.setSky(skyOn ? (msaaOn ? pipeSky4 : pipeSky1) : PipelineHandle{});
        renderer.setTonemap(fxaaOn ? tmLDR : tmSwap, linSamp);
        renderer.setFXAA(fxaaOn ? fxPipe : PipelineHandle{}, fxaaOn ? linSamp : SamplerHandle{});
    };

    // Ground + a long avenue of pillars receding in -Z, heights varying, both sides of the path.
    std::vector<render::InstanceData> inst;
    std::vector<render::MaterialGPU> mats;
    mats.push_back({}); mats[0].baseColorFactor = {0.34f, 0.36f, 0.32f, 1.0f};   // ground
    { render::InstanceData g; g.model = glm::scale(glm::mat4(1.0f), glm::vec3(500.0f, 0.4f, 500.0f)); g.normalModel = g.model; g.materialIndex = 0; inst.push_back(g); }

    auto rnd = [](float s) { float v = std::sin(s * 91.71f) * 43758.5453f; return v - std::floor(v); };
    const float xs[] = { -16.0f, -9.0f, 9.0f, 16.0f };
    for (int row = 0; row < 26; ++row) {
        float z = -6.0f - row * 8.5f;                       // recede far into the distance
        for (float x : xs) {
            float r = rnd(z * 3.1f + x);
            float ht = 3.0f + 9.0f * r;                     // 3..12 tall
            glm::vec3 p(x + (rnd(x + z) - 0.5f) * 2.0f, ht * 0.5f, z);
            render::InstanceData c;
            c.model = glm::translate(glm::mat4(1.0f), p) * glm::scale(glm::mat4(1.0f), glm::vec3(2.2f, ht, 2.2f));
            c.normalModel = c.model; c.materialIndex = static_cast<uint32_t>(mats.size());
            inst.push_back(c);
            render::MaterialGPU m;
            m.baseColorFactor = glm::vec4(0.45f + 0.35f * rnd(x * 1.3f + z), 0.42f, 0.40f + 0.3f * r, 1.0f);
            mats.push_back(m);
        }
    }
    item.instanceCount = static_cast<uint32_t>(inst.size());
    applyAA();   // set the initial MSAA/FXAA/sky pipeline configuration

    std::printf("atmosphere_scene: %zu pillars receding to the horizon. Close to exit.\n", inst.size() - 1);
    std::printf("  keys:  F = fog   K = sky   M = MSAA(4x)   X = FXAA\n");

    engine::demo::KeyToggle fogT { GLFW_KEY_F, true,  "fog" };
    engine::demo::KeyToggle skyT { GLFW_KEY_K, true,  "sky" };
    engine::demo::KeyToggle msaaT{ GLFW_KEY_M, true,  "MSAA 4x" };
    engine::demo::KeyToggle fxaaT{ GLFW_KEY_X, false, "FXAA" };

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (fogT.poll(window))  { if (fogT.on) enableFog(); else renderer.setFog(0.0f); }
        if (skyT.poll(window))  { skyOn  = skyT.on;  applyAA(); }
        if (msaaT.poll(window)) { msaaOn = msaaT.on; applyAA(); }
        if (fxaaT.poll(window)) { fxaaOn = fxaaT.on; applyAA(); }

        const float t = static_cast<float>(glfwGetTime());

        render::RenderView view;
        glm::vec3 eye(std::sin(t * 0.15f) * 6.0f, 3.5f, 14.0f);   // gentle sway for parallax
        view.view = glm::lookAt(eye, glm::vec3(0, 5.0f, -70.0f), glm::vec3(0, 1, 0));
        view.proj = glm::perspective(glm::radians(55.0f), float(W) / float(H), 0.3f, 600.0f);
        view.width = W; view.height = H;
        view.clearColor[0] = 0.0f; view.clearColor[1] = 0.0f; view.clearColor[2] = 0.0f; view.clearColor[3] = 1.0f;
        // Low sun ahead + slightly to the side: rim-lights the pillars and glows warm through the haze.
        view.light.direction = glm::normalize(glm::vec3(-0.35f, -0.22f, 1.0f));
        view.light.color = glm::vec3(1.0f, 0.93f, 0.82f);
        view.light.intensity = 1.5f;
        view.light.ambient = glm::vec3(0.14f, 0.16f, 0.20f);
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
    std::printf("atmosphere_scene: closed.\n");
}
