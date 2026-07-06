#include "harness/harness.h"
//
//  input_demo.cpp
//  engine::tst — graphics / visual
//
//  Windowed demo of the Phase A ECS input/camera structure: the camera is an *entity*
//  (Transform + Camera + FlyController). A Schedule runs an input-pump system (GlfwInput →
//  InputState resource) then the fly-controller system (InputState + Time → the camera's
//  Transform). scene::extractViews turns the camera entity into a RenderView; Background /
//  SceneLighting resources drive the clear color + world light.
//
//  Controls:
//    W/A/S/D move, Q/E (or Space/LCtrl) down/up, hold LeftShift to sprint
//    Right-drag to look,  B cycle background,  L toggle light,  Esc quit
//
//  Run:  ./build/tst/visuals input_demo
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "engine/core/core.h"                 // Transform, Camera, Time
#include "engine/ecs/ecs.h"
#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
#include "engine/scene/extract.h"
#include "engine/scene/environment.h"
#include "engine/input/input.h"
#include "engine/input_glfw/glfw_input.h"
#include "engine/controls/fly_controller.h"

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

TST_CASE(graphics, visual, input_demo) {
    using namespace engine;
    using namespace engine::rhi;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — input demo", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return; }

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw), H = static_cast<uint32_t>(fbh);

    WindowSurface surface{window, W, H};
    Device device = Device::createWindowed(surface, {});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); return; }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const rhi::VertexLayout layout = render::coreVertexLayout();
    const Format colorFormat = Format::BGRA8Unorm;
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = layout;
    pdesc.topology = Topology::TriangleList;
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = {.test = true, .write = true, .op = CompareOp::Less};
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);

    render::GeometryStore geometry(device);
    render::MeshHandle ground = geometry.upload(primitives::makePlane(30.0f, 1));
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 16, 32));
    render::Renderer renderer(device, geometry);

    // Scene: a ground plane (material 0) + a grid of colored spheres (materials 1..N).
    ecs::World world;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({.baseColorFactor = glm::vec4(0.55f, 0.55f, 0.58f, 1.0f)});  // ground
    world.spawn(Transform{.position = glm::vec3(0, 0, 0)},
                scene::RenderMesh{ground}, scene::RenderMaterial{0});

    const int grid = 6;
    const float spacing = 2.0f;
    const float extent  = (grid - 1) * spacing * 0.5f;
    for (int iz = 0; iz < grid; ++iz) {
        for (int ix = 0; ix < grid; ++ix) {
            const auto mi = static_cast<uint32_t>(materials.size());
            materials.push_back({.baseColorFactor = glm::vec4(
                0.3f + 0.7f * float(ix) / grid, 0.5f, 0.3f + 0.7f * float(iz) / grid, 1.0f)});
            world.spawn(
                Transform{.position = glm::vec3(ix * spacing - extent, 0.6f, iz * spacing - extent)},
                scene::RenderMesh{sphere}, scene::RenderMaterial{mi});
        }
    }

    // The camera is an entity: Transform (pose) + Camera (projection) + FlyController (control).
    world.spawn(
        Transform{.position = glm::vec3(0.0f, 6.0f, extent + 10.0f)},
        Camera{.fovY = glm::radians(60.0f), .nearZ = 0.1f, .farZ = 200.0f},
        controls::FlyController{.pitch = -20.0f});

    // Scene-level environment (ECS resources) + input/time resources.
    const std::array<glm::vec4, 3> backgrounds = {
        glm::vec4(0.08f, 0.10f, 0.14f, 1.0f),   // dusk
        glm::vec4(0.55f, 0.70f, 0.90f, 1.0f),   // day sky
        glm::vec4(0.02f, 0.02f, 0.03f, 1.0f),   // night
    };
    int bgIndex = 0;
    world.setResource(scene::Background{backgrounds[bgIndex]});
    scene::SceneLighting lighting;
    world.setResource(lighting);
    world.setResource(input::InputState{});
    world.setResource(Time{});

    // Input adapter (GLFW, app-owned) + a Schedule: input-pump system, then the fly system.
    input::GlfwInput adapter(window);
    ecs::Schedule schedule;
    schedule.add("input", [&](ecs::World& w) { adapter.update(*w.getResource<input::InputState>()); });
    schedule.add("fly-camera", controls::flyControllerSystem);

    std::vector<render::RenderView> views;
    scene::ExtractedScene extracted;
    bool lightOn = true;
    double lastTime = glfwGetTime();

    std::printf("input_demo: WASD move, right-drag look, B background, L light, Esc quit.\n");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        const double now = glfwGetTime();
        world.getResource<Time>()->dt = static_cast<float>(now - lastTime);
        lastTime = now;

        schedule.run(world);   // input-pump → fly-camera (updates the camera entity's Transform)

        // App-level actions read the freshly-pumped InputState resource.
        const input::InputState& in = *world.getResource<input::InputState>();
        if (in.keyPressed(input::Key::Escape)) glfwSetWindowShouldClose(window, GLFW_TRUE);
        if (in.mousePressed(input::MouseButton::Right))  adapter.setCursorCaptured(true);
        if (in.mouseReleased(input::MouseButton::Right)) adapter.setCursorCaptured(false);
        if (in.keyPressed(input::Key::B)) {
            bgIndex = (bgIndex + 1) % static_cast<int>(backgrounds.size());
            world.setResource(scene::Background{backgrounds[bgIndex]});
        }
        if (in.keyPressed(input::Key::L)) {
            lightOn = !lightOn;
            lighting.light.intensity = lightOn ? 1.0f : 0.0f;
            world.setResource(lighting);
        }

        scene::extract(world, extracted); renderer.setMeshPipeline(pipe);
        scene::extractViews(world, extracted, views, W, H);   // camera entity → RenderView(s)

        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        for (auto& v : views) {
            v.materials = std::span<const render::MaterialGPU>(materials);
            v.target = frame.swapchainTarget();
        }
        renderer.render(frame, std::span<const render::RenderView>(views));
        device.endFrame(std::move(frame));
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("input_demo: closed.\n");
}
