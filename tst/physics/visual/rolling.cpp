#include "harness/harness.h"
//
//  physics_window.cpp
//  engine::tst
//
//  The driving milestone, visualized: spheres under gravity rolling down an inclined plane.
//  Ties every subsystem together — ecs::World + Schedule drive the PhysicsWorld through the
//  physics_ecs bridge (step + sync → Transform), scene::extract builds the RenderView, and the
//  Metal Renderer draws it in a window. Close the window to exit.
//
//  Run:  ./build/tst/physics_window   (optionally ENGINE_BALLS=N)
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "engine/core/core.h"
#include "engine/core/geometry/primitives.h"
#include "engine/ecs/ecs.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
#include "engine/physics/world.h"
#include "engine/physics_ecs/components.h"
#include "engine/physics_ecs/systems.h"
#include "engine/scene/extract.h"

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

// Quaternion rotating unit vector `from` onto unit vector `to`.
glm::quat rotationBetween(const glm::vec3& from, const glm::vec3& to) {
    const glm::vec3 axis = glm::cross(from, to);
    const float s = glm::length(axis);
    const float c = glm::dot(from, to);
    if (s < 1e-6f) return glm::quat(1, 0, 0, 0);
    return glm::angleAxis(std::atan2(s, c), axis / s);
}
}

TST_CASE(physics, visual, rolling) {
    using namespace engine;
    using namespace engine::rhi;
    namespace phys = engine::physics;

    int balls = 12;
    if (const char* b = std::getenv("ENGINE_BALLS")) { balls = std::atoi(b); if (balls < 1) balls = 1; }

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — ball rolling down a plane", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return; }

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw), H = static_cast<uint32_t>(fbh);

    WindowSurface surface{ window, W, H };
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
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);

    render::GeometryStore geometry(device);
    const float r = 0.5f;
    render::MeshHandle sphereMesh = geometry.upload(primitives::makeSphere(r, 20, 40));
    render::MeshHandle planeMesh  = geometry.upload(primitives::makePlane(14.0f, 1));
    render::Renderer renderer(device, geometry);

    // --- physics world: inclined plane + falling spheres ---
    const float angle = glm::radians(25.0f);
    const glm::vec3 n = glm::normalize(glm::vec3(std::sin(angle), std::cos(angle), 0.0f));
    phys::WorldDef wd;
    wd.gravity = glm::vec3(0, -9.81f, 0);
    wd.velocityIterations = 12;
    wd.substeps = 2;
    auto world = phys::createPhysicsWorld(phys::Backend::Realtime, wd);

    phys::BodyDef planeDef;
    planeDef.type = phys::BodyType::Static;
    planeDef.collider.type = phys::ColliderDesc::Type::Plane;
    planeDef.collider.plane = phys::Plane{ n, 0.0f };
    planeDef.material.friction = 0.9f;
    world->createBody(planeDef);

    const glm::vec3 upslope = -glm::normalize(wd.gravity - glm::dot(wd.gravity, n) * n);

    // ECS scene.
    ecs::World ecsWorld;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({});                              // 0: the ground plane
    materials[0].baseColorFactor = { 0.35f, 0.35f, 0.4f, 1.0f };

    // Ground entity: a big tilted quad matching the collider plane (no RigidBody → static).
    const glm::quat planeRot = rotationBetween(glm::vec3(0, 1, 0), n);
    ecsWorld.spawn(Transform{ .rotation = planeRot },
                   scene::RenderMesh{ planeMesh }, scene::RenderMaterial{ 0 });

    // Spheres, spread up-slope and across, dropped just above the surface.
    for (int i = 0; i < balls; ++i) {
        const float up   = 2.0f + (i % 4) * 1.3f;
        const float side = ((i / 4) - 1) * 1.2f;
        const glm::vec3 pos = n * (r + 0.3f) + upslope * up + glm::vec3(0, 0, side);

        phys::BodyDef ball;
        ball.type = phys::BodyType::Dynamic;
        ball.mass = 1.0f;
        ball.collider.type = phys::ColliderDesc::Type::Sphere;
        ball.collider.sphere = phys::Sphere{ r };
        ball.material.friction = 0.9f;
        ball.material.restitution = 0.1f;
        ball.position = pos;
        const phys::BodyHandle h = world->createBody(ball);

        const uint32_t mat = static_cast<uint32_t>(materials.size());
        render::MaterialGPU m;
        m.baseColorFactor = { 0.3f + 0.6f * float(i) / balls, 0.5f, 0.9f - 0.5f * float(i) / balls, 1.0f };
        materials.push_back(m);

        ecsWorld.spawn(Transform{ .position = pos },
                       scene::RenderMesh{ sphereMesh }, scene::RenderMaterial{ mat },
                       physics_ecs::RigidBody{ h });
    }

    ecsWorld.setResource(physics_ecs::PhysicsWorldRef{ world.get() });
    ecsWorld.setResource(physics_ecs::FixedStep{ 1.0f / 120.0f });
    ecs::Schedule sim;
    sim.add("physics.step", physics_ecs::stepSystem);
    sim.add("physics.sync", physics_ecs::syncSystem);

    scene::ExtractedScene extracted;
    std::printf("physics_window: %d spheres rolling down a %.0f-deg incline — close to exit.\n",
                balls, glm::degrees(angle));

    double last = glfwGetTime();
    double accumulator = 0.0;
    const double fixed = 1.0 / 120.0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double now = glfwGetTime();
        accumulator += std::min(now - last, 0.1);   // clamp to avoid spiral-of-death
        last = now;
        while (accumulator >= fixed) { sim.run(ecsWorld); accumulator -= fixed; }

        scene::extract(ecsWorld, extracted); renderer.setMeshPipeline(pipe);

        const glm::vec3 eye(9.0f, 7.0f, 11.0f);   // static camera
        render::RenderView view;
        view.view = glm::lookAt(eye, glm::vec3(2.0f, 0.0f, 0.0f), glm::vec3(0, 1, 0));
        view.proj = glm::perspective(glm::radians(55.0f), float(W) / float(H), 0.1f, 200.0f);
        view.width = W; view.height = H;
        view.items = std::span<const render::RenderItem>(extracted.items);
        view.instances = std::span<const render::InstanceData>(extracted.instances);
        view.materials = std::span<const render::MaterialGPU>(materials);

        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        view.target = frame.swapchainTarget();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("physics_window: closed.\n");
}
