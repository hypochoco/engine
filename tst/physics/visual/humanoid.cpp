#include "harness/harness.h"
//
//  humanoid.cpp
//  engine::tst — physics / visual
//
//  Milestone 2, Phase B5: the actuated humanoid, visualized. The physics::makeHumanoid preset is
//  built into a PhysicsWorld with its pelvis pinned and PD servos holding the neutral pose (so it
//  stands). Each body is an ECS entity (Transform scaled to its collider + RenderMesh + Material +
//  RigidBody); a Schedule runs input → fly-camera → actuatorFlush → step → sync. Limbs are drawn
//  as boxes (capsules approximated by their bounding box). Fly around with WASD + right-drag.
//
//  Run:  ./build/tst/visuals humanoid
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
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
#include "engine/scene/extract.h"
#include "engine/scene/environment.h"
#include "engine/input/input.h"
#include "engine/input_glfw/glfw_input.h"
#include "engine/controls/fly_controller.h"
#include "engine/physics/world.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_ecs/components.h"
#include "engine/physics_ecs/systems.h"

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

// Render scale (full extents) of a collider so a unit box approximates it.
glm::vec3 colliderScale(const engine::physics::ColliderDesc& c) {
    using T = engine::physics::ColliderDesc::Type;
    switch (c.type) {
        case T::Box:     return 2.0f * glm::vec3(c.box.halfExtents);
        case T::Capsule: return glm::vec3(2 * c.capsule.radius,
                                          2 * (c.capsule.halfHeight + c.capsule.radius),
                                          2 * c.capsule.radius);
        case T::Sphere:  return glm::vec3(2 * c.sphere.radius);
        default:         return glm::vec3(0.2f);
    }
}
}

TST_CASE(physics, visual, humanoid) {
    using namespace engine;
    using namespace engine::rhi;
    namespace phys = engine::physics;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — humanoid", nullptr, nullptr);
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
    render::MeshHandle boxMesh   = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));  // unit cube
    render::MeshHandle planeMesh = geometry.upload(primitives::makePlane(30.0f, 1));
    render::Renderer renderer(device, geometry);

    // --- physics: ground + humanoid ---
    // Two modes via ENGINE_HUMANOID:
    //   default "ragdoll" — free (unpinned, unactuated), starts just above the floor and falls +
    //                       settles; the clearest "physics working" demo, no PD/contact fighting.
    //   "pose"            — pelvis pinned and SUSPENDED clear of the floor, PD servos holding the
    //                       neutral pose; still + jitter-free (no foot-ground contact vs the PD).
    const char* modeEnv = std::getenv("ENGINE_HUMANOID");
    const bool poseMode = modeEnv && std::string(modeEnv) == "pose";

    phys::WorldDef wd;
    wd.gravity = glm::vec3(0, -9.81f, 0);
    wd.velocityIterations = 24;
    wd.substeps = 4;
    wd.linearDamping = 0.2f;    // mild drag so free DOFs settle (ragdoll) instead of jittering
    wd.angularDamping = 0.8f;
    auto world = phys::createPhysicsWorld(phys::Backend::Realtime, wd);
    {
        phys::BodyDef g;
        g.type = phys::BodyType::Static;
        g.collider.type = phys::ColliderDesc::Type::Plane;
        g.collider.plane = phys::Plane{ glm::vec3(0, 1, 0), 0.0f };
        g.material.friction = 0.9f;
        world->createBody(g);
    }
    phys::ArticulationDef def = phys::makeHumanoid(glm::vec3(0, poseMode ? 1.45f : 1.35f, 0));
    if (poseMode) def.bodies[0].type = phys::BodyType::Static;   // pin the pelvis (suspended)
    const phys::Articulation art = phys::buildArticulation(*world, def);
    if (poseMode) {
        phys::Actuator a;
        a.mode = phys::ActuatorMode::PDTarget;
        a.target = 0.0f; a.ballTarget = glm::quat(1, 0, 0, 0);
        a.kp = 600.0f; a.kd = 60.0f; a.maxTorque = 400.0f;
        for (const phys::JointHandle j : art.joints) world->setJointActuator(j, a);
    }

    // --- ECS scene: one render entity per body (box scaled to the collider) ---
    ecs::World ecsWorld;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({ .baseColorFactor = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f) });   // 0: ground
    ecsWorld.spawn(Transform{}, scene::RenderMesh{ planeMesh }, scene::RenderMaterial{ 0 });

    for (size_t i = 0; i < art.bodies.size(); ++i) {
        const phys::BodyHandle h = art.bodies[i];
        const engine::Transform pose = world->pose(h);
        const auto mat = static_cast<uint32_t>(materials.size());
        materials.push_back({ .baseColorFactor = glm::vec4(
            0.4f + 0.5f * float(i) / art.bodies.size(), 0.45f, 0.8f - 0.4f * float(i) / art.bodies.size(), 1.0f) });
        ecsWorld.spawn(
            Transform{ .position = pose.position, .rotation = pose.rotation,
                       .scale = colliderScale(def.bodies[i].collider) },
            scene::RenderMesh{ boxMesh }, scene::RenderMaterial{ mat },
            physics_ecs::RigidBody{ h });
    }

    // Camera entity + environment + input/time resources.
    ecsWorld.spawn(Transform{ .position = glm::vec3(0.0f, 1.3f, 4.0f) },
                   Camera{ .fovY = glm::radians(55.0f), .nearZ = 0.05f, .farZ = 200.0f },
                   controls::FlyController{ .pitch = -8.0f });
    ecsWorld.setResource(scene::Background{ glm::vec4(0.10f, 0.12f, 0.16f, 1.0f) });
    ecsWorld.setResource(scene::SceneLighting{});
    ecsWorld.setResource(input::InputState{});
    ecsWorld.setResource(Time{});
    ecsWorld.setResource(physics_ecs::PhysicsWorldRef{ world.get() });
    ecsWorld.setResource(physics_ecs::FixedStep{ 1.0f / 120.0f });

    input::GlfwInput adapter(window);
    ecs::Schedule frameSched;   // per-frame (variable dt): input then camera
    frameSched.add("input", [&](ecs::World& w) { adapter.update(*w.getResource<input::InputState>()); });
    frameSched.add("fly-camera", controls::flyControllerSystem);
    ecs::Schedule simSched;     // fixed-step: flush actuator commands, step physics, sync poses
    simSched.add("actuators", physics_ecs::actuatorFlushSystem);
    simSched.add("step", physics_ecs::stepSystem);
    simSched.add("sync", physics_ecs::syncSystem);

    std::vector<render::RenderView> views;
    scene::ExtractedScene extracted;
    double last = glfwGetTime();
    double accumulator = 0.0;
    const double fixed = 1.0 / 120.0;
    std::printf("humanoid: mode=%s (set ENGINE_HUMANOID=pose|ragdoll). WASD move, right-drag look, Esc quit.\n",
                poseMode ? "pose" : "ragdoll");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double now = glfwGetTime();
        ecsWorld.getResource<Time>()->dt = static_cast<float>(now - last);
        accumulator += std::min(now - last, 0.1);
        last = now;

        frameSched.run(ecsWorld);
        const input::InputState& in = *ecsWorld.getResource<input::InputState>();
        if (in.keyPressed(input::Key::Escape)) glfwSetWindowShouldClose(window, GLFW_TRUE);
        if (in.mousePressed(input::MouseButton::Right))  adapter.setCursorCaptured(true);
        if (in.mouseReleased(input::MouseButton::Right)) adapter.setCursorCaptured(false);
        while (accumulator >= fixed) { simSched.run(ecsWorld); accumulator -= fixed; }

        scene::extract(ecsWorld, pipe, extracted);
        scene::extractViews(ecsWorld, extracted, views, W, H);

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
    std::printf("humanoid: closed.\n");
}
