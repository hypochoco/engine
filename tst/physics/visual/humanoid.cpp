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
    render::MeshHandle boxMesh    = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));   // unit cube (feet)
    render::MeshHandle sphereMesh = geometry.upload(primitives::makeSphere(0.5f, 20, 32));   // unit sphere (limbs → ellipsoids)
    render::MeshHandle planeMesh  = geometry.upload(primitives::makePlane(30.0f, 1));
    render::Renderer renderer(device, geometry);

    // Rotate a +Y capsule mesh so its long axis is +X (for the horizontal shoulders).
    auto rotateZ90 = [](MeshData mesh) {
        const glm::mat3 R = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0, 0, 1)));
        for (Vertex& v : mesh.vertices) { v.position = R * v.position; v.normal = R * v.normal; }
        return mesh;
    };
    // Preset body order: 0 pelvis, 1 torso, 2 shoulders, 3 head, 4-11 limbs, 12-13 feet.
    // Pelvis/shoulders = horizontal capsules, torso = vertical capsule, limbs = capsules,
    // head = a perfect sphere, feet = boxes.
    const phys::ArticulationDef refDef = phys::makeHumanoid();
    std::vector<render::MeshHandle> bodyMesh(refDef.bodies.size());
    std::vector<glm::vec3> bodyScale(refDef.bodies.size(), glm::vec3(1.0f));
    for (size_t i = 0; i < refDef.bodies.size(); ++i) {
        const phys::ColliderDesc& col = refDef.bodies[i].collider;
        const bool isFoot = i >= refDef.bodies.size() - 2;
        if (i == 0) {                 // pelvis → horizontal capsule (hips are wider than tall)
            bodyMesh[i] = geometry.upload(rotateZ90(primitives::makeCapsule(0.08f, 0.045f)));
        } else if (i == 1) {          // torso → vertical capsule
            bodyMesh[i] = geometry.upload(primitives::makeCapsule(0.11f, 0.075f));
        } else if (i == 2) {          // shoulders → horizontal capsule (long axis X)
            bodyMesh[i] = geometry.upload(rotateZ90(primitives::makeCapsule(0.06f, 0.12f)));
        } else if (i == 3) {          // head → perfect sphere (uniform scale)
            bodyMesh[i] = sphereMesh; bodyScale[i] = glm::vec3(0.18f);
        } else if (col.type == phys::ColliderDesc::Type::Capsule) {   // arms + legs
            bodyMesh[i] = geometry.upload(primitives::makeCapsule(col.capsule.radius, col.capsule.halfHeight));
        } else if (isFoot) {          // feet
            bodyMesh[i] = boxMesh;    bodyScale[i] = colliderScale(col);
        } else {                      // fallback
            bodyMesh[i] = sphereMesh; bodyScale[i] = colliderScale(col);
        }
    }

    // --- physics: ground + a gallery of humanoids in different states ---
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

    ecs::World ecsWorld;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({ .baseColorFactor = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f) });   // 0: ground
    ecsWorld.spawn(Transform{}, scene::RenderMesh{ planeMesh }, scene::RenderMaterial{ 0 });

    // Spawn one humanoid: build it, optionally pin the pelvis + PD-hold a pose, and create a
    // render entity per body (limbs = stretched spheres, feet = boxes). `kneeBend` bends the knees.
    auto spawnHumanoid = [&](glm::vec3 rootPos, bool pinned, bool kneeBend, glm::vec4 tint) {
        phys::ArticulationDef def = phys::makeHumanoid(rootPos);
        if (pinned) def.bodies[0].type = phys::BodyType::Static;
        const phys::Articulation art = phys::buildArticulation(*world, def);
        if (pinned) {
            phys::Actuator a;
            a.mode = phys::ActuatorMode::PDTarget;
            a.target = 0.0f; a.ballTarget = glm::quat(1, 0, 0, 0);
            a.kp = 600.0f; a.kd = 60.0f; a.maxTorque = 400.0f;
            for (const phys::JointHandle j : art.joints) world->setJointActuator(j, a);
            // The feet are ~40x lighter than the hips/knees; uniform high gains make explicit PD
            // overshoot and buzz. Scale the ankle gains + torque limit down to the foot's inertia.
            phys::Actuator ankle = a; ankle.kp = 25.0f; ankle.kd = 5.0f; ankle.maxTorque = 6.0f;
            world->setJointActuator(art.joints[11], ankle);   // ankle L
            world->setJointActuator(art.joints[12], ankle);   // ankle R
            if (kneeBend) {
                phys::Actuator k = a; k.target = -1.2f;         // hinge knees (joints 9,10)
                world->setJointActuator(art.joints[9], k);
                world->setJointActuator(art.joints[10], k);
            }
        }
        for (size_t i = 0; i < art.bodies.size(); ++i) {
            const engine::Transform pose = world->pose(art.bodies[i]);
            const auto mat = static_cast<uint32_t>(materials.size());
            glm::vec4 c = tint * (0.72f + 0.4f * float(i) / float(art.bodies.size()));
            c.a = 1.0f;
            materials.push_back({ .baseColorFactor = c });
            ecsWorld.spawn(
                Transform{ .position = pose.position, .rotation = pose.rotation, .scale = bodyScale[i] },
                scene::RenderMesh{ bodyMesh[i] }, scene::RenderMaterial{ mat },
                physics_ecs::RigidBody{ art.bodies[i] });
        }
    };

    // Left: free ragdoll (falls + settles). Middle: pinned, PD holding the neutral pose.
    // Right: pinned, PD holding a deep knee bend. Pinned ones are suspended so the feet clear the
    // floor (no foot-ground contact fighting the PD → they stand still for inspection).
    spawnHumanoid(glm::vec3(-2.6f, 1.15f, 0.0f), false, false, glm::vec4(0.85f, 0.42f, 0.38f, 1.0f));
    spawnHumanoid(glm::vec3( 0.0f, 1.40f, 0.0f), true,  false, glm::vec4(0.42f, 0.55f, 0.85f, 1.0f));
    spawnHumanoid(glm::vec3( 2.6f, 1.40f, 0.0f), true,  true,  glm::vec4(0.45f, 0.80f, 0.52f, 1.0f));

    // Camera entity, framed to see all three ~1.70 m figures.
    ecsWorld.spawn(Transform{ .position = glm::vec3(0.0f, 1.4f, 6.0f) },
                   Camera{ .fovY = glm::radians(55.0f), .nearZ = 0.05f, .farZ = 200.0f },
                   controls::FlyController{ .pitch = -6.0f });
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
    std::printf("humanoid: left=ragdoll (falls), middle=PD neutral hold, right=PD knee-bend hold. "
                "WASD move, right-drag look, Esc quit.\n");

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
