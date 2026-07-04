#include "harness/harness.h"
//
//  reduced_joints.cpp
//  engine::tst — physics / visual
//
//  A gallery of articulated systems on the REDUCED (Featherstone/ABA) backend, arranged left→right
//  from simplest to most complex, so you can eyeball that each joint type behaves correctly. All
//  are passive (gravity-driven) and start off-equilibrium so they swing on launch, then settle
//  under mild joint damping:
//
//    1. single revolute pendulum          (1 hinge)
//    2. double revolute pendulum          (2 hinges — chaotic)
//    3. four-link revolute chain          ("rope" — settles into a hang)
//    4. ball-joint pendulum               (3-DOF spherical — swings out of plane)
//    5. fixed weld + revolute elbow arm   (rigid upper arm, free forearm hinge)
//
//  Each body is an ECS entity (Transform + RenderMesh + Material + RigidBody); the shared sim
//  schedule steps the world and syncs poses. Fly around with WASD + right-drag.
//
//  Run:  ./build/tst/visuals reduced_joints
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
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
}

TST_CASE(physics, visual, reduced_joints) {
    using namespace engine;
    using namespace engine::rhi;
    namespace phys = engine::physics;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1100, 720, "engine — reduced joints", nullptr, nullptr);
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
    render::MeshHandle sphereMesh = geometry.upload(primitives::makeSphere(0.5f, 20, 32));   // unit-diameter
    render::MeshHandle boxMesh    = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));    // unit cube
    render::MeshHandle planeMesh  = geometry.upload(primitives::makePlane(40.0f, 1));
    render::Renderer renderer(device, geometry);

    // --- physics world (reduced backend) ---
    phys::WorldDef wd;
    wd.gravity = glm::vec3(0, -9.81f, 0);
    wd.substeps = 8;
    wd.angularDamping = 0.4f;                 // mild joint damping so swings settle
    auto world = phys::createPhysicsWorld(phys::Backend::Reduced, wd);

    ecs::World ecsWorld;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({ .baseColorFactor = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f) });   // 0: ground
    ecsWorld.spawn(Transform{}, scene::RenderMesh{ planeMesh }, scene::RenderMaterial{ 0 });

    // Spawn a render entity mirroring a physics body (Transform synced from the world each step).
    auto spawnBody = [&](phys::BodyHandle h, render::MeshHandle mesh, glm::vec3 scale, glm::vec4 color) {
        const auto mat = static_cast<uint32_t>(materials.size());
        materials.push_back({ .baseColorFactor = color });
        const engine::Transform pose = world->pose(h);
        ecsWorld.spawn(Transform{ .position = pose.position, .rotation = pose.rotation, .scale = scale },
                       scene::RenderMesh{ mesh }, scene::RenderMaterial{ mat },
                       physics_ecs::RigidBody{ h });
    };
    auto sphereBody = [&](glm::vec3 pos, float r, glm::vec4 color) {
        phys::BodyDef d; d.type = phys::BodyType::Dynamic; d.mass = 1.0f;
        d.collider.type = phys::ColliderDesc::Type::Sphere; d.collider.sphere.radius = r;
        d.position = pos;
        const phys::BodyHandle h = world->createBody(d);
        spawnBody(h, sphereMesh, glm::vec3(2 * r), color);
        return h;
    };
    auto boxBody = [&](glm::vec3 pos, glm::vec3 half, glm::vec4 color) {
        phys::BodyDef d; d.type = phys::BodyType::Dynamic; d.mass = 1.0f;
        d.collider.type = phys::ColliderDesc::Type::Box; d.collider.box.halfExtents = half;
        d.position = pos;
        const phys::BodyHandle h = world->createBody(d);
        spawnBody(h, boxMesh, 2.0f * half, color);
        return h;
    };
    // Static pivot post (rendered as a small cube) at a world point.
    auto anchorAt = [&](glm::vec3 p) {
        phys::BodyDef d; d.type = phys::BodyType::Static;
        d.collider.type = phys::ColliderDesc::Type::Box; d.collider.box.halfExtents = glm::vec3(0.05f);
        d.position = p;
        const phys::BodyHandle h = world->createBody(d);
        spawnBody(h, boxMesh, glm::vec3(0.1f), glm::vec4(0.30f, 0.30f, 0.34f, 1.0f));
        return h;
    };
    // Revolute hinge connecting parent→child; pivot given in world space (bodies unrotated).
    auto hinge = [&](phys::BodyHandle parent, glm::vec3 parentCom, phys::BodyHandle child,
                     glm::vec3 childCom, glm::vec3 pivot, glm::vec3 axis = glm::vec3(0, 0, 1)) {
        phys::JointDef jd; jd.type = phys::JointType::Revolute; jd.a = parent; jd.b = child;
        jd.localAnchorA = pivot - parentCom; jd.localAnchorB = pivot - childCom;
        jd.localAxisA = axis; jd.localAxisB = axis;
        return world->createJoint(jd);
    };
    auto ball = [&](phys::BodyHandle parent, glm::vec3 parentCom, phys::BodyHandle child,
                    glm::vec3 childCom, glm::vec3 pivot) {
        phys::JointDef jd; jd.type = phys::JointType::Ball; jd.a = parent; jd.b = child;
        jd.localAnchorA = pivot - parentCom; jd.localAnchorB = pivot - childCom;
        return world->createJoint(jd);
    };
    auto weld = [&](phys::BodyHandle parent, glm::vec3 parentCom, phys::BodyHandle child,
                    glm::vec3 childCom, glm::vec3 pivot) {
        phys::JointDef jd; jd.type = phys::JointType::Fixed; jd.a = parent; jd.b = child;
        jd.localAnchorA = pivot - parentCom; jd.localAnchorB = pivot - childCom;
        return world->createJoint(jd);
    };

    const float topY = 2.6f, L = 0.7f, r = 0.09f;
    const glm::vec4 red(0.85f, 0.42f, 0.38f, 1), blu(0.42f, 0.55f, 0.85f, 1),
                    grn(0.45f, 0.80f, 0.52f, 1), yel(0.86f, 0.78f, 0.36f, 1), pur(0.66f, 0.45f, 0.82f, 1);

    // 1) Single revolute pendulum — built horizontal (+x) so it swings down on release.
    {
        const glm::vec3 piv(-6, topY, 0);
        const phys::BodyHandle a = anchorAt(piv);
        const glm::vec3 com = piv + glm::vec3(L, 0, 0);
        const phys::BodyHandle b = sphereBody(com, r, red);
        hinge(a, piv, b, com, piv);
    }
    // 2) Double revolute pendulum — both links horizontal ⇒ large, chaotic swing.
    {
        const glm::vec3 piv(-3, topY, 0);
        const phys::BodyHandle a = anchorAt(piv);
        const glm::vec3 c1 = piv + glm::vec3(L, 0, 0);
        const phys::BodyHandle b1 = sphereBody(c1, r, blu);
        hinge(a, piv, b1, c1, piv);
        const glm::vec3 piv2 = piv + glm::vec3(2 * L, 0, 0);      // end of link 1
        const glm::vec3 c2 = piv2 + glm::vec3(L, 0, 0);
        const phys::BodyHandle b2 = sphereBody(c2, r, blu * 0.8f + glm::vec4(0.1f));
        hinge(b1, c1, b2, c2, piv2);
    }
    // 3) Four-link revolute chain ("rope") — hangs from the pivot, built horizontal.
    {
        glm::vec3 piv(0, topY, 0);
        phys::BodyHandle parent = anchorAt(piv);
        glm::vec3 parentCom = piv;
        for (int i = 0; i < 4; ++i) {
            const glm::vec3 com = piv + glm::vec3(L, 0, 0);
            const glm::vec4 c = grn * (0.6f + 0.12f * float(i)) + glm::vec4(0, 0, 0, 1);
            const phys::BodyHandle b = sphereBody(com, r * 0.8f, glm::vec4(c.r, c.g, c.b, 1.0f));
            hinge(parent, parentCom, b, com, piv);
            parent = b; parentCom = com; piv = com + glm::vec3(L, 0, 0);
        }
    }
    // 4) Ball-joint pendulum — offset in z so gravity makes it swing OUT of the x–y plane.
    {
        const glm::vec3 piv(3, topY, 0);
        const phys::BodyHandle a = anchorAt(piv);
        const glm::vec3 com = piv + glm::vec3(0.6f * L, 0, 0.5f * L);
        const phys::BodyHandle b = sphereBody(com, r * 1.3f, yel);
        ball(a, piv, b, com, piv);
    }
    // 5) Fixed weld + revolute elbow — rigid upper arm (welded horizontal), free forearm hinge.
    {
        const glm::vec3 piv(6, topY, 0);
        const phys::BodyHandle a = anchorAt(piv);
        const glm::vec3 upperCom = piv + glm::vec3(0.35f, 0, 0);
        const phys::BodyHandle upper = boxBody(upperCom, glm::vec3(0.32f, 0.05f, 0.05f), pur);
        weld(a, piv, upper, upperCom, piv);                       // rigid: upper arm cannot droop
        const glm::vec3 elbow = piv + glm::vec3(0.7f, 0, 0);      // tip of the upper arm
        const glm::vec3 foreCom = elbow + glm::vec3(0.35f, 0, 0); // forearm built horizontal
        const phys::BodyHandle fore = boxBody(foreCom, glm::vec3(0.32f, 0.05f, 0.05f), pur * 0.8f + glm::vec4(0.1f, 0.1f, 0.1f, 0.2f));
        hinge(upper, upperCom, fore, foreCom, elbow);             // free hinge: forearm swings down
    }

    // Camera framing the whole row (x ∈ [-7, 7]).
    ecsWorld.spawn(Transform{ .position = glm::vec3(0.0f, 2.0f, 12.0f) },
                   Camera{ .fovY = glm::radians(55.0f), .nearZ = 0.05f, .farZ = 200.0f },
                   controls::FlyController{ .pitch = -4.0f });
    ecsWorld.setResource(scene::Background{ glm::vec4(0.10f, 0.12f, 0.16f, 1.0f) });
    ecsWorld.setResource(scene::SceneLighting{});
    ecsWorld.setResource(input::InputState{});
    ecsWorld.setResource(Time{});
    ecsWorld.setResource(physics_ecs::PhysicsWorldRef{ world.get() });
    ecsWorld.setResource(physics_ecs::FixedStep{ 1.0f / 120.0f });

    input::GlfwInput adapter(window);
    ecs::Schedule frameSched;
    frameSched.add("input", [&](ecs::World& w) { adapter.update(*w.getResource<input::InputState>()); });
    frameSched.add("fly-camera", controls::flyControllerSystem);
    ecs::Schedule simSched;
    simSched.add("actuators", physics_ecs::actuatorFlushSystem);
    simSched.add("step", physics_ecs::stepSystem);
    simSched.add("sync", physics_ecs::syncSystem);

    std::vector<render::RenderView> views;
    scene::ExtractedScene extracted;
    double last = glfwGetTime();
    double accumulator = 0.0;
    const double fixed = 1.0 / 120.0;
    std::printf("reduced_joints: L→R = single pendulum, double pendulum, 4-link chain, "
                "ball pendulum, fixed-weld+elbow arm. WASD move, right-drag look, Esc quit.\n");

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
    std::printf("reduced_joints: closed.\n");
}
