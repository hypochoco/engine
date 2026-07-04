#include "harness/harness.h"
//
//  amp_humanoid.cpp
//  engine::tst — physics / visual
//
//  The DeepMimic/AMP humanoid rig (makeAMPHumanoid — 15 bodies / 28 DOF), visualized as a passive
//  all-body-contact ragdoll driven by the differentiable engine (converter + Scalar-generic ABA +
//  soft ground contact), exactly like diff_humanoid but on the richer mocap-friendly rig. Meshes are
//  chosen per collider type (sphere/capsule/box), so the same loop renders any rig. This is the
//  eyeball check that the AMP rig flows through the whole differentiable stack.
//
//  Run:  ./build/tst/visuals amp_humanoid
//        ENGINE_JOINT_DAMPING=0 / ENGINE_GROUND_C=... / ENGINE_SUBSTEPS=... / ENGINE_SEMI=1 to tune.
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
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
#include "engine/physics/diff/diff_environment.h"
#include "engine/physics/dynamics/articulation.h"
#include "engine/physics_ecs/components.h"

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
// diff M3<double> (row-major math) → glm::quat.
glm::quat toQuat(const engine::physics::diff::M3<double>& R) {
    glm::mat3 g;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) g[j][i] = static_cast<float>(R.m[i][j]);
    return glm::quat_cast(g);
}
}

TST_CASE(physics, visual, amp_humanoid) {
    using namespace engine;
    using namespace engine::rhi;
    namespace phys = engine::physics;
    namespace diff = engine::physics::diff;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — AMP humanoid (28 DOF)", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return; }

    int fbw = 0, fbh = 0; glfwGetFramebufferSize(window, &fbw, &fbh);
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
    pdesc.vertex = vs; pdesc.fragment = fs; pdesc.vertexLayout = layout;
    pdesc.topology = Topology::TriangleList;
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);

    render::GeometryStore geometry(device);
    render::MeshHandle boxMesh    = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::MeshHandle sphereMesh = geometry.upload(primitives::makeSphere(0.5f, 20, 32));
    render::MeshHandle planeMesh  = geometry.upload(primitives::makePlane(30.0f, 1));
    render::Renderer renderer(device, geometry);

    // Rig meshes chosen per collider type — works for any ArticulationDef (rig-agnostic).
    const phys::ArticulationDef refDef = phys::makeAMPHumanoid();
    std::vector<render::MeshHandle> bodyMesh(refDef.bodies.size());
    std::vector<glm::vec3> bodyScale(refDef.bodies.size(), glm::vec3(1.0f));
    for (size_t i = 0; i < refDef.bodies.size(); ++i) {
        const phys::ColliderDesc& c = refDef.bodies[i].collider;
        switch (c.type) {
            case phys::ColliderDesc::Type::Capsule:
                bodyMesh[i] = geometry.upload(primitives::makeCapsule(c.capsule.radius, c.capsule.halfHeight));
                break;
            case phys::ColliderDesc::Type::Box:
                bodyMesh[i] = boxMesh; bodyScale[i] = 2.0f * glm::vec3(c.box.halfExtents);
                break;
            default:  // Sphere
                bodyMesh[i] = sphereMesh; bodyScale[i] = glm::vec3(2.0f * c.sphere.radius);
                break;
        }
    }

    // The differentiable engine drives the sim. All-body shape-aware contact so the passive ragdoll
    // piles ON the plane; shipped contact defaults (k=4e4, C=1000) + a little joint damping.
    diff::DiffModel model = diff::articulationToDiffModel(refDef, diff::DiffContact::All);
    diff::DiffState<double> state = diff::makeState<double>(model);
    state.basePos = { -0.7, 1.05, 0 };   // ragdoll on the left (the static reference stands on the right)
    const diff::V3<double> gravity{ 0, -9.81, 0 };
    const bool semiImplicit = std::getenv("ENGINE_SEMI") != nullptr;
    model.contactIntegration = semiImplicit ? diff::ContactIntegration::SemiImplicit : diff::ContactIntegration::Explicit;
    const int substeps = std::getenv("ENGINE_SUBSTEPS") ? std::max(1, std::atoi(std::getenv("ENGINE_SUBSTEPS"))) : 64;
    model.jointDamping = std::getenv("ENGINE_JOINT_DAMPING") ? std::atof(std::getenv("ENGINE_JOINT_DAMPING")) : 0.3;
    if (std::getenv("ENGINE_GROUND_K")) model.groundK = std::atof(std::getenv("ENGINE_GROUND_K"));
    if (std::getenv("ENGINE_GROUND_C")) model.groundC = std::atof(std::getenv("ENGINE_GROUND_C"));
    const double substepDt = (1.0 / 60.0) / substeps;
    const std::vector<double> tau(static_cast<size_t>(model.ndofJoints), 0.0);   // passive ragdoll
    std::printf("amp_humanoid: 15 bodies / %d DOF, contact=%s substeps=%d jointDamping=%.3f groundK=%.0f groundC=%.0f\n",
                model.ndofJoints, semiImplicit ? "SemiImplicit" : "Explicit", substeps, model.jointDamping, model.groundK, model.groundC);
    auto links = [&] { return diff::linkWorld<double>(model, state); };

    ecs::World ecsWorld;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({ .baseColorFactor = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f) });
    ecsWorld.spawn(Transform{}, scene::RenderMesh{ planeMesh }, scene::RenderMaterial{ 0 });

    std::vector<ecs::Entity> bodyEntity(refDef.bodies.size());
    const auto lw0 = links();
    for (size_t i = 0; i < refDef.bodies.size(); ++i) {
        const auto mat = static_cast<uint32_t>(materials.size());
        glm::vec4 c = glm::vec4(0.42f, 0.62f, 0.85f, 1.0f) * (0.7f + 0.42f * float(i) / float(refDef.bodies.size())); c.a = 1.0f;
        materials.push_back({ .baseColorFactor = c });
        bodyEntity[i] = ecsWorld.spawn(
            Transform{ .position = glm::vec3(lw0[i].pos.x, lw0[i].pos.y, lw0[i].pos.z), .rotation = toQuat(lw0[i].rot), .scale = bodyScale[i] },
            scene::RenderMesh{ bodyMesh[i] }, scene::RenderMaterial{ mat });
    }

    // A STATIC reference AMP rig: the authored standing (rest) pose, frozen (never stepped), standing on
    // the right so its 15-body/28-DOF layout is visible next to the settling ragdoll.
    diff::DiffState<double> staticState = diff::makeState<double>(model);
    staticState.basePos = { 0.7, 1.022, 0 };   // rest pose stands the rig with its soles on y≈0
    const auto lwStatic = diff::linkWorld<double>(model, staticState);
    for (size_t i = 0; i < refDef.bodies.size(); ++i) {
        const auto mat = static_cast<uint32_t>(materials.size());
        glm::vec4 c = glm::vec4(0.85f, 0.72f, 0.42f, 1.0f) * (0.7f + 0.42f * float(i) / float(refDef.bodies.size())); c.a = 1.0f;
        materials.push_back({ .baseColorFactor = c });   // warm tint to distinguish from the (blue) ragdoll
        ecsWorld.spawn(
            Transform{ .position = glm::vec3(lwStatic[i].pos.x, lwStatic[i].pos.y, lwStatic[i].pos.z), .rotation = toQuat(lwStatic[i].rot), .scale = bodyScale[i] },
            scene::RenderMesh{ bodyMesh[i] }, scene::RenderMaterial{ mat });
    }

    ecsWorld.spawn(Transform{ .position = glm::vec3(0.0f, 1.2f, 4.5f) },
                   Camera{ .fovY = glm::radians(55.0f), .nearZ = 0.05f, .farZ = 200.0f },
                   controls::FlyController{ .pitch = -8.0f });
    ecsWorld.setResource(scene::Background{ glm::vec4(0.10f, 0.12f, 0.16f, 1.0f) });
    ecsWorld.setResource(scene::SceneLighting{});
    ecsWorld.setResource(input::InputState{});
    ecsWorld.setResource(Time{});

    input::GlfwInput adapter(window);
    ecs::Schedule frameSched;
    frameSched.add("input", [&](ecs::World& w) { adapter.update(*w.getResource<input::InputState>()); });
    frameSched.add("fly-camera", controls::flyControllerSystem);

    std::vector<render::RenderView> views;
    scene::ExtractedScene extracted;
    double last = glfwGetTime(), accumulator = 0.0;
    const double fixed = 1.0 / 60.0;
    std::printf("amp_humanoid: passive AMP ragdoll (differentiable engine). WASD move, right-drag look, Esc quit.\n");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double now = glfwGetTime();
        ecsWorld.getResource<Time>()->dt = static_cast<float>(now - last);
        accumulator += std::min(now - last, 0.1); last = now;

        frameSched.run(ecsWorld);
        const input::InputState& in = *ecsWorld.getResource<input::InputState>();
        if (in.keyPressed(input::Key::Escape)) glfwSetWindowShouldClose(window, GLFW_TRUE);
        if (in.mousePressed(input::MouseButton::Right))  adapter.setCursorCaptured(true);
        if (in.mouseReleased(input::MouseButton::Right)) adapter.setCursorCaptured(false);

        while (accumulator >= fixed) {
            for (int i = 0; i < substeps; ++i) diff::diffSubstep(model, state, tau, gravity, substepDt);
            const auto lw = links();
            for (size_t i = 0; i < bodyEntity.size(); ++i) {
                Transform* tr = ecsWorld.get<Transform>(bodyEntity[i]);
                tr->position = glm::vec3(lw[i].pos.x, lw[i].pos.y, lw[i].pos.z);
                tr->rotation = toQuat(lw[i].rot);
            }
            accumulator -= fixed;
        }

        scene::extract(ecsWorld, pipe, extracted);
        scene::extractViews(ecsWorld, extracted, views, W, H);
        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        for (auto& v : views) { v.materials = std::span<const render::MaterialGPU>(materials); v.target = frame.swapchainTarget(); }
        renderer.render(frame, std::span<const render::RenderView>(views));
        device.endFrame(std::move(frame));
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("amp_humanoid: closed.\n");
}
