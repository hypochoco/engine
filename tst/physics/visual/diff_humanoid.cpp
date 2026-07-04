#include "harness/harness.h"
//
//  diff_humanoid.cpp
//  engine::tst — physics / visual
//
//  Phase F: the DIFFERENTIABLE humanoid, visualized. The whole rollout is driven by the
//  header-only differentiable engine (converter + Scalar-generic ABA + soft ground contact used for
//  gradients) — NOT the production PhysicsWorld. Each frame we step the differentiable model (double)
//  and copy every link's world pose (pos + orientation from linkWorld) into its ECS render entity.
//  So this is an eyeball check that the differentiable dynamics + converter + contact produce
//  physical motion: an uncontrolled humanoid released at standing height collapses under gravity and
//  piles onto the plane (a passive ragdoll). Contact spheres are attached to EVERY body (not just
//  the feet) so a fallen body rests on the ground instead of phasing through, and it runs passively
//  (no torque) so it can't pump itself unstable. Uses softened contact + substeps=64.
//
//  Run:  ./build/tst/visuals diff_humanoid
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
glm::vec3 colliderScale(const engine::physics::ColliderDesc& c) {
    using T = engine::physics::ColliderDesc::Type;
    switch (c.type) {
        case T::Box:     return 2.0f * glm::vec3(c.box.halfExtents);
        case T::Capsule: return glm::vec3(2 * c.capsule.radius, 2 * (c.capsule.halfHeight + c.capsule.radius), 2 * c.capsule.radius);
        case T::Sphere:  return glm::vec3(2 * c.sphere.radius);
        default:         return glm::vec3(0.2f);
    }
}
// diff M3<double> (row-major math) → glm::quat.
glm::quat toQuat(const engine::physics::diff::M3<double>& R) {
    glm::mat3 g;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) g[j][i] = static_cast<float>(R.m[i][j]);
    return glm::quat_cast(g);
}
}

TST_CASE(physics, visual, diff_humanoid) {
    using namespace engine;
    using namespace engine::rhi;
    namespace phys = engine::physics;
    namespace diff = engine::physics::diff;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — differentiable humanoid", nullptr, nullptr);
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

    auto rotateZ90 = [](MeshData mesh) {
        const glm::mat3 R = glm::mat3(glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0, 0, 1)));
        for (Vertex& v : mesh.vertices) { v.position = R * v.position; v.normal = R * v.normal; }
        return mesh;
    };
    const phys::ArticulationDef refDef = phys::makeHumanoid();
    std::vector<render::MeshHandle> bodyMesh(refDef.bodies.size());
    std::vector<glm::vec3> bodyScale(refDef.bodies.size(), glm::vec3(1.0f));
    for (size_t i = 0; i < refDef.bodies.size(); ++i) {
        const phys::ColliderDesc& col = refDef.bodies[i].collider;
        const bool isFoot = i >= refDef.bodies.size() - 2;
        if (i == 0)      bodyMesh[i] = geometry.upload(rotateZ90(primitives::makeCapsule(0.08f, 0.045f)));
        else if (i == 1) bodyMesh[i] = geometry.upload(primitives::makeCapsule(0.11f, 0.075f));
        else if (i == 2) bodyMesh[i] = geometry.upload(rotateZ90(primitives::makeCapsule(0.06f, 0.12f)));
        else if (i == 3) { bodyMesh[i] = sphereMesh; bodyScale[i] = glm::vec3(0.18f); }
        else if (col.type == phys::ColliderDesc::Type::Capsule) bodyMesh[i] = geometry.upload(primitives::makeCapsule(col.capsule.radius, col.capsule.halfHeight));
        else if (isFoot) { bodyMesh[i] = boxMesh; bodyScale[i] = colliderScale(col); }
        else { bodyMesh[i] = sphereMesh; bodyScale[i] = colliderScale(col); }
    }

    // The differentiable engine IS the simulation. Contact geometry on EVERY body — shape-aware
    // (capsule end-caps + box corners, Feature 3) so a collapsing, uncontrolled ragdoll piles ON the
    // plane and its limbs rest at their true surface instead of clipping through. Softened contact +
    // 64 substeps for stability; passive (no torque) so it just falls and settles.
    diff::DiffModel model = diff::articulationToDiffModel(refDef, diff::DiffContact::All);
    diff::DiffState<double> state = diff::makeState<double>(model);
    state.basePos = { 0, 0.99, 0 };
    const diff::V3<double> gravity{ 0, -9.81, 0 };
    const int substeps = 64;
    const double substepDt = (1.0 / 60.0) / substeps;
    const std::vector<double> tau(static_cast<size_t>(model.ndofJoints), 0.0);   // passive ragdoll
    auto links = [&] { return diff::linkWorld<double>(model, state); };

    ecs::World ecsWorld;
    std::vector<render::MaterialGPU> materials;
    materials.push_back({ .baseColorFactor = glm::vec4(0.5f, 0.5f, 0.55f, 1.0f) });
    ecsWorld.spawn(Transform{}, scene::RenderMesh{ planeMesh }, scene::RenderMaterial{ 0 });

    std::vector<ecs::Entity> bodyEntity(refDef.bodies.size());
    const auto lw0 = links();
    for (size_t i = 0; i < refDef.bodies.size(); ++i) {
        const auto mat = static_cast<uint32_t>(materials.size());
        glm::vec4 c = glm::vec4(0.85f, 0.42f, 0.38f, 1.0f) * (0.72f + 0.4f * float(i) / float(refDef.bodies.size())); c.a = 1.0f;
        materials.push_back({ .baseColorFactor = c });
        bodyEntity[i] = ecsWorld.spawn(
            Transform{ .position = glm::vec3(lw0[i].pos.x, lw0[i].pos.y, lw0[i].pos.z), .rotation = toQuat(lw0[i].rot), .scale = bodyScale[i] },
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
    std::printf("diff_humanoid: differentiable-engine passive ragdoll (all-body soft contact). WASD move, right-drag look, Esc quit.\n");

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
    std::printf("diff_humanoid: closed.\n");
}
