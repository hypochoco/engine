#include "harness/harness.h"
//
//  visual_window.cpp
//  engine::tst
//
//  ECS-driven windowed demo: a grid of sphere entities (Transform + RenderMesh +
//  RenderMaterial) in an ecs::World. Each frame a "bob" system animates the transforms, the
//  extraction system builds a RenderView, and the Renderer draws them in one instanced call.
//  Camera orbits the grid. Close the window to exit.
//
//  Run:  ./build/tst/visual_window   (optionally ENGINE_GRID=N for an NxN grid)
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

#include "engine/core/core.h"                     // engine::Transform
#include "engine/ecs/ecs.h"
#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
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
}

TST_CASE(graphics, visual, grid) {
    using namespace engine;
    using namespace engine::rhi;

    int grid = 32;
    if (const char* g = std::getenv("ENGINE_GRID")) { grid = std::atoi(g); if (grid < 1) grid = 1; }
    const int count = grid * grid;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — ECS instancing demo", nullptr, nullptr);
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
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.35f, 16, 32));
    render::Renderer renderer(device, geometry);

    // ECS world: one entity per grid cell, each with a material color.
    ecs::World world;
    std::vector<render::MaterialGPU> materials(count);
    const float spacing = 1.0f;
    const float extent  = (grid - 1) * spacing * 0.5f;
    for (int iz = 0; iz < grid; ++iz) {
        for (int ix = 0; ix < grid; ++ix) {
            const int i = iz * grid + ix;
            materials[i].baseColorFactor = glm::vec4(
                0.25f + 0.75f * float(ix) / grid, 0.35f, 0.25f + 0.75f * float(iz) / grid, 1.0f);
            world.spawn(
                Transform{ .position = glm::vec3(ix * spacing - extent, 0.0f, iz * spacing - extent) },
                scene::RenderMesh{ sphere },
                scene::RenderMaterial{ static_cast<uint32_t>(i) });
        }
    }

    scene::ExtractedScene extracted;
    std::printf("visual_window: ECS-driven %d spheres (%dx%d) at %ux%u — close to exit.\n",
                count, grid, grid, W, H);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const float t = static_cast<float>(glfwGetTime());

        // "bob" system: animate each transform's Y from its X/Z (a query over the world).
        world.query<Transform>().each([&](ecs::Entity, Transform& tr) {
            tr.position.y = 0.35f * std::sin(t * 1.5f + (tr.position.x + tr.position.z) * 0.6f);
        });

        // extraction system: world -> render lists.
        scene::extract(world, extracted); renderer.setMeshPipeline(pipe);

        const float r = extent * 2.2f + 3.0f;
        glm::vec3 eye(std::sin(t * 0.3f) * r, extent * 0.9f + 2.0f, std::cos(t * 0.3f) * r);
        render::RenderView view;
        view.view = glm::lookAt(eye, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
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
    std::printf("visual_window: closed.\n");
}
