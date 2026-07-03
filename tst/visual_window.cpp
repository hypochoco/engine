//
//  visual_window.cpp
//  engine::tst
//
//  Windowed instancing demo: a large grid of core spheres drawn in a single instanced draw
//  call through the Renderer/RenderView path, with a perspective camera orbiting the grid.
//  Each sphere bobs so the motion is obvious. Close the window to exit.
//
//  Run:  ./build/tst/visual_window   (optionally set ENGINE_GRID=N for an NxN grid)
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

#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"

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

int main() {
    using namespace engine;
    using namespace engine::rhi;

    int grid = 32;
    if (const char* g = std::getenv("ENGINE_GRID")) { grid = std::atoi(g); if (grid < 1) grid = 1; }
    const int count = grid * grid;

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return 1; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1000, 750, "engine — instancing demo", nullptr, nullptr);
    if (!window) { std::printf("FAIL: window\n"); glfwTerminate(); return 1; }

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw), H = static_cast<uint32_t>(fbh);

    WindowSurface surface{ window, W, H };
    Device device = Device::createWindowed(surface, {});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); return 1; }
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
    if (!pipe.valid()) { std::printf("FAIL: pipeline\n"); return 1; }

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.35f, 16, 32));
    render::Renderer renderer(device, geometry);

    // Grid layout in the XZ plane, centered at the origin. Per-instance color varies by cell.
    const float spacing = 1.0f;
    const float extent  = (grid - 1) * spacing * 0.5f;
    std::vector<render::InstanceData> instances(count);
    auto baseColor = [&](int ix, int iz) {
        return glm::vec3(0.3f + 0.7f * float(ix) / grid, 0.4f, 0.3f + 0.7f * float(iz) / grid);
    };

    render::RenderItem item;
    item.mesh = sphere; item.pipeline = pipe;
    item.firstInstance = 0; item.instanceCount = static_cast<uint32_t>(count);

    std::printf("visual_window: instancing %d spheres (%dx%d grid) at %ux%u — close to exit.\n",
                count, grid, grid, W, H);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const float t = static_cast<float>(glfwGetTime());

        // Animate per-instance model matrices (each sphere bobs on Y).
        for (int iz = 0; iz < grid; ++iz) {
            for (int ix = 0; ix < grid; ++ix) {
                const float x = ix * spacing - extent;
                const float z = iz * spacing - extent;
                const float y = 0.35f * std::sin(t * 1.5f + (x + z) * 0.6f);
                glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, y, z));
                render::InstanceData& d = instances[iz * grid + ix];
                d.model = m;
                d.normalModel = m;
                // color isn't per-instance in the vertex stream; bake into geometry later.
                (void)baseColor;
            }
        }

        // Orbiting camera looking at the grid center.
        const float r = extent * 2.2f + 3.0f;
        glm::vec3 eye(std::sin(t * 0.3f) * r, extent * 0.9f + 2.0f, std::cos(t * 0.3f) * r);
        render::RenderView view;
        view.view = glm::lookAt(eye, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
        view.proj = glm::perspective(glm::radians(55.0f), float(W) / float(H), 0.1f, 200.0f);
        view.width = W; view.height = H;
        view.items = std::span<const render::RenderItem>(&item, 1);
        view.instances = std::span<const render::InstanceData>(instances);

        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        view.target = frame.swapchainTarget();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("visual_window: closed.\n");
    return 0;
}
