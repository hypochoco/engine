//
//  visual_window.cpp
//  engine::tst
//
//  Opens a real window (GLFW + CAMetalLayer) and renders a lit core sphere with a proper
//  perspective camera (MVP uniform). The sphere orbits so the motion + perspective are
//  obvious, and the background gently pulses. Close the window to exit.
//
//  Run:  ./build/tst/visual_window
//

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE   // Metal clip-space depth is [0,1]
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

namespace {
struct Uniforms { glm::mat4 mvp; glm::mat4 normalMatrix; };

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

    if (!glfwInit()) { std::printf("FAIL: glfwInit\n"); return 1; }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "engine — visual test", nullptr, nullptr);
    if (!window) { std::printf("FAIL: create window\n"); glfwTerminate(); return 1; }

    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    const auto W = static_cast<uint32_t>(fbw);
    const auto H = static_cast<uint32_t>(fbh);

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
    pdesc.vertex = vs;
    pdesc.fragment = fs;
    pdesc.vertexLayout = layout;
    pdesc.topology = Topology::TriangleList;
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    if (!pipe.valid()) { std::printf("FAIL: pipeline\n"); return 1; }

    TextureHandle depthTex = device.createTexture(
        { .width = W, .height = H, .format = Format::Depth32Float, .usage = TextureUsage::DepthTarget });
    RenderTargetHandle depthRT = device.createRenderTarget(depthTex);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 32, 64));
    const auto range = geometry.range(sphere);

    BufferHandle ubuf = device.createBuffer(
        { .size = sizeof(Uniforms), .usage = BufferUsage::Uniform, .memory = MemoryMode::CpuToGpu });

    const float aspect = float(W) / float(H);
    const glm::mat4 proj = glm::perspective(glm::radians(50.0f), aspect, 0.1f, 20.0f);
    const glm::mat4 view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));

    std::printf("visual_window: perspective camera, orbiting lit sphere at %ux%u — close the window to exit.\n", W, H);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const float t = static_cast<float>(glfwGetTime());

        // Orbit the sphere; perspective makes the near/far motion read as depth.
        glm::mat4 model = glm::translate(glm::mat4(1.0f),
                                         glm::vec3(std::sin(t) * 0.9f, std::cos(t * 1.3f) * 0.4f,
                                                   std::cos(t) * 0.9f));
        Uniforms u{ proj * view * model, model };
        device.updateBuffer(ubuf, 0, std::as_bytes(std::span<const Uniforms>(&u, 1)));

        FrameContext frame = device.beginFrame();
        if (!frame.swapchainTarget().valid()) { device.endFrame(std::move(frame)); continue; }
        CommandList cl = device.commandList(frame);

        ColorAttachment ca;
        ca.target = frame.swapchainTarget();
        ca.load = LoadOp::Clear; ca.store = StoreOp::Store;
        ca.clearColor[0] = 0.08f + 0.04f * std::sin(t);
        ca.clearColor[1] = 0.10f;
        ca.clearColor[2] = 0.14f + 0.04f * std::cos(t);
        ca.clearColor[3] = 1.0f;

        DepthAttachment da;
        da.target = depthRT;
        da.load = LoadOp::Clear; da.store = StoreOp::DontCare;
        da.clearDepth = 1.0f;

        RenderTargetDesc rtd;
        rtd.color = std::span<const ColorAttachment>(&ca, 1);
        rtd.depth = &da;
        rtd.width = W; rtd.height = H;

        BufferBinding ub{ .binding = 0, .buffer = ubuf };
        ResourceBindings rb; rb.buffers = std::span<const BufferBinding>(&ub, 1);

        cl.beginRendering(rtd);
        cl.bindPipeline(pipe);
        cl.setViewport(0, 0, float(W), float(H));
        cl.bindResources(rb);
        cl.bindVertexBuffer(geometry.vertexBuffer(), 0);
        cl.bindIndexBuffer(geometry.indexBuffer(), IndexType::Uint32);
        cl.drawIndexed(range.indexCount, 1, range.firstIndex, range.vertexOffset, 0);
        cl.endRendering();

        device.submit(frame, cl);
        device.endFrame(std::move(frame));
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    std::printf("visual_window: closed.\n");
    return 0;
}
