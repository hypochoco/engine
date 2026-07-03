#include "harness/harness.h"
//
//  mesh_offscreen.cpp
//  engine::tst
//
//  Headless test of the full render path: GeometryStore + Renderer + RenderView, drawing
//  several instanced core spheres offscreen with a perspective camera + depth, then reading
//  pixels back and checking the center sphere is lit while a corner stays the clear color.
//  This is the verifiable counterpart to the windowed instancing demo.
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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

TST_CASE(graphics, integration, mesh) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;

    Device device = Device::createHeadless({});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFile(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup/verification failed"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const rhi::VertexLayout layout = render::coreVertexLayout();
    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = layout;
    pdesc.topology = Topology::TriangleList;
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    if (!pipe.valid()) { std::printf("FAIL: pipeline\n"); TST_REQUIRE_MSG(false, "setup/verification failed"); }

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm,
          .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 24, 48));
    render::Renderer renderer(device, geometry);

    // Three instances (center + two offset), each with its own material color.
    std::vector<render::MaterialGPU> materials(3);
    materials[0].baseColorFactor = {1.0f, 0.2f, 0.2f, 1.0f};   // red   (center)
    materials[1].baseColorFactor = {0.2f, 1.0f, 0.2f, 1.0f};   // green (left)
    materials[2].baseColorFactor = {0.2f, 0.2f, 1.0f, 1.0f};   // blue  (right)

    const glm::vec3 positions[3] = { {0, 0, 0}, {-1.2f, 0, 0}, {1.2f, 0, 0} };
    std::vector<render::InstanceData> instances(3);
    for (uint32_t i = 0; i < 3; ++i) {
        instances[i].model = glm::translate(glm::mat4(1.0f), positions[i]);
        instances[i].normalModel = instances[i].model;
        instances[i].materialIndex = i;
    }
    render::RenderItem item;
    item.mesh = sphere; item.pipeline = pipe;
    item.firstInstance = 0; item.instanceCount = static_cast<uint32_t>(instances.size());

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 20.0f);
    view.target = colorRT;
    view.width = W; view.height = H;
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(instances);
    view.materials = std::span<const render::MaterialGPU>(materials);

    FrameContext frame = device.beginFrame();
    renderer.render(frame, std::span<const render::RenderView>(&view, 1));
    device.endFrame(std::move(frame));

    std::vector<uint8_t> pixels(static_cast<size_t>(W) * H * 4);
    device.readback(color, std::as_writable_bytes(std::span<uint8_t>(pixels)));
    auto px = [&](uint32_t x, uint32_t y) { return &pixels[(static_cast<size_t>(y) * W + x) * 4]; };
    const uint8_t* center = px(W / 2, H / 2);
    const uint8_t* corner = px(2, 2);

    std::printf("center rgba = %u %u %u %u\n", center[0], center[1], center[2], center[3]);
    std::printf("corner rgba = %u %u %u %u\n", corner[0], corner[1], corner[2], corner[3]);
    std::printf("instances = %zu, materials = %zu\n", instances.size(), materials.size());

    // Center instance uses the red material → red channel should dominate.
    const bool centerRed    = center[0] > 110 && center[0] > center[1] + 40 && center[0] > center[2] + 40;
    const bool cornerIsClear = corner[0] < 70 && corner[1] < 70 && corner[2] < 70;
    if (!centerRed)     { std::printf("FAIL: center is not the red-material sphere\n"); TST_REQUIRE_MSG(false, "setup/verification failed"); }
    if (!cornerIsClear) { std::printf("FAIL: corner is not the clear color\n"); TST_REQUIRE_MSG(false, "setup/verification failed"); }

    std::printf("mesh offscreen ok\n");
}
