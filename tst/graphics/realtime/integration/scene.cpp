#include "harness/harness.h"
//
//  scene_offscreen.cpp
//  engine::tst
//
//  Headless test of the ECS -> extraction -> Renderer path: spawn entities with
//  Transform + RenderMesh + RenderMaterial, extract into a RenderView, render offscreen, and
//  verify the center entity shows its material color.
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

TST_CASE(graphics, integration, scene) {
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

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm,
          .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 24, 48));
    render::Renderer renderer(device, geometry);

    // Materials (red center, green/blue offsets).
    std::vector<render::MaterialGPU> materials(3);
    materials[0].baseColorFactor = {1.0f, 0.2f, 0.2f, 1.0f};
    materials[1].baseColorFactor = {0.2f, 1.0f, 0.2f, 1.0f};
    materials[2].baseColorFactor = {0.2f, 0.2f, 1.0f, 1.0f};

    // ECS world: three entities.
    ecs::World world;
    const glm::vec3 positions[3] = { {0, 0, 0}, {-1.2f, 0, 0}, {1.2f, 0, 0} };
    for (uint32_t i = 0; i < 3; ++i) {
        world.spawn(Transform{ .position = positions[i] },
                    scene::RenderMesh{ sphere },
                    scene::RenderMaterial{ i });
    }

    // Extract -> render lists.
    scene::ExtractedScene extracted;
    scene::extract(world, extracted);
    renderer.setMeshPipeline(pipe);
    std::printf("extracted: %zu items, %zu instances\n", extracted.items.size(), extracted.instances.size());

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 20.0f);
    view.target = colorRT;
    view.width = W; view.height = H;
    view.items = std::span<const render::RenderItem>(extracted.items);
    view.instances = std::span<const render::InstanceData>(extracted.instances);
    view.materials = std::span<const render::MaterialGPU>(materials);

    FrameContext frame = device.beginFrame();
    renderer.render(frame, std::span<const render::RenderView>(&view, 1));
    device.endFrame(std::move(frame));

    std::vector<uint8_t> pixels(static_cast<size_t>(W) * H * 4);
    device.readback(color, std::as_writable_bytes(std::span<uint8_t>(pixels)));
    const uint8_t* c = &pixels[(static_cast<size_t>(H / 2) * W + W / 2) * 4];
    std::printf("center rgba = %u %u %u %u\n", c[0], c[1], c[2], c[3]);

    if (!(c[0] > 110 && c[0] > c[1] + 40 && c[0] > c[2] + 40)) {
        std::printf("FAIL: center is not the red-material entity\n"); TST_REQUIRE_MSG(false, "setup/verification failed");
    }
    std::printf("scene offscreen ok\n");
}
