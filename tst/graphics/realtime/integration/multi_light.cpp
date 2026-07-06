#include "harness/harness.h"
//
//  multi_light.cpp
//  engine::tst
//
//  RF4a: multi-light forward path. Renders a sphere twice — once with only a dim ambient term
//  (directional intensity 0, no point lights) and once with a bright point light in front — and
//  asserts the point light substantially brightens the surface facing it. This verifies the
//  PointLight upload + binding (buffer 3), the light-count param, and the shader's punctual
//  lighting loop. Also renders with several point lights to exercise the count path.
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <span>
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
std::vector<std::byte> readFileBin(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> data(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}
}

TST_CASE(graphics, integration, multi_light) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 128, H = 128;
    Device device = Device::createHeadless({});

    const std::string metallib = std::string(ENGINE_SHADER_DIR) + "/mesh.metallib";
    const auto blob = readFileBin(metallib);
    if (blob.empty()) { std::printf("FAIL: read %s\n", metallib.c_str()); TST_REQUIRE_MSG(false, "setup/verification failed"); }
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pdesc;
    pdesc.vertex = vs; pdesc.fragment = fs;
    pdesc.vertexLayout = render::coreVertexLayout();
    pdesc.colorFormats = std::span<const Format>(&colorFormat, 1);
    pdesc.depthFormat = Format::Depth32Float;
    pdesc.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pdesc);
    TST_REQUIRE_MSG(pipe.valid(), "pipeline creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.5f, 24, 48));
    render::Renderer renderer(device, geometry);

    render::MaterialGPU white; white.baseColorFactor = {1.0f, 1.0f, 1.0f, 1.0f};
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
    render::RenderItem item{ sphere, 0, 1 };
    renderer.setMeshPipeline(pipe);

    auto baseView = [&]() {
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0), glm::vec3(0, 1, 0));
        v.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 20.0f);
        v.target = colorRT; v.width = W; v.height = H;
        v.clearColor[0] = 0.0f; v.clearColor[1] = 0.0f; v.clearColor[2] = 0.0f; v.clearColor[3] = 1.0f;
        // Isolate the point lights: kill the directional sun, keep only a faint ambient.
        v.light.intensity = 0.0f;
        v.light.ambient = glm::vec3(0.05f, 0.05f, 0.05f);
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&white, 1);
        return v;
    };

    auto renderCenter = [&](const render::RenderView& v) {
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(frame));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        return int(px[(static_cast<size_t>(H / 2) * W + (W / 2)) * 4 + 0]);   // red channel
    };

    // (1) No point lights → ambient only (dim).
    render::RenderView unlit = baseView();
    const int unlitR = renderCenter(unlit);

    // (2) One bright point light in front of the sphere → surface facing camera lights up.
    render::PointLight pl; pl.position = {0.0f, 0.0f, 2.0f}; pl.radius = 5.0f;
    pl.color = {1.0f, 1.0f, 1.0f}; pl.intensity = 5.0f;
    render::RenderView lit = baseView();
    lit.pointLights = std::span<const render::PointLight>(&pl, 1);
    const int litR = renderCenter(lit);

    // (3) Several point lights (exercise the count loop; must stay valid + bright).
    render::PointLight many[4] = {
        { {0.0f, 0.0f, 2.0f}, 5.0f, {1.0f, 0.2f, 0.2f}, 3.0f },
        { {1.5f, 0.0f, 1.5f}, 5.0f, {0.2f, 1.0f, 0.2f}, 3.0f },
        { {-1.5f, 0.0f, 1.5f}, 5.0f, {0.2f, 0.2f, 1.0f}, 3.0f },
        { {0.0f, 1.5f, 1.5f}, 5.0f, {1.0f, 1.0f, 1.0f}, 3.0f },
    };
    render::RenderView multi = baseView();
    multi.pointLights = std::span<const render::PointLight>(many, 4);
    const int multiR = renderCenter(multi);

    std::printf("center red: unlit=%d  1-point-light=%d  4-point-lights=%d\n", unlitR, litR, multiR);

    TST_REQUIRE_MSG(unlitR < 40, "ambient-only center should be dim");
    TST_REQUIRE_MSG(litR > unlitR + 80, "point light did not brighten the surface");
    TST_REQUIRE_MSG(multiR > unlitR + 60, "multiple point lights did not contribute");

    std::printf("multi-light ok\n");
}
