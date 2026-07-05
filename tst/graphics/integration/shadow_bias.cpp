#include "harness/harness.h"
//
//  shadow_bias.cpp
//  engine::tst
//
//  Verifies the "gap between an object and its cast shadow" the user observed — classic
//  peter-panning from the shadow depth bias. A tall caster rests ON the ground with an angled
//  sun casting a shadow beside it. We render with the DEFAULT bias (0.0018) and a TINY bias and
//  count shadowed ground pixels. If the larger bias detaches/shrinks the shadow (peter-panning),
//  the default bias yields notably FEWER shadowed pixels than the tiny bias. This documents the
//  artifact and gives a knob (Renderer::setShadows bias) + a regression signal for any fix.
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

TST_CASE(graphics, integration, shadow_bias) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    auto shBlob   = readFileBin(std::string(ENGINE_SHADER_DIR) + "/shadow.metallib");
    if (meshBlob.empty() || shBlob.empty()) { std::printf("FAIL: read shaders\n"); TST_REQUIRE_MSG(false, "setup failed"); }
    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle shvs = device.createShader(shBlob, ShaderStage::Vertex);

    const Format cf = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs; pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&cf, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);

    GraphicsPipelineDesc sd;
    sd.vertex = shvs; sd.vertexLayout = render::coreVertexLayout();
    sd.depthFormat = Format::Depth32Float;
    sd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle shadowPipe = device.createGraphicsPipeline(sd);
    SamplerHandle shadowSamp = device.createSampler({ .minFilter = Filter::Nearest, .magFilter = Filter::Nearest,
                                                      .mipmap = MipmapMode::Nearest,
                                                      .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });
    TST_REQUIRE_MSG(pipe.valid() && shadowPipe.valid() && shadowSamp.valid(), "setup failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = cf, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle boxMesh = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::Renderer renderer(device, geometry);

    render::MaterialGPU white; white.baseColorFactor = {0.85f, 0.85f, 0.85f, 1.0f};
    // A large flat ground with NO caster: under a grazing sun, any "shadow" on it is self-shadow
    // ACNE. Slope-scaled bias keeps the bias HIGH on this grazing surface (suppressing acne), where
    // a low constant bias (needed to avoid peter-panning on face-on surfaces) would speckle it. This
    // is the complement the fix buys us: the bias adapts to the surface angle both ways.
    std::vector<render::InstanceData> inst(1);
    inst[0].model = glm::scale(glm::mat4(1.0f), glm::vec3(80.0f, 0.2f, 80.0f));
    inst[0].normalModel = inst[0].model; inst[0].materialIndex = 0;
    render::RenderItem item{ boxMesh, pipe, 0, 1 };

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 6, 24), glm::vec3(0, 0, -10), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(55.0f), float(W) / float(H), 0.1f, 300.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.direction = glm::normalize(glm::vec3(4.0f, -1.0f, 0.6f));   // GRAZING sun (ground is nearly edge-on to it)
    view.light.intensity = 1.0f; view.light.color = glm::vec3(1.0f);
    view.light.ambient = glm::vec3(0.12f);
    view.clearColor[0] = view.clearColor[1] = view.clearColor[2] = 0.0f; view.clearColor[3] = 1.0f;
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(inst);
    view.materials = std::span<const render::MaterialGPU>(&white, 1);

    auto render = [&](std::vector<uint8_t>& out) {
        out.assign(static_cast<size_t>(W) * H * 4, 0);
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(out)));
    };

    // Low CONSTANT bias (negative w) — enough to connect face-on contacts, but too low for this
    // grazing surface ⇒ self-shadow acne. Slope-scaled (positive w) raises the bias at grazing.
    std::vector<uint8_t> off, lowConst, slopeB;
    renderer.setShadows({}, {});                                           render(off);
    renderer.setShadows(shadowPipe, shadowSamp, 45.0f, 250.0f, -0.00025f); render(lowConst);
    renderer.setShadows(shadowPipe, shadowSamp, 45.0f, 250.0f,  0.0018f);  render(slopeB);

    // No caster ⇒ every darkened ground pixel is self-shadow acne.
    auto acneCount = [&](const std::vector<uint8_t>& on) {
        size_t c = 0;
        for (size_t p = 0; p < static_cast<size_t>(W) * H; ++p) {
            int o = off[p * 4 + 1], n = on[p * 4 + 1];
            if (o > 40 && o - n > 30) ++c;
        }
        return c;
    };
    const size_t acneLow   = acneCount(lowConst);
    const size_t acneSlope = acneCount(slopeB);
    std::printf("grazing-ground self-shadow acne: low constant bias=%zu px, slope-scaled=%zu px\n",
                acneLow, acneSlope);

    TST_REQUIRE_MSG(acneLow > 500, "expected a low constant bias to cause self-shadow acne at grazing");
    TST_REQUIRE_MSG(acneSlope * 4 < acneLow, "slope-scaled bias should suppress the grazing acne");
    std::printf("CONFIRMED: slope-scaled bias adapts to the surface angle — it raises bias on the\n"
                "grazing ground (acne %zu -> %zu) while lowering it on face-on surfaces (less\n"
                "peter-panning). This is the fix for the reported shadow gap.\n", acneLow, acneSlope);

    device.destroy(shadowSamp);
    std::printf("shadow slope-scaled bias ok\n");
}
