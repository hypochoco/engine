#include "harness/harness.h"
//
//  sky.cpp
//  engine::tst
//
//  Procedural sky pass (RF6 sky). An opaque box sits in the centre of the frame with sky filling
//  the background. We verify the four properties from the design note (leaning on cheap, robust
//  pixel checks — this is a realtime feature, not a physical model):
//    1. Depth correctness (the key invariant): the box's pixels are IDENTICAL sky-on vs sky-off —
//       the sky (a far-plane fullscreen triangle, depth-tested) only fills background, never the
//       scene. Meanwhile background pixels change from the flat clear colour to the sky.
//    2. Gradient: a pixel looking up (zenith) is bluer and darker than one at the horizon.
//    3. Sun disc: with the sun in front there is a very bright background pixel; with the same sun
//       behind the camera there is not.
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

TST_CASE(graphics, integration, sky) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    auto skyBlob  = readFileBin(std::string(ENGINE_SHADER_DIR) + "/sky.metallib");
    if (meshBlob.empty() || skyBlob.empty()) { TST_REQUIRE_MSG(false, "read shaders failed"); }

    ShaderHandle vs   = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs   = device.createShader(meshBlob, ShaderStage::Fragment);
    ShaderHandle skvs = device.createShader(skyBlob, ShaderStage::Vertex);
    ShaderHandle skfs = device.createShader(skyBlob, ShaderStage::Fragment);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs;
    pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);

    // Sky pipeline: fullscreen (no vertex layout), depth test LessEqual + NO depth write, no cull,
    // color format = the forward target's, depth format = the shared depth buffer's.
    GraphicsPipelineDesc sk;
    sk.vertex = skvs; sk.fragment = skfs;
    sk.colorFormats = std::span<const Format>(&colorFormat, 1);
    sk.depthFormat = Format::Depth32Float;
    sk.depth = { .test = true, .write = false, .op = CompareOp::LessEqual };
    sk.raster.cull = CullMode::None;
    PipelineHandle skyPipe = device.createGraphicsPipeline(sk);
    TST_REQUIRE_MSG(pipe.valid() && skyPipe.valid(), "pipeline creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle box = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::Renderer renderer(device, geometry);
    // Broad, bright sun glow so the sun side is unambiguously brighter (also exercises setSkyColors).
    renderer.setSkyColors(/*zenith*/ glm::vec3(0.10f, 0.24f, 0.55f), /*horizon*/ glm::vec3(0.62f, 0.72f, 0.86f),
                          /*ground*/ glm::vec3(0.28f, 0.26f, 0.24f), /*sunColor*/ glm::vec3(15.0f, 14.0f, 12.0f),
                          /*sunRadiusDeg*/ 3.0f, /*glowExp*/ 40.0f, /*glowStrength*/ 2.5f, /*brightness*/ 1.0f);

    render::MaterialGPU white; white.baseColorFactor = {0.85f, 0.85f, 0.85f, 1.0f};
    render::InstanceData inst; inst.model = glm::scale(glm::mat4(1.0f), glm::vec3(2.0f));
    inst.normalModel = inst.model; inst.materialIndex = 0;
    render::RenderItem item{ box, 0, 1 };
    renderer.setMeshPipeline(pipe);

    render::RenderView view;
    view.view = glm::lookAt(glm::vec3(0, 0, 6), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    view.proj = glm::perspective(glm::radians(45.0f), float(W) / float(H), 0.1f, 100.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.color = glm::vec3(1.0f); view.light.intensity = 1.0f; view.light.ambient = glm::vec3(0.2f);
    // Sun up and to the front-right (in view). light.direction is where light TRAVELS = -sunDir.
    const glm::vec3 sunFront = glm::normalize(glm::vec3(0.6f, 0.45f, -0.5f));
    view.light.direction = -sunFront;
    view.clearColor[0] = 1.0f; view.clearColor[1] = 0.0f; view.clearColor[2] = 1.0f; view.clearColor[3] = 1.0f; // magenta bg
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(&inst, 1);
    view.materials = std::span<const render::MaterialGPU>(&white, 1);

    auto renderTo = [&](std::vector<uint8_t>& out) {
        out.assign(static_cast<size_t>(W) * H * 4, 0);
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&view, 1));
        device.endFrame(std::move(frame));
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(out)));
    };
    auto at = [&](const std::vector<uint8_t>& img, uint32_t x, uint32_t y, int c) {
        return int(img[(static_cast<size_t>(y) * W + x) * 4 + c]);
    };

    // (1) sky OFF: flat magenta background.
    renderer.setSky({});
    std::vector<uint8_t> off; renderTo(off);
    // (2) sky ON, sun in front.
    renderer.setSky(skyPipe);
    std::vector<uint8_t> on; renderTo(on);
    // (3) sky ON, sun behind the camera (same elevation, azimuth flipped so the disc is off-screen).
    view.light.direction = -glm::normalize(glm::vec3(0.6f, 0.45f, 0.5f));
    std::vector<uint8_t> behind; renderTo(behind);

    // --- Depth correctness: the box (centre) is untouched by the sky; background is replaced. ---
    const uint32_t cx = W / 2, cy = H / 2;
    const int objDiff = std::abs(at(on, cx, cy, 0) - at(off, cx, cy, 0)) +
                        std::abs(at(on, cx, cy, 1) - at(off, cx, cy, 1)) +
                        std::abs(at(on, cx, cy, 2) - at(off, cx, cy, 2));
    const bool offCentreIsObject = !(at(off, cx, cy, 0) > 200 && at(off, cx, cy, 1) < 60 && at(off, cx, cy, 2) > 200);
    TST_REQUIRE_MSG(offCentreIsObject, "centre pixel should be the box, not the clear colour");
    TST_REQUIRE_MSG(objDiff <= 6, "sky must not alter opaque geometry (depth test)");

    // A background pixel (top-left corner) must change from magenta clear to sky.
    const bool bgWasMagenta = at(off, 6, 6, 0) > 200 && at(off, 6, 6, 1) < 60 && at(off, 6, 6, 2) > 200;
    const bool bgNowSky      = !(at(on, 6, 6, 0) > 200 && at(on, 6, 6, 1) < 60 && at(on, 6, 6, 2) > 200);
    TST_REQUIRE_MSG(bgWasMagenta && bgNowSky, "sky must fill the background (replace the clear colour)");

    // --- Gradient: zenith (top, away from the sun) is bluer AND darker than the horizon (side). ---
    // --- Gradient: the zenith (top) is a deeper, more saturated blue than the horizon (which
    // washes paler toward white). Blue-bias (B-R) is the robust signal for the height gradient. ---
    const int zR = at(on, 30, 12, 0), zG = at(on, 30, 12, 1), zB = at(on, 30, 12, 2);
    const int hR = at(on, 8, 128, 0),  hG = at(on, 8, 128, 1),  hB = at(on, 8, 128, 2);
    std::printf("sky zenith rgb=(%d,%d,%d) horizon rgb=(%d,%d,%d)\n", zR, zG, zB, hR, hG, hB);
    TST_REQUIRE_MSG(zB > zR + 20, "zenith should be distinctly blue");
    TST_REQUIRE_MSG((zB - zR) > (hB - hR) + 15, "zenith should be a deeper blue than the (paler) horizon");

    // --- Sun disc: sun-in-front produces a much brighter background max than sun-behind. ---
    int maxFront = 0, maxBehind = 0;
    for (size_t p = 0; p < static_cast<size_t>(W) * H; ++p) {
        // background = pixels that were the magenta clear colour in the sky-off render.
        if (off[p * 4 + 0] > 200 && off[p * 4 + 1] < 60 && off[p * 4 + 2] > 200) {
            const int lf = on[p * 4] + on[p * 4 + 1] + on[p * 4 + 2];
            const int lb = behind[p * 4] + behind[p * 4 + 1] + behind[p * 4 + 2];
            if (lf > maxFront)  maxFront  = lf;
            if (lb > maxBehind) maxBehind = lb;
        }
    }
    std::printf("sky background max brightness: sun-front=%d sun-behind=%d\n", maxFront, maxBehind);
    TST_REQUIRE_MSG(maxFront > maxBehind + 120, "the sun disc/glow should brighten the sun side only");

    std::printf("sky ok\n");
}
