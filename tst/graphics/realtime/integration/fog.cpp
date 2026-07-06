#include "harness/harness.h"
//
//  fog.cpp
//  engine::tst
//
//  Aerial-perspective + height fog (RF6 atmosphere), applied in the forward shader. Verifies:
//    1. Parity: fog OFF leaves distant geometry at its lit albedo (not the fog colour).
//    2. Distance washout: fog ON pulls a FAR object toward the fog colour, and a far object is
//       closer to the fog colour than a near one (monotonic with distance).
//    3. Sun in-scatter: with the sun ahead, a fogged object is brighter than with the sun behind.
//    4. Height fog: on a tall box, the (low) base is foggier than the (high) top.
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

TST_CASE(graphics, integration, fog) {
    using namespace engine;
    using namespace engine::rhi;

    constexpr uint32_t W = 256, H = 256;
    Device device = Device::createHeadless({});

    auto meshBlob = readFileBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    if (meshBlob.empty()) { TST_REQUIRE_MSG(false, "read shaders failed"); }
    ShaderHandle vs = device.createShader(meshBlob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(meshBlob, ShaderStage::Fragment);

    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs;
    pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    TST_REQUIRE_MSG(pipe.valid(), "pipeline creation failed");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFormat, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle box = geometry.upload(primitives::makeBox(glm::vec3(0.5f)));
    render::Renderer renderer(device, geometry);

    render::MaterialGPU redd; redd.baseColorFactor = {0.9f, 0.2f, 0.2f, 1.0f};   // distinct from the fog color
    render::InstanceData inst; inst.materialIndex = 0;
    render::RenderItem item{ box, 0, 1 };
    renderer.setMeshPipeline(pipe);

    const glm::vec3 fogColor(0.55f, 0.62f, 0.78f);   // pale blue, distinct from the red box

    render::RenderView view;
    view.proj = glm::perspective(glm::radians(45.0f), float(W) / float(H), 0.1f, 200.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.color = glm::vec3(1.0f); view.light.intensity = 1.0f; view.light.ambient = glm::vec3(0.25f);
    view.light.direction = glm::normalize(glm::vec3(0.0f, -0.3f, -1.0f));   // sun roughly ahead
    view.clearColor[0] = 0.0f; view.clearColor[1] = 0.0f; view.clearColor[2] = 0.0f; view.clearColor[3] = 1.0f;
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(&inst, 1);
    view.materials = std::span<const render::MaterialGPU>(&redd, 1);

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
    // Distance from a sampled pixel's rgb to the fog color (in 0..255 units).
    auto distToFog = [&](const std::vector<uint8_t>& img, uint32_t x, uint32_t y) {
        float dr = at(img, x, y, 0) - fogColor.r * 255.0f;
        float dg = at(img, x, y, 1) - fogColor.g * 255.0f;
        float db = at(img, x, y, 2) - fogColor.b * 255.0f;
        return dr * dr + dg * dg + db * db;
    };

    const glm::mat4 nearM = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, -8))  * glm::scale(glm::mat4(1.0f), glm::vec3(6.0f));
    const glm::mat4 farM  = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, -70)) * glm::scale(glm::mat4(1.0f), glm::vec3(6.0f));
    view.view = glm::lookAt(glm::vec3(0, 0, 2), glm::vec3(0, 0, -1), glm::vec3(0, 1, 0));
    const uint32_t cx = W / 2, cy = H / 2;

    // --- (1) parity + (2) distance washout (pure distance fog, no in-scatter). ---
    std::vector<uint8_t> nearOff, nearOn, farOff, farOn;
    renderer.setFog(0.0f);                                        // OFF
    inst.model = nearM; inst.normalModel = nearM; renderTo(nearOff);
    inst.model = farM;  inst.normalModel = farM;  renderTo(farOff);
    renderer.setFog(0.03f, 0.0f, 0.0f, fogColor, glm::vec3(0.0f), 8.0f);   // distance fog only
    inst.model = nearM; inst.normalModel = nearM; renderTo(nearOn);
    inst.model = farM;  inst.normalModel = farM;  renderTo(farOn);

    std::printf("fog center rgb  nearOff=(%d,%d,%d) farOff=(%d,%d,%d) nearOn=(%d,%d,%d) farOn=(%d,%d,%d)\n",
                at(nearOff,cx,cy,0),at(nearOff,cx,cy,1),at(nearOff,cx,cy,2),
                at(farOff,cx,cy,0), at(farOff,cx,cy,1), at(farOff,cx,cy,2),
                at(nearOn,cx,cy,0), at(nearOn,cx,cy,1), at(nearOn,cx,cy,2),
                at(farOn,cx,cy,0),  at(farOn,cx,cy,1),  at(farOn,cx,cy,2));

    // (1) Parity: with fog OFF the far box keeps its (red) albedo — far from the fog colour.
    TST_REQUIRE_MSG(distToFog(farOff, cx, cy) > 4000.0f, "fog OFF should leave the albedo untouched");
    // (2) Distance washout: fog ON pulls the far box toward the fog colour...
    TST_REQUIRE_MSG(distToFog(farOn, cx, cy) < distToFog(farOff, cx, cy),
                    "fog ON should pull the far object toward the fog color");
    // ...and the far object is closer to the fog colour than the near object (monotonic distance).
    TST_REQUIRE_MSG(distToFog(farOn, cx, cy) < distToFog(nearOn, cx, cy),
                    "farther geometry should be foggier than nearer geometry");

    // --- (3) Sun in-scatter: sun ahead (bright fog) vs sun behind (no in-scatter). ---
    std::vector<uint8_t> sunAhead, sunBehind;
    renderer.setFog(0.03f, 0.0f, 0.0f, fogColor, glm::vec3(2.5f, 2.0f, 1.2f), 6.0f);   // warm in-scatter
    inst.model = farM; inst.normalModel = farM;
    view.light.direction = glm::normalize(glm::vec3(0.0f, -0.3f,  1.0f)); renderTo(sunAhead);   // sunDir→ -z = ahead in view
    view.light.direction = glm::normalize(glm::vec3(0.0f, -0.3f, -1.0f)); renderTo(sunBehind);  // sunDir→ +z = behind camera
    const int briAhead  = at(sunAhead, cx, cy, 0) + at(sunAhead, cx, cy, 1) + at(sunAhead, cx, cy, 2);
    const int briBehind = at(sunBehind, cx, cy, 0) + at(sunBehind, cx, cy, 1) + at(sunBehind, cx, cy, 2);
    std::printf("fog sun in-scatter: ahead brightness=%d behind=%d\n", briAhead, briBehind);
    TST_REQUIRE_MSG(briAhead > briBehind + 40, "sun-ahead fog should in-scatter brighter than sun-behind");

    // --- (4) Height fog: a tall box, base (low) foggier than top (high). ---
    const glm::mat4 tallM = glm::translate(glm::mat4(1.0f), glm::vec3(0, 4, -20)) * glm::scale(glm::mat4(1.0f), glm::vec3(2, 10, 2));
    view.view = glm::lookAt(glm::vec3(0, 4, 6), glm::vec3(0, 4, -1), glm::vec3(0, 1, 0));   // level, box fills vertical
    view.light.direction = glm::normalize(glm::vec3(0.0f, -0.3f, -1.0f));
    renderer.setFog(0.02f, 0.35f, -1.0f, fogColor, glm::vec3(0.0f), 8.0f);   // strong height falloff
    std::vector<uint8_t> heightOn; inst.model = tallM; inst.normalModel = tallM; renderTo(heightOn);
    const uint32_t basePy = static_cast<uint32_t>(H * 0.70f), topPy = static_cast<uint32_t>(H * 0.30f);
    std::printf("fog height: base rgb=(%d,%d,%d) top rgb=(%d,%d,%d)\n",
                at(heightOn,cx,basePy,0),at(heightOn,cx,basePy,1),at(heightOn,cx,basePy,2),
                at(heightOn,cx,topPy,0), at(heightOn,cx,topPy,1), at(heightOn,cx,topPy,2));
    TST_REQUIRE_MSG(distToFog(heightOn, cx, basePy) < distToFog(heightOn, cx, topPy),
                    "height fog: the low base should be foggier than the high top");

    std::printf("fog ok\n");
}
