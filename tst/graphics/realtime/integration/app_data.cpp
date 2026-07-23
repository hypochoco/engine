#include "harness/harness.h"
//
//  app_data.cpp
//  engine::tst — graphics / integration
//
//  The per-view appData channel: render::RenderView::appData must reach the shader as
//  globals.appData intact. A probe shader (tst/shaders/app_data_probe.slang) outputs
//  globals.appData.rgb; we upload a known value and check the rendered pixel. This proves the
//  engine-agnostic app-data path game shaders (foliage/wind) rely on.
//

#include <array>
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

#include "engine/core/geometry/mesh.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"

using namespace engine;
using namespace engine::rhi;

namespace {
std::vector<std::byte> readBin(const std::string& p) {
    std::ifstream f(p, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto n = static_cast<std::streamsize>(f.tellg()); f.seekg(0);
    std::vector<std::byte> d(static_cast<size_t>(n)); f.read(reinterpret_cast<char*>(d.data()), n); return d;
}
MeshData quadAt(float z) {
    MeshData m;
    auto v = [z](float x, float y) { Vertex t; t.position={x,y,z}; t.normal={0,0,1}; t.uv={0,0}; t.color={1,1,1}; return t; };
    m.vertices = { v(-1,-1), v(1,-1), v(1,1), v(-1,1) };
    m.indices = { 0,1,2, 0,2,3 };
    return m;
}
} // namespace

TST_CASE(graphics, integration, app_data) {
    constexpr uint32_t W = 64, H = 64;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(TST_SHADER_DIR) + "/app_data_probe.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read app_data_probe.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle quad = geometry.upload(quadAt(0.0f));
    render::Renderer renderer(device, geometry);

    PipelineHandle probe = renderer.createMeshPipeline({ .vertex = vs, .fragment = fs });
    TST_REQUIRE_MSG(probe.valid(), "probe pipeline creation failed");
    render::RenderResources res; res.mesh = probe; renderer.setResources(res);

    render::MaterialGPU m;
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f);
    std::array<render::RenderItem, 1> items{ render::RenderItem{ quad, 0, 1, {} } };

    const glm::vec4 kAppData(0.2f, 0.4f, 0.6f, 0.0f);   // expected pixel ≈ (51, 102, 153)
    render::RenderView v;
    v.view = glm::lookAt(glm::vec3(0,0,3), glm::vec3(0), glm::vec3(0,1,0));
    v.proj = glm::perspective(glm::radians(50.0f), float(W)/float(H), 0.1f, 50.0f);
    v.target = colorRT; v.width = W; v.height = H;
    v.appData = kAppData;
    v.items = std::span<const render::RenderItem>(items.data(), items.size());
    v.instances = std::span<const render::InstanceData>(&inst, 1);
    v.materials = std::span<const render::MaterialGPU>(&m, 1);

    FrameContext fr = device.beginFrame();
    renderer.render(fr, std::span<const render::RenderView>(&v, 1));
    device.endFrame(std::move(fr));

    std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
    device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
    const size_t c = (static_cast<size_t>(H/2)*W + W/2) * 4;
    const int r = px[c], g = px[c+1], b = px[c+2];
    std::printf("appData probe center rgb = %d %d %d (expected ~51 102 153)\n", r, g, b);
    TST_REQUIRE_MSG(std::abs(r - 51)  <= 6, "appData.x did not reach the shader");
    TST_REQUIRE_MSG(std::abs(g - 102) <= 6, "appData.y did not reach the shader");
    TST_REQUIRE_MSG(std::abs(b - 153) <= 6, "appData.z did not reach the shader");
    std::printf("app data channel ok\n");
}
