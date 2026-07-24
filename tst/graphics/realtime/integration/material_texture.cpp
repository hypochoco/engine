//
//  material_texture.cpp
//  engine::tst — graphics / integration
//
//  Regression guard for the bindless material-texture binding contract. A game shader that samples
//  the material texture table WITHOUT referencing the shadow map (material_probe.slang) must still
//  read the correct bindless slot. This reproduces the "white trees" bug: if the texture array's slot
//  depends on whether the shader references the shadow map, the sample misaligns with the renderer's
//  binding and returns the wrong texel. We register a known solid-color texture and check the pixel.
//

#include "harness/harness.h"

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
MeshData quad() {
    MeshData m;
    auto v = [](float x, float y) { Vertex t; t.position={x,y,0}; t.normal={0,0,1}; t.uv={0.5f,0.5f}; t.color={1,1,1}; return t; };
    m.vertices = { v(-1,-1), v(1,-1), v(1,1), v(-1,1) };
    m.indices = { 0,1,2, 0,2,3 };
    return m;
}
} // namespace

TST_CASE(graphics, integration, material_texture_binding) {
    constexpr uint32_t W = 64, H = 64;
    Device device = Device::createHeadless({});
    const auto blob = readBin(std::string(TST_SHADER_DIR) + "/material_probe.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read material_probe.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    // A known solid-color 1x1 material texture, registered in the bindless table.
    const std::array<uint8_t, 4> texel = { 51, 102, 153, 255 };   // ≈ (0.2, 0.4, 0.6)
    TextureHandle tex = device.createTexture(
        { .width = 1, .height = 1, .format = Format::RGBA8Unorm, .usage = TextureUsage::Sampled },
        std::as_bytes(std::span<const uint8_t>(texel)));
    const uint32_t slot = device.registerBindlessTexture(tex);

    SamplerHandle samp = device.createSampler(
        { .minFilter = Filter::Linear, .magFilter = Filter::Linear,
          .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });

    render::GeometryStore geometry(device);
    render::MeshHandle mesh = geometry.upload(quad());
    render::Renderer renderer(device, geometry);

    PipelineHandle probe = renderer.createMeshPipeline({ .vertex = vs, .fragment = fs });
    TST_REQUIRE_MSG(probe.valid(), "probe pipeline creation failed");
    render::RenderResources res; res.mesh = probe; res.materialSampler = samp;
    renderer.setResources(res);

    render::MaterialGPU m;                       // white factor; texture supplies the color
    m.baseColorFactor = glm::vec4(1.0f);
    m.baseColorTexture = static_cast<int32_t>(slot);
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
    std::array<render::RenderItem, 1> items{ render::RenderItem{ mesh, 0, 1, {} } };

    render::RenderView v;
    v.view = glm::lookAt(glm::vec3(0,0,3), glm::vec3(0), glm::vec3(0,1,0));
    v.proj = glm::perspective(glm::radians(50.0f), float(W)/float(H), 0.1f, 50.0f);
    v.target = colorRT; v.width = W; v.height = H;
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
    std::printf("material texture probe center rgb = %d %d %d (expected ~51 102 153)\n", r, g, b);
    TST_REQUIRE_MSG(std::abs(r - 51)  <= 6, "bindless material texture R wrong (slot misaligned?)");
    TST_REQUIRE_MSG(std::abs(g - 102) <= 6, "bindless material texture G wrong (slot misaligned?)");
    TST_REQUIRE_MSG(std::abs(b - 153) <= 6, "bindless material texture B wrong (slot misaligned?)");
    std::printf("material texture binding ok (no shadow reference)\n");
}
