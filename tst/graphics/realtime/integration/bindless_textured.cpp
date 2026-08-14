#include "harness/harness.h"
//
//  bindless_textured.cpp
//  engine::tst — graphics / integration
//
//  End-to-end proof of the Phase-1 texture foundation through the real mesh.slang + Renderer:
//    (1) bindless ALBEDO sampling — a solid-color texture registered in the bindless table shows
//        through a white material (distinct from the material factor).
//    (2) tangent-space NORMAL mapping — swapping a flat normal map for a tilted one measurably
//        changes the Lambert term under a fixed directional light (proves the TBN frame + tangents).
//    (3) MIPMAP generation — a black/white checkerboard albedo with a generated mip chain, sampled
//        under heavy minification, resolves toward the mid-gray average instead of aliasing.
//

#include <array>
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

#include "engine/core/geometry/mesh.h"
#include "engine/core/geometry/obj_loader.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"

using namespace engine;
using namespace engine::rhi;

namespace {

std::vector<std::byte> readBin(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    f.seekg(0);
    std::vector<std::byte> d(static_cast<size_t>(size));
    f.read(reinterpret_cast<char*>(d.data()), size);
    return d;
}

// A camera-facing unit quad in the XY plane (normal +Z), UVs 0..1 (U→+X), with a computed tangent.
MeshData facingQuad() {
    MeshData m;
    auto vert = [](glm::vec3 p, glm::vec2 uv) {
        Vertex v; v.position = p; v.normal = {0, 0, 1}; v.uv = uv; v.color = {1, 1, 1}; return v;
    };
    m.vertices = { vert({-1, -1, 0}, {0, 0}), vert({1, -1, 0}, {1, 0}),
                   vert({1, 1, 0}, {1, 1}),  vert({-1, 1, 0}, {0, 1}) };
    m.indices = { 0, 1, 2, 0, 2, 3 };
    geometry::computeTangents(m);
    return m;
}

TextureHandle solidTexture(Device& d, uint8_t r, uint8_t g, uint8_t b, uint32_t n = 4) {
    std::vector<uint8_t> px(static_cast<size_t>(n) * n * 4);
    for (size_t i = 0; i < static_cast<size_t>(n) * n; ++i) { px[i*4]=r; px[i*4+1]=g; px[i*4+2]=b; px[i*4+3]=255; }
    return d.createTexture({ .width = n, .height = n, .format = Format::RGBA8Unorm, .usage = TextureUsage::Sampled },
                           std::as_bytes(std::span<const uint8_t>(px)));
}

} // namespace

TST_CASE(graphics, integration, bindless_textured) {
    constexpr uint32_t W = 128, H = 128;
    Device device = Device::createHeadless({});

    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "failed to read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);

    const Format colorFmt = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs;
    pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&colorFmt, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    TST_REQUIRE_MSG(pipe.valid(), "mesh pipeline creation failed");

    SamplerHandle matSamp = device.createSampler(
        { .minFilter = Filter::Linear, .magFilter = Filter::Linear, .mipmap = MipmapMode::Linear,
          .addressU = AddressMode::Repeat, .addressV = AddressMode::Repeat });

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = colorFmt, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    // Bindless textures: a solid orange albedo + a flat and a tilted normal map.
    const uint32_t albedoSlot   = device.registerBindlessTexture(solidTexture(device, 220, 90, 30));
    const uint32_t flatNrmSlot  = device.registerBindlessTexture(solidTexture(device, 128, 128, 255)); // +Z
    const uint32_t tiltNrmSlot  = device.registerBindlessTexture(solidTexture(device, 235, 128, 150)); // toward +X
    TST_REQUIRE_MSG(albedoSlot != 0xFFFFFFFFu && flatNrmSlot != 0xFFFFFFFFu && tiltNrmSlot != 0xFFFFFFFFu,
                    "bindless registration failed");

    render::GeometryStore geometry(device);
    render::MeshHandle quad = geometry.upload(facingQuad());
    render::Renderer renderer(device, geometry);
    render::RenderResources res; res.mesh = pipe; res.materialSampler = matSamp;
    renderer.setResources(res);

    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
    render::RenderItem item{ quad, 0, 1 };

    auto renderOnce = [&](const render::MaterialGPU& material, const render::DirectionalLight& light,
                          float camZ) {
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0, 0, camZ), glm::vec3(0), glm::vec3(0, 1, 0));
        v.proj = glm::perspective(glm::radians(50.0f), float(W) / float(H), 0.1f, 50.0f);
        v.target = colorRT; v.width = W; v.height = H;
        v.light = light;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&material, 1);
        FrameContext frame = device.beginFrame();
        renderer.render(frame, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(frame));
        std::vector<uint8_t> px(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(px)));
        const size_t c = (static_cast<size_t>(H / 2) * W + W / 2) * 4;
        return std::array<int, 3>{ px[c], px[c + 1], px[c + 2] };
    };

    // (1) Albedo: white material factor + full ambient ⇒ center ≈ the albedo texel (orange).
    render::DirectionalLight flatLit;
    flatLit.intensity = 0.0f; flatLit.ambient = glm::vec3(1.0f);   // lit color == albedo
    render::MaterialGPU texMat; texMat.baseColorFactor = glm::vec4(1.0f);
    texMat.baseColorTexture = static_cast<int>(albedoSlot); texMat.normalTexture = -1;
    auto albedoPx = renderOnce(texMat, flatLit, 2.2f);
    std::printf("albedo center rgb = %d %d %d\n", albedoPx[0], albedoPx[1], albedoPx[2]);
    TST_REQUIRE_MSG(albedoPx[0] > 180 && albedoPx[0] > albedoPx[1] + 60 && albedoPx[0] > albedoPx[2] + 90,
                    "albedo texture did not show through (expected orange-dominant)");

    // (2) Normal mapping: directional light from +X+Z, no ambient. Flat vs tilted normal map must
    // change the diffuse term (tilt turns the surface toward the light ⇒ brighter).
    render::DirectionalLight dir;
    dir.intensity = 1.0f; dir.color = glm::vec3(1.0f); dir.ambient = glm::vec3(0.0f);
    dir.direction = glm::normalize(glm::vec3(-1.0f, 0.0f, -1.0f));   // travels -x-z ⇒ light is at +x+z
    render::MaterialGPU whiteMat; whiteMat.baseColorFactor = glm::vec4(1.0f); whiteMat.baseColorTexture = -1;

    render::MaterialGPU flatN = whiteMat; flatN.normalTexture = static_cast<int>(flatNrmSlot);
    render::MaterialGPU tiltN = whiteMat; tiltN.normalTexture = static_cast<int>(tiltNrmSlot);
    auto flatShade = renderOnce(flatN, dir, 2.2f);
    auto tiltShade = renderOnce(tiltN, dir, 2.2f);
    std::printf("normal map flat R=%d  tilted R=%d\n", flatShade[0], tiltShade[0]);
    TST_REQUIRE_MSG(tiltShade[0] > flatShade[0] + 15,
                    "tilted normal map should brighten the surface vs the flat one");

    // (3) Mipmaps: a black/white checkerboard albedo with a generated mip chain, viewed heavily
    // minified, should resolve toward the mid-gray average (not a single checker color / aliasing).
    constexpr uint32_t N = 64, mips = 7;   // 64 = 2^6 ⇒ 7 mip levels
    std::vector<uint8_t> checker(static_cast<size_t>(N) * N * 4);
    for (uint32_t y = 0; y < N; ++y)
        for (uint32_t x = 0; x < N; ++x) {
            const uint8_t c = ((x ^ y) & 1) ? 255 : 0;
            uint8_t* p = &checker[(static_cast<size_t>(y) * N + x) * 4];
            p[0] = p[1] = p[2] = c; p[3] = 255;
        }
    TextureHandle checkTex = device.createTexture(
        { .width = N, .height = N, .mipLevels = mips, .format = Format::RGBA8Unorm, .usage = TextureUsage::Sampled },
        std::as_bytes(std::span<const uint8_t>(checker)));
    device.generateMipmaps(checkTex);
    const uint32_t checkSlot = device.registerBindlessTexture(checkTex);

    render::MaterialGPU checkMat; checkMat.baseColorFactor = glm::vec4(1.0f);
    checkMat.baseColorTexture = static_cast<int>(checkSlot); checkMat.normalTexture = -1;
    // Push the quad far back so one screen pixel covers many texels ⇒ minified sampling uses high mips.
    auto miniPx = renderOnce(checkMat, flatLit, 40.0f);
    std::printf("minified checker center rgb = %d %d %d (expect ~128 gray)\n", miniPx[0], miniPx[1], miniPx[2]);
    TST_REQUIRE_MSG(miniPx[0] > 80 && miniPx[0] < 175,
                    "minified checkerboard should average toward gray via mipmaps (not alias to 0/255)");

    device.destroy(matSamp);
    std::printf("bindless textured ok\n");
}
