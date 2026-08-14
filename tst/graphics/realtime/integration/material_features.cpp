#include "harness/harness.h"
//
//  material_features.cpp
//  engine::tst — graphics / integration
//
//  Exercises the Phase-2 material upgrade through mesh.slang + Renderer:
//    (1) EMISSIVE — an emissive material shows its color with zero lighting (ambient + sun off).
//    (2) SPECULAR — a low-roughness material produces a brighter peak (GGX sun highlight) than the
//        default Lambert material under the same light.
//    (3) ALPHA CUTOUT — with the cutout flag set, fragments whose albedo alpha < cutoff are
//        discarded (background shows through); with it off they stay opaque.
//

#include <algorithm>
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
#include "engine/core/geometry/primitives.h"
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
MeshData facingQuad() {
    MeshData m;
    auto v = [](glm::vec3 p, glm::vec2 uv) { Vertex x; x.position=p; x.normal={0,0,1}; x.uv=uv; x.color={1,1,1}; return x; };
    m.vertices = { v({-1,-1,0},{0,0}), v({1,-1,0},{1,0}), v({1,1,0},{1,1}), v({-1,1,0},{0,1}) };
    m.indices = { 0,1,2, 0,2,3 };
    geometry::computeTangents(m);
    return m;
}
} // namespace

TST_CASE(graphics, integration, material_features) {
    constexpr uint32_t W = 128, H = 128;
    Device device = Device::createHeadless({});

    const auto blob = readBin(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);
    const Format fmt = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs; pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&fmt, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    SamplerHandle samp = device.createSampler({ .minFilter = Filter::Linear, .magFilter = Filter::Linear,
                                                .addressU = AddressMode::ClampToEdge, .addressV = AddressMode::ClampToEdge });
    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = fmt, .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    render::MeshHandle quad   = geometry.upload(facingQuad());
    render::MeshHandle sphere = geometry.upload(primitives::makeSphere(0.9f, 48, 96));
    render::Renderer renderer(device, geometry);
    render::RenderResources res; res.mesh = pipe; res.materialSampler = samp;
    renderer.setResources(res);

    auto render = [&](render::MeshHandle mesh, const render::MaterialGPU& m,
                      const render::DirectionalLight& light, std::vector<uint8_t>& out) {
        render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
        render::RenderItem item{ mesh, 0, 1 };
        render::RenderView v;
        v.view = glm::lookAt(glm::vec3(0,0,3), glm::vec3(0), glm::vec3(0,1,0));
        v.proj = glm::perspective(glm::radians(45.0f), float(W)/float(H), 0.1f, 50.0f);
        v.target = colorRT; v.width = W; v.height = H; v.light = light;
        v.items = std::span<const render::RenderItem>(&item, 1);
        v.instances = std::span<const render::InstanceData>(&inst, 1);
        v.materials = std::span<const render::MaterialGPU>(&m, 1);
        FrameContext fr = device.beginFrame();
        renderer.render(fr, std::span<const render::RenderView>(&v, 1));
        device.endFrame(std::move(fr));
        out.resize(static_cast<size_t>(W) * H * 4);
        device.readback(color, std::as_writable_bytes(std::span<uint8_t>(out)));
    };
    auto at = [&](const std::vector<uint8_t>& px, uint32_t x, uint32_t y) {
        return &px[(static_cast<size_t>(y) * W + x) * 4];
    };

    // (1) EMISSIVE: no lighting at all → only the emissive term survives.
    render::DirectionalLight dark; dark.intensity = 0.0f; dark.ambient = glm::vec3(0.0f);
    render::MaterialGPU emis; emis.baseColorFactor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    emis.emissiveFactor = glm::vec4(0.0f, 0.7f, 0.0f, 0.0f);
    std::vector<uint8_t> pxE; render(quad, emis, dark, pxE);
    const uint8_t* ec = at(pxE, W/2, H/2);
    std::printf("emissive center rgb = %u %u %u\n", ec[0], ec[1], ec[2]);
    TST_REQUIRE_MSG(ec[1] > 120 && ec[0] < 40 && ec[2] < 40, "emissive green should show with no lighting");

    // (2) SPECULAR: same white sphere + sun; glossy (low roughness) has a brighter peak than Lambert.
    render::DirectionalLight sun; sun.intensity = 1.0f; sun.color = glm::vec3(1.0f);
    sun.ambient = glm::vec3(0.05f); sun.direction = glm::normalize(glm::vec3(-0.25f, -0.25f, -1.0f));
    auto peakLuma = [&](const std::vector<uint8_t>& px) {
        int best = 0;
        for (size_t i = 0; i < px.size(); i += 4)
            best = std::max<int>(best, int(px[i]) + int(px[i+1]) + int(px[i+2]));
        return best;
    };
    render::MaterialGPU lambert; lambert.baseColorFactor = glm::vec4(0.8f, 0.8f, 0.8f, 1.0f); // roughness 1, metallic 0
    render::MaterialGPU glossy = lambert; glossy.roughnessFactor = 0.12f;                     // dielectric gloss
    std::vector<uint8_t> pxL, pxG; render(sphere, lambert, sun, pxL); render(sphere, glossy, sun, pxG);
    const int lPeak = peakLuma(pxL), gPeak = peakLuma(pxG);
    std::printf("specular peak luma: lambert=%d glossy=%d\n", lPeak, gPeak);
    TST_REQUIRE_MSG(gPeak > lPeak + 40, "low-roughness material should have a brighter specular peak");

    // (3) ALPHA CUTOUT: albedo texture with left column opaque, right column transparent.
    std::array<uint8_t, 2*2*4> tex{};
    for (int y = 0; y < 2; ++y) {
        uint8_t* l = &tex[(y*2 + 0)*4]; l[0]=230; l[1]=60; l[2]=60; l[3]=255;   // opaque red
        uint8_t* r = &tex[(y*2 + 1)*4]; r[0]=230; r[1]=60; r[2]=60; r[3]=0;     // transparent
    }
    const uint32_t slot = device.registerBindlessTexture(device.createTexture(
        { .width = 2, .height = 2, .format = fmt, .usage = TextureUsage::Sampled },
        std::as_bytes(std::span<const uint8_t>(tex))));

    render::DirectionalLight amb; amb.intensity = 0.0f; amb.ambient = glm::vec3(1.0f);   // flat lit
    render::MaterialGPU cut; cut.baseColorFactor = glm::vec4(1.0f);
    cut.baseColorTexture = int(slot); cut.alphaCutoff = 0.5f; cut.flags = render::MaterialFlagAlphaCutout;
    std::vector<uint8_t> pxC; render(quad, cut, amb, pxC);
    const uint8_t* leftPx  = at(pxC, W/4, H/2);       // uv≈0.25 → opaque
    const uint8_t* rightPx = at(pxC, (3*W)/4, H/2);   // uv≈0.75 → transparent → discarded → clear
    std::printf("cutout left rgb = %u %u %u   right rgb = %u %u %u\n",
                leftPx[0], leftPx[1], leftPx[2], rightPx[0], rightPx[1], rightPx[2]);
    TST_REQUIRE_MSG(leftPx[0] > 180 && leftPx[0] > leftPx[1] + 80, "opaque half should show the red albedo");
    TST_REQUIRE_MSG(rightPx[0] < 60 && rightPx[1] < 60, "transparent half should be discarded (clear color)");

    // Same texture, cutout OFF → right half stays opaque red (no discard).
    render::MaterialGPU noCut = cut; noCut.flags = render::MaterialFlagNone;
    std::vector<uint8_t> pxN; render(quad, noCut, amb, pxN);
    const uint8_t* rightN = at(pxN, (3*W)/4, H/2);
    std::printf("cutout-off right rgb = %u %u %u\n", rightN[0], rightN[1], rightN[2]);
    TST_REQUIRE_MSG(rightN[0] > 180, "with cutout off, the transparent-alpha region should still draw");

    device.destroy(samp);
    std::printf("material features ok\n");
}
