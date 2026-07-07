#include "harness/harness.h"
//
//  raster_vs_pathtraced.cpp
//  engine::tst / graphics / path-tracer / integration
//
//  The two renderer heads, same scene, side by side. A sphere is registered ONCE in a core
//  GeometryCatalog; the raster head (GeometryStore + Renderer, offscreen → readback) and the path
//  tracer (pt::Scene built from the SAME catalog mesh) each render it from the same camera. We write
//  a side-by-side PNG for eyeballing and assert cross-head consistency: the raster silhouette matches
//  the path tracer's geometric coverage (its own primary-ray intersector), independent of shading.
//
//  This proves the catalog is a shared, renderer-neutral geometry source and that both heads agree on
//  camera + geometry. (A live windowed A/B is deferred — needs texture-blit plumbing.)
//

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "engine/core/geometry/geometry_catalog.h"
#include "engine/core/geometry/primitives.h"
#include "engine/graphics/rhi/rhi.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/renderer.h"
#include "engine/pathtracer/integrator.h"
#include "engine/pathtracer/scene.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

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

float lum8(const uint8_t* p) { return (0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2]) / 255.0f; }

// Shared camera for both heads.
struct Cam { glm::vec3 eye{0, 0, 3.5f}, center{0, 0, 0}, up{0, 1, 0}; float fovY = glm::radians(45.0f); };

} // namespace

TST_CASE(pathtracer, integration, raster_vs_pathtraced) {
    using namespace engine;
    using namespace engine::rhi;
    constexpr uint32_t W = 200, H = 200;
    const Cam cam;
    const float aspect = float(W) / float(H);

    // --- one shared, renderer-neutral geometry source ---
    GeometryCatalog catalog;
    const MeshId sphereId = catalog.add(primitives::makeSphere(1.0f, 32, 64));

    // ================= raster head (offscreen → readback) =================
    Device device = Device::createHeadless({});
    const auto blob = readFile(std::string(ENGINE_SHADER_DIR) + "/mesh.metallib");
    TST_REQUIRE_MSG(!blob.empty(), "read mesh.metallib");
    ShaderHandle vs = device.createShader(blob, ShaderStage::Vertex);
    ShaderHandle fs = device.createShader(blob, ShaderStage::Fragment);
    const Format colorFormat = Format::RGBA8Unorm;
    GraphicsPipelineDesc pd;
    pd.vertex = vs; pd.fragment = fs; pd.vertexLayout = render::coreVertexLayout();
    pd.colorFormats = std::span<const Format>(&colorFormat, 1);
    pd.depthFormat = Format::Depth32Float;
    pd.depth = { .test = true, .write = true, .op = CompareOp::Less };
    PipelineHandle pipe = device.createGraphicsPipeline(pd);
    TST_REQUIRE_MSG(pipe.valid(), "pipeline");

    TextureHandle color = device.createTexture(
        { .width = W, .height = H, .format = Format::RGBA8Unorm,
          .usage = TextureUsage::ColorTarget | TextureUsage::Sampled });
    RenderTargetHandle colorRT = device.createRenderTarget(color);

    render::GeometryStore geometry(device);
    const render::MeshHandle rasterMesh = geometry.upload(catalog.mesh(sphereId));   // ← from the catalog
    render::Renderer renderer(device, geometry);
    renderer.setMeshPipeline(pipe);

    render::MaterialGPU mat; mat.baseColorFactor = {0.7f, 0.7f, 0.7f, 1.0f};
    render::InstanceData inst; inst.model = glm::mat4(1.0f); inst.normalModel = glm::mat4(1.0f); inst.materialIndex = 0;
    render::RenderItem item{ rasterMesh, 0, 1 };

    render::RenderView view;
    view.view = glm::lookAt(cam.eye, cam.center, cam.up);
    view.proj = glm::perspective(cam.fovY, aspect, 0.1f, 100.0f);
    view.target = colorRT; view.width = W; view.height = H;
    view.light.direction = glm::normalize(glm::vec3(-0.4f, -0.8f, -0.4f));
    view.light.intensity = 1.5f; view.light.color = glm::vec3(1.0f); view.light.ambient = glm::vec3(0.18f);
    view.clearColor[0] = view.clearColor[1] = view.clearColor[2] = 0.0f; view.clearColor[3] = 1.0f;
    view.items = std::span<const render::RenderItem>(&item, 1);
    view.instances = std::span<const render::InstanceData>(&inst, 1);
    view.materials = std::span<const render::MaterialGPU>(&mat, 1);

    FrameContext frame = device.beginFrame();
    renderer.render(frame, std::span<const render::RenderView>(&view, 1));
    device.endFrame(std::move(frame));

    std::vector<uint8_t> rasterPx(static_cast<size_t>(W) * H * 4);
    device.readback(color, std::as_writable_bytes(std::span<uint8_t>(rasterPx)));

    // ================= path-traced head (same catalog mesh) =================
    pt::Scene scene;
    const uint32_t gray = scene.addMaterial({ .albedo = glm::vec3(0.7f) });
    scene.addMesh(catalog.mesh(sphereId), glm::mat4(1.0f), gray);   // ← same source geometry
    const uint32_t sphereTriCount = static_cast<uint32_t>(scene.triangles.size());
    // Off-screen overhead area light (above + slightly toward the camera) so the sphere is lit.
    pt::Material lightMat; lightMat.emission = glm::vec3(20.0f);
    const uint32_t light = scene.addMaterial(lightMat);
    {
        const float y = 3.0f;
        const glm::vec3 a(-1.5f, y, 0.5f), b(-1.5f, y, 2.5f), c(1.5f, y, 2.5f), d(1.5f, y, 0.5f);
        const glm::vec3 fn = glm::normalize(glm::cross(b - a, c - a));
        auto tri = [&](glm::vec3 p0, glm::vec3 p1, glm::vec3 p2) {
            pt::Triangle t; t.v0 = p0; t.v1 = p1; t.v2 = p2; t.n0 = t.n1 = t.n2 = fn; t.material = light;
            scene.triangles.push_back(t);
        };
        tri(a, b, c); tri(a, c, d);
    }
    scene.camera.eye = cam.eye; scene.camera.forward = glm::normalize(cam.center - cam.eye);
    scene.camera.up = cam.up; scene.camera.fovY = cam.fovY;
    scene.finalize();

    pt::Settings set; set.samplesPerPixel = 48; set.maxDepth = 4; set.seed = 2;
    const std::vector<glm::vec3> ptHDR = pt::render(scene, W, H, set);
    const std::vector<uint8_t>  ptPx   = pt::toneMap(ptHDR);

    // ================= geometric coverage from the PT intersector (lighting-independent) =========
    const glm::vec3 fwd   = glm::normalize(scene.camera.forward);
    const glm::vec3 right = glm::normalize(glm::cross(fwd, scene.camera.up));
    const glm::vec3 up    = glm::cross(right, fwd);
    const float tanHalf = std::tan(cam.fovY * 0.5f);
    std::vector<uint8_t> geoCov(static_cast<size_t>(W) * H, 0), rasCov(static_cast<size_t>(W) * H, 0);
    uint32_t geoN = 0, rasN = 0, inter = 0, uni = 0;
    for (uint32_t y = 0; y < H; ++y) {
        for (uint32_t x = 0; x < W; ++x) {
            const float px = ((x + 0.5f) / W) * 2.0f - 1.0f;
            const float py = 1.0f - ((y + 0.5f) / H) * 2.0f;
            const glm::vec3 dir = glm::normalize(fwd + right * (px * aspect * tanHalf) + up * (py * tanHalf));
            const pt::Hit h = scene.intersect(cam.eye, dir);
            const bool g = h.valid && h.tri < sphereTriCount;        // primary ray hits the sphere (not the light)
            const bool r = lum8(&rasterPx[(size_t(y) * W + x) * 4]) > 0.04f;
            const size_t i = size_t(y) * W + x;
            geoCov[i] = g ? 1 : 0; rasCov[i] = r ? 1 : 0;
            geoN += g; rasN += r;
            inter += (g && r); uni += (g || r);
        }
    }
    const double iou = uni ? double(inter) / uni : 0.0;
    const double geoFrac = double(geoN) / (W * H), rasFrac = double(rasN) / (W * H);

    // PT shaded brightness over the sphere (it must actually be lit) + dark background corner.
    double ptOnSphere = 0.0; uint32_t ns = 0;
    for (size_t i = 0; i < geoCov.size(); ++i) if (geoCov[i]) { ptOnSphere += lum8(&ptPx[i * 4]); ++ns; }
    ptOnSphere = ns ? ptOnSphere / ns : 0.0;
    const float ptCorner = lum8(&ptPx[0]);

    // --- write the side-by-side PNG (raster | path-traced) for inspection ---
    std::vector<uint8_t> combo(static_cast<size_t>(2 * W) * H * 4, 0);
    for (uint32_t y = 0; y < H; ++y) {
        std::memcpy(&combo[(size_t(y) * (2 * W) + 0) * 4], &rasterPx[size_t(y) * W * 4], size_t(W) * 4);
        std::memcpy(&combo[(size_t(y) * (2 * W) + W) * 4], &ptPx[size_t(y) * W * 4],     size_t(W) * 4);
    }
    const char* outPath = "raster_vs_pathtraced.png";
    const int wrote = stbi_write_png(outPath, int(2 * W), int(H), 4, combo.data(), int(2 * W) * 4);

    std::printf("raster_vs_pathtraced: IoU=%.3f (raster cov=%.3f geo cov=%.3f) | PT sphere L=%.3f corner=%.3f | png=%s\n",
                iou, rasFrac, geoFrac, ptOnSphere, ptCorner, wrote ? outPath : "(write failed)");

    // Cross-head consistency: both see a sphere of sane size, silhouettes overlap strongly.
    TST_REQUIRE_MSG(geoFrac > 0.15 && geoFrac < 0.65, "sphere covers a sane fraction of the frame");
    TST_REQUIRE_MSG(rasFrac > 0.15 && rasFrac < 0.65, "raster sphere covers a sane fraction");
    TST_REQUIRE_MSG(iou > 0.80, "raster silhouette must match the path tracer's geometry (same camera+mesh)");
    // The path tracer actually lit the sphere, and the background is dark.
    TST_REQUIRE_MSG(ptOnSphere > 0.02f, "path tracer should light the sphere");
    TST_REQUIRE_MSG(ptCorner < 0.05f, "path tracer background corner should be dark");
}
