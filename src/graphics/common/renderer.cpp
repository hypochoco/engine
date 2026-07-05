//
//  renderer.cpp
//  engine::graphics / render
//
//  Consumes per-view render lists: uploads the camera (view-projection) and per-instance
//  data, then issues one instanced indexed draw per RenderItem against the shared geometry.
//  Owns the per-frame camera/instance buffers and the depth target; owns no scene data.
//
//  Resource binding matches the mesh shader (Slang-assigned Metal indices): camera uniform
//  at binding 0, instance storage buffer at binding 1, vertex data at the backend's vertex
//  base index.
//

#include "engine/graphics/render/renderer.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <span>

#include <glm/glm.hpp>

#include "engine/graphics/rhi/device.h"
#include "engine/graphics/render/frame_ring.h"
#include "engine/graphics/render/geometry_store.h"
#include "engine/graphics/render/render_graph.h"

namespace engine::render {

// CPU mirror of the mesh shader's `Globals` constant buffer (binding 0). std140/constant-buffer
// layout: mat4 (64B) then three vec4 (16B each) — all naturally 16-byte aligned. Column-major
// matrices match slangc's -matrix-layout-column-major and glm.
struct GlobalUniforms {
    glm::mat4 viewProj{1.0f};
    glm::vec4 lightDir{0.0f};     // xyz = normalized direction TO the light; w unused
    glm::vec4 lightColor{1.0f};   // rgb = color * intensity; w unused
    glm::vec4 ambient{0.0f};      // rgb = ambient term; w unused
    glm::vec4 params{0.0f};       // x = point light count; y = clustered? ; zw reserved
};

// CPU mirror of cluster.slang's ClusterParams (std140/constant-buffer layout).
struct ClusterParamsGPU {
    glm::mat4  view{1.0f};
    glm::vec4  frustum{0.0f};   // tanHalfFovY, aspect, zNear, zFar
    glm::vec4  screen{0.0f};    // W, H
    glm::uvec4 gridDim{0};      // CX, CY, CZ, maxLightsPerCluster
    glm::uvec4 misc{0};         // lightCount
};

// Froxel grid dimensions for clustered forward+. Fixed for now (tunable later per the note).
namespace cluster {
    constexpr uint32_t CX = 12, CY = 12, CZ = 24, MAX_PER = 64;
    constexpr uint32_t COUNT = CX * CY * CZ;
}

struct Renderer::Impl {
    rhi::Device*   device   = nullptr;
    GeometryStore* geometry = nullptr;

    // Per-frame-in-flight upload arenas: camera uniform + instance SoA + materials are
    // sub-allocated per view from here, fixing the old single-buffer per-frame hazard and the
    // multi-view clobber (see frame_ring.h / the render-framework note §1).
    FrameRingAllocator      ring;
    rhi::TextureHandle      depthTex;
    rhi::RenderTargetHandle depthRT;
    uint32_t                depthW = 0, depthH = 0;
    rhi::PipelineHandle     clusterPipeline;   // compute; clustered lighting active when valid

    // HDR + tone mapping (opt-in via setTonemap). Forward renders into hdrTex (RGBA16F), then a
    // fullscreen tonemap pass resolves to the view target.
    rhi::PipelineHandle     tonemapPipeline;
    rhi::SamplerHandle      tonemapSampler;
    rhi::TextureHandle      hdrTex;
    rhi::RenderTargetHandle hdrRT;
    uint32_t                hdrW = 0, hdrH = 0;

    void ensureDepth(uint32_t w, uint32_t h) {
        if (depthTex.valid() && w == depthW && h == depthH) return;
        if (depthTex.valid()) device->destroy(depthTex);
        if (depthRT.valid())  device->destroy(depthRT);
        depthTex = device->createTexture(
            { .width = w, .height = h, .format = rhi::Format::Depth32Float,
              .usage = rhi::TextureUsage::DepthTarget, .transient = true });
        depthRT = device->createRenderTarget(depthTex);
        depthW = w; depthH = h;
    }

    void ensureHDR(uint32_t w, uint32_t h) {
        if (hdrTex.valid() && w == hdrW && h == hdrH) return;
        if (hdrTex.valid()) device->destroy(hdrTex);
        if (hdrRT.valid())  device->destroy(hdrRT);
        hdrTex = device->createTexture(
            { .width = w, .height = h, .format = rhi::Format::RGBA16Float,
              .usage = rhi::TextureUsage::ColorTarget | rhi::TextureUsage::Sampled });
        hdrRT = device->createRenderTarget(hdrTex);
        hdrW = w; hdrH = h;
    }
};

Renderer::Renderer(rhi::Device& device, GeometryStore& geometry) : impl_(std::make_unique<Impl>()) {
    impl_->device   = &device;
    impl_->geometry = &geometry;
    impl_->ring     = FrameRingAllocator(device, device.framesInFlight());
}
Renderer::Renderer(Renderer&&) noexcept = default;
Renderer& Renderer::operator=(Renderer&&) noexcept = default;
Renderer::~Renderer() {
    if (impl_ && impl_->device) {
        impl_->ring.destroy();
        if (impl_->depthTex.valid()) impl_->device->destroy(impl_->depthTex);
        if (impl_->depthRT.valid())  impl_->device->destroy(impl_->depthRT);
        if (impl_->hdrTex.valid())   impl_->device->destroy(impl_->hdrTex);
        if (impl_->hdrRT.valid())    impl_->device->destroy(impl_->hdrRT);
    }
}

void Renderer::setClusterBinning(rhi::PipelineHandle binningPipeline) {
    impl_->clusterPipeline = binningPipeline;
}

void Renderer::setTonemap(rhi::PipelineHandle tonemapPipeline, rhi::SamplerHandle sampler) {
    impl_->tonemapPipeline = tonemapPipeline;
    impl_->tonemapSampler  = sampler;
}

void Renderer::render(rhi::FrameContext& frame, std::span<const RenderView> views) {
    Impl& I = *impl_;
    I.ring.beginFrame(frame.frameIndex());

    // Build a render graph: one forward raster pass per view. The graph owns pass ordering +
    // barriers + the begin/endRendering scope; the pass callback only issues draws. (This is the
    // structure the clustered forward+ path grows on — a compute cluster pass + shadow passes
    // become additional graph nodes.)
    RenderGraph graph(*I.device);

    for (const auto& view : views) {
        I.ensureDepth(view.width, view.height);

        // Clustered forward+ active for this view? (Must be known BEFORE uploading the camera
        // uniform, since it sets params.y which the forward shader reads.)
        const bool clustered = I.clusterPipeline.valid() && !view.pointLights.empty();

        GlobalUniforms g;
        g.viewProj   = view.proj * view.view;
        g.lightDir   = glm::vec4(glm::normalize(-view.light.direction), 0.0f);  // surface→light
        g.lightColor = glm::vec4(view.light.color * view.light.intensity, 0.0f);
        g.ambient    = glm::vec4(view.light.ambient, 0.0f);
        g.params     = glm::vec4(static_cast<float>(view.pointLights.size()),
                                 clustered ? 1.0f : 0.0f, 0.0f, 0.0f);

        // Sub-allocate this view's upload data from the frame ring (distinct regions per view).
        const FrameRingAllocator::Alloc cam = I.ring.upload(&g, 1);
        FrameRingAllocator::Alloc inst{}, mat{}, lights{};
        if (!view.instances.empty())
            inst = I.ring.upload(view.instances.data(), view.instances.size());
        if (!view.materials.empty())
            mat  = I.ring.upload(view.materials.data(), view.materials.size());
        if (!view.pointLights.empty())
            lights = I.ring.upload(view.pointLights.data(), view.pointLights.size());

        // Clustered forward+: if enabled and this view has point lights, add a froxel light-binning
        // compute pass before the forward pass, and bind the cluster buffers to shading.
        FrameRingAllocator::Alloc cparamsA{}, gridA{}, idxA{};
        if (clustered) {
            const glm::mat4& P = view.proj;
            ClusterParamsGPU cpp;
            cpp.view    = view.view;
            const float tanH = 1.0f / P[1][1];
            const float aspect = P[1][1] / P[0][0];
            const float zn = P[3][2] / P[2][2];
            const float zf = P[2][2] * zn / (1.0f + P[2][2]);
            cpp.frustum = glm::vec4(tanH, aspect, zn, zf);
            cpp.screen  = glm::vec4(float(view.width), float(view.height), 0.0f, 0.0f);
            cpp.gridDim = glm::uvec4(cluster::CX, cluster::CY, cluster::CZ, cluster::MAX_PER);
            cpp.misc    = glm::uvec4(static_cast<uint32_t>(view.pointLights.size()), 0, 0, 0);
            cparamsA = I.ring.upload(&cpp, 1);
            gridA = I.ring.alloc(cluster::COUNT * sizeof(uint32_t));
            idxA  = I.ring.alloc(static_cast<uint64_t>(cluster::COUNT) * cluster::MAX_PER * sizeof(uint32_t));
            // Thread-per-light scatter accumulates counts atomically, so the count buffer must
            // start zeroed. Ring memory is CPU-visible (Shared) and this frame's arena is not in
            // GPU use (frames-in-flight throttle), so a host memset here is safe + cheap.
            if (gridA.ptr) std::memset(gridA.ptr, 0, cluster::COUNT * sizeof(uint32_t));

            rhi::PipelineHandle binPipe = I.clusterPipeline;
            FrameRingAllocator::Alloc lightsA = lights;
            const uint32_t nLights = static_cast<uint32_t>(view.pointLights.size());
            graph.addComputePass(
                "cluster-binning", /*reads=*/{}, /*writes=*/{},
                [binPipe, cparamsA, lightsA, gridA, idxA, nLights](rhi::CommandList& cl) {
                    cl.bindPipeline(binPipe);
                    std::array<rhi::BufferBinding, 4> b{};
                    b[0] = { .binding = 0, .buffer = cparamsA.buffer, .offset = cparamsA.offset };
                    b[1] = { .binding = 1, .buffer = lightsA.buffer,  .offset = lightsA.offset };
                    b[2] = { .binding = 2, .buffer = gridA.buffer,    .offset = gridA.offset };
                    b[3] = { .binding = 3, .buffer = idxA.buffer,     .offset = idxA.offset };
                    rhi::ResourceBindings rb; rb.buffers = std::span<const rhi::BufferBinding>(b.data(), 4);
                    cl.bindResources(rb);
                    cl.dispatch((nLights + 63) / 64, 1, 1);   // one thread per light
                });
        }

        const bool hdr = I.tonemapPipeline.valid();
        if (hdr) I.ensureHDR(view.width, view.height);

        RenderGraph::ColorTarget color;
        color.rt  = hdr ? I.hdrRT  : view.target;
        color.tex = hdr ? I.hdrTex : rhi::TextureHandle{};
        for (int i = 0; i < 4; ++i) color.clear[i] = view.clearColor[i];

        RenderGraph::DepthTarget depth;
        depth.rt = I.depthRT; depth.tex = I.depthTex; depth.used = true;
        depth.load = rhi::LoadOp::Clear; depth.store = rhi::StoreOp::DontCare;

        // Everything the draw loop needs, captured by value (the graph executes within this
        // render() call, so the item span still points at valid caller memory).
        GeometryStore* geom = I.geometry;
        rhi::BufferHandle vtx = geom->vertexBuffer();
        rhi::BufferHandle idx = geom->indexBuffer();
        auto items = view.items;

        graph.addRasterPass(
            "forward", color, depth, view.width, view.height, /*reads=*/{},
            [geom, vtx, idx, items, cam, inst, mat, lights, clustered, cparamsA, gridA, idxA](rhi::CommandList& cl) {
                std::array<rhi::BufferBinding, 7> binds{};
                uint32_t n = 0;
                binds[n++] = { .binding = 0, .buffer = cam.buffer, .offset = cam.offset };
                if (inst.buffer.valid())   binds[n++] = { .binding = 1, .buffer = inst.buffer,   .offset = inst.offset };
                if (mat.buffer.valid())    binds[n++] = { .binding = 2, .buffer = mat.buffer,    .offset = mat.offset };
                if (lights.buffer.valid()) binds[n++] = { .binding = 3, .buffer = lights.buffer, .offset = lights.offset };
                if (clustered) {
                    binds[n++] = { .binding = 4, .buffer = cparamsA.buffer, .offset = cparamsA.offset };
                    binds[n++] = { .binding = 5, .buffer = gridA.buffer,    .offset = gridA.offset };
                    binds[n++] = { .binding = 6, .buffer = idxA.buffer,     .offset = idxA.offset };
                }
                rhi::ResourceBindings rb;
                rb.buffers = std::span<const rhi::BufferBinding>(binds.data(), n);

                cl.bindResources(rb);
                cl.bindVertexBuffer(vtx, 0);
                cl.bindIndexBuffer(idx, rhi::IndexType::Uint32);
                for (const auto& item : items) {
                    cl.bindPipeline(item.pipeline);
                    const auto range = geom->range(item.mesh);
                    cl.drawIndexed(range.indexCount, item.instanceCount,
                                   range.firstIndex, range.vertexOffset, item.firstInstance);
                }
            });

        // HDR resolve: a fullscreen pass samples the HDR target and tone-maps it to view.target.
        // The graph transitions hdrTex RenderTarget -> ShaderRead between the two passes.
        if (hdr) {
            rhi::PipelineHandle tmPipe = I.tonemapPipeline;
            rhi::SamplerHandle  tmSamp = I.tonemapSampler;
            rhi::TextureHandle  hdrTex = I.hdrTex;
            RenderGraph::ColorTarget tmColor;
            tmColor.rt = view.target;
            tmColor.load = rhi::LoadOp::DontCare;   // fullscreen triangle writes every pixel
            RenderGraph::DepthTarget noDepth;       // used = false
            graph.addRasterPass(
                "tonemap", tmColor, noDepth, view.width, view.height, /*reads=*/{ hdrTex },
                [tmPipe, tmSamp, hdrTex](rhi::CommandList& cl) {
                    cl.bindPipeline(tmPipe);
                    rhi::TextureBinding tb{ .binding = 0, .texture = hdrTex };
                    rhi::SamplerBinding sb{ .binding = 0, .sampler = tmSamp };
                    rhi::ResourceBindings rb;
                    rb.textures = std::span<const rhi::TextureBinding>(&tb, 1);
                    rb.samplers = std::span<const rhi::SamplerBinding>(&sb, 1);
                    cl.bindResources(rb);
                    cl.draw(3, 1, 0, 0);   // fullscreen triangle from SV_VertexID
                });
        }
    }

    graph.execute(frame);
}

} // namespace engine::render
