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
#include <cmath>
#include <cstddef>
#include <cstring>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
    glm::mat4 lightViewProj{1.0f}; // world → light clip (directional shadow map)
    glm::vec4 lightDir{0.0f};     // xyz = normalized direction TO the light; w = shadow bias
    glm::vec4 lightColor{1.0f};   // rgb = color * intensity; w unused
    glm::vec4 ambient{0.0f};      // rgb = ambient term; w unused
    glm::vec4 params{0.0f};       // x = point light count; y = clustered?; z = shadows?; w = shadowMapSize
    glm::vec4 camPos{0.0f};       // xyz = camera world pos; w = fog enabled? (>0.5)
    glm::vec4 fogColor{0.0f};     // rgb = base extinction tint (scene-referred)
    glm::vec4 fogInscatter{0.0f}; // rgb = sun in-scatter color (scene-referred)
    glm::vec4 fogParams{0.0f};    // x = density; y = heightFalloff; z = baseY; w = inscatter exponent
};

// CPU mirror of shadow.slang's ShadowGlobals.
struct ShadowGlobalsGPU {
    glm::mat4 lightViewProj{1.0f};
};

// CPU mirror of sky.slang's SkyGlobals (constant-buffer layout: mat4 then vec4s, 16B-aligned).
struct SkyGlobalsGPU {
    glm::mat4 invViewProj{1.0f};
    glm::vec4 camPos{0.0f};    // xyz camera world pos
    glm::vec4 sunDir{0.0f};    // xyz toward the sun
    glm::vec4 zenith{0.0f};
    glm::vec4 horizon{0.0f};
    glm::vec4 ground{0.0f};
    glm::vec4 sunColor{0.0f};  // scene-referred
    glm::vec4 params{0.0f};    // x=cos(sunRadius) y=glowExp z=glowStrength w=brightness
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

    // Centralized config: tunable params + per-feature enable flags, and the app-supplied GPU
    // handles. A feature runs when its config flag is on AND its resource is valid.
    GraphicsConfig  config_{};
    RenderResources resources_{};

    // HDR + tone mapping (config_.hdr + resources_.tonemap). Forward renders into hdrTex (RGBA16F),
    // then a fullscreen tonemap pass resolves it.
    rhi::TextureHandle      hdrTex;
    rhi::RenderTargetHandle hdrRT;
    uint32_t                hdrW = 0, hdrH = 0;

    // MSAA (config_.aa.msaaSamples): the forward pass renders into these multisampled targets and
    // resolves into the single-sample HDR/view target. Memoryless (on-tile) — never read back.
    rhi::TextureHandle      msaaColorTex;
    rhi::RenderTargetHandle msaaColorRT;
    rhi::TextureHandle      msaaDepthTex;
    rhi::RenderTargetHandle msaaDepthRT;
    uint32_t                msaaW = 0, msaaH = 0, msaaN = 0;
    rhi::Format             msaaFmt = rhi::Format::Undefined;

    void ensureMSAA(uint32_t w, uint32_t h, uint32_t samples, rhi::Format colorFmt) {
        if (msaaColorTex.valid() && w == msaaW && h == msaaH && samples == msaaN && colorFmt == msaaFmt)
            return;
        if (msaaColorTex.valid()) device->destroy(msaaColorTex);
        if (msaaColorRT.valid())  device->destroy(msaaColorRT);
        if (msaaDepthTex.valid()) device->destroy(msaaDepthTex);
        if (msaaDepthRT.valid())  device->destroy(msaaDepthRT);
        msaaColorTex = device->createTexture(
            { .width = w, .height = h, .sampleCount = samples, .format = colorFmt,
              .usage = rhi::TextureUsage::ColorTarget, .transient = true });
        msaaColorRT = device->createRenderTarget(msaaColorTex);
        msaaDepthTex = device->createTexture(
            { .width = w, .height = h, .sampleCount = samples, .format = rhi::Format::Depth32Float,
              .usage = rhi::TextureUsage::DepthTarget, .transient = true });
        msaaDepthRT = device->createRenderTarget(msaaDepthTex);
        msaaW = w; msaaH = h; msaaN = samples; msaaFmt = colorFmt;
    }

    // FXAA (config_.aa.fxaa + resources_.fxaa). The pre-FXAA stage writes this intermediate LDR
    // texture; the FXAA pass samples it and writes the view target. Fixed RGBA8Unorm (the app builds
    // its tonemap pipeline to output RGBA8Unorm when FXAA is on).
    rhi::TextureHandle      ldrTex;
    rhi::RenderTargetHandle ldrRT;
    uint32_t                ldrW = 0, ldrH = 0;

    void ensureLDR(uint32_t w, uint32_t h) {
        if (ldrTex.valid() && w == ldrW && h == ldrH) return;
        if (ldrTex.valid()) device->destroy(ldrTex);
        if (ldrRT.valid())  device->destroy(ldrRT);
        ldrTex = device->createTexture(
            { .width = w, .height = h, .format = rhi::Format::RGBA8Unorm,
              .usage = rhi::TextureUsage::ColorTarget | rhi::TextureUsage::Sampled });
        ldrRT = device->createRenderTarget(ldrTex);
        ldrW = w; ldrH = h;
    }

    // Directional sun shadow map (config_.shadow + resources_.shadow).
    rhi::TextureHandle      shadowTex;
    rhi::RenderTargetHandle shadowRT;
    uint32_t                shadowMapW = 0;   // tracks the created map size for lazy re-alloc

    void ensureShadowMap() {
        const uint32_t sz = config_.shadow.mapSize;
        if (shadowTex.valid() && shadowMapW == sz) return;
        if (shadowTex.valid()) device->destroy(shadowTex);
        if (shadowRT.valid())  device->destroy(shadowRT);
        shadowTex = device->createTexture(
            { .width = sz, .height = sz, .format = rhi::Format::Depth32Float,
              .usage = rhi::TextureUsage::DepthTarget | rhi::TextureUsage::Sampled });
        shadowRT = device->createRenderTarget(shadowTex);
        shadowMapW = sz;
    }

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
        if (impl_->shadowTex.valid()) impl_->device->destroy(impl_->shadowTex);
        if (impl_->shadowRT.valid())  impl_->device->destroy(impl_->shadowRT);
        if (impl_->msaaColorTex.valid()) impl_->device->destroy(impl_->msaaColorTex);
        if (impl_->msaaColorRT.valid())  impl_->device->destroy(impl_->msaaColorRT);
        if (impl_->msaaDepthTex.valid()) impl_->device->destroy(impl_->msaaDepthTex);
        if (impl_->msaaDepthRT.valid())  impl_->device->destroy(impl_->msaaDepthRT);
        if (impl_->ldrTex.valid()) impl_->device->destroy(impl_->ldrTex);
        if (impl_->ldrRT.valid())  impl_->device->destroy(impl_->ldrRT);
    }
}

void Renderer::setConfig(const GraphicsConfig& config) { impl_->config_ = config; }
const GraphicsConfig& Renderer::config() const { return impl_->config_; }
void Renderer::setResources(const RenderResources& resources) { impl_->resources_ = resources; }

void Renderer::setMeshPipeline(rhi::PipelineHandle meshPipeline) { impl_->resources_.mesh = meshPipeline; }

void Renderer::setClusterBinning(rhi::PipelineHandle binningPipeline) {
    impl_->resources_.clusterBinning = binningPipeline;
    impl_->config_.cluster.enabled   = binningPipeline.valid();
}

void Renderer::setShadows(rhi::PipelineHandle shadowPipeline, rhi::SamplerHandle sampler,
                          float orthoHalfExtent, float depthRange, float bias) {
    impl_->resources_.shadow        = shadowPipeline;
    impl_->resources_.shadowSampler = sampler;
    impl_->config_.shadow.enabled         = shadowPipeline.valid();
    impl_->config_.shadow.orthoHalfExtent = orthoHalfExtent;
    impl_->config_.shadow.depthRange      = depthRange;
    impl_->config_.shadow.bias            = bias;
}

void Renderer::setTonemap(rhi::PipelineHandle tonemapPipeline, rhi::SamplerHandle sampler) {
    impl_->resources_.tonemap        = tonemapPipeline;
    impl_->resources_.tonemapSampler = sampler;
    impl_->config_.hdr               = tonemapPipeline.valid();
}

void Renderer::setSky(rhi::PipelineHandle skyPipeline) {
    impl_->resources_.sky    = skyPipeline;
    impl_->config_.sky.enabled = skyPipeline.valid();
}

void Renderer::setSkyColors(const glm::vec3& zenith, const glm::vec3& horizon,
                            const glm::vec3& ground, const glm::vec3& sunColor,
                            float sunAngularRadiusDeg, float glowExponent,
                            float glowStrength, float brightness) {
    SkyConfig& s = impl_->config_.sky;
    s.zenith    = zenith;
    s.horizon   = horizon;
    s.ground    = ground;
    s.sunColor  = sunColor;
    s.sunAngularRadiusDeg = sunAngularRadiusDeg;
    s.glowExponent = glowExponent;
    s.glowStrength = glowStrength;
    s.brightness   = brightness;
}

void Renderer::setFog(float density, float heightFalloff, float baseHeight,
                      const glm::vec3& color, const glm::vec3& inscatterColor,
                      float inscatterExponent) {
    FogConfig& f = impl_->config_.fog;
    f.enabled           = density > 0.0f;
    f.density           = density;
    f.heightFalloff     = heightFalloff;
    f.baseHeight        = baseHeight;
    f.color             = color;
    f.inscatterColor    = inscatterColor;
    f.inscatterExponent = inscatterExponent;
}

void Renderer::setMSAA(uint32_t sampleCount) {
    impl_->config_.aa.msaaSamples = sampleCount < 1 ? 1 : sampleCount;
}

void Renderer::setFXAA(rhi::PipelineHandle fxaaPipeline, rhi::SamplerHandle sampler) {
    impl_->resources_.fxaa        = fxaaPipeline;
    impl_->resources_.fxaaSampler = sampler;
    impl_->config_.aa.fxaa        = fxaaPipeline.valid();
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
        const bool clustered = I.config_.cluster.enabled && I.resources_.clusterBinning.valid()
                               && !view.pointLights.empty();

        // Directional sun shadow map active? Compute the light's orthographic view-proj (centered
        // at the origin, looking along the sun direction).
        const bool shadows = I.config_.shadow.enabled && I.resources_.shadow.valid();
        glm::mat4 lightVP(1.0f);
        if (shadows) {
            I.ensureShadowMap();
            const glm::vec3 dir = glm::normalize(view.light.direction);   // direction light travels
            const glm::vec3 center(0.0f);
            const glm::vec3 lightPos = center - dir * (I.config_.shadow.depthRange * 0.5f);
            const glm::vec3 up = (std::abs(dir.y) > 0.99f) ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
            const glm::mat4 lv = glm::lookAt(lightPos, center, up);
            const float ext = I.config_.shadow.orthoHalfExtent;
            const glm::mat4 lp = glm::ortho(-ext, ext, -ext, ext, 0.0f, I.config_.shadow.depthRange);
            lightVP = lp * lv;
        }

        GlobalUniforms g;
        g.viewProj      = view.proj * view.view;
        g.lightViewProj = lightVP;
        g.lightDir   = glm::vec4(glm::normalize(-view.light.direction), I.config_.shadow.bias);  // xyz→light, w=bias
        g.lightColor = glm::vec4(view.light.color * view.light.intensity, 0.0f);
        g.ambient    = glm::vec4(view.light.ambient, 0.0f);
        g.params     = glm::vec4(static_cast<float>(view.pointLights.size()),
                                 clustered ? 1.0f : 0.0f,
                                 shadows ? 1.0f : 0.0f,
                                 static_cast<float>(I.config_.shadow.mapSize));
        // Camera world position + aerial-perspective fog (applied in the forward shader).
        const glm::vec3 camPos = glm::vec3(glm::inverse(view.view)[3]);
        const FogConfig& fog = I.config_.fog;
        g.camPos       = glm::vec4(camPos, fog.enabled ? 1.0f : 0.0f);
        g.fogColor     = glm::vec4(fog.color, 0.0f);
        g.fogInscatter = glm::vec4(fog.inscatterColor, 0.0f);
        g.fogParams    = glm::vec4(fog.density, fog.heightFalloff, fog.baseHeight, fog.inscatterExponent);

        // Sub-allocate this view's upload data from the frame ring (distinct regions per view).
        const FrameRingAllocator::Alloc cam = I.ring.upload(&g, 1);
        FrameRingAllocator::Alloc inst{}, mat{}, lights{};
        if (!view.instances.empty())
            inst = I.ring.upload(view.instances.data(), view.instances.size());
        if (!view.materials.empty())
            mat  = I.ring.upload(view.materials.data(), view.materials.size());
        if (!view.pointLights.empty())
            lights = I.ring.upload(view.pointLights.data(), view.pointLights.size());

        GeometryStore* geom = I.geometry;
        rhi::BufferHandle vtx = geom->vertexBuffer();
        rhi::BufferHandle idx = geom->indexBuffer();
        auto items = view.items;

        // Shadow pass (depth-only): render the scene from the sun into the shadow map, BEFORE the
        // forward pass. The forward pass declares the shadow map as a read ⇒ the graph transitions
        // it RenderTarget→ShaderRead.
        if (shadows) {
            ShadowGlobalsGPU sg; sg.lightViewProj = lightVP;
            const FrameRingAllocator::Alloc sgA = I.ring.upload(&sg, 1);
            rhi::PipelineHandle shPipe = I.resources_.shadow;
            FrameRingAllocator::Alloc instS = inst;
            RenderGraph::ColorTarget noColor;   // invalid rt ⇒ depth-only
            RenderGraph::DepthTarget shDepth;
            shDepth.rt = I.shadowRT; shDepth.tex = I.shadowTex; shDepth.used = true;
            shDepth.load = rhi::LoadOp::Clear; shDepth.store = rhi::StoreOp::Store;
            const uint32_t shMap = I.config_.shadow.mapSize;
            graph.addRasterPass(
                "shadow", noColor, shDepth, shMap, shMap, /*reads=*/{},
                [geom, vtx, idx, items, sgA, instS, shPipe](rhi::CommandList& cl) {
                    cl.bindPipeline(shPipe);
                    std::array<rhi::BufferBinding, 2> b{};
                    uint32_t n = 0;
                    b[n++] = { .binding = 0, .buffer = sgA.buffer, .offset = sgA.offset };
                    if (instS.buffer.valid()) b[n++] = { .binding = 1, .buffer = instS.buffer, .offset = instS.offset };
                    rhi::ResourceBindings rb; rb.buffers = std::span<const rhi::BufferBinding>(b.data(), n);
                    cl.bindResources(rb);
                    cl.bindVertexBuffer(vtx, 0);
                    cl.bindIndexBuffer(idx, rhi::IndexType::Uint32);
                    for (const auto& item : items) {
                        const auto range = geom->range(item.mesh);
                        cl.drawIndexed(range.indexCount, item.instanceCount,
                                       range.firstIndex, range.vertexOffset, item.firstInstance);
                    }
                });
        }

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

            rhi::PipelineHandle binPipe = I.resources_.clusterBinning;
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

        const bool hdr = I.config_.hdr && I.resources_.tonemap.valid();
        const bool fxaaOn = I.config_.aa.fxaa && I.resources_.fxaa.valid();
        if (hdr) I.ensureHDR(view.width, view.height);
        if (fxaaOn) I.ensureLDR(view.width, view.height);

        // Output chain: forward[+MSAA resolve] -> [tonemap] -> [FXAA] -> view.target.
        // The stage before FXAA writes `preRT` (an intermediate LDR texture when FXAA is on).
        const rhi::RenderTargetHandle finalRT = view.target;
        const rhi::RenderTargetHandle preRT  = fxaaOn ? I.ldrRT  : finalRT;
        const rhi::TextureHandle      preTex = fxaaOn ? I.ldrTex : rhi::TextureHandle{};

        const uint32_t samples = I.config_.aa.msaaSamples;
        const bool msaa = samples > 1;
        // The single-sample destination the forward pass writes (HDR buffer when tonemapping, else
        // the pre-FXAA target). With MSAA this becomes the resolve destination.
        const rhi::RenderTargetHandle singleRT  = hdr ? I.hdrRT  : preRT;
        const rhi::TextureHandle      singleTex = hdr ? I.hdrTex : preTex;
        if (msaa) I.ensureMSAA(view.width, view.height, samples,
                               hdr ? rhi::Format::RGBA16Float : rhi::Format::RGBA8Unorm);

        RenderGraph::ColorTarget color;
        if (msaa) {
            color.rt = I.msaaColorRT; color.tex = I.msaaColorTex;   // render into MSAA...
            color.resolveRT = singleRT; color.resolveTex = singleTex;   // ...resolve into single-sample
        } else {
            color.rt = singleRT; color.tex = singleTex;
        }
        for (int i = 0; i < 4; ++i) color.clear[i] = view.clearColor[i];

        RenderGraph::DepthTarget depth;
        depth.rt = msaa ? I.msaaDepthRT : I.depthRT;
        depth.tex = msaa ? I.msaaDepthTex : I.depthTex;
        depth.used = true;
        depth.load = rhi::LoadOp::Clear; depth.store = rhi::StoreOp::DontCare;

        rhi::TextureHandle shadowTex = I.shadowTex;
        rhi::SamplerHandle shadowSamp = I.resources_.shadowSampler;
        std::vector<rhi::TextureHandle> forwardReads;
        if (shadows) forwardReads.push_back(shadowTex);

        // Procedural sky: drawn at the END of the forward pass (same render pass ⇒ on-tile on
        // Apple, depth-tested against opaque so it only fills background). Uploaded per view.
        const bool skyOn = I.config_.sky.enabled && I.resources_.sky.valid();
        FrameRingAllocator::Alloc skyA{};
        if (skyOn) {
            const SkyConfig& sky = I.config_.sky;
            SkyGlobalsGPU sk;
            sk.invViewProj = glm::inverse(g.viewProj);
            sk.camPos   = glm::vec4(glm::vec3(glm::inverse(view.view)[3]), 0.0f);
            sk.sunDir   = glm::vec4(glm::normalize(-view.light.direction), 0.0f);
            sk.zenith   = glm::vec4(sky.zenith, 0.0f);
            sk.horizon  = glm::vec4(sky.horizon, 0.0f);
            sk.ground   = glm::vec4(sky.ground, 0.0f);
            sk.sunColor = glm::vec4(sky.sunColor, 0.0f);
            const float sunCos = std::cos(glm::radians(sky.sunAngularRadiusDeg));
            sk.params   = glm::vec4(sunCos, sky.glowExponent, sky.glowStrength, sky.brightness);
            skyA = I.ring.upload(&sk, 1);
        }
        rhi::PipelineHandle skyPipe = I.resources_.sky;
        rhi::PipelineHandle meshPipe = I.resources_.mesh;

        graph.addRasterPass(
            "forward", color, depth, view.width, view.height, forwardReads,
            [geom, vtx, idx, items, cam, inst, mat, lights, clustered, cparamsA, gridA, idxA,
             shadows, shadowTex, shadowSamp, skyOn, skyPipe, skyA, meshPipe](rhi::CommandList& cl) {
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
                rhi::TextureBinding tb{ .binding = 0, .texture = shadowTex };
                rhi::SamplerBinding sb{ .binding = 0, .sampler = shadowSamp };
                if (shadows) {
                    rb.textures = std::span<const rhi::TextureBinding>(&tb, 1);
                    rb.samplers = std::span<const rhi::SamplerBinding>(&sb, 1);
                }

                cl.bindResources(rb);
                cl.bindVertexBuffer(vtx, 0);
                cl.bindIndexBuffer(idx, rhi::IndexType::Uint32);
                cl.bindPipeline(meshPipe);   // one opaque scene pipeline for all items (single-pipeline scene)
                for (const auto& item : items) {
                    const auto range = geom->range(item.mesh);
                    cl.drawIndexed(range.indexCount, item.instanceCount,
                                   range.firstIndex, range.vertexOffset, item.firstInstance);
                }

                // Sky fills the background (far-plane fullscreen triangle, depth-tested LessEqual,
                // no depth write). Drawn last so opaque depth is already populated.
                if (skyOn) {
                    cl.bindPipeline(skyPipe);
                    rhi::BufferBinding sb{ .binding = 0, .buffer = skyA.buffer, .offset = skyA.offset };
                    rhi::ResourceBindings srb;
                    srb.buffers = std::span<const rhi::BufferBinding>(&sb, 1);
                    cl.bindResources(srb);
                    cl.draw(3, 1, 0, 0);   // fullscreen triangle from SV_VertexID
                }
            });

        // HDR resolve: a fullscreen pass samples the HDR target and tone-maps it to the pre-FXAA
        // target (the view target, or the LDR intermediate when FXAA is on).
        if (hdr) {
            rhi::PipelineHandle tmPipe = I.resources_.tonemap;
            rhi::SamplerHandle  tmSamp = I.resources_.tonemapSampler;
            rhi::TextureHandle  hdrTex = I.hdrTex;
            RenderGraph::ColorTarget tmColor;
            tmColor.rt = preRT; tmColor.tex = preTex;
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

        // FXAA: a fullscreen pass samples the pre-FXAA LDR intermediate and anti-aliases it to the
        // final view target. The graph transitions preTex RenderTarget -> ShaderRead.
        if (fxaaOn) {
            rhi::PipelineHandle fxPipe = I.resources_.fxaa;
            rhi::SamplerHandle  fxSamp = I.resources_.fxaaSampler;
            rhi::TextureHandle  srcTex = preTex;
            RenderGraph::ColorTarget fxColor;
            fxColor.rt = finalRT;
            fxColor.load = rhi::LoadOp::DontCare;
            RenderGraph::DepthTarget noDepth;
            graph.addRasterPass(
                "fxaa", fxColor, noDepth, view.width, view.height, /*reads=*/{ srcTex },
                [fxPipe, fxSamp, srcTex](rhi::CommandList& cl) {
                    cl.bindPipeline(fxPipe);
                    rhi::TextureBinding tb{ .binding = 0, .texture = srcTex };
                    rhi::SamplerBinding sb{ .binding = 0, .sampler = fxSamp };
                    rhi::ResourceBindings rb;
                    rb.textures = std::span<const rhi::TextureBinding>(&tb, 1);
                    rb.samplers = std::span<const rhi::SamplerBinding>(&sb, 1);
                    cl.bindResources(rb);
                    cl.draw(3, 1, 0, 0);
                });
        }
    }

    graph.execute(frame);
}

} // namespace engine::render
