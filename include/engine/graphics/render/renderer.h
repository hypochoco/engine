//
//  renderer.h
//  engine::graphics / render
//
//  Mid-level renderer. Owns pipeline definitions, pass/target setup, and the per-frame
//  instance/uniform ring buffers. Consumes per-view render lists and records RHI commands.
//  Owns NO scene data — that flows in from the ECS as RenderViews.
//

#pragma once

#include <memory>
#include <span>

#include "engine/graphics/render/graphics_config.h"
#include "engine/graphics/view/render_view.h"

namespace engine::rhi { class Device; class FrameContext; }

namespace engine::render {

class GeometryStore;

class Renderer {
public:
    Renderer(rhi::Device&, GeometryStore&);
    Renderer(Renderer&&) noexcept;
    Renderer& operator=(Renderer&&) noexcept;
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    ~Renderer();

    // Records the given views into the frame's command list: uploads per-view camera +
    // instance data, then binds the scene mesh pipeline once and issues one instanced indexed
    // draw per RenderItem. The Device owns beginFrame/endFrame (present or readback); the Renderer
    // owns pipelinesʼ per-frame buffers + the depth target. Manages no scene data.
    void render(rhi::FrameContext& frame, std::span<const RenderView> views);

    // --- Centralized config (preferred) -------------------------------------------------------
    // The renderer holds one GraphicsConfig (tunable params + per-feature enable flags) and one
    // RenderResources bundle (the app-supplied pipeline/sampler handles). A feature runs when its
    // config flag is on AND its resource is valid. Set the resources once (or on pipeline rebuild)
    // and drive everything else by mutating the config — see engine/graphics/render/graphics_config.h.
    void setConfig(const GraphicsConfig& config);
    const GraphicsConfig& config() const;
    void setResources(const RenderResources& resources);

    // Set the opaque scene (mesh) pipeline the forward pass binds for every RenderItem. Convenience
    // wrapper over RenderResources.mesh (the app builds the pipeline; the engine builds none).
    void setMeshPipeline(rhi::PipelineHandle meshPipeline);

    // --- Feature setters (thin wrappers over the config/resources above; kept for convenience) --
    // Enable clustered forward+ lighting. Pass a compute pipeline built from cluster.metallib
    // (the engine loads no shaders itself). When enabled, each view with point lights runs a
    // froxel light-binning compute pass before its forward pass, and the forward shader loops
    // only its cluster's lights. Pass an invalid handle to disable (loop-all fallback).
    void setClusterBinning(rhi::PipelineHandle binningPipeline);

    // Enable HDR + tone mapping. Pass a fullscreen tonemap pipeline built from tonemap.metallib
    // (color format = the final target's) and a sampler. When set, each view renders into an
    // RGBA16F HDR target, then a fullscreen tonemap pass resolves it to view.target. The scene
    // (mesh) pipeline must then target RGBA16Float. Pass an invalid pipeline to disable.
    void setTonemap(rhi::PipelineHandle tonemapPipeline, rhi::SamplerHandle sampler);

    // Enable directional (sun) shadow mapping. Pass a depth-only pipeline built from shadow.metallib
    // (no fragment; depthFormat Depth32Float) and a Nearest sampler. Each view renders the scene
    // into an orthographic shadow map from the sun's direction (view.light.direction), then the
    // forward shader PCF-samples it. `orthoHalfExtent`/`depthRange` size the light volume (centered
    // at the origin). Pass an invalid pipeline to disable.
    void setShadows(rhi::PipelineHandle shadowPipeline, rhi::SamplerHandle sampler,
                    float orthoHalfExtent = 25.0f, float depthRange = 100.0f, float bias = 0.0018f);

    // Enable the procedural sky. Pass a fullscreen pipeline built from sky.metallib (color format =
    // the forward color target's — RGBA16Float when tonemapping, else the final target; depthFormat
    // Depth32Float; depth test LessEqual, depth write OFF). The sky is drawn at the far plane at the
    // END of each forward pass, filling only background pixels; it is coupled to the view's sun
    // (view.light.direction). Pass an invalid pipeline to disable (flat clear color). `setSkyColors`
    // optionally overrides the default palette / sun look. Colors are scene-referred (HDR).
    void setSky(rhi::PipelineHandle skyPipeline);
    void setSkyColors(const glm::vec3& zenith, const glm::vec3& horizon, const glm::vec3& ground,
                      const glm::vec3& sunColor, float sunAngularRadiusDeg, float glowExponent,
                      float glowStrength, float brightness);

    // Enable aerial-perspective + height fog on opaque geometry (applied in the forward shader —
    // no pipeline needed). Distant/low fragments blend toward `color` (a sky-consistent tint) plus
    // a sun in-scatter glow (`inscatterColor`, sharpened by `inscatterExponent`) toward the view's
    // sun. `density` sets distance extinction; fog thins above `baseHeight` at `heightFalloff` per
    // world unit. Colors are scene-referred (HDR). OFF by default; call setFog(0) to disable.
    void setFog(float density, float heightFalloff = 0.0f, float baseHeight = 0.0f,
                const glm::vec3& color = glm::vec3(0.7f, 0.8f, 0.9f),
                const glm::vec3& inscatterColor = glm::vec3(0.0f),
                float inscatterExponent = 8.0f);

    // Enable hardware MSAA on the forward pass. The forward pass renders into `sampleCount`× MSAA
    // color + depth targets and resolves into the single-sample target (the HDR buffer when
    // tonemapping, else the view target). `sampleCount` must be a power of two the device supports
    // (2/4/8); 1 disables. **The app must build its mesh AND sky pipelines with a matching
    // `GraphicsPipelineDesc.sampleCount`** (the engine builds no pipelines).
    void setMSAA(uint32_t sampleCount);

    // Enable FXAA post anti-aliasing. Pass a fullscreen pipeline built from fxaa.metallib (color
    // format = the FINAL/present target's) and a linear-clamp sampler. When set, the pre-FXAA stage
    // (tonemap if HDR, else the forward pass) writes to an intermediate RGBA8 LDR texture, then FXAA
    // resolves it to the view target. So: **when FXAA is on, build the tonemap pipeline to output
    // RGBA8Unorm** (the intermediate) and the FXAA pipeline to output the final target format. Pass
    // an invalid pipeline to disable. Independent of MSAA (use either or both).
    void setFXAA(rhi::PipelineHandle fxaaPipeline, rhi::SamplerHandle sampler);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace engine::render
