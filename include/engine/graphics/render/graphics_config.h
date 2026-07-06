//
//  graphics_config.h
//  engine::graphics / render
//
//  Centralized, tunable renderer configuration (G1 of the graphics-config plan; see
//  notes/investigations/2026-07-05-graphics-config-system.md). One place that lists every graphics
//  feature toggle + tuning knob, un-burying what used to be scattered across the `Renderer::setX()`
//  setters and two file-scope `constexpr` blocks (shadow map size, froxel grid).
//
//  Two disjoint surfaces (graphics' wrinkle vs the physics config):
//    • GraphicsConfig  — plain data: tuning params + explicit `enabled` flags. Value-type, no
//                        singleton; pass by value, override, serialize.
//    • RenderResources — the GPU pipeline/sampler handles the app must supply (the engine builds no
//                        shaders/pipelines, so these can't have data defaults).
//  A feature is ACTIVE when `config.<feature>.enabled` AND its resource handle is valid.
//
//  Defaults here MUST equal the previous hardcoded values (G1 is a no-behavior-change refactor).
//

#pragma once

#include <cstdint>
#include <optional>

#include <glm/glm.hpp>

#include "engine/graphics/rhi/types.h"

namespace engine::render {

// Directional (sun) shadow mapping. `enabled` needs `RenderResources.shadow` (a depth-only
// pipeline) + `shadowSampler` to actually run.
struct ShadowConfig {
    bool     enabled        = false;
    float    orthoHalfExtent = 25.0f;    // half-size of the light's ortho volume (world units)
    float    depthRange     = 100.0f;    // ortho depth range along the light
    float    bias           = 0.0018f;   // slope-scaled base depth-compare bias (peter-panning knob)
    uint32_t mapSize        = 2048;      // shadow map resolution (square); was shadow::MAP_SIZE
};

// Procedural sky (far-plane fullscreen triangle at the end of the forward pass). Colors are
// scene-referred (HDR). `enabled` needs `RenderResources.sky`.
struct SkyConfig {
    bool      enabled       = false;
    glm::vec3 zenith        {0.10f, 0.24f, 0.55f};
    glm::vec3 horizon       {0.62f, 0.72f, 0.86f};
    glm::vec3 ground        {0.28f, 0.26f, 0.24f};
    glm::vec3 sunColor      {12.0f, 10.5f, 8.0f};
    float     sunAngularRadiusDeg = 1.0f;   // apparent sun radius; cos() computed at upload
    float     glowExponent  = 250.0f;
    float     glowStrength  = 0.5f;
    float     brightness    = 1.0f;
};

// Aerial-perspective + height fog, applied in the forward fragment shader (no pipeline needed).
// Colors are scene-referred (HDR).
struct FogConfig {
    bool      enabled           = false;
    float     density           = 0.0f;    // distance extinction
    float     heightFalloff     = 0.0f;    // per-world-unit thinning above baseHeight
    float     baseHeight        = 0.0f;
    float     inscatterExponent = 8.0f;    // sharpness of the sun in-scatter lobe
    glm::vec3 color             {0.7f, 0.8f, 0.9f};   // base extinction tint
    glm::vec3 inscatterColor    {0.0f};                // sun in-scatter color
};

// Anti-aliasing. MSAA runs on the forward pass (see decision 4 in the note: the app must build its
// mesh+sky pipelines with a matching sampleCount). FXAA needs `RenderResources.fxaa` + `fxaaSampler`.
struct AAConfig {
    uint32_t msaaSamples = 1;      // 1 = off; 2/4/8 = hardware MSAA
    bool     fxaa        = false;  // FXAA post pass
};

// Clustered forward+ froxel grid. Moved here for visibility/recording, but EFFECTIVELY COMPILE-TIME
// in G1 (froxel buffer sizing + shader loop bounds are coupled to these; runtime resize is out of
// scope — see decision 5). Defaults mirror the former `cluster::` constexpr. `enabled` needs
// `RenderResources.clusterBinning` + a view with point lights.
struct ClusterConfig {
    bool     enabled       = false;
    uint32_t gridX         = 12;
    uint32_t gridY         = 12;
    uint32_t gridZ         = 24;
    uint32_t maxPerCluster = 64;
};

// The single renderer-global config surface. Value-type: copy it, tweak it, hand it to the renderer.
struct GraphicsConfig {
    bool          hdr = false;   // HDR render target + tone-map resolve (needs RenderResources.tonemap)
    ShadowConfig  shadow;
    SkyConfig     sky;
    FogConfig     fog;
    AAConfig      aa;
    ClusterConfig cluster;
};

// The GPU resources the app supplies (the engine loads no shaders/pipelines). Split from
// GraphicsConfig because handles aren't tunable data and can't have meaningful defaults. Set once
// (or whenever pipelines are rebuilt); the enable flags in GraphicsConfig gate whether they run.
struct RenderResources {
    rhi::PipelineHandle mesh;            // the opaque scene (mesh) pipeline, bound for every RenderItem
    rhi::PipelineHandle tonemap;         // fullscreen HDR tone-map (color format = pre-FXAA target)
    rhi::PipelineHandle shadow;          // depth-only shadow caster
    rhi::PipelineHandle sky;             // fullscreen procedural sky
    rhi::PipelineHandle fxaa;            // fullscreen FXAA post
    rhi::PipelineHandle clusterBinning;  // froxel light-binning compute

    rhi::SamplerHandle  tonemapSampler;
    rhi::SamplerHandle  shadowSampler;   // Nearest (PCF taps offset in-shader)
    rhi::SamplerHandle  fxaaSampler;     // linear clamp
};

// ---- Override layering (G2) ------------------------------------------------------------------
// A sparse override: only the set (engaged optional) fields replace the base. Lets a scene / quality
// preset specify just the knobs it changes on top of a base (defaults or another preset). Mirrors
// the physics config override system. Resources are NOT overridable here (they're handles, wired
// separately) — overrides tune data + toggles only.
template <class T> void applyOverride(T& dst, const std::optional<T>& o) { if (o) dst = *o; }

struct ShadowConfigOverride {
    std::optional<bool>     enabled;
    std::optional<float>    orthoHalfExtent, depthRange, bias;
    std::optional<uint32_t> mapSize;
};
struct SkyConfigOverride {
    std::optional<bool>      enabled;
    std::optional<glm::vec3> zenith, horizon, ground, sunColor;
    std::optional<float>     sunAngularRadiusDeg, glowExponent, glowStrength, brightness;
};
struct FogConfigOverride {
    std::optional<bool>      enabled;
    std::optional<float>     density, heightFalloff, baseHeight, inscatterExponent;
    std::optional<glm::vec3> color, inscatterColor;
};
struct AAConfigOverride {
    std::optional<uint32_t> msaaSamples;
    std::optional<bool>     fxaa;
};
struct ClusterConfigOverride {
    std::optional<bool>     enabled;
    std::optional<uint32_t> gridX, gridY, gridZ, maxPerCluster;
};
struct GraphicsConfigOverride {
    std::optional<bool>   hdr;
    ShadowConfigOverride  shadow;
    SkyConfigOverride     sky;
    FogConfigOverride     fog;
    AAConfigOverride      aa;
    ClusterConfigOverride cluster;
};

// Resolve an override onto a base, returning the effective GraphicsConfig (base for every unset field).
inline GraphicsConfig resolve(GraphicsConfig base, const GraphicsConfigOverride& o) {
    applyOverride(base.hdr, o.hdr);
    applyOverride(base.shadow.enabled,         o.shadow.enabled);
    applyOverride(base.shadow.orthoHalfExtent, o.shadow.orthoHalfExtent);
    applyOverride(base.shadow.depthRange,      o.shadow.depthRange);
    applyOverride(base.shadow.bias,            o.shadow.bias);
    applyOverride(base.shadow.mapSize,         o.shadow.mapSize);
    applyOverride(base.sky.enabled,             o.sky.enabled);
    applyOverride(base.sky.zenith,              o.sky.zenith);
    applyOverride(base.sky.horizon,             o.sky.horizon);
    applyOverride(base.sky.ground,              o.sky.ground);
    applyOverride(base.sky.sunColor,            o.sky.sunColor);
    applyOverride(base.sky.sunAngularRadiusDeg, o.sky.sunAngularRadiusDeg);
    applyOverride(base.sky.glowExponent,        o.sky.glowExponent);
    applyOverride(base.sky.glowStrength,        o.sky.glowStrength);
    applyOverride(base.sky.brightness,          o.sky.brightness);
    applyOverride(base.fog.enabled,           o.fog.enabled);
    applyOverride(base.fog.density,           o.fog.density);
    applyOverride(base.fog.heightFalloff,     o.fog.heightFalloff);
    applyOverride(base.fog.baseHeight,        o.fog.baseHeight);
    applyOverride(base.fog.inscatterExponent, o.fog.inscatterExponent);
    applyOverride(base.fog.color,             o.fog.color);
    applyOverride(base.fog.inscatterColor,    o.fog.inscatterColor);
    applyOverride(base.aa.msaaSamples, o.aa.msaaSamples);
    applyOverride(base.aa.fxaa,        o.aa.fxaa);
    applyOverride(base.cluster.enabled,       o.cluster.enabled);
    applyOverride(base.cluster.gridX,         o.cluster.gridX);
    applyOverride(base.cluster.gridY,         o.cluster.gridY);
    applyOverride(base.cluster.gridZ,         o.cluster.gridZ);
    applyOverride(base.cluster.maxPerCluster, o.cluster.maxPerCluster);
    return base;
}

// ---- Named presets (G2) ----------------------------------------------------------------------
// Base quality configs; a scene layers a sparse GraphicsConfigOverride on top via resolve(). Enable
// flags express intent — a feature still only runs when its RenderResources handle is also valid.
namespace presets {

// Everything off — the engine defaults / parity baseline.
inline GraphicsConfig baseline() { return GraphicsConfig{}; }

// Cheap: FXAA (one fullscreen pass) instead of MSAA, a smaller shadow map. Favors framerate.
inline GraphicsConfig performance() {
    GraphicsConfig c;
    c.aa.msaaSamples = 1;
    c.aa.fxaa        = true;
    c.shadow.mapSize = 1024;
    return c;
}

// High quality: HDR + 4× MSAA + a large shadow map + sky + subtle fog. Favors image quality.
inline GraphicsConfig cinematic() {
    GraphicsConfig c;
    c.hdr             = true;
    c.aa.msaaSamples  = 4;
    c.shadow.enabled  = true;
    c.shadow.mapSize  = 4096;
    c.sky.enabled     = true;
    c.fog.enabled     = true;
    c.fog.density     = 0.01f;
    c.fog.heightFalloff = 0.02f;
    return c;
}

} // namespace presets

} // namespace engine::render
